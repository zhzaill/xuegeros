// Dependency-free RFC 6455 client for the robotd daemon (JSON text frames).
import net from "node:net";
import crypto from "node:crypto";
import { EventEmitter } from "node:events";
import { URL, URLSearchParams } from "node:url";

const GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
const OP = { CONT: 0, TEXT: 1, BINARY: 2, CLOSE: 8, PING: 9, PONG: 10 };
const MAX_PAYLOAD = 1 << 20;
const EMPTY = Buffer.alloc(0);

const acceptKey = (key) => crypto.createHash("sha1").update(key + GUID, "ascii").digest("base64");

// Client-to-server frames are always masked; server-to-client frames never are.
function encodeFrame(opcode, payload, mask) {
  const len = payload.length;
  let header;
  if (len < 126) { header = Buffer.alloc(2); header[1] = len; }
  else if (len < 65536) { header = Buffer.alloc(4); header[1] = 126; header.writeUInt16BE(len, 2); }
  else { header = Buffer.alloc(10); header[1] = 127; header.writeBigUInt64BE(BigInt(len), 2); }
  header[0] = 0x80 | opcode;
  if (!mask) return Buffer.concat([header, payload]);
  header[1] |= 0x80;
  const key = crypto.randomBytes(4);
  const body = Buffer.allocUnsafe(len);
  for (let i = 0; i < len; i++) body[i] = payload[i] ^ key[i & 3];
  return Buffer.concat([header, key, body]);
}

function handshakeRequest(host, path, key) {
  return `GET ${path} HTTP/1.1\r\nHost: ${host}\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n` +
    `Sec-WebSocket-Key: ${key}\r\nSec-WebSocket-Version: 13\r\n\r\n`;
}

export class RobotdClient extends EventEmitter {
  constructor({ url = "ws://127.0.0.1:18202/ws", token = "", reconnectMs = 1000, requestTimeoutMs = 4000 } = {}) {
    super();
    this.url = url;
    this.token = token;
    this.reconnectMs = reconnectMs;
    this.requestTimeoutMs = requestTimeoutMs;
    this._socket = null; this._buf = EMPTY; this._open = false;
    this._started = false; this._reconnectTimer = null;
    this._everConnected = false; this._state = null;
    this._inflight = new Map(); // message.t -> FIFO of {resolve, reject, timer}
  }

  get connected() { return this._open; }
  get state() { return this._state; }

  start() { if (this._started) return; this._started = true; this._connect(); }

  stop() {
    this._started = false;
    if (this._reconnectTimer) clearTimeout(this._reconnectTimer);
    this._reconnectTimer = null;
    const sock = this._socket;
    this._socket = null; this._open = false; this._buf = EMPTY;
    if (sock) sock.destroy();
    this._failInflight();
  }

  send(message) {
    const type = String((message && message.t) || "");
    if (!this._open || !this._socket) {
      return Promise.reject(Object.assign(new Error("not_connected"), { code: "not_connected" }));
    }
    let payload;
    try { payload = Buffer.from(JSON.stringify(message), "utf8"); }
    catch (err) { return Promise.reject(err); }
    const sock = this._socket;
    return new Promise((resolve, reject) => {
      const entry = {
        resolve,
        reject,
        timer: setTimeout(() => {
          this._remove(type, entry);
          reject(Object.assign(new Error(`timeout:${type}`), { code: "timeout" }));
        }, this.requestTimeoutMs),
      };
      if (entry.timer.unref) entry.timer.unref();
      const queue = this._inflight.get(type);
      if (queue) queue.push(entry);
      else this._inflight.set(type, [entry]);
      try { sock.write(encodeFrame(OP.TEXT, payload, true)); }
      catch (err) { this._remove(type, entry); reject(err); }
    });
  }

  request(command) { return this.send(command); }

  _connect() {
    if (!this._started) return;
    let target;
    try { target = new URL(this.url); }
    catch (err) { this._emitError(err); this._scheduleReconnect(); return; }
    const host = target.hostname.replace(/^\[|\]$/g, "");
    const port = Number(target.port || 80);
    let path = target.pathname || "/";
    const query = new URLSearchParams(target.search);
    if (this.token) query.set("token", this.token);
    const qs = query.toString();
    if (qs) path += `?${qs}`;
    const key = crypto.randomBytes(16).toString("base64");
    const expected = acceptKey(key);
    const sock = net.connect({ host, port });
    this._socket = sock; this._buf = EMPTY;
    let handshook = false, errored = false;
    let head = EMPTY;
    sock.setNoDelay(true);
    sock.on("connect", () => sock.write(handshakeRequest(target.host, path, key)));
    sock.on("error", (err) => { if (!errored) { errored = true; this._emitError(err); } });
    sock.on("close", () => this._onClose(sock));
    sock.on("data", (chunk) => {
      if (!handshook) {
        head = Buffer.concat([head, chunk]);
        const end = head.indexOf("\r\n\r\n");
        if (end < 0) { if (head.length > 16 * 1024) sock.destroy(); return; }
        const text = head.subarray(0, end).toString("latin1");
        const rest = head.subarray(end + 4);
        const lines = text.split("\r\n");
        const status = Number(lines[0].split(" ")[1] || "0");
        const headers = {};
        for (const line of lines.slice(1)) {
          const at = line.indexOf(":");
          if (at > 0) headers[line.slice(0, at).trim().toLowerCase()] = line.slice(at + 1).trim();
        }
        if (status !== 101) {
          return this._handshakeFailure(sock, Object.assign(new Error(`handshake_failed:${status}`), { status }));
        }
        if (headers["sec-websocket-accept"] !== expected) {
          return this._handshakeFailure(sock, new Error("bad_accept_key"));
        }
        handshook = true;
        this._open = true;
        const again = this._everConnected;
        this._everConnected = true;
        this._buf = rest;
        this.emit(again ? "reconnect" : "open");
        if (this._socket === sock && rest.length) this._drain(sock);
        return;
      }
      this._buf = Buffer.concat([this._buf, chunk]);
      this._drain(sock);
    });
  }

  _handshakeFailure(sock, err) { this._emitError(err); sock.destroy(); }

  _onClose(sock) {
    if (this._socket !== sock) return;
    const wasOpen = this._open;
    this._socket = null; this._open = false; this._buf = EMPTY;
    this._failInflight();
    if (wasOpen) this.emit("close");
    this._scheduleReconnect();
  }

  _scheduleReconnect() {
    if (!this._started || this._reconnectTimer) return;
    this._reconnectTimer = setTimeout(() => {
      this._reconnectTimer = null;
      if (this._started) this._connect();
    }, this.reconnectMs);
    if (this._reconnectTimer.unref) this._reconnectTimer.unref();
  }

  _drain(sock) {
    let buf = this._buf;
    while (this._socket === sock) {
      if (buf.length < 2) break;
      const b0 = buf[0], b1 = buf[1];
      const op = b0 & 0x0f, fin = (b0 & 0x80) !== 0, masked = (b1 & 0x80) !== 0;
      let len = b1 & 0x7f, off = 2;
      if (len === 126) {
        if (buf.length < 4) break;
        len = buf.readUInt16BE(2);
        off = 4;
      } else if (len === 127) {
        if (buf.length < 10) break;
        const big = buf.readBigUInt64BE(2);
        if (big > BigInt(MAX_PAYLOAD)) { sock.destroy(); return; }
        len = Number(big);
        off = 10;
      }
      if (len > MAX_PAYLOAD) { sock.destroy(); return; }
      const start = off + (masked ? 4 : 0);
      const end = start + len;
      if (buf.length < end) break;
      const payload = Buffer.from(buf.subarray(start, end));
      if (masked) {
        const mask = buf.subarray(off, off + 4);
        for (let i = 0; i < payload.length; i++) payload[i] ^= mask[i & 3];
      }
      buf = buf.subarray(end);
      if (!fin) { sock.destroy(); return; } // fragmentation unsupported
      this._onFrame(sock, op, payload);
    }
    this._buf = buf;
  }

  _onFrame(sock, op, payload) {
    if (op === OP.TEXT) {
      let msg;
      try { msg = JSON.parse(payload.toString("utf8")); }
      catch (err) { this._emitError(err); sock.destroy(); return; }
      if (msg && typeof msg === "object") this._handleMessage(msg);
      return;
    }
    if (op === OP.PING) { this._writeFrame(sock, OP.PONG, payload); return; }
    if (op === OP.PONG) return;
    if (op === OP.CLOSE) {
      this._writeFrame(sock, OP.CLOSE, payload.subarray(0, 2));
      sock.end();
      return;
    }
    sock.destroy();
  }

  _writeFrame(sock, op, payload) {
    try { sock.write(encodeFrame(op, payload, true)); }
    catch (err) { this._emitError(err); }
  }

  _handleMessage(msg) {
    this.emit("frame", msg);
    if (msg.t === "state") { this._state = msg; this.emit("state", msg); }
    if (msg.t !== "answer" || !msg.request) return;
    const queue = this._inflight.get(msg.request);
    const entry = queue && queue.shift();
    if (!entry) return;
    if (!queue.length) this._inflight.delete(msg.request);
    clearTimeout(entry.timer);
    entry.resolve(msg);
  }

  _remove(type, entry) {
    clearTimeout(entry.timer);
    const queue = this._inflight.get(type);
    if (!queue) return;
    const idx = queue.indexOf(entry);
    if (idx >= 0) queue.splice(idx, 1);
    if (!queue.length) this._inflight.delete(type);
  }

  _failInflight() {
    if (!this._inflight.size) return;
    const err = Object.assign(new Error("disconnected"), { code: "disconnected" });
    for (const queue of this._inflight.values()) {
      for (const entry of queue) { clearTimeout(entry.timer); entry.reject(err); }
    }
    this._inflight.clear();
  }

  // EventEmitter throws on an unhandled "error" event; keep connect failures non-fatal.
  _emitError(err) { if (this.listenerCount("error") > 0) this.emit("error", err); }
}
