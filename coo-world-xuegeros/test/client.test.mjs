import net from "node:net";
import crypto from "node:crypto";
import { test } from "node:test";
import assert from "node:assert/strict";
import { RobotdClient } from "../src/robotd_client.js";

const GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

function acceptKey(key) {
  return crypto.createHash("sha1").update(key + GUID, "ascii").digest("base64");
}

function serverFrame(opcode, payload) {
  const len = payload.length;
  let header;
  if (len < 126) {
    header = Buffer.alloc(2);
    header[1] = len;
  } else if (len < 65536) {
    header = Buffer.alloc(4);
    header[1] = 126;
    header.writeUInt16BE(len, 2);
  } else {
    header = Buffer.alloc(10);
    header[1] = 127;
    header.writeBigUInt64BE(BigInt(len), 2);
  }
  header[0] = 0x80 | opcode;
  return Buffer.concat([header, payload]);
}

// Minimal in-process robotd substitute: handshake, token check, unmasked text frames.
class TestServer {
  constructor({ token = "", onOpen = null, onText = null } = {}) {
    this.token = token;
    this.onOpen = onOpen;
    this.onText = onText;
    this.sockets = new Set();
    this.maskViolation = false;
  }

  async listen() {
    this.server = net.createServer((sock) => this._accept(sock));
    await new Promise((resolve) => this.server.listen(0, "127.0.0.1", resolve));
    this.port = this.server.address().port;
    return this;
  }

  send(sock, obj) {
    sock.write(serverFrame(1, Buffer.from(JSON.stringify(obj), "utf8")));
  }

  dropAll() {
    for (const sock of [...this.sockets]) sock.destroy();
    this.sockets.clear();
  }

  async close() {
    this.dropAll();
    await new Promise((resolve) => this.server.close(resolve));
  }

  _accept(sock) {
    this.sockets.add(sock);
    let buf = Buffer.alloc(0);
    let open = false;
    sock.on("error", () => {});
    sock.on("close", () => this.sockets.delete(sock));
    sock.on("data", (chunk) => {
      buf = Buffer.concat([buf, chunk]);
      if (!open) {
        const end = buf.indexOf("\r\n\r\n");
        if (end < 0) return;
        const text = buf.subarray(0, end).toString("latin1");
        buf = buf.subarray(end + 4);
        const lines = text.split("\r\n");
        const path = lines[0].split(" ")[1] || "/";
        const headers = {};
        for (const line of lines.slice(1)) {
          const at = line.indexOf(":");
          if (at > 0) headers[line.slice(0, at).trim().toLowerCase()] = line.slice(at + 1).trim();
        }
        const supplied = new URL(path, "http://robotd.test").searchParams.get("token") || "";
        if (this.token && supplied !== this.token) {
          const body = '{"ok":false,"error":"token_required"}';
          sock.write(
            "HTTP/1.1 401 Unauthorized\r\ncontent-type: application/json\r\n" +
              `content-length: ${Buffer.byteLength(body)}\r\nconnection: close\r\n\r\n${body}`,
          );
          sock.end();
          return;
        }
        const key = headers["sec-websocket-key"];
        if (!key) {
          sock.destroy();
          return;
        }
        sock.write(
          "HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n" +
            `Sec-WebSocket-Accept: ${acceptKey(key)}\r\n\r\n`,
        );
        open = true;
        if (this.onOpen) this.onOpen(sock, this);
      }
      buf = this._readFrames(sock, buf);
    });
  }

  _readFrames(sock, buf) {
    while (buf.length >= 2) {
      const op = buf[0] & 0x0f;
      const masked = (buf[1] & 0x80) !== 0;
      let len = buf[1] & 0x7f;
      let off = 2;
      if (len === 126) {
        if (buf.length < 4) break;
        len = buf.readUInt16BE(2);
        off = 4;
      } else if (len === 127) {
        if (buf.length < 10) break;
        len = Number(buf.readBigUInt64BE(2));
        off = 10;
      }
      const start = off + (masked ? 4 : 0);
      const end = start + len;
      if (buf.length < end) break;
      let payload = Buffer.from(buf.subarray(start, end));
      if (masked) {
        const mask = buf.subarray(off, off + 4);
        for (let i = 0; i < payload.length; i++) payload[i] ^= mask[i & 3];
      } else {
        this.maskViolation = true;
      }
      buf = buf.subarray(end);
      if (op === 1 && this.onText) this.onText(JSON.parse(payload.toString("utf8")), sock, this);
      else if (op === 8) sock.end();
    }
    return buf;
  }
}

function clientUrl(server) {
  return `ws://127.0.0.1:${server.port}/ws`;
}

function waitFor(client, event, predicate) {
  return new Promise((resolve, reject) => {
    const timer = setTimeout(() => {
      client.off(event, onEvent);
      reject(new Error(`timeout waiting for ${event}`));
    }, 2000);
    function onEvent(value) {
      if (predicate && !predicate(value)) return;
      clearTimeout(timer);
      client.off(event, onEvent);
      resolve(value);
    }
    client.on(event, onEvent);
  });
}

test("connects with token, streams state, answers propose", async () => {
  const server = await new TestServer({
    token: "s3cret",
    onOpen(sock, srv) {
      srv.send(sock, { t: "hello", ok: true, state_hz: 50, skills: [] });
      srv.send(sock, { t: "state", seq: 7, physical: false });
    },
    onText(msg, sock, srv) {
      if (msg.t === "propose") {
        srv.send(sock, { t: "answer", request: "propose", ok: true, accepted: true, proposal_id: "p1" });
      }
    },
  }).listen();
  const client = new RobotdClient({ url: clientUrl(server), token: "s3cret", reconnectMs: 50 });
  const openP = waitFor(client, "open");
  const stateP = waitFor(client, "state");
  client.start();
  try {
    await openP;
    const state = await stateP;
    assert.equal(state.seq, 7);
    assert.equal(client.connected, true);
    assert.equal(client.state, state);
    const answer = await client.request({ t: "propose", goal: "forward" });
    assert.equal(answer.request, "propose");
    assert.equal(answer.accepted, true);
    assert.equal(answer.proposal_id, "p1");
    assert.equal(server.maskViolation, false, "client frames must be masked");
  } finally {
    client.stop();
    await server.close();
  }
});

test("rejects a connection without the token", async () => {
  const server = await new TestServer({ token: "s3cret" }).listen();
  const client = new RobotdClient({ url: clientUrl(server), reconnectMs: 50 });
  const errorP = waitFor(client, "error");
  client.start();
  try {
    const err = await errorP;
    assert.equal(err.status, 401);
    assert.equal(client.connected, false);
  } finally {
    client.stop();
    await server.close();
  }
});

test("reconnects after the server drops the socket", async () => {
  let connections = 0;
  const server = await new TestServer({
    onOpen(sock, srv) {
      connections += 1;
      srv.send(sock, { t: "state", seq: connections });
    },
  }).listen();
  const client = new RobotdClient({ url: clientUrl(server), reconnectMs: 50, requestTimeoutMs: 1000 });
  const openP = waitFor(client, "open");
  const reconnectP = waitFor(client, "reconnect");
  const secondStateP = waitFor(client, "state", (state) => state.seq === 2);
  client.start();
  try {
    await openP;
    assert.equal(connections, 1);
    server.dropAll();
    await reconnectP;
    const second = await secondStateP;
    assert.equal(second.seq, 2);
    assert.equal(client.connected, true);
    assert.equal(connections, 2);
  } finally {
    client.stop();
    await server.close();
  }
});
