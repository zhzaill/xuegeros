// Dependency-free append-only JSONL event log with cursor replay.
// Coo World persists robotd events here and remembers its delivery position.
import fs from "node:fs";
import path from "node:path";
import { createHash } from "node:crypto";

export class EventLog {
  /**
   * @param {string} filePath JSONL file path (directory is created if missing)
   * @param {{maxReplay?: number}} [options]
   */
  constructor(filePath, { maxReplay = 200 } = {}) {
    this.path = filePath;
    this.cursorPath = `${filePath}.cursor`;
    this.maxReplay = Number.isFinite(maxReplay) ? Math.max(0, Math.floor(maxReplay)) : 200;
    fs.mkdirSync(path.dirname(filePath), { recursive: true });
    this.receiptsPath = `${filePath}.delivered.jsonl`;
    this.delivered = new Set();
    // Migrate the old sequence cursor once. Timestamp is part of the new key:
    // a restarted robotd may legitimately reuse sequence numbers.
    if (!fs.existsSync(this.receiptsPath)) {
      const cursor = this.readCursor();
      for (const event of this.replayAll()) {
        if (event.seq <= cursor) this.markDelivered(event);
      }
      fs.appendFileSync(this.receiptsPath, "", "utf8");
    } else {
      for (const line of fs.readFileSync(this.receiptsPath, "utf8").split("\n")) {
        if (/^[a-f0-9]{64}$/.test(line)) this.delivered.add(line);
      }
    }
    this.stored = new Set(this.replayAll().map((event) => this.key(event)));
  }

  key(event) {
    return createHash("sha256").update(JSON.stringify([
      event._origin || "", event.seq, event.ts ?? null, event.kind,
    ])).digest("hex");
  }

  wasDelivered(event) { return this.delivered.has(this.key(event)); }

  markDelivered(event) {
    const key = this.key(event);
    if (this.delivered.has(key)) return;
    fs.appendFileSync(this.receiptsPath, `${key}\n`, "utf8");
    this.delivered.add(key);
  }

  /** Append one event as a single JSON line. Uses event.seq; assigns nothing. */
  append(event) {
    const key = this.key(event);
    if (this.stored.has(key)) return;
    // appendFileSync is append-only: a crash cannot rewrite existing history.
    fs.appendFileSync(this.path, `${JSON.stringify(event)}\n`, "utf8");
    this.stored.add(key);
  }

  /** Last delivered event.seq, or 0 if no cursor exists. */
  readCursor() {
    try {
      const raw = fs.readFileSync(this.cursorPath, "utf8").trim();
      const seq = Number.parseInt(raw, 10);
      return Number.isFinite(seq) ? seq : 0;
    } catch {
      return 0; // absent or unreadable cursor
    }
  }

  /** Persist delivery position: write temp file, then rename over the cursor. */
  writeCursor(seq) {
    const tmp = `${this.path}.cursor.tmp`;
    fs.writeFileSync(tmp, `${seq}\n`, "utf8");
    fs.renameSync(tmp, this.cursorPath); // rename replaces atomically-ish
  }

  /** Events with seq > given seq, capped to the newest maxReplay, oldest first. */
  replaySince(seq) {
    const lines = this.#lines();
    const events = [];
    for (const line of lines) {
      let event;
      try {
        event = JSON.parse(line);
      } catch {
        continue; // malformed JSON line: skip, do not fail
      }
      if (event && typeof event.seq === "number" && event.seq > seq) {
        events.push(event);
      }
    }
    if (this.maxReplay === 0) return [];
    return events.slice(-this.maxReplay);
  }

  replayAll() {
    return this.#lines().flatMap((line) => {
      try { const event = JSON.parse(line); return event && typeof event.seq === "number" ? [event] : []; }
      catch { return []; }
    });
  }

  pending() {
    const events = this.replayAll().filter((event) => !this.wasDelivered(event));
    return this.maxReplay === 0 ? [] : events.slice(-this.maxReplay);
  }

  /** Number of stored non-empty lines. */
  size() {
    return this.#lines().length;
  }

  #lines() {
    let raw;
    try {
      raw = fs.readFileSync(this.path, "utf8");
    } catch {
      return [];
    }
    return raw.split("\n").filter((line) => line.trim() !== "");
  }
}
