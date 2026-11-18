import { test } from "node:test";
import assert from "node:assert/strict";
import fs from "node:fs";
import os from "node:os";
import path from "node:path";
import { EventLog } from "../src/events.js";

function tempLog(t, name, opts) {
  const dir = fs.mkdtempSync(path.join(os.tmpdir(), "coo-events-"));
  t.after(() => fs.rmSync(dir, { recursive: true, force: true }));
  return new EventLog(path.join(dir, name), opts);
}

const ev = (seq) => ({ seq, ts: 1000 + seq, kind: "wake" });

test("append 5, cursor defaults to 0, replaySince(0) returns all in order", (t) => {
  const log = tempLog(t, "events.jsonl");
  for (let i = 1; i <= 5; i++) log.append(ev(i));
  assert.equal(log.size(), 5);
  assert.equal(log.readCursor(), 0);
  assert.deepEqual(log.replaySince(0).map((e) => e.seq), [1, 2, 3, 4, 5]);
});

test("writeCursor(3) then replaySince(3) returns only 4 and 5", (t) => {
  const log = tempLog(t, "events.jsonl");
  for (let i = 1; i <= 5; i++) log.append(ev(i));
  log.writeCursor(3);
  assert.equal(log.readCursor(), 3);
  assert.deepEqual(log.replaySince(3).map((e) => e.seq), [4, 5]);
});

test("maxReplay cap keeps the newest N in chronological order", (t) => {
  const log = tempLog(t, "events.jsonl", { maxReplay: 3 });
  for (let i = 1; i <= 10; i++) log.append(ev(i));
  assert.deepEqual(log.replaySince(0).map((e) => e.seq), [8, 9, 10]);
});

test("malformed JSON lines are skipped, not fatal", (t) => {
  const log = tempLog(t, "events.jsonl");
  log.append(ev(1));
  fs.appendFileSync(log.path, "{not json\n", "utf8");
  log.append(ev(2));
  assert.deepEqual(log.replaySince(0).map((e) => e.seq), [1, 2]);
  assert.equal(log.size(), 3);
});

test("two EventLog instances on the same path share the cursor", (t) => {
  const dir = fs.mkdtempSync(path.join(os.tmpdir(), "coo-events-"));
  t.after(() => fs.rmSync(dir, { recursive: true, force: true }));
  const p = path.join(dir, "events.jsonl");
  const a = new EventLog(p);
  const b = new EventLog(p);
  assert.equal(a.readCursor(), 0);
  assert.equal(b.readCursor(), 0);
  a.writeCursor(7);
  assert.equal(b.readCursor(), 7);
});
