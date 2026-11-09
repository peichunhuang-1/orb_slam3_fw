#!/usr/bin/env python3
"""Live statistics of running fw boxes, from fw_agent (the same numbers the editor's panel shows).

  python3 tools/fw_stats.py [--block orb_matcher ...] [--interval 1]

Run it next to `fw launch <graph>`. Every interval, per block (all blocks when none is given):
  out  <port>   Hz over the interval, total sent
  run  <handler> calls/s, mean and max execution time over the interval
  in   <port>   received, dropped (queue full), sync_unmatched (sync items no pair took)
  sync          aligned groups
At the end (Ctrl-C, or every box gone) a summary over the whole run per output: mean Hz and handler time.
End-to-end latency is not in these statistics: orb_matcher prints its own (sensor stamp -> match) at EOS.
"""
import argparse
import json
import os
import socket
import sys
import time


def agent_socket():
    run = os.environ.get("XDG_RUNTIME_DIR")
    return os.path.join(run, "fw", "agent.sock") if run else f"/tmp/fw-{os.getuid()}/agent.sock"


def lines(sock):
    buf = b""
    while True:
        try:
            d = sock.recv(1 << 20)
        except socket.timeout:
            yield None
            continue
        if not d:
            return
        buf += d
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            if line.strip():
                yield line.decode(errors="replace")


def ms(ns):
    return ns / 1e6


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--block", action="append", help="block instance name (repeatable); default: all")
    ap.add_argument("--interval", type=float, default=1.0, help="seconds between prints (stats arrive once a second)")
    ap.add_argument("--wait", type=float, default=30.0, help="seconds to wait for a box to come up")
    a = ap.parse_args()
    want = set(a.block or [])

    sock = socket.socket(socket.AF_UNIX)
    try:
        sock.connect(agent_socket())
    except OSError as e:
        sys.exit(f"fw_stats: cannot reach fw_agent at {agent_socket()} ({e}); is a graph running?")
    sock.settimeout(1.0)
    sock.sendall(b'{"op":"subscribe"}\n')

    prev = {}      # (box, pid) -> (time, stats)
    first = {}     # (box, pid) -> (time, stats)
    last_print = 0.0
    seen_box = time.time()
    last = {}
    try:
        for line in lines(sock):
            now = time.time()
            if line is None:
                if now - seen_box > (a.wait if not last else 3.0):
                    break   # no box (left)
                continue
            msg = json.loads(line)
            if msg.get("op") != "stats":
                continue
            seen_box = now
            key = (msg["box"], msg["pid"])
            st = msg["stats"]
            first.setdefault(key, (now, st))
            last[key] = (now, st)
            if key in prev and now - last_print >= a.interval - 0.05:
                print_delta(key, prev[key], (now, st), want)
                last_print = now
            if key not in prev or now - prev[key][0] >= a.interval - 0.05:
                prev[key] = (now, st)
    except KeyboardInterrupt:
        pass
    for key in last:
        summary(key, first[key], last[key], want)


def by(items, *fields):
    return {tuple(x[f] for f in fields): x for x in items}


def print_delta(key, a, b, want):
    (t0, s0), (t1, s1) = a, b
    dt = max(t1 - t0, 1e-9)
    o0, h0 = by(s0["outputs"], "block", "port"), by(s0["handlers"], "block", "handler")
    rows = []
    for o in s1["outputs"]:
        if (want and o["block"] not in want) or o["block"].startswith("config://"):
            continue
        p = o0.get((o["block"], o["port"]), {"sent": 0})
        rows.append(f"  {o['block']:<22} out  {o['port']:<12} {(o['sent'] - p['sent']) / dt:6.1f} Hz  (sent {o['sent']})")
    for h in s1["handlers"]:
        if want and h["block"] not in want:
            continue
        p = h0.get((h["block"], h["handler"]), {"calls": 0, "exec_total_ns": 0})
        n = h["calls"] - p["calls"]
        if n <= 0:
            continue
        mean = (h["exec_total_ns"] - p["exec_total_ns"]) / n
        rows.append(f"  {h['block']:<22} run  {h['handler']:<12} {n / dt:6.1f} /s  mean {ms(mean):7.2f} ms  "
                    f"max(all) {ms(h['exec_max_ns']):7.2f} ms")
    for p in s1["ports"]:
        if (want and p["block"] not in want) or (not p["dropped"] and not p["sync_unmatched"]):
            continue
        rows.append(f"  {p['block']:<22} in   {p['port']:<12} received {p['received']}  dropped {p['dropped']}  "
                    f"sync_unmatched {p['sync_unmatched']}")
    print(f"[{key[0]} pid {key[1]}] {time.strftime('%H:%M:%S')}  health {s1.get('box_health')}")
    print("\n".join(rows), flush=True)


def summary(key, a, b, want):
    (t0, s0), (t1, s1) = a, b
    dt = max(t1 - t0, 1e-9)
    print(f"\n== summary [{key[0]} pid {key[1]}] over {dt:.1f} s ==")
    o0, h0 = by(s0["outputs"], "block", "port"), by(s0["handlers"], "block", "handler")
    for o in s1["outputs"]:
        if (want and o["block"] not in want) or o["block"].startswith("config://"):
            continue
        p = o0.get((o["block"], o["port"]), {"sent": 0})
        print(f"  {o['block']:<22} out  {o['port']:<12} {(o['sent'] - p['sent']) / dt:6.1f} Hz mean  (sent {o['sent']})")
    for h in s1["handlers"]:
        if (want and h["block"] not in want) or not h["calls"]:
            continue
        print(f"  {h['block']:<22} run  {h['handler']:<12} {h['calls']} calls  mean {ms(h['exec_total_ns'] / h['calls']):7.2f} ms  "
              f"max {ms(h['exec_max_ns']):7.2f} ms")
    for p in s1["ports"]:
        if (want and p["block"] not in want) or (not p["dropped"] and not p["sync_unmatched"]):
            continue
        print(f"  {p['block']:<22} in   {p['port']:<12} received {p['received']}  dropped {p['dropped']}  "
              f"sync_unmatched {p['sync_unmatched']}")


if __name__ == "__main__":
    main()
