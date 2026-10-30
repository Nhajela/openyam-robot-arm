#!/usr/bin/env python3
"""Answer look_and_grasp's y/N prompts automatically, but only inside a safety box.

Runs the command after `--`, watches its stdout for the engine's ">>> ..." prompts and answers
  y  for "MOVE to ready" / "MOVE to home", any GRIPPER command, and joint MOVEs whose tip target
     is within R m (xy) of the object and at least Z m high (base frame);
  n  for anything else, or when any of j2..j4 is at or above the temperature limit
     (rotor °C from the lab console's /api/state).
Every prompt and answer is printed with the temperatures. Rewritten 30 Oct 2026 from the
29 Oct purple-ball runs (the original lived only in a terminal).

usage: pick_guard.py <obj x,y base> <R m> <Z min m> <temp max °C> -- <command ...>
"""
import json, re, subprocess, sys, time, urllib.request

def temps():
    try:
        d = json.load(urllib.request.urlopen("http://127.0.0.1:8090/api/state", timeout=2))
        t = {m["name"]: m.get("rotor_temp", m.get("t_rotor")) for m in d["motors"]}
        return [t.get(f"joint{i}") for i in (2, 3, 4)]
    except Exception:
        return [None, None, None]

def decide(what, obj, R, Z, tmax):
    ts = temps()
    if any(t is None or t >= tmax for t in ts):
        return "n", ts, f"temperature {ts} (limit {tmax})"
    if re.match(r"MOVE to (ready|home)\b", what) or what.startswith("GRIPPER"):
        return "y", ts, ""
    m = re.search(r"tip -> \(([-\d.]+), ([-\d.]+), ([-\d.]+)\)", what)
    if what.startswith("MOVE") and m:
        x, y, z = map(float, m.groups())
        d = ((x - obj[0]) ** 2 + (y - obj[1]) ** 2) ** 0.5
        if d <= R and z >= Z:
            return "y", ts, ""
        return "n", ts, f"tip ({x:.3f}, {y:.3f}, {z:.3f}) is {d:.3f} m from the object / below {Z}"
    return "n", ts, "not a recognised prompt"

def main():
    sep = sys.argv.index("--")
    a = sys.argv[1:sep]; cmd = sys.argv[sep + 1:]
    obj = tuple(map(float, a[0].split(","))); R, Z, tmax = float(a[1]), float(a[2]), float(a[3])
    p = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=0)
    buf, last = "", ""
    while True:
        ch = p.stdout.read(1)
        if not ch:
            break
        sys.stdout.write(ch); sys.stdout.flush()
        buf += ch
        if ch == "\n":
            if buf.startswith(">>> "):
                last = buf[4:].strip()
            buf = ""
        elif buf.endswith("anything else aborts: ") and last:
            ans, ts, why = decide(last, obj, R, Z, tmax)
            print(f"\n[guard {time.strftime('%H:%M:%S')}] {ans} [j2 {ts[0]} j3 {ts[1]} j4 {ts[2]}] {last}" + (f"  REFUSED: {why}" if why else ""), flush=True)
            p.stdin.write(ans + "\n"); p.stdin.flush()
            buf, last = "", ""
    print("program exited", p.wait(), flush=True)

main()
