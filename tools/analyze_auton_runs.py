#!/usr/bin/env python3
"""Analyze AUTONCSV event logs (Python standard library only).

python tools/analyze_auton_runs.py capture.txt --group segment
python tools/analyze_auton_runs.py *.txt --group route --event AUTON_END
Use '-' for stdin. Concatenate captures freely; file/session boundaries distinguish
run IDs after Brain reboots. Physical measurements must be recorded separately.
"""
import argparse
import collections
import csv
import json
import math
import statistics
import sys


def number(value):
    try:
        result = float(value)
        return result if math.isfinite(result) else None
    except (TypeError, ValueError):
        return None


def linear(values):
    if not values:
        return {"n": 0}
    return {"n": len(values), "mean": statistics.mean(values), "min": min(values),
            "max": max(values), "range": max(values)-min(values),
            "sample_stddev": statistics.stdev(values) if len(values)>1 else None}


def headings(values):
    if not values:
        return {"n": 0}
    angles = sorted(v % 360 for v in values)
    c = statistics.mean(math.cos(math.radians(v)) for v in angles)
    s = statistics.mean(math.sin(math.radians(v)) for v in angles)
    strength = min(1.0, math.hypot(c, s))
    gaps = [b-a for a,b in zip(angles, angles[1:])]+[angles[0]+360-angles[-1]]
    return {"n": len(angles), "circular_mean": math.degrees(math.atan2(s,c)) % 360 if strength>1e-12 else None,
            "circular_stddev": math.degrees(math.sqrt(-2*math.log(strength))) if strength>1e-12 else None,
            "smallest_covering_arc": 360-max(gaps), "resultant_length": strength}


def parse(stream, source):
    header = None
    capture = 0
    for line_number, line in enumerate(stream, 1):
        if line.startswith("AUTON_LOG_DROPPED,"):
            print(f"WARNING {source}:{line_number}: {line.strip()}; capture is incomplete", file=sys.stderr)
        if not line.startswith("AUTONCSV,"):
            continue
        try:
            cells = next(csv.reader([line], strict=True))
        except csv.Error as exc:
            print(f"WARNING {source}:{line_number}: malformed CSV ({exc})", file=sys.stderr)
            continue
        if len(cells)>1 and cells[1]=="run":
            header = cells[1:]
            capture += 1
            continue
        if header is None or len(cells)-1!=len(header):
            print(f"WARNING {source}:{line_number}: missing header or truncated row", file=sys.stderr)
            continue
        row = dict(zip(header,cells[1:]))
        if not {"run","event","route","segment","motion","result","end_x","end_y","end_h"} <= row.keys():
            print(f"WARNING {source}:{line_number}: unsupported schema", file=sys.stderr)
            continue
        row["_run"]=(source,capture,row["run"])
        yield row


def summarize(rows):
    def values(key):
        return [v for r in rows if (v:=number(r.get(key))) is not None]
    dx, dy, errors = [], [], []
    for r in rows:
        x,y,tx,ty=(number(r.get(k)) for k in ("end_x","end_y","target_x","target_y"))
        if None not in (x,y,tx,ty):
            dx.append(x-tx); dy.append(y-ty); errors.append(math.hypot(x-tx,y-ty))
    counts=collections.Counter(r["result"] for r in rows)
    return {"records":len(rows),"runs":len({r["_run"] for r in rows}),
            "result_counts":dict(counts), "success":counts["PASS"],
            "failure":sum(counts[k] for k in ("FAIL","TIMEOUT","SENSOR_FAULT")),
            "aborted":counts["ABORTED"], "unverified":counts["RETURNED_UNVERIFIED"],
            "final_x":linear(values("end_x")), "final_y":linear(values("end_y")),
            "final_heading":headings(values("end_h")),
            "wrapped_heading_error":linear(values("heading_error")),
            "error_x_actual_minus_target":linear(dx), "error_y_actual_minus_target":linear(dy),
            "euclidean_error":linear(errors),
            "bias_vector_magnitude":math.hypot(statistics.mean(dx),statistics.mean(dy)) if dx else None}


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("files",nargs="+")
    parser.add_argument("--group",choices=("route","segment","motion"),default="segment")
    parser.add_argument("--event",choices=("AUTON_END","SEGMENT_END","MOTION_END"))
    args=parser.parse_args(argv)
    event=args.event or {"route":"AUTON_END","segment":"SEGMENT_END","motion":"MOTION_END"}[args.group]
    groups=collections.defaultdict(list)
    for path in args.files:
        stream=sys.stdin if path=="-" else open(path,encoding="utf-8-sig",errors="replace")
        try:
            for row in parse(stream,path):
                if row["event"]==event:
                    # Segment/motion identities include route to avoid mixing sides.
                    key=row["route"] if args.group=="route" else row["route"]+" / "+row["segment"]
                    if args.group=="motion": key+=" / "+row["motion"]
                    groups[key].append(row)
        finally:
            if stream is not sys.stdin: stream.close()
    if not groups:
        print("No matching AUTONCSV records; check event selection and complete headers.",file=sys.stderr)
        return 1
    print(json.dumps({k:summarize(v) for k,v in sorted(groups.items())},indent=2,allow_nan=False))
    print("Mean error vector suggests systematic bias; spread suggests variability. Neither proves cause. "
          "Do not pool different targets, configurations or test conditions. Odom error is not physical ground truth.",file=sys.stderr)
    return 0


if __name__=="__main__":
    raise SystemExit(main())
