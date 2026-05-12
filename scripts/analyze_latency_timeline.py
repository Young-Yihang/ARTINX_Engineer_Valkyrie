#!/usr/bin/env python3
"""Correlate ARV hardware and torque-controller latency events by ROS log time."""

from __future__ import annotations

import argparse
import glob
import os
import re
from dataclasses import dataclass
from typing import Iterable


ROS_TS_RE = re.compile(r"\[(\d+\.\d+)\]")


@dataclass
class Event:
    ts: float
    source: str
    kind: str
    detail: str
    value_us: int = 0


def latest_log(pattern: str) -> str | None:
    files = glob.glob(os.path.expanduser(pattern))
    if not files:
        return None
    return max(files, key=os.path.getmtime)


def ros_ts(line: str) -> float | None:
    match = ROS_TS_RE.search(line)
    return float(match.group(1)) if match else None


def parse_int_field(line: str, field: str) -> int | None:
    match = re.search(rf"{re.escape(field)}=([-0-9]+)us", line)
    return int(match.group(1)) if match else None


def parse_hardware_log(path: str) -> list[Event]:
    events: list[Event] = []
    with open(path, "r", errors="replace") as fh:
        for line in fh:
            ts = ros_ts(line)
            if ts is None:
                continue
            if "[SEND/TIMING] OVERRUN" in line:
                total = parse_int_field(line, "total") or 0
                cache = parse_int_field(line, "cache") or 0
                gap = parse_int_field(line, "gap") or 0
                send = parse_int_field(line, "send_torque") or 0
                events.append(
                    Event(
                        ts,
                        "hardware",
                        "send_overrun",
                        f"total={total}us gap={gap}us cache={cache}us send_torque={send}us",
                        total,
                    )
                )
            elif "[SAFETY] Torque data stale" in line:
                events.append(Event(ts, "hardware", "torque_stale", line.strip()))
            elif "[WARN] No torque data received yet" in line:
                events.append(Event(ts, "hardware", "no_torque", line.strip()))
    return events


def parse_torque_log(path: str) -> list[Event]:
    events: list[Event] = []
    with open(path, "r", errors="replace") as fh:
        for line in fh:
            ts = ros_ts(line)
            if ts is None:
                continue
            if "[TIMING] OVERRUN" in line:
                old = re.search(r"total=\s*([0-9]+)us", line)
                total = int(old.group(1)) if old else (parse_int_field(line, "total") or 0)
                path_match = re.search(r"path=([A-Z_+\-a-z0-9]+)", line)
                path_name = path_match.group(1) if path_match else "unknown"
                gap = parse_int_field(line, "gap")
                js_age = parse_int_field(line, "js_age")
                extra = []
                if gap is not None:
                    extra.append(f"gap={gap}us")
                if js_age is not None:
                    extra.append(f"js_age={js_age}us")
                suffix = " " + " ".join(extra) if extra else ""
                events.append(
                    Event(ts, "torque", "control_overrun", f"path={path_name} total={total}us{suffix}", total)
                )
            elif "[SAFETY] Joint state timeout" in line:
                events.append(Event(ts, "torque", "joint_state_timeout", line.strip()))
            elif "[SAFETY] Control loop slow" in line:
                events.append(Event(ts, "torque", "control_gap_slow", line.strip()))
    return events


def parse_dataflow_summary(path: str | None) -> dict[str, str]:
    if not path:
        return {}
    summary = os.path.join(path, "summary.txt") if os.path.isdir(path) else path
    if not os.path.exists(summary):
        return {}
    values: dict[str, str] = {}
    with open(summary, "r", errors="replace") as fh:
        for line in fh:
            line = line.strip()
            if "=" in line and not line.startswith("---"):
                key, value = line.split("=", 1)
                values[key] = value
    return values


def nearest(events: Iterable[Event], ts: float, source: str, window_s: float) -> Event | None:
    candidates = [event for event in events if event.source == source and abs(event.ts - ts) <= window_s]
    if not candidates:
        return None
    return min(candidates, key=lambda event: abs(event.ts - ts))


def print_dataflow(values: dict[str, str]) -> None:
    if not values:
        return
    keys = [
        "JOINT_STATES_STATUS",
        "JOINT_STATES_RATE_AVG_HZ",
        "JOINT_STATES_RATE_MIN_HZ",
        "JOINT_STATES_MAX_INTERVAL_MS",
        "EFFORT_CONTROLLER_COMMANDS_STATUS",
        "EFFORT_CONTROLLER_COMMANDS_RATE_AVG_HZ",
        "EFFORT_CONTROLLER_COMMANDS_RATE_MIN_HZ",
        "EFFORT_CONTROLLER_COMMANDS_RATE_MAX_HZ",
        "EFFORT_CONTROLLER_COMMANDS_MAX_INTERVAL_MS",
    ]
    print("\n[dataflow]")
    for key in keys:
        if key in values:
            print(f"{key}={values[key]}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hardware-log", default=None)
    parser.add_argument("--torque-log", default=None)
    parser.add_argument("--dataflow", default=None, help="dataflow output dir or summary.txt")
    parser.add_argument("--window-sec", type=float, default=1.0)
    parser.add_argument("--limit", type=int, default=80)
    args = parser.parse_args()

    hardware_log = args.hardware_log or latest_log("~/.ros/log/hardware_interface_node_*.log")
    torque_log = args.torque_log or latest_log("~/.ros/log/torque_controller_node_*.log")

    if not hardware_log:
        raise SystemExit("hardware log not found")
    if not torque_log:
        raise SystemExit("torque controller log not found")

    events = parse_hardware_log(hardware_log) + parse_torque_log(torque_log)
    events.sort(key=lambda event: event.ts)

    print(f"hardware_log={hardware_log}")
    print(f"torque_log={torque_log}")
    print_dataflow(parse_dataflow_summary(args.dataflow))

    hardware_overruns = [event for event in events if event.source == "hardware" and event.kind == "send_overrun"]
    torque_overruns = [event for event in events if event.source == "torque" and event.kind == "control_overrun"]
    torque_timeouts = [event for event in events if event.source == "torque" and event.kind == "joint_state_timeout"]

    print("\n[summary]")
    print(f"hardware_send_overruns={len(hardware_overruns)}")
    print(f"hardware_max_overrun_us={max((event.value_us for event in hardware_overruns), default=0)}")
    print(f"torque_control_overruns={len(torque_overruns)}")
    print(f"torque_max_overrun_us={max((event.value_us for event in torque_overruns), default=0)}")
    print(f"torque_joint_state_timeouts={len(torque_timeouts)}")

    print(f"\n[timeline first {args.limit}]")
    for event in events[: args.limit]:
      print(f"{event.ts:.9f} {event.source:8s} {event.kind:20s} {event.detail}")

    print(f"\n[hardware overrun correlation window=+/-{args.window_sec:.3f}s]")
    for event in hardware_overruns[: args.limit]:
        peer = nearest(events, event.ts, "torque", args.window_sec)
        if peer:
            delta_ms = (peer.ts - event.ts) * 1000.0
            print(
                f"{event.ts:.9f} hardware {event.detail} | nearest torque {delta_ms:+.1f}ms "
                f"{peer.kind} {peer.detail}"
            )
        else:
            print(f"{event.ts:.9f} hardware {event.detail} | nearest torque none")

    print("\n[inference]")
    if hardware_overruns and torque_timeouts:
        first_hw = hardware_overruns[0].ts
        first_timeout = torque_timeouts[0].ts
        delta_ms = (first_timeout - first_hw) * 1000.0
        if delta_ms > 0:
            print(
                "first_hardware_overrun_precedes_first_torque_timeout=true "
                f"delta_ms={delta_ms:.1f}"
            )
        else:
            print(
                "first_torque_timeout_precedes_first_hardware_overrun=true "
                f"delta_ms={-delta_ms:.1f}"
            )
    elif hardware_overruns:
        print("hardware_overruns_present_but_no_torque_timeouts_found=true")
    elif torque_timeouts:
        print("torque_timeouts_present_but_no_hardware_overruns_found=true")
    else:
        print("no_major_log_events_found=true")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
