#!/usr/bin/env python3
"""Replay TGW global paths with a simple 2D lookahead tracker.

The input is JSONL exported by tgw_experience_global_sweep.  This is not a
robot dynamics simulator.  It checks whether the global path contract can be
consumed by a local controller that follows x/y/yaw while treating z as expected
surface height.
"""

import argparse
import json
import math
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


@dataclass
class PathSample:
    x: float
    y: float
    z: float
    kind: str
    speed: float


@dataclass
class ReplayOptions:
    dt: float
    lookahead_m: float
    projection_window_m: float
    max_yaw_rate_radps: float
    goal_tolerance_m: float
    max_time_s: float
    max_lateral_error_m: float


@dataclass
class ReplayResult:
    query_id: int
    success: bool
    failure_reason: str
    steps: int
    duration_s: float
    final_error_m: float
    max_lateral_error_m: float
    mean_lateral_error_m: float
    max_z_step_m: float
    portal_count: int
    kind_transitions: int
    trace: List[Dict[str, float]]


def load_records(path: Path) -> List[dict]:
    records = []
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if line:
                records.append(json.loads(line))
    return records


def path_samples(record: dict) -> List[PathSample]:
    samples = []
    for point in record.get("path", []):
        samples.append(
            PathSample(
                x=float(point["x"]),
                y=float(point["y"]),
                z=float(point["z"]),
                kind=str(point.get("kind", "unknown")),
                speed=max(0.02, float(point.get("target_speed_mps", 0.3))),
            )
        )
    return remove_duplicate_points(samples)


def remove_duplicate_points(samples: Sequence[PathSample]) -> List[PathSample]:
    out: List[PathSample] = []
    for sample in samples:
        if not out:
            out.append(sample)
            continue
        if distance_xy(out[-1], sample) < 1.0e-6 and abs(out[-1].z - sample.z) < 1.0e-6:
            if out[-1].kind != "portal" and sample.kind == "portal":
                out[-1] = sample
            continue
        out.append(sample)
    return out


def distance_xy(a: PathSample, b: PathSample) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def cumulative_lengths(samples: Sequence[PathSample]) -> List[float]:
    cumulative = [0.0]
    for previous, current in zip(samples, samples[1:]):
        cumulative.append(cumulative[-1] + distance_xy(previous, current))
    return cumulative


def wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


def interpolate(samples: Sequence[PathSample], cumulative: Sequence[float], s: float) -> PathSample:
    if s <= 0.0:
        return samples[0]
    if s >= cumulative[-1]:
        return samples[-1]
    lo = 0
    hi = len(cumulative) - 1
    while lo + 1 < hi:
        mid = (lo + hi) // 2
        if cumulative[mid] <= s:
            lo = mid
        else:
            hi = mid
    segment_len = cumulative[lo + 1] - cumulative[lo]
    t = 0.0 if segment_len <= 1.0e-9 else (s - cumulative[lo]) / segment_len
    a = samples[lo]
    b = samples[lo + 1]
    return PathSample(
        x=a.x + t * (b.x - a.x),
        y=a.y + t * (b.y - a.y),
        z=a.z + t * (b.z - a.z),
        kind=b.kind if t > 0.5 else a.kind,
        speed=max(0.02, a.speed + t * (b.speed - a.speed)),
    )


def project_progress(
    samples: Sequence[PathSample],
    cumulative: Sequence[float],
    x: float,
    y: float,
    min_s: float,
    max_s: float,
) -> Tuple[float, float]:
    best_s = min_s
    best_dist = float("inf")
    for i, (a, b) in enumerate(zip(samples, samples[1:])):
        if cumulative[i] > max_s or cumulative[i + 1] < min_s:
            continue
        ax, ay = a.x, a.y
        bx, by = b.x, b.y
        vx = bx - ax
        vy = by - ay
        seg2 = vx * vx + vy * vy
        if seg2 <= 1.0e-12:
            continue
        t = ((x - ax) * vx + (y - ay) * vy) / seg2
        t = min(1.0, max(0.0, t))
        s = cumulative[i] + t * (cumulative[i + 1] - cumulative[i])
        if s + 1.0e-6 < min_s or s > max_s + 1.0e-6:
            continue
        px = ax + t * vx
        py = ay + t * vy
        dist = math.hypot(x - px, y - py)
        if dist < best_dist:
            best_dist = dist
            best_s = s
    return best_s, best_dist


def max_path_z_step(samples: Sequence[PathSample]) -> float:
    return max((abs(b.z - a.z) for a, b in zip(samples, samples[1:])), default=0.0)


def initial_yaw(samples: Sequence[PathSample]) -> float:
    for a, b in zip(samples, samples[1:]):
        dx = b.x - a.x
        dy = b.y - a.y
        if math.hypot(dx, dy) > 1.0e-6:
            return math.atan2(dy, dx)
    return 0.0


def replay_record(record: dict, options: ReplayOptions) -> ReplayResult:
    samples = path_samples(record)
    query_id = int(record.get("query_id", -1))
    if len(samples) < 2:
        return ReplayResult(query_id, False, "path_too_short", 0, 0.0, float("inf"), 0.0, 0.0, 0.0, 0, 0, [])

    cumulative = cumulative_lengths(samples)
    if cumulative[-1] <= 1.0e-6:
        return ReplayResult(query_id, False, "zero_length_path", 0, 0.0, float("inf"), 0.0, 0.0, 0.0, 0, 0, [])

    x = samples[0].x
    y = samples[0].y
    yaw = initial_yaw(samples)
    progress_s = 0.0
    trace: List[Dict[str, float]] = []
    lateral_errors: List[float] = []
    max_z_step = max_path_z_step(samples)
    last_kind = samples[0].kind
    kind_transitions = 0
    portal_count = sum(1 for sample in samples if sample.kind == "portal")
    max_steps = int(math.ceil(options.max_time_s / options.dt))

    for step in range(max_steps):
        progress_s, lateral_error = project_progress(
            samples,
            cumulative,
            x,
            y,
            progress_s,
            min(cumulative[-1], progress_s + options.projection_window_m),
        )
        lateral_errors.append(lateral_error)
        current = interpolate(samples, cumulative, progress_s)
        lookahead = interpolate(samples, cumulative, min(cumulative[-1], progress_s + options.lookahead_m))

        dx = lookahead.x - x
        dy = lookahead.y - y
        desired_yaw = yaw if math.hypot(dx, dy) < 1.0e-6 else math.atan2(dy, dx)
        yaw_delta = wrap_angle(desired_yaw - yaw)
        yaw_step = max(-options.max_yaw_rate_radps * options.dt, min(options.max_yaw_rate_radps * options.dt, yaw_delta))
        yaw = wrap_angle(yaw + yaw_step)

        target_speed = max(0.02, current.speed)
        x += target_speed * options.dt * math.cos(yaw)
        y += target_speed * options.dt * math.sin(yaw)
        next_progress_s, _ = project_progress(
            samples,
            cumulative,
            x,
            y,
            progress_s,
            min(cumulative[-1], progress_s + options.projection_window_m),
        )
        next_point = interpolate(samples, cumulative, next_progress_s)
        progress_s = max(progress_s, next_progress_s)

        if current.kind != last_kind:
            kind_transitions += 1
            last_kind = current.kind

        trace.append(
            {
                "t": step * options.dt,
                "x": x,
                "y": y,
                "yaw": yaw,
                "progress_m": progress_s,
                "lateral_error_m": lateral_error,
                "target_x": lookahead.x,
                "target_y": lookahead.y,
                "target_z": lookahead.z,
                "speed_mps": target_speed,
            }
        )

        final_error = math.hypot(x - samples[-1].x, y - samples[-1].y)
        if final_error <= options.goal_tolerance_m and cumulative[-1] - progress_s <= options.goal_tolerance_m:
            max_lateral = max(lateral_errors) if lateral_errors else 0.0
            mean_lateral = sum(lateral_errors) / len(lateral_errors) if lateral_errors else 0.0
            success = max_lateral <= options.max_lateral_error_m
            reason = "" if success else "lateral_error_exceeded"
            return ReplayResult(
                query_id,
                success,
                reason,
                step + 1,
                (step + 1) * options.dt,
                final_error,
                max_lateral,
                mean_lateral,
                max_z_step,
                portal_count,
                kind_transitions,
                trace,
            )

    final_error = math.hypot(x - samples[-1].x, y - samples[-1].y)
    max_lateral = max(lateral_errors) if lateral_errors else 0.0
    mean_lateral = sum(lateral_errors) / len(lateral_errors) if lateral_errors else 0.0
    reason = "timeout" if final_error > options.goal_tolerance_m else "lateral_error_exceeded"
    return ReplayResult(
        query_id,
        False,
        reason,
        max_steps,
        max_steps * options.dt,
        final_error,
        max_lateral,
        mean_lateral,
        max_z_step,
        portal_count,
        kind_transitions,
        trace,
    )


def kind_groups(samples: Sequence[PathSample]) -> Dict[str, List[PathSample]]:
    grouped: Dict[str, List[PathSample]] = {"surface": [], "backbone": [], "portal": [], "unknown": []}
    for sample in samples:
        grouped.setdefault(sample.kind, []).append(sample)
    return grouped


def plot_replay(record: dict, result: ReplayResult, output: Path) -> None:
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")
    import matplotlib.pyplot as plt

    samples = path_samples(record)
    grouped = kind_groups(samples)
    fig, ax = plt.subplots(figsize=(10, 8))
    backbone = record.get("global_backbone", [])
    if len(backbone) >= 2:
        ax.plot([p["x"] for p in backbone], [p["y"] for p in backbone], color="#b0b0b0", linewidth=0.7, alpha=0.45, label="global backbone")

    path = record.get("path", [])
    if len(path) >= 2:
        ax.plot([p["x"] for p in path], [p["y"] for p in path], color="black", linewidth=1.0, alpha=0.50, label="global path")

    colors = {"surface": "#2ca02c", "backbone": "#1f77b4", "portal": "#d62728", "unknown": "#7f7f7f"}
    sizes = {"surface": 6, "backbone": 8, "portal": 28, "unknown": 6}
    for kind, points in grouped.items():
        if not points:
            continue
        ax.scatter([p.x for p in points], [p.y for p in points], s=sizes.get(kind, 6), c=colors.get(kind, "#7f7f7f"), label=kind)

    if result.trace:
        ax.plot([p["x"] for p in result.trace], [p["y"] for p in result.trace], color="#ff7f0e", linewidth=1.6, label="replay pose")
        ax.scatter([result.trace[-1]["x"]], [result.trace[-1]["y"]], c="#ff7f0e", s=60, marker="x", label="replay final")

    start = record.get("start", path[0] if path else {"x": 0.0, "y": 0.0})
    goal = record.get("goal", path[-1] if path else {"x": 0.0, "y": 0.0})
    ax.scatter([start["x"]], [start["y"]], c="#00aa00", s=80, label="start")
    ax.scatter([goal["x"]], [goal["y"]], c="#cc0000", s=80, label="goal")

    status = "PASS" if result.success else "FAIL"
    ax.set_title(
        f"query {result.query_id} {status} final={result.final_error_m:.2f}m "
        f"lat_max={result.max_lateral_error_m:.2f}m portals={result.portal_count}"
    )
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linewidth=0.3)
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(output, dpi=170)
    plt.close(fig)


def write_summary(output: Path, source_jsonl: Path, results: Sequence[ReplayResult], plotted: Sequence[Path]) -> None:
    passed = sum(1 for result in results if result.success)
    max_lat = max((result.max_lateral_error_m for result in results), default=0.0)
    mean_lat = sum(result.mean_lateral_error_m for result in results) / len(results) if results else 0.0
    max_final = max((result.final_error_m for result in results), default=0.0)
    with output.open("w", encoding="utf-8") as handle:
        handle.write("# TGW Global Path Tracking Replay\n\n")
        handle.write(f"Source JSONL: `{source_jsonl}`\n\n")
        handle.write("This is a core-only 2D lookahead replay. It does not simulate robot dynamics; it checks whether the global path contract can be consumed by a local x/y/yaw tracker.\n\n")
        handle.write("## Summary\n\n")
        handle.write(f"- Queries: {len(results)}\n")
        handle.write(f"- Passed: {passed}\n")
        handle.write(f"- Failed: {len(results) - passed}\n")
        handle.write(f"- Max final error: {max_final:.3f} m\n")
        handle.write(f"- Max lateral error: {max_lat:.3f} m\n")
        handle.write(f"- Mean lateral error: {mean_lat:.3f} m\n\n")
        handle.write("## Results\n\n")
        handle.write("| Query | Result | Reason | Duration s | Final error m | Max lateral m | Mean lateral m | Portals | Image |\n")
        handle.write("| ---: | --- | --- | ---: | ---: | ---: | ---: | ---: | --- |\n")
        plotted_by_query = {int(path.stem.split("_")[1]): path.name for path in plotted if "_" in path.stem}
        for result in results:
            image = plotted_by_query.get(result.query_id, "")
            image_link = f"[{image}]({image})" if image else ""
            handle.write(
                f"| {result.query_id} | {'PASS' if result.success else 'FAIL'} | {result.failure_reason} | "
                f"{result.duration_s:.2f} | {result.final_error_m:.3f} | {result.max_lateral_error_m:.3f} | "
                f"{result.mean_lateral_error_m:.3f} | {result.portal_count} | {image_link} |\n"
            )


def parse_indices(text: str, count: int) -> List[int]:
    if text == "all":
        return list(range(count))
    result = []
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            left, right = part.split("-", 1)
            result.extend(range(int(left), int(right) + 1))
        else:
            result.append(int(part))
    return [idx for idx in result if 0 <= idx < count]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("jsonl", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--indices", default="all")
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument("--lookahead", type=float, default=0.8)
    parser.add_argument("--projection-window", type=float, default=5.0)
    parser.add_argument("--max-yaw-rate", type=float, default=1.2)
    parser.add_argument("--goal-tolerance", type=float, default=0.35)
    parser.add_argument("--max-time", type=float, default=900.0)
    parser.add_argument("--max-lateral-error", type=float, default=1.20)
    parser.add_argument("--plot-limit", type=int, default=20)
    args = parser.parse_args()

    records = load_records(args.jsonl)
    indices = parse_indices(args.indices, len(records))
    args.output_dir.mkdir(parents=True, exist_ok=True)
    options = ReplayOptions(
        dt=args.dt,
        lookahead_m=args.lookahead,
        projection_window_m=args.projection_window,
        max_yaw_rate_radps=args.max_yaw_rate,
        goal_tolerance_m=args.goal_tolerance,
        max_time_s=args.max_time,
        max_lateral_error_m=args.max_lateral_error,
    )

    results: List[ReplayResult] = []
    plotted: List[Path] = []
    for rank, idx in enumerate(indices):
        record = records[idx]
        if not record.get("success", False):
            continue
        result = replay_record(record, options)
        results.append(result)
        if rank < args.plot_limit:
            output = args.output_dir / f"query_{result.query_id:02d}_tracking.png"
            plot_replay(record, result, output)
            plotted.append(output)

    json_out = args.output_dir / "tracking_replay_results.jsonl"
    with json_out.open("w", encoding="utf-8") as handle:
        for result in results:
            payload = {
                "query_id": result.query_id,
                "success": result.success,
                "failure_reason": result.failure_reason,
                "steps": result.steps,
                "duration_s": result.duration_s,
                "final_error_m": result.final_error_m,
                "max_lateral_error_m": result.max_lateral_error_m,
                "mean_lateral_error_m": result.mean_lateral_error_m,
                "max_z_step_m": result.max_z_step_m,
                "portal_count": result.portal_count,
                "kind_transitions": result.kind_transitions,
            }
            handle.write(json.dumps(payload, sort_keys=True) + "\n")

    write_summary(args.output_dir / "tracking_replay_summary.md", args.jsonl, results, plotted)
    passed = sum(1 for result in results if result.success)
    print(f"tracking_replay queries={len(results)} passed={passed} failed={len(results) - passed}")
    print(f"summary={args.output_dir / 'tracking_replay_summary.md'}")


if __name__ == "__main__":
    main()
