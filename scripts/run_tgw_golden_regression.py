#!/usr/bin/env python3
"""Run core-only TGW golden scene regressions.

This script intentionally avoids ROS launch files. It runs the C++ global sweep
tool, then replays the exported paths with the lightweight tracking replay.
"""

import argparse
import json
import os
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Sequence


@dataclass(frozen=True)
class QuerySet:
    name: str
    mode: str
    low_z_max: float
    high_z_min: float
    same_z_min: float = 0.0
    same_z_max: float = 0.0


@dataclass(frozen=True)
class GoldenScene:
    name: str
    pbstream: str
    notes: str
    query_sets: Sequence[QuerySet]


SCENES: Dict[str, GoldenScene] = {
    "scene_20260608": GoldenScene(
        name="scene_20260608",
        pbstream="docs/data/tgw_n3map_nav_filtered.pbstream",
        notes="0608 stair / cross-floor experience scene.",
        query_sets=(
            QuerySet("cross_floor", "low-high", -0.20, 7.80),
            QuerySet("same_floor_low", "same-band", 0.0, 1.0, -0.20, 0.20),
            QuerySet("same_floor_high", "same-band", 0.0, 1.0, 7.80, 8.50),
        ),
    ),
    "scene_20260610": GoldenScene(
        name="scene_20260610",
        pbstream="docs/data/tgw_n3map_nav_filtered_20260610.pbstream",
        notes="0610 ramp / grade-change scene.",
        query_sets=(
            QuerySet("cross_floor", "low-high", 2.20, 3.10),
            QuerySet("same_floor_low", "same-band", 0.0, 1.0, 2.20, 2.70),
            QuerySet("same_floor_high", "same-band", 0.0, 1.0, 3.10, 3.60),
        ),
    ),
}


def package_root() -> Path:
    return Path(__file__).resolve().parents[1]


def workspace_root() -> Path:
    return package_root().parent.parent


def run_command(command: Sequence[str], cwd: Path, env: Dict[str, str]) -> None:
    print("+ " + " ".join(command), flush=True)
    subprocess.run(command, cwd=str(cwd), env=env, check=True)


def command_for_docs(command: Sequence[str], cwd: Path) -> str:
    parts: List[str] = []
    for index, arg in enumerate(command):
        if index == 0 and arg == sys.executable:
            parts.append("python3")
            continue
        try:
            path = Path(arg)
            if path.is_absolute():
                parts.append(str(path.relative_to(cwd)))
                continue
        except ValueError:
            pass
        parts.append(arg)
    return " ".join(parts)


def load_jsonl(path: Path) -> List[dict]:
    records: List[dict] = []
    if not path.exists():
        return records
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if line:
                records.append(json.loads(line))
    return records


def summarize_global(records: Sequence[dict]) -> Dict[str, float]:
    successes = [record for record in records if record.get("success", False)]
    metrics = [record.get("metrics", {}) for record in successes]
    return {
        "queries": float(len(records)),
        "success": float(len(successes)),
        "max_detour": max((float(metric.get("detour_ratio", 0.0)) for metric in metrics), default=0.0),
        "mean_detour": (
            sum(float(metric.get("detour_ratio", 0.0)) for metric in metrics) / len(metrics)
            if metrics else 0.0
        ),
        "max_backbone_ratio": max(
            (float(metric.get("backbone_ratio", 0.0)) for metric in metrics), default=0.0
        ),
        "max_portal_switch_count": max(
            (float(metric.get("portal_switch_count", 0.0)) for metric in metrics), default=0.0
        ),
        "max_path_edge_dz_m": max(
            (float(metric.get("max_path_edge_dz_m", 0.0)) for metric in metrics), default=0.0
        ),
    }


def summarize_replay(records: Sequence[dict]) -> Dict[str, float]:
    successes = [record for record in records if record.get("success", False)]
    return {
        "queries": float(len(records)),
        "passed": float(len(successes)),
        "max_final_error_m": max(
            (float(record.get("final_error_m", 0.0)) for record in records), default=0.0
        ),
        "max_lateral_error_m": max(
            (float(record.get("max_lateral_error_m", 0.0)) for record in records), default=0.0
        ),
        "mean_lateral_error_m": (
            sum(float(record.get("mean_lateral_error_m", 0.0)) for record in records) / len(records)
            if records else 0.0
        ),
        "max_z_step_m": max((float(record.get("max_z_step_m", 0.0)) for record in records), default=0.0),
    }


@dataclass(frozen=True)
class QualityGate:
    max_detour: float
    max_same_floor_detour: float
    max_backbone_ratio: float
    max_portal_switch_count: float
    max_path_edge_dz_m: float
    max_tracking_final_error_m: float
    max_tracking_lateral_error_m: float
    max_tracking_z_step_m: float


def global_quality_failures(query_set_name: str, stats: Dict[str, float], gate: QualityGate) -> List[str]:
    failures: List[str] = []
    if stats.get("max_detour", 0.0) > gate.max_detour:
        failures.append(f"max_detour>{gate.max_detour:.3f}")
    if query_set_name.startswith("same_floor") and stats.get("max_detour", 0.0) > gate.max_same_floor_detour:
        failures.append(f"same_floor_max_detour>{gate.max_same_floor_detour:.3f}")
    if stats.get("max_backbone_ratio", 0.0) > gate.max_backbone_ratio:
        failures.append(f"max_backbone_ratio>{gate.max_backbone_ratio:.3f}")
    if stats.get("max_portal_switch_count", 0.0) > gate.max_portal_switch_count:
        failures.append(f"max_portal_switch_count>{gate.max_portal_switch_count:.0f}")
    if stats.get("max_path_edge_dz_m", 0.0) > gate.max_path_edge_dz_m:
        failures.append(f"max_path_edge_dz_m>{gate.max_path_edge_dz_m:.3f}")
    return failures


def replay_quality_failures(stats: Dict[str, float], gate: QualityGate) -> List[str]:
    failures: List[str] = []
    if stats.get("max_final_error_m", 0.0) > gate.max_tracking_final_error_m:
        failures.append(f"max_final_error_m>{gate.max_tracking_final_error_m:.3f}")
    if stats.get("max_lateral_error_m", 0.0) > gate.max_tracking_lateral_error_m:
        failures.append(f"max_lateral_error_m>{gate.max_tracking_lateral_error_m:.3f}")
    if stats.get("max_z_step_m", 0.0) > gate.max_tracking_z_step_m:
        failures.append(f"max_z_step_m>{gate.max_tracking_z_step_m:.3f}")
    return failures


def format_count(value: float) -> str:
    return str(int(value))


def write_scene_summary(
    scene: GoldenScene,
    output_dir: Path,
    sample_pairs: int,
    gate: QualityGate,
    global_stats: Dict[str, Dict[str, float]],
    replay_stats: Dict[str, Dict[str, float]],
    quality_failures: Dict[str, Sequence[str]],
    commands: Sequence[str],
) -> None:
    summary = output_dir / "README.md"
    with summary.open("w", encoding="utf-8") as handle:
        handle.write(f"# TGW Golden Regression - {scene.name}\n\n")
        handle.write(f"Input map: `{scene.pbstream}`\n\n")
        handle.write(f"{scene.notes}\n\n")
        handle.write(f"Sample pairs per query set: `{sample_pairs}`\n\n")
        handle.write("## Commands\n\n")
        handle.write("```bash\n")
        for command in commands:
            handle.write(command + "\n")
        handle.write("```\n\n")
        handle.write("## Global Sweep\n\n")
        handle.write(
            "| Set | Queries | Success | Mean detour | Max detour | Max backbone ratio | "
            "Max portal switches | Max path dz m |\n"
        )
        handle.write("| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |\n")
        for query_set in scene.query_sets:
            stats = global_stats.get(query_set.name, {})
            handle.write(
                f"| {query_set.name} | {format_count(stats.get('queries', 0.0))} | "
                f"{format_count(stats.get('success', 0.0))} | "
                f"{stats.get('mean_detour', 0.0):.3f} | {stats.get('max_detour', 0.0):.3f} | "
                f"{stats.get('max_backbone_ratio', 0.0):.3f} | "
                f"{format_count(stats.get('max_portal_switch_count', 0.0))} | "
                f"{stats.get('max_path_edge_dz_m', 0.0):.3f} |\n"
            )
        handle.write("\n## Tracking Replay\n\n")
        handle.write(
            "| Set | Queries | Passed | Max final error m | Max lateral error m | "
            "Mean lateral error m | Max path z step m | Summary |\n"
        )
        handle.write("| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |\n")
        for query_set in scene.query_sets:
            stats = replay_stats.get(query_set.name, {})
            tracking_dir = f"{query_set.name}_tracking"
            handle.write(
                f"| {query_set.name} | {format_count(stats.get('queries', 0.0))} | "
                f"{format_count(stats.get('passed', 0.0))} | "
                f"{stats.get('max_final_error_m', 0.0):.3f} | "
                f"{stats.get('max_lateral_error_m', 0.0):.3f} | "
                f"{stats.get('mean_lateral_error_m', 0.0):.3f} | "
                f"{stats.get('max_z_step_m', 0.0):.3f} | "
                f"[summary]({tracking_dir}/tracking_replay_summary.md) |\n"
            )
        handle.write("\n## Interpretation\n\n")
        handle.write(
            "This is a core-only regression gate. It validates global path generation and "
            "the lightweight tracking replay contract, not robot dynamics or hardware command safety.\n"
        )
        handle.write("\n## Quality Gate\n\n")
        handle.write(
            f"- max detour: `{gate.max_detour:.3f}`\n"
            f"- same-floor max detour: `{gate.max_same_floor_detour:.3f}`\n"
            f"- max backbone ratio: `{gate.max_backbone_ratio:.3f}`\n"
            f"- max portal switches: `{gate.max_portal_switch_count:.0f}`\n"
            f"- max path edge dz: `{gate.max_path_edge_dz_m:.3f}m`\n"
            f"- max tracking final error: `{gate.max_tracking_final_error_m:.3f}m`\n"
            f"- max tracking lateral error: `{gate.max_tracking_lateral_error_m:.3f}m`\n"
            f"- max tracking z step: `{gate.max_tracking_z_step_m:.3f}m`\n\n"
        )
        if not quality_failures:
            handle.write("All configured core path-quality gates passed.\n")
        else:
            for query_set, failures in quality_failures.items():
                handle.write(f"- `{query_set}`: {', '.join(failures)}\n")


def sweep_command(
    workspace: Path,
    package: Path,
    scene: GoldenScene,
    query_set: QuerySet,
    sample_pairs: int,
    jsonl_path: Path,
) -> List[str]:
    binary = workspace / "build/tgw_planner/tgw_experience_global_sweep"
    command = [
        str(binary),
        str(package / scene.pbstream),
        f"{query_set.low_z_max:.6g}",
        f"{query_set.high_z_min:.6g}",
        str(sample_pairs),
        "--dominant-only",
        "--export-jsonl",
        str(jsonl_path),
    ]
    if query_set.mode == "same-band":
        command.extend(
            [
                "--mode",
                "same-band",
                "--same-z-min",
                f"{query_set.same_z_min:.6g}",
                "--same-z-max",
                f"{query_set.same_z_max:.6g}",
            ]
        )
    return command


def replay_command(package: Path, jsonl_path: Path, output_dir: Path, plot_limit: int) -> List[str]:
    return [
        sys.executable,
        str(package / "scripts/replay_tgw_global_path.py"),
        str(jsonl_path),
        "--output-dir",
        str(output_dir),
        "--plot-limit",
        str(plot_limit),
    ]


def normalize_scene_names(names: Iterable[str]) -> List[str]:
    out: List[str] = []
    for name in names:
        normalized = name
        if normalized in ("all", "*"):
            out.extend(SCENES.keys())
            continue
        if normalized == "20260608":
            normalized = "scene_20260608"
        elif normalized == "20260610":
            normalized = "scene_20260610"
        if normalized not in SCENES:
            raise ValueError(f"unknown scene: {name}")
        out.append(normalized)
    return sorted(set(out), key=out.index)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--scene",
        action="append",
        default=None,
        help="scene_20260608, scene_20260610, 20260608, 20260610, or all",
    )
    parser.add_argument("--sample-pairs", type=int, default=50)
    parser.add_argument("--plot-limit", type=int, default=8)
    parser.add_argument("--output-name", default="golden_regression")
    parser.add_argument("--max-detour", type=float, default=100.0)
    parser.add_argument("--max-same-floor-detour", type=float, default=6.0)
    parser.add_argument("--max-backbone-ratio", type=float, default=0.85)
    parser.add_argument("--max-portal-switch-count", type=float, default=12.0)
    parser.add_argument("--max-path-edge-dz-m", type=float, default=0.85)
    parser.add_argument("--max-tracking-final-error-m", type=float, default=0.40)
    parser.add_argument("--max-tracking-lateral-error-m", type=float, default=0.90)
    parser.add_argument("--max-tracking-z-step-m", type=float, default=0.85)
    args = parser.parse_args()

    package = package_root()
    workspace = workspace_root()
    binary = workspace / "build/tgw_planner/tgw_experience_global_sweep"
    if not binary.exists():
        raise SystemExit(f"missing global sweep binary: {binary}. Run colcon build first.")

    env = os.environ.copy()
    env.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")
    gate = QualityGate(
        max_detour=args.max_detour,
        max_same_floor_detour=args.max_same_floor_detour,
        max_backbone_ratio=args.max_backbone_ratio,
        max_portal_switch_count=args.max_portal_switch_count,
        max_path_edge_dz_m=args.max_path_edge_dz_m,
        max_tracking_final_error_m=args.max_tracking_final_error_m,
        max_tracking_lateral_error_m=args.max_tracking_lateral_error_m,
        max_tracking_z_step_m=args.max_tracking_z_step_m,
    )
    commands_for_final: List[str] = []
    failed = False

    requested_scenes = args.scene if args.scene is not None else ["all"]
    for scene_name in normalize_scene_names(requested_scenes):
        scene = SCENES[scene_name]
        scene_output = package / "docs/exp" / scene.name / args.output_name
        scene_output.mkdir(parents=True, exist_ok=True)
        global_stats: Dict[str, Dict[str, float]] = {}
        replay_stats: Dict[str, Dict[str, float]] = {}
        quality_failures: Dict[str, Sequence[str]] = {}
        scene_commands: List[str] = []

        for query_set in scene.query_sets:
            jsonl_path = scene_output / f"{query_set.name}_paths.jsonl"
            tracking_output = scene_output / f"{query_set.name}_tracking"

            sweep = sweep_command(workspace, package, scene, query_set, args.sample_pairs, jsonl_path)
            replay = replay_command(package, jsonl_path, tracking_output, args.plot_limit)
            scene_commands.extend(
                [
                    command_for_docs(sweep, workspace),
                    "MPLCONFIGDIR=/tmp/matplotlib " + command_for_docs(replay, workspace),
                ]
            )

            run_command(sweep, workspace, env)
            run_command(replay, workspace, env)

            global_records = load_jsonl(jsonl_path)
            replay_records = load_jsonl(tracking_output / "tracking_replay_results.jsonl")
            global_stats[query_set.name] = summarize_global(global_records)
            replay_stats[query_set.name] = summarize_replay(replay_records)

            if global_stats[query_set.name]["success"] < args.sample_pairs:
                failed = True
            if replay_stats[query_set.name]["passed"] < args.sample_pairs:
                failed = True
            failures = (
                global_quality_failures(query_set.name, global_stats[query_set.name], gate) +
                replay_quality_failures(replay_stats[query_set.name], gate)
            )
            if failures:
                quality_failures[query_set.name] = failures
                failed = True

        write_scene_summary(
            scene,
            scene_output,
            args.sample_pairs,
            gate,
            global_stats,
            replay_stats,
            quality_failures,
            scene_commands,
        )
        commands_for_final.append(str(scene_output / "README.md"))
        print(f"scene_summary={scene_output / 'README.md'}", flush=True)

    if failed:
        raise SystemExit(1)
    print("golden_regression_success=true", flush=True)
    for summary in commands_for_final:
        print(f"summary={summary}", flush=True)


if __name__ == "__main__":
    main()
