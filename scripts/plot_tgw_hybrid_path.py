#!/usr/bin/env python3
"""Plot one JSONL record exported by tgw_experience_global_sweep."""

import argparse
import json
import os
from pathlib import Path


def load_record(path: Path, index: int):
    with path.open("r", encoding="utf-8") as handle:
        for line_index, line in enumerate(handle):
            if line_index == index:
                return json.loads(line)
    raise RuntimeError(f"record index {index} not found in {path}")


def points_by_kind(record):
    grouped = {"surface": [], "backbone": [], "portal": [], "unknown": []}
    for point in record.get("path", []):
        grouped.setdefault(point.get("kind", "unknown"), []).append(point)
    return grouped


def add_points(ax, points, label, color, size):
    if not points:
        return
    ax.scatter(
        [point["x"] for point in points],
        [point["y"] for point in points],
        [point["z"] for point in points],
        label=label,
        c=color,
        s=size,
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("jsonl", type=Path)
    parser.add_argument("--index", type=int, default=0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    record = load_record(args.jsonl, args.index)

    os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    path = record.get("path", [])
    if path:
        ax.plot(
            [point["x"] for point in path],
            [point["y"] for point in path],
            [point["z"] for point in path],
            color="black",
            linewidth=1.0,
            alpha=0.55,
            label="path",
        )

    grouped = points_by_kind(record)
    add_points(ax, grouped.get("surface", []), "surface", "#2ca02c", 5)
    add_points(ax, grouped.get("backbone", []), "backbone", "#1f77b4", 8)
    add_points(ax, grouped.get("portal", []), "portal", "#d62728", 24)
    add_points(ax, grouped.get("unknown", []), "unknown", "#7f7f7f", 5)

    start = record["start"]
    goal = record["goal"]
    ax.scatter([start["x"]], [start["y"]], [start["z"]], c="#00aa00", s=80, label="start")
    ax.scatter([goal["x"]], [goal["y"]], [goal["z"]], c="#cc0000", s=80, label="goal")

    metrics = record.get("metrics", {})
    ax.set_title(
        "query {qid} detour={detour:.2f} len={length:.1f}m backbone={backbone:.1f}m".format(
            qid=record.get("query_id", args.index),
            detour=metrics.get("detour_ratio", 0.0),
            length=metrics.get("path_length_m", 0.0),
            backbone=metrics.get("backbone_path_length_m", 0.0),
        )
    )
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    ax.legend(loc="best")
    ax.view_init(elev=35, azim=-55)
    fig.tight_layout()

    if args.output:
        fig.savefig(args.output, dpi=180)
    else:
        plt.show()


if __name__ == "__main__":
    main()
