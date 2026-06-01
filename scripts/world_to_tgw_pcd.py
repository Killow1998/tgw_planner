#!/usr/bin/env python3
"""Convert simple Gazebo SDF/world geometry to an ASCII PCD for tgw_planner.

This mirrors the sampling policy used by jie_octomap/world_to_octomap_node:
thin horizontal boxes and stair-step-like boxes are sampled on their top
surface, while other boxes are sampled volumetrically.
"""

from __future__ import annotations

import argparse
import math
import xml.etree.ElementTree as ET
from pathlib import Path


def parse_floats(text: str | None) -> list[float]:
    if not text:
        return []
    return [float(x) for x in text.split()]


def pose_from_element(elem: ET.Element | None) -> tuple[float, float, float, float, float, float]:
    values = parse_floats(elem.text if elem is not None else None)
    values += [0.0] * (6 - len(values))
    return tuple(values[:6])  # type: ignore[return-value]


def matmul(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    out = [[0.0] * 3 for _ in range(3)]
    for i in range(3):
        for j in range(3):
            out[i][j] = sum(a[i][k] * b[k][j] for k in range(3))
    return out


def rot_from_rpy(roll: float, pitch: float, yaw: float) -> list[list[float]]:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rz = [[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]]
    ry = [[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]]
    rx = [[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]]
    return matmul(matmul(rz, ry), rx)


class Transform:
    def __init__(self, xyz=(0.0, 0.0, 0.0), r=None):
        self.xyz = xyz
        self.r = r if r is not None else [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

    @staticmethod
    def from_pose(pose: tuple[float, float, float, float, float, float]) -> "Transform":
        x, y, z, roll, pitch, yaw = pose
        return Transform((x, y, z), rot_from_rpy(roll, pitch, yaw))

    def compose(self, other: "Transform") -> "Transform":
        ox, oy, oz = other.xyz
        rx = self.r[0][0] * ox + self.r[0][1] * oy + self.r[0][2] * oz
        ry = self.r[1][0] * ox + self.r[1][1] * oy + self.r[1][2] * oz
        rz = self.r[2][0] * ox + self.r[2][1] * oy + self.r[2][2] * oz
        sx, sy, sz = self.xyz
        return Transform((sx + rx, sy + ry, sz + rz), matmul(self.r, other.r))

    def apply(self, point: tuple[float, float, float]) -> tuple[float, float, float]:
        x, y, z = point
        tx, ty, tz = self.xyz
        return (
            tx + self.r[0][0] * x + self.r[0][1] * y + self.r[0][2] * z,
            ty + self.r[1][0] * x + self.r[1][1] * y + self.r[1][2] * z,
            tz + self.r[2][0] * x + self.r[2][1] * y + self.r[2][2] * z,
        )


def add_point(points: set[tuple[int, int, int]], p: tuple[float, float, float], resolution: float, half_xy: float):
    x, y, z = p
    if half_xy > 0.0 and (abs(x) > half_xy or abs(y) > half_xy):
        return
    points.add((math.floor(x / resolution), math.floor(y / resolution), math.floor(z / resolution)))


def sample_box(
    points: set[tuple[int, int, int]],
    tf: Transform,
    size: tuple[float, float, float],
    resolution: float,
    half_xy: float,
):
    sx, sy, sz = size
    min_xy = min(sx, sy)
    max_xy = max(sx, sy)
    local_z_world = tf.r[2][2]
    near_horizontal = abs(local_z_world) > 0.9
    thin_ground_like = near_horizontal and sz <= 0.6
    stair_step_like = near_horizontal and sz <= 0.5 and min_xy <= 0.8 and max_xy >= 1.0

    if thin_ground_like or stair_step_like:
        step = max(resolution * 0.5, 1e-3)
        nx = max(1, math.ceil(sx / step))
        ny = max(1, math.ceil(sy / step))
        for ix in range(nx + 1):
            x = -sx * 0.5 + sx * ix / nx
            for iy in range(ny + 1):
                y = -sy * 0.5 + sy * iy / ny
                add_point(points, tf.apply((x, y, sz * 0.5)), resolution, half_xy)
        return

    min_dim = min(sx, sy, sz)
    step = max(resolution * 0.5, 1e-3) if min_dim <= 4.0 * resolution else resolution
    nx = max(1, math.ceil(sx / step))
    ny = max(1, math.ceil(sy / step))
    nz = max(1, math.ceil(sz / step))
    for ix in range(nx + 1):
        x = -sx * 0.5 + sx * ix / nx
        for iy in range(ny + 1):
            y = -sy * 0.5 + sy * iy / ny
            for iz in range(nz + 1):
                z = -sz * 0.5 + sz * iz / nz
                add_point(points, tf.apply((x, y, z)), resolution, half_xy)


def sample_plane(
    points: set[tuple[int, int, int]],
    tf: Transform,
    size: tuple[float, float],
    resolution: float,
    half_xy: float,
):
    sx, sy = size
    step = max(resolution * 0.5, 1e-3)
    nx = max(1, math.ceil(sx / step))
    ny = max(1, math.ceil(sy / step))
    for ix in range(nx + 1):
        x = -sx * 0.5 + sx * ix / nx
        for iy in range(ny + 1):
            y = -sy * 0.5 + sy * iy / ny
            add_point(points, tf.apply((x, y, 0.0)), resolution, half_xy)


def convert_world(world_path: Path, resolution: float, half_xy: float) -> list[tuple[float, float, float]]:
    root = ET.parse(world_path).getroot()
    world = root.find("world")
    if world is None:
        raise RuntimeError(f"no <world> in {world_path}")

    points: set[tuple[int, int, int]] = set()
    identity = Transform()
    for model in world.findall("model"):
        model_tf = identity.compose(Transform.from_pose(pose_from_element(model.find("pose"))))
        for link in model.findall("link"):
            link_tf = model_tf.compose(Transform.from_pose(pose_from_element(link.find("pose"))))
            for collision in link.findall("collision"):
                collision_tf = link_tf.compose(Transform.from_pose(pose_from_element(collision.find("pose"))))
                geom = collision.find("geometry")
                if geom is None:
                    continue
                box = geom.find("box")
                if box is not None:
                    size = parse_floats(box.findtext("size"))
                    if len(size) >= 3:
                        sample_box(points, collision_tf, (size[0], size[1], size[2]), resolution, half_xy)
                    continue
                plane = geom.find("plane")
                if plane is not None:
                    size = parse_floats(plane.findtext("size"))
                    if len(size) >= 2:
                        sample_plane(points, collision_tf, (size[0], size[1]), resolution, half_xy)

    return [
        ((ix + 0.5) * resolution, (iy + 0.5) * resolution, (iz + 0.5) * resolution)
        for ix, iy, iz in sorted(points)
    ]


def write_pcd(path: Path, points: list[tuple[float, float, float]]):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="ascii") as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z intensity\n")
        f.write("SIZE 4 4 4 4\n")
        f.write("TYPE F F F F\n")
        f.write("COUNT 1 1 1 1\n")
        f.write(f"WIDTH {len(points)}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {len(points)}\n")
        f.write("DATA ascii\n")
        for x, y, z in points:
            f.write(f"{x:.3f} {y:.3f} {z:.3f} 1.0\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--world", required=True)
    parser.add_argument("--output", default="/tmp/tgw_from_world.pcd")
    parser.add_argument("--resolution", type=float, default=0.20)
    parser.add_argument("--xy-range", type=float, default=24.0)
    args = parser.parse_args()

    points = convert_world(Path(args.world), args.resolution, args.xy_range * 0.5)
    write_pcd(Path(args.output), points)
    print(f"wrote {len(points)} points to {args.output}")


if __name__ == "__main__":
    main()
