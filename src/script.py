import argparse
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
import trimesh
from shapely.geometry import Polygon

WINDOW_W = 1024
WINDOW_H = 768
UI_BOTTOM = 100
MARGIN = 40

USABLE_W = WINDOW_W - 2 * MARGIN
USABLE_H = WINDOW_H - UI_BOTTOM - MARGIN

DEFAULT_Z = 10.0
DEFAULT_TOL = 0.5
RETRY_TOL_FACTOR = 4

POLYGONS_SUBDIR = ("assets", "polygons")


def close_ring(verts):
    if np.allclose(verts[0], verts[-1]):
        return verts
    return np.vstack([verts, verts[0]])


def diagonal(verts):
    return np.linalg.norm(verts.max(axis=0) - verts.min(axis=0))


def simplify(verts, tolerance):
    if len(verts) < 4:
        return verts

    pts = verts[:-1] if np.allclose(verts[0], verts[-1]) else verts.copy()
    try:
        simplified = Polygon(pts).simplify(tolerance, preserve_topology=True)
        coords = np.array(simplified.exterior.coords)[:-1]
        if len(coords) >= 3:
            return coords
    except Exception:
        pass
    return pts


def is_simple(verts):
    if len(verts) < 3:
        return False
    try:
        return Polygon(verts).is_valid
    except Exception:
        return False


def extract_contours(path_2d, tolerance):
    raw = [close_ring(path_2d.vertices[e.points]) for e in path_2d.entities]
    if not raw:
        return None, []

    raw.sort(key=diagonal, reverse=True)

    valid = []
    for i, verts in enumerate(raw):
        candidate = simplify(verts, tolerance)

        if not is_simple(candidate):
            candidate = simplify(verts, tolerance * RETRY_TOL_FACTOR)
            if not is_simple(candidate):
                if i == 0:
                    return None, []
                continue

        valid.append(candidate)

    return valid[0], valid[1:]


def normalize(contours):
    all_pts = np.vstack(contours)
    min_xy = all_pts.min(axis=0)
    extent = all_pts.max(axis=0) - min_xy

    if extent[0] < 1e-9 or extent[1] < 1e-9:
        return contours

    scale = min(USABLE_W / extent[0], USABLE_H / extent[1])
    offset = np.array([
        MARGIN + (USABLE_W - extent[0] * scale) / 2.0,
        MARGIN + (USABLE_H - extent[1] * scale) / 2.0,
    ])

    return [(verts - min_xy) * scale + offset for verts in contours]


def export(outer, holes, output_path):
    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)

    lines = ["OUTER"]
    lines += [f"{x:.6f},{y:.6f}" for x, y in outer]
    for hole in holes:
        lines.append("HOLE")
        lines += [f"{x:.6f},{y:.6f}" for x, y in hole]

    with open(output_path, "w") as f:
        f.write("\n".join(lines) + "\n")


def preview(outer, holes, height_z, name):
    fig, ax = plt.subplots(figsize=(9, 7))

    ax.plot(*close_ring(outer).T, "b-", linewidth=1.5, label="Outer contour")
    ax.scatter(*outer.T, color="blue", s=8, zorder=3)

    for i, hole in enumerate(holes):
        ax.plot(*close_ring(hole).T, "r-", linewidth=1.2,
                label="Hole" if i == 0 else "_nolegend_")
        ax.scatter(*hole.T, color="red", s=6, zorder=3)

    ax.add_patch(plt.Rectangle((MARGIN, MARGIN), USABLE_W, USABLE_H,
                               fill=False, edgecolor="gray",
                               linestyle="--", linewidth=0.8, label="Usable area"))

    ax.set_aspect("equal")
    ax.invert_yaxis()
    ax.set_xlim(-10, WINDOW_W + 10)
    ax.set_ylim(WINDOW_H + 10, -10)
    ax.set_title(f"{name}  |  Z = {height_z} mm  |  "
                 f"{len(outer)} pts + {len(holes)} holes")
    ax.legend()
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()


def load_mesh(path):
    loaded = trimesh.load(path)

    if not isinstance(loaded, trimesh.Scene):
        return loaded

    geometries = list(loaded.geometry.values())
    if not geometries:
        return None
    return max(geometries, key=lambda m: m.volume if hasattr(m, "volume") else 0)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Slice a .3mf and export the contour as a .txt."
    )
    parser.add_argument("name",
                        help="file name without extension (assets/polygons/<name>.3mf)")
    parser.add_argument("--z", type=float, default=DEFAULT_Z, metavar="HEIGHT",
                        help=f"slice height Z in mm (default: {DEFAULT_Z})")
    parser.add_argument("--tol", type=float, default=DEFAULT_TOL, metavar="TOL",
                        help=f"RDP tolerance in mm (default: {DEFAULT_TOL})")
    parser.add_argument("--no-preview", action="store_true",
                        help="skip the preview window")
    return parser.parse_args()


def main():
    args = parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    polygons_dir = os.path.join(script_dir, *POLYGONS_SUBDIR)
    input_path = os.path.join(polygons_dir, f"{args.name}.3mf")
    output_path = os.path.join(polygons_dir, f"{args.name}.txt")

    if not os.path.isfile(input_path):
        sys.exit(1)

    mesh = load_mesh(input_path)
    if mesh is None:
        sys.exit(1)

    section = mesh.section(plane_origin=[0, 0, args.z], plane_normal=[0, 0, 1])
    if section is None:
        sys.exit(1)

    path_2d, _ = section.to_planar()

    outer, holes = extract_contours(path_2d, args.tol)
    if outer is None:
        sys.exit(1)

    outer_norm, *holes_norm = normalize([outer] + holes)

    export(outer_norm, holes_norm, output_path)

    if not args.no_preview:
        preview(outer_norm, holes_norm, args.z, args.name)


if __name__ == "__main__":
    main()
