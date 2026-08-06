#!/usr/bin/env python3
"""
stl_face_survey.py - measure the FLAT FACES of the printed parts so that
fiducial-marker stickers can be sized from real geometry instead of guessed.

WHY THIS EXISTS
    The marker design in Documentation/MARKER-SYSTEM.md needs one number per
    candidate sticker location: how big a square can actually be stuck on that
    face. A bounding box does not answer that - Alt_Kapak.stl is 203.6 x 113.6
    but it is a cover full of cutouts, and El_Ust.stl loses about a quarter of
    its footprint to holes. The answer is the LARGEST INSCRIBED AXIS-ALIGNED
    SQUARE of the actual triangle coverage of one planar patch, which is what
    this script computes.

WHAT IT DOES
    1. Reads a binary STL with the standard library + numpy. The file is
       validated as binary by the exact arithmetic 84 + 50*n == filesize; a
       mismatch is a hard error, never a silent misparse.
    2. Groups triangles into PLANAR PATCHES: first by facet normal direction
       (greedy angular clustering, default 5 deg), then by plane offset
       d = n . p (default 0.4 mm) so that two parallel faces at different
       heights are two patches, not one.
    3. For each patch: total triangle area, the in-plane 2-D bounding box, the
       fill ratio (patch area / bbox area - the hole tell), and the largest
       inscribed square, found by rasterising the patch and binary-searching
       square size against a summed-area table.

WHAT IT CANNOT TELL YOU
    These STLs are individual parts in PRINT orientation. There is no assembly
    file in the repository, so part-to-part and link-to-link transforms are
    UNKNOWN. Every coordinate this script prints is in the part's own STL frame.
    Do not read a normal of (0,0,1) as "up on the assembled arm".

USAGE
    python Software/vision/stl_face_survey.py                    # the 9 candidates
    python Software/vision/stl_face_survey.py --all              # all 21 parts
    python Software/vision/stl_face_survey.py --csv out.csv      # machine-readable
    python Software/vision/stl_face_survey.py Backups/STL_parts/El.stl

DEPENDENCIES
    numpy      - BSD-3-Clause. FLAGGED: the repository rule is Apache-2.0/MIT
                 only. numpy is already an unavoidable dependency of the vision
                 and LeRobot work; it is flagged here, not silently accepted.
    opencv     - Apache-2.0. Used only for polygon rasterisation. If cv2 is
                 missing the script falls back to a numpy barycentric
                 rasteriser, which is slower but gives the same answer.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import struct
import sys
from dataclasses import dataclass

import numpy as np

try:
    import cv2

    _HAVE_CV2 = True
except ImportError:  # pragma: no cover - exercised only on a machine without cv2
    _HAVE_CV2 = False


# The parts that could plausibly carry a sticker. Gears, shafts, the jack cover
# and the cable holder are excluded on purpose - see MARKER-SYSTEM.md "where NOT
# to place". Pass --all to survey everything anyway.
CANDIDATE_PARTS = [
    "Alt_Kasa.stl",
    "Alt_Kapak.stl",
    "Alt_Govde.stl",
    "Tabla_Alt.stl",
    "Alt_Kol.stl",
    "On_Kol.stl",
    "Bilek.stl",
    "El.stl",
    "El_Ust.stl",
    "Parmak_2 X 2.stl",
]

DEFAULT_STL_DIR = os.path.join("Backups", "STL_parts")


# --------------------------------------------------------------------------
# binary STL reading
# --------------------------------------------------------------------------


class NotBinarySTL(ValueError):
    """Raised when 84 + 50*n does not equal the file size."""


def read_binary_stl(path: str) -> np.ndarray:
    """Return an (N, 3, 3) float64 array of triangle vertices, in mm.

    Validates the binary layout arithmetically and raises NotBinarySTL rather
    than returning a plausible-looking wrong answer.
    """
    size = os.path.getsize(path)
    with open(path, "rb") as fh:
        fh.read(80)  # header, ignored
        (count,) = struct.unpack("<I", fh.read(4))
        expected = 84 + 50 * count
        if expected != size:
            raise NotBinarySTL(
                f"{os.path.basename(path)}: header claims {count} triangles "
                f"=> expected {expected} bytes, file is {size}. "
                "Not a well-formed binary STL; refusing to guess."
            )
        raw = np.frombuffer(fh.read(50 * count), dtype=np.uint8)

    # 50 bytes per triangle: 12 little-endian float32 then a uint16 attribute.
    # Drop the trailing 2 bytes, then reinterpret the remaining 48 as floats.
    tri = raw.reshape(count, 50)[:, :48].copy()
    floats = tri.view("<f4").reshape(count, 4, 3)
    return floats[:, 1:4, :].astype(np.float64)  # index 0 is the stored normal


def triangle_normals_and_areas(v: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Recompute normals from the vertices - the stored normal is not trusted."""
    cross = np.cross(v[:, 1] - v[:, 0], v[:, 2] - v[:, 0])
    twice_area = np.linalg.norm(cross, axis=1)
    areas = 0.5 * twice_area
    safe = np.where(twice_area > 1e-12, twice_area, 1.0)
    normals = cross / safe[:, None]
    return normals, areas


# --------------------------------------------------------------------------
# planar patch grouping
# --------------------------------------------------------------------------


@dataclass
class Patch:
    part: str
    normal: np.ndarray
    offset: float
    area: float
    n_tris: int
    bbox_u: float
    bbox_v: float
    fill_ratio: float
    inscribed_square: float
    centroid: np.ndarray
    square_centre: np.ndarray
    second_square: float
    second_centre: np.ndarray
    pair_baseline: float

    @property
    def label(self) -> str:
        n = self.normal
        return f"n=({n[0]:+.2f},{n[1]:+.2f},{n[2]:+.2f})"


def in_plane_basis(normal: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Two orthonormal vectors spanning the plane with this normal."""
    seed = np.array([1.0, 0.0, 0.0])
    if abs(float(np.dot(seed, normal))) > 0.9:
        seed = np.array([0.0, 1.0, 0.0])
    u = np.cross(normal, seed)
    u /= np.linalg.norm(u)
    v = np.cross(normal, u)
    return u, v


def group_planar_patches(
    verts: np.ndarray,
    normals: np.ndarray,
    areas: np.ndarray,
    angle_tol_deg: float,
    offset_tol_mm: float,
    min_area_mm2: float,
) -> list[tuple[np.ndarray, float, np.ndarray]]:
    """Greedy cluster by normal direction, then split each cluster by plane offset.

    Returns a list of (mean_normal, offset, triangle_index_array).
    """
    cos_tol = math.cos(math.radians(angle_tol_deg))
    order = np.argsort(-areas)  # seed clusters from the biggest facets

    seeds: list[np.ndarray] = []
    assignment = np.full(len(normals), -1, dtype=np.int64)
    for idx in order:
        n = normals[idx]
        placed = False
        for k, seed in enumerate(seeds):
            if float(np.dot(n, seed)) >= cos_tol:
                assignment[idx] = k
                placed = True
                break
        if not placed:
            seeds.append(n)
            assignment[idx] = len(seeds) - 1

    patches: list[tuple[np.ndarray, float, np.ndarray]] = []
    centroids = verts.mean(axis=1)
    for k, seed in enumerate(seeds):
        members = np.flatnonzero(assignment == k)
        if members.size == 0:
            continue
        # Area-weighted mean normal is more stable than the seed facet alone.
        mean_n = (normals[members] * areas[members, None]).sum(axis=0)
        norm = np.linalg.norm(mean_n)
        if norm < 1e-12:
            continue
        mean_n /= norm

        d = centroids[members] @ mean_n
        sub_order = np.argsort(d)
        run_start = 0
        for i in range(1, len(sub_order) + 1):
            at_end = i == len(sub_order)
            if at_end or (d[sub_order[i]] - d[sub_order[i - 1]]) > offset_tol_mm:
                run = members[sub_order[run_start:i]]
                if areas[run].sum() >= min_area_mm2:
                    patches.append((mean_n, float(d[sub_order[run_start:i]].mean()), run))
                run_start = i
    return patches


# --------------------------------------------------------------------------
# largest inscribed square
# --------------------------------------------------------------------------


def rasterise(poly_px: list[np.ndarray], w: int, h: int) -> np.ndarray:
    """Union-rasterise triangles given as integer pixel polygons."""
    mask = np.zeros((h, w), dtype=np.uint8)
    if _HAVE_CV2:
        cv2.fillPoly(mask, poly_px, 1)
        return mask
    for tri in poly_px:  # pragma: no cover - numpy fallback
        x0, y0 = tri[:, 0].min(), tri[:, 1].min()
        x1, y1 = tri[:, 0].max() + 1, tri[:, 1].max() + 1
        if x1 <= x0 or y1 <= y0:
            continue
        xs, ys = np.meshgrid(np.arange(x0, x1), np.arange(y0, y1))
        ax, ay = tri[0]
        bx, by = tri[1]
        cx, cy = tri[2]
        den = (by - cy) * (ax - cx) + (cx - bx) * (ay - cy)
        if abs(den) < 1e-9:
            continue
        wa = ((by - cy) * (xs - cx) + (cx - bx) * (ys - cy)) / den
        wb = ((cy - ay) * (xs - cx) + (ax - cx) * (ys - cy)) / den
        inside = (wa >= -0.02) & (wb >= -0.02) & (wa + wb <= 1.02)
        mask[y0:y1, x0:x1][inside] = 1
    return mask


def largest_inscribed_square_px(mask: np.ndarray) -> tuple[int, int, int]:
    """Largest all-ones axis-aligned square: (side, top_row, left_col).

    Binary search on side length; each test is one vectorised comparison over a
    summed-area table, so this is O(HW log min(H, W)).
    """
    sat = mask.astype(np.int64).cumsum(0).cumsum(1)
    sat = np.pad(sat, ((1, 0), (1, 0)))
    h, w = mask.shape
    best = (0, 0, 0)

    def probe(k: int) -> tuple[int, int] | None:
        if k <= 0 or k > h or k > w:
            return None
        block = sat[k:, k:] - sat[:-k, k:] - sat[k:, :-k] + sat[:-k, :-k]
        hits = np.flatnonzero(block.ravel() == k * k)
        if hits.size == 0:
            return None
        r, c = divmod(int(hits[0]), block.shape[1])
        return r, c

    lo, hi = 0, min(h, w)
    while lo < hi:
        mid = (lo + hi + 1) // 2
        found = probe(mid)
        if found is not None:
            lo = mid
            best = (mid, found[0], found[1])
        else:
            hi = mid - 1
    return best


def two_disjoint_squares_px(
    mask: np.ndarray,
) -> tuple[tuple[int, int, int], tuple[int, int, int]]:
    """Greedy pair of NON-OVERLAPPING squares: the best one, then the best one
    remaining after the first is erased together with a one-square-side
    exclusion margin around it.

    This is the evidence behind every "two markers on this link" claim. A link
    that cannot seat two disjoint squares does not get a two-marker design.
    """
    first = largest_inscribed_square_px(mask)
    side, r, c = first
    if side == 0:
        return first, (0, 0, 0)
    margin = max(2, side // 3)
    scratch = mask.copy()
    scratch[
        max(0, r - margin) : r + side + margin,
        max(0, c - margin) : c + side + margin,
    ] = 0
    return first, largest_inscribed_square_px(scratch)


def measure_patch(
    part: str,
    verts: np.ndarray,
    areas: np.ndarray,
    normal: np.ndarray,
    offset: float,
    idx: np.ndarray,
    pitch_mm: float,
) -> Patch:
    u, v = in_plane_basis(normal)
    pts = verts[idx].reshape(-1, 3)
    uu = pts @ u
    vv = pts @ v
    u0, v0 = uu.min(), vv.min()
    span_u, span_v = uu.max() - u0, vv.max() - v0

    w = max(int(math.ceil(span_u / pitch_mm)) + 2, 2)
    h = max(int(math.ceil(span_v / pitch_mm)) + 2, 2)
    # Keep the raster bounded; the biggest part is ~210 mm so this never bites
    # at the default 0.4 mm pitch, but a pathological input will not eat the RAM.
    if w * h > 12_000_000:
        raise MemoryError(f"{part}: raster {w}x{h} too large at pitch {pitch_mm}")

    tri_u = ((verts[idx] @ u) - u0) / pitch_mm
    tri_v = ((verts[idx] @ v) - v0) / pitch_mm
    polys = [
        np.round(np.stack([tri_u[i], tri_v[i]], axis=1)).astype(np.int32)
        for i in range(len(idx))
    ]
    mask = rasterise(polys, w, h)
    (s1, r1, c1), (s2, r2, c2) = two_disjoint_squares_px(mask)

    def to_part_coords(side: int, row: int, col: int) -> np.ndarray:
        """Centre of a raster square, back in the part's own STL frame (mm)."""
        cu = u0 + (col + side / 2.0) * pitch_mm
        cv = v0 + (row + side / 2.0) * pitch_mm
        return offset * normal + cu * u + cv * v

    c_1 = to_part_coords(s1, r1, c1)
    c_2 = to_part_coords(s2, r2, c2) if s2 > 0 else np.zeros(3)
    baseline = float(np.linalg.norm(c_1 - c_2)) if s2 > 0 else 0.0

    area = float(areas[idx].sum())
    bbox_area = float(span_u * span_v) if span_u > 0 and span_v > 0 else 0.0
    return Patch(
        part=part,
        normal=normal,
        offset=offset,
        area=area,
        n_tris=int(len(idx)),
        bbox_u=float(span_u),
        bbox_v=float(span_v),
        fill_ratio=(area / bbox_area) if bbox_area > 0 else 0.0,
        inscribed_square=s1 * pitch_mm,
        centroid=verts[idx].reshape(-1, 3).mean(axis=0),
        square_centre=c_1,
        second_square=s2 * pitch_mm,
        second_centre=c_2,
        pair_baseline=baseline,
    )


# --------------------------------------------------------------------------
# driver
# --------------------------------------------------------------------------


def survey(path: str, args: argparse.Namespace) -> list[Patch]:
    verts = read_binary_stl(path)
    normals, areas = triangle_normals_and_areas(verts)
    groups = group_planar_patches(
        verts, normals, areas, args.angle_tol, args.offset_tol, args.min_area
    )
    part = os.path.basename(path)
    out = [
        measure_patch(part, verts, areas, n, d, idx, args.pitch) for n, d, idx in groups
    ]
    out.sort(key=lambda p: -p.inscribed_square)
    return out[: args.top]


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("files", nargs="*", help="STL paths; default is the candidate set")
    ap.add_argument("--stl-dir", default=DEFAULT_STL_DIR)
    ap.add_argument("--all", action="store_true", help="survey every STL in the dir")
    ap.add_argument("--pitch", type=float, default=0.4, help="raster mm/pixel")
    ap.add_argument("--angle-tol", type=float, default=5.0, help="normal cluster deg")
    ap.add_argument("--offset-tol", type=float, default=0.4, help="coplanarity mm")
    ap.add_argument("--min-area", type=float, default=60.0, help="ignore tiny patches")
    ap.add_argument("--top", type=int, default=6, help="patches reported per part")
    ap.add_argument("--csv", help="also write the survey to this CSV path")
    args = ap.parse_args(argv)

    if args.files:
        paths = args.files
    else:
        names = sorted(os.listdir(args.stl_dir)) if args.all else CANDIDATE_PARTS
        paths = [os.path.join(args.stl_dir, n) for n in names if n.lower().endswith(".stl")]

    if not _HAVE_CV2:
        print("NOTE: cv2 not importable - using the slower numpy rasteriser.\n")

    rows: list[Patch] = []
    failures = 0
    for path in paths:
        if not os.path.exists(path):
            print(f"MISSING  {path}", file=sys.stderr)
            failures += 1
            continue
        try:
            patches = survey(path, args)
        except (NotBinarySTL, MemoryError) as exc:
            print(f"FAIL     {exc}", file=sys.stderr)
            failures += 1
            continue
        rows.extend(patches)
        print(f"\n=== {os.path.basename(path)} ===")
        print(
            f"{'normal':>26} {'offset':>8} {'area':>10} {'bbox u x v':>18} "
            f"{'fill':>6} {'sq1':>6} {'sq2':>6} {'base':>7}"
        )
        for p in patches:
            print(
                f"{p.label:>26} {p.offset:8.1f} {p.area:9.0f}  "
                f"{p.bbox_u:7.1f} x {p.bbox_v:6.1f} {p.fill_ratio:6.2f} "
                f"{p.inscribed_square:6.1f} {p.second_square:6.1f} "
                f"{p.pair_baseline:7.1f}"
            )

    if args.csv:
        with open(args.csv, "w", newline="", encoding="utf-8") as fh:
            wr = csv.writer(fh)
            wr.writerow(
                [
                    "part_file", "normal_x", "normal_y", "normal_z", "plane_offset_mm",
                    "patch_area_mm2", "n_triangles", "bbox_u_mm", "bbox_v_mm",
                    "fill_ratio", "largest_inscribed_square_mm",
                    "square1_cx_mm", "square1_cy_mm", "square1_cz_mm",
                    "second_square_mm", "square2_cx_mm", "square2_cy_mm",
                    "square2_cz_mm", "pair_baseline_mm",
                    "centroid_x_mm", "centroid_y_mm", "centroid_z_mm",
                ]
            )
            for p in rows:
                wr.writerow(
                    [
                        p.part, f"{p.normal[0]:.4f}", f"{p.normal[1]:.4f}",
                        f"{p.normal[2]:.4f}", f"{p.offset:.2f}", f"{p.area:.1f}",
                        p.n_tris, f"{p.bbox_u:.2f}", f"{p.bbox_v:.2f}",
                        f"{p.fill_ratio:.3f}", f"{p.inscribed_square:.2f}",
                        f"{p.square_centre[0]:.2f}", f"{p.square_centre[1]:.2f}",
                        f"{p.square_centre[2]:.2f}", f"{p.second_square:.2f}",
                        f"{p.second_centre[0]:.2f}", f"{p.second_centre[1]:.2f}",
                        f"{p.second_centre[2]:.2f}", f"{p.pair_baseline:.2f}",
                        f"{p.centroid[0]:.2f}", f"{p.centroid[1]:.2f}",
                        f"{p.centroid[2]:.2f}",
                    ]
                )
        print(f"\nwrote {len(rows)} patch rows to {args.csv}")

    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
