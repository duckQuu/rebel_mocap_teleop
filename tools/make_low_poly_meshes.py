#!/usr/bin/env python3
"""Generate low-poly copies of the description meshes into meshes_low/.

meshes_low/ mirrors meshes/ (same sub-folders and file names), so a URDF only
has to swap the mesh root folder to switch resolution -- see the `mesh_lod`
xacro arg in the URDFs.  Units, frames and per-part colors are preserved; only
the triangle count changes.

Usage (needs: pip install trimesh fast-simplification pycollada):
    python3 tools/make_low_poly_meshes.py               # default ratio 0.15
    python3 tools/make_low_poly_meshes.py --ratio 0.1 --min-faces 500
"""
import argparse
import os
import re
import shutil
import sys

import numpy as np
import trimesh

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(PKG, "meshes")
DST = os.path.join(PKG, "meshes_low")
# Folders referenced by the rebel2 arm, xeg32, dual_arm_rig(_v2) and suction URDFs.
FOLDERS = ["rebel_d00617809", "xeg32", "dual_arm_rig", "dual_arm_rig_v2", "suction_approx"]


def decimate(mesh, ratio, min_faces, aggression=3):
    mesh.merge_vertices()  # STL/DAE triangle soup cannot be collapsed until welded
    n = len(mesh.faces)
    target = max(min_faces, int(n * ratio))
    if n <= target:
        return mesh
    out = mesh.simplify_quadric_decimation(face_count=target, aggression=aggression)
    out.visual = trimesh.visual.ColorVisuals(
        out, face_colors=np.tile(_color(mesh), (len(out.faces), 1)))
    return out


def _color(mesh):
    v = mesh.visual
    if hasattr(v, "material") and v.material is not None:
        return np.asarray(v.material.main_color, dtype=np.uint8)
    if v.kind == "face":
        return np.asarray(v.face_colors[0], dtype=np.uint8)
    return np.array([200, 200, 200, 255], dtype=np.uint8)


def process(src, dst, ratio, min_faces, keep_below):
    ext = os.path.splitext(src)[1].lower()
    if ext == ".stl":
        mesh = trimesh.load(src, force="mesh")
        before = len(mesh.faces)
        if before < keep_below:
            shutil.copyfile(src, dst)
            return before, before
        low = decimate(mesh, ratio, min_faces)
        low.export(dst)
        return before, len(low.faces)
    if ext == ".dae":
        scene = trimesh.load(src, force="scene")
        before = sum(len(g.faces) for g in scene.geometry.values())
        if before < keep_below:
            shutil.copyfile(src, dst)
            return before, before
        out = trimesh.Scene()
        # Bake node transforms so the result is one flat, correctly placed scene.
        for node in scene.graph.nodes_geometry:
            T, gname = scene.graph[node]
            g = decimate(scene.geometry[gname].copy(), ratio, min_faces)
            g.apply_transform(T)
            out.add_geometry(g, geom_name=node)
        xml = trimesh.exchange.dae.export_collada(list(out.geometry.values())).decode()
        # trimesh writes Y_UP; the vertices are already in the source Z-up metre frame.
        xml = re.sub(r"<up_axis>.*?</up_axis>", "<up_axis>Z_UP</up_axis>", xml)
        if "<unit" not in xml:
            xml = xml.replace("<up_axis>", '<unit name="meter" meter="1"/><up_axis>', 1)
        with open(dst, "w") as f:
            f.write(xml)
        return before, sum(len(g.faces) for g in out.geometry.values())
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--ratio", type=float, default=0.15, help="kept fraction of faces")
    ap.add_argument("--min-faces", type=int, default=300, help="never go below this per part")
    ap.add_argument("--keep-below", type=int, default=20000,
                    help="files with fewer faces are copied unchanged (collision / approx meshes)")
    args = ap.parse_args()

    total_before = total_after = 0
    for folder in FOLDERS:
        for name in sorted(os.listdir(os.path.join(SRC, folder))):
            src = os.path.join(SRC, folder, name)
            dst = os.path.join(DST, folder, name)
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            res = process(src, dst, args.ratio, args.min_faces, args.keep_below)
            if res is None:
                continue
            total_before += res[0]
            total_after += res[1]
            print(f"{folder}/{name:32s} {res[0]:8d} -> {res[1]:6d} tris")
    print(f"total {total_before} -> {total_after} tris", file=sys.stderr)


if __name__ == "__main__":
    main()
