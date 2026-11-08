"""Self-check of tet_gmsh.py's embedded ribs (2026-11-08), outside the application.

    python tet_gmsh_check.py [--keep DIR]

A 1 x 10 x 1 box as its 2 x 20 x 2 grid (the application's box primitive;
its vertices lie on the grid rows, so the rib planes below need no cut),
three ribs normal to y at y = -2.5, 0, 2.5 and a spar normal to x at
x = 0 crossing them. Checks: every rib face is a face of exactly two
tetrahedra (the embedding conformed), the rib area is the sections'
(three 1 x 1 discs and a 10 x 1 web), the rib rims are the surface's own
nodes, the spar and the ribs share their chord nodes, the tetrahedra
still fill the box, and the volume-less form (shells with ribs, no
tetrahedra) returns the same discs on the surface's nodes.
"""

import json
import os
import shutil
import struct
import sys
import tempfile
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tet_gmsh as tg  # noqa: E402  (bootstraps the DLLs)
import structural_fea_check as sfc  # noqa: E402  (box_shell)

np = tg.np


def check(cond, msg):
    print(("PASS " if cond else "FAIL ") + msg)
    return bool(cond)


def write_tri(path, verts, faces):
    with open(path, "wb") as f:
        f.write(tg.TRI_MAGIC)
        f.write(struct.pack("<II", len(verts), len(faces)))
        f.write(np.ascontiguousarray(verts, dtype="<f8").tobytes())
        f.write(np.ascontiguousarray(faces, dtype="<i4").tobytes())


def read_tet(path):
    with open(path, "rb") as f:
        assert f.read(8) == tg.TET_MAGIC
        nn, nt = struct.unpack("<II", f.read(8))
        nodes = np.frombuffer(f.read(nn * 24), dtype="<f8").reshape(nn, 3)
        tets = np.frombuffer(f.read(nt * 16), dtype="<i4").reshape(nt, 4)
        material = np.frombuffer(f.read(nt * 4), dtype="<i4")
    return np.array(nodes), np.array(tets, dtype=np.int64), np.array(material)


def read_ribs(path):
    with open(path, "rb") as f:
        (nf,) = struct.unpack("<I", f.read(4))
        faces = np.frombuffer(f.read(nf * 12), dtype="<i4").reshape(nf, 3)
        ids = np.frombuffer(f.read(nf * 4), dtype="<i4")
    return np.array(faces, dtype=np.int64), np.array(ids)


def main(argv):
    keep = None
    args = list(argv[1:])
    while args:
        a = args.pop(0)
        if a == "--keep":
            keep = args.pop(0)
    job_dir = keep if keep else tempfile.mkdtemp(prefix="aerotelic_tet_ribs_")
    os.makedirs(job_dir, exist_ok=True)
    ok = True
    try:
        verts, faces = sfc.box_shell(1.0, 10.0, 1.0, 2, 20, 2)
        write_tri(os.path.join(job_dir, "outer.tri"), verts, faces)
        ribs = [{"id": 1, "normal": [0.0, 1.0, 0.0], "offset": -2.5, "height": 0.0},
                {"id": 1, "normal": [0.0, 1.0, 0.0], "offset": 0.0, "height": 0.0},
                {"id": 1, "normal": [0.0, 1.0, 0.0], "offset": 2.5, "height": 0.0},
                {"id": 2, "normal": [1.0, 0.0, 0.0], "offset": 0.0, "height": 0.0}]
        job = {"outer": "outer.tri", "inner": "", "core_size": 0.5, "algorithm": 1, "optimize": True, "optimize_netgen": True,
               "ribs": ribs, "rib_size": 0.25, "volume": True,
               "outputs": {"mesh": "mesh.tet", "result": "result.json", "ribs": "ribs.bin"}}
        with open(os.path.join(job_dir, "job.json"), "w", encoding="utf-8") as f:
            json.dump(job, f, indent=1)
        t0 = time.time()
        code = tg.main(["tet_gmsh.py", job_dir])
        print("run: exit %d in %.2f s" % (code, time.time() - t0))
        with open(os.path.join(job_dir, "result.json"), "r", encoding="utf-8") as f:
            result = json.load(f)
        ok &= check(code == 0 and result.get("ok"), "ribs: the job succeeded: %s" % result.get("error", ""))
        if not ok:
            return 1
        nodes, tets, material = read_tet(os.path.join(job_dir, "mesh.tet"))
        rib_faces, rib_ids = read_ribs(os.path.join(job_dir, "ribs.bin"))
        ok &= check(len(rib_faces) > 0 and result["rib_faces"] == len(rib_faces), "ribs: %d rib faces written" % len(rib_faces))
        ok &= check(result["rib_faces_missing"] == 0, "ribs: every rib face is a face of exactly two tetrahedra (%d missing; %s)" % (result["rib_faces_missing"], "; ".join(result.get("rib_notes", []))))
        ok &= check(abs(result["volume_error"]) < 1e-9, "ribs: the tetrahedra still fill the box (relative %.2e)" % result["volume_error"])
        expected_area = 3.0 * 1.0 * 1.0 + 10.0 * 1.0
        ok &= check(abs(result["rib_area"] - expected_area) < 1e-6 * expected_area, "ribs: the rib area is three discs and a web, %.4g (got %.6g)" % (expected_area, result["rib_area"]))
        # The first len(verts) nodes are the surface's, in order.
        ok &= check(len(nodes) >= len(verts) and np.allclose(nodes[:len(verts)], verts, atol=1e-12), "ribs: the mesh's first nodes are the surface's vertices in order")
        # Every rib rim node (a rib face node on the surface) is a surface node;
        # the rib interior nodes are not.
        on_surface = np.zeros(len(nodes), dtype=bool)
        on_surface[:len(verts)] = True
        rib_nodes = np.unique(rib_faces)
        d_rim = np.abs(np.abs(nodes[rib_nodes][:, 0]) - 0.5) < 1e-9
        d_rim |= np.abs(np.abs(nodes[rib_nodes][:, 2]) - 0.5) < 1e-9
        d_rim |= np.abs(np.abs(nodes[rib_nodes][:, 1]) - 5.0) < 1e-9
        ok &= check(bool(np.all(on_surface[rib_nodes[d_rim]])), "ribs: every rim node of a rib is a surface node")
        ok &= check(bool(np.all(~on_surface[rib_nodes[~d_rim]])), "ribs: no interior rib node is a surface node")
        # The spar (id 2) and the ribs (id 1) share nodes along their chords
        # (x = 0, y = -2.5 / 0 / 2.5): the chord nodes appear in both.
        spar_nodes = set(np.unique(rib_faces[rib_ids == 2]).tolist())
        rib_only = set(np.unique(rib_faces[rib_ids == 1]).tolist())
        shared = spar_nodes & rib_only
        chord_nodes = [n for n in shared if abs(nodes[n][0]) < 1e-9 and min(abs(nodes[n][1] + 2.5), abs(nodes[n][1]), abs(nodes[n][1] - 2.5)) < 1e-9]
        # Each chord is 1 long at rib size 0.25: 5 nodes, two of them on the rim.
        ok &= check(len(chord_nodes) >= 3 * 5, "ribs: the spar and the ribs share their chord nodes (%d shared on the chords)" % len(chord_nodes))
        # No duplicate node positions among the rib nodes (a missed share
        # would leave two nodes at one place).
        keyed = np.round(nodes[rib_nodes] * 1e9).astype(np.int64)
        ok &= check(len(np.unique(keyed, axis=0)) == len(rib_nodes), "ribs: no two rib nodes coincide")

        # The volume-less form: discs alone on the surface's nodes.
        job2 = dict(job)
        job2["volume"] = False
        job2["outputs"] = {"mesh": "mesh2.tet", "result": "result2.json", "ribs": "ribs2.bin"}
        with open(os.path.join(job_dir, "job.json"), "w", encoding="utf-8") as f:
            json.dump(job2, f, indent=1)
        code = tg.main(["tet_gmsh.py", job_dir])
        with open(os.path.join(job_dir, "result2.json"), "r", encoding="utf-8") as f:
            result2 = json.load(f)
        ok &= check(code == 0 and result2.get("ok"), "ribs (no volume): the job succeeded: %s" % result2.get("error", ""))
        if result2.get("ok"):
            nodes2, tets2, _ = read_tet(os.path.join(job_dir, "mesh2.tet"))
            rib_faces2, _ = read_ribs(os.path.join(job_dir, "ribs2.bin"))
            ok &= check(len(tets2) == 0 and result2["tets"] == 0, "ribs (no volume): no tetrahedra")
            ok &= check(len(rib_faces2) > 0 and abs(result2["rib_area"] - expected_area) < 1e-6 * expected_area, "ribs (no volume): the same rib area (%.6g)" % result2["rib_area"])
            ok &= check(np.allclose(nodes2[:len(verts)], verts, atol=1e-12), "ribs (no volume): the surface's nodes come first")

        # A limited height: the ribs become frames 0.3 deep along the rim.
        job3 = dict(job)
        job3["ribs"] = [dict(r, height=0.3) for r in ribs[:3]]
        job3["outputs"] = {"mesh": "mesh3.tet", "result": "result3.json", "ribs": "ribs3.bin"}
        with open(os.path.join(job_dir, "job.json"), "w", encoding="utf-8") as f:
            json.dump(job3, f, indent=1)
        code = tg.main(["tet_gmsh.py", job_dir])
        with open(os.path.join(job_dir, "result3.json"), "r", encoding="utf-8") as f:
            result3 = json.load(f)
        ok &= check(code == 0 and result3.get("ok"), "ribs (height): the job succeeded: %s" % result3.get("error", ""))
        if result3.get("ok"):
            # A 1 x 1 disc less its 0.4 x 0.4 centre, per rib, roughly (the
            # triangles are kept or dropped whole).
            frame = 3.0 * (1.0 - 0.4 * 0.4)
            ok &= check(abs(result3["rib_area"] - frame) < 0.25 * frame, "ribs (height): the frames are the discs less their middles (%.4g vs %.4g)" % (result3["rib_area"], frame))
            ok &= check(result3["rib_faces_missing"] == 0, "ribs (height): the frames still conform (%d missing)" % result3["rib_faces_missing"])
    finally:
        if keep:
            print("kept %s" % keep)
        else:
            shutil.rmtree(job_dir, ignore_errors=True)
    print("RESULT: %s" % ("all checks passed" if ok else "some checks FAILED"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
