"""Self-check of rib_gmsh.py (2026-11-08), outside the application.

    python rib_gmsh_check.py [--keep DIR]

A 1 x 10 x 1 box as its 2 x 20 x 2 grid (the application's box primitive;
its vertices lie on the grid rows, so the rib planes below need no cut),
three ribs normal to y at y = -2.5, 0, 2.5 and a spar normal to x at
x = 0 crossing them. Checks: the rib area is the sections' (three 1 x 1
discs and a 10 x 1 web), the nodes written are the surface's own first
and in order, the rib rims are the surface's nodes and no interior node
is, every rib face is indexed within the nodes written, the spar and the
ribs share their chord nodes, no two rib nodes coincide, and a limited
height turns the ribs into frames. A failing job (a plane that meets no
edge of the surface) is reported as a note, not an error.
"""

import json
import os
import shutil
import struct
import sys
import tempfile
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gmsh_common as tg  # noqa: E402  (bootstraps the DLLs)
import rib_gmsh as rg  # noqa: E402
import structural_fea_check as sfc  # noqa: E402  (box_shell)

np = tg.np


def check(cond, msg):
    print(("PASS " if cond else "FAIL ") + msg)
    return bool(cond)


def read_nodes(path):
    with open(path, "rb") as f:
        assert f.read(8) == rg.NODES_MAGIC
        (nn,) = struct.unpack("<I", f.read(4))
        nodes = np.frombuffer(f.read(nn * 24), dtype="<f8").reshape(nn, 3)
    return np.array(nodes)


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
    job_dir = keep if keep else tempfile.mkdtemp(prefix="aerotelic_ribs_")
    os.makedirs(job_dir, exist_ok=True)
    ok = True
    try:
        verts, faces = sfc.box_shell(1.0, 10.0, 1.0, 2, 20, 2)
        tg.write_tri(os.path.join(job_dir, "outer.tri"), verts, faces)
        ribs = [{"id": 1, "normal": [0.0, 1.0, 0.0], "offset": -2.5, "height": 0.0},
                {"id": 1, "normal": [0.0, 1.0, 0.0], "offset": 0.0, "height": 0.0},
                {"id": 1, "normal": [0.0, 1.0, 0.0], "offset": 2.5, "height": 0.0},
                {"id": 2, "normal": [1.0, 0.0, 0.0], "offset": 0.0, "height": 0.0}]
        job = {"outer": "outer.tri", "ribs": ribs, "rib_size": 0.25, "plane_tolerance": 0.0,
               "outputs": {"nodes": "nodes.bin", "result": "result.json", "ribs": "ribs.bin"}}
        with open(os.path.join(job_dir, "job.json"), "w", encoding="utf-8") as f:
            json.dump(job, f, indent=1)
        t0 = time.time()
        code = rg.main(["rib_gmsh.py", job_dir])
        print("run: exit %d in %.2f s" % (code, time.time() - t0))
        with open(os.path.join(job_dir, "result.json"), "r", encoding="utf-8") as f:
            result = json.load(f)
        ok &= check(code == 0 and result.get("ok"), "ribs: the job succeeded: %s" % result.get("error", ""))
        if not ok:
            return 1
        nodes = read_nodes(os.path.join(job_dir, "nodes.bin"))
        rib_faces, rib_ids = read_ribs(os.path.join(job_dir, "ribs.bin"))
        ok &= check(len(rib_faces) > 0 and result["rib_faces"] == len(rib_faces), "ribs: %d rib faces written" % len(rib_faces))
        ok &= check(result["nodes"] == len(nodes) and result["surface_nodes"] == len(verts), "ribs: %d nodes written, %d of them the surface's" % (len(nodes), result["surface_nodes"]))
        ok &= check(result["rib_notes"] == [], "ribs: no notes (%s)" % "; ".join(result["rib_notes"]))
        expected_area = 3.0 * 1.0 * 1.0 + 10.0 * 1.0
        ok &= check(abs(result["rib_area"] - expected_area) < 1e-6 * expected_area, "ribs: the rib area is three discs and a web, %.4g (got %.6g)" % (expected_area, result["rib_area"]))
        # The first len(verts) nodes are the surface's, in order.
        ok &= check(len(nodes) >= len(verts) and np.allclose(nodes[:len(verts)], verts, atol=1e-12), "ribs: the first nodes are the surface's vertices in order")
        ok &= check(rib_faces.min() >= 0 and rib_faces.max() < len(nodes), "ribs: every rib face is indexed within the %d nodes" % len(nodes))
        # Every rim node of a rib (a rib node on the box's surface) is a
        # surface node; the interior ones are not.
        on_surface = np.zeros(len(nodes), dtype=bool)
        on_surface[:len(verts)] = True
        rib_nodes = np.unique(rib_faces)
        d_rim = np.abs(np.abs(nodes[rib_nodes][:, 0]) - 0.5) < 1e-9
        d_rim |= np.abs(np.abs(nodes[rib_nodes][:, 2]) - 0.5) < 1e-9
        d_rim |= np.abs(np.abs(nodes[rib_nodes][:, 1]) - 5.0) < 1e-9
        ok &= check(bool(np.all(on_surface[rib_nodes[d_rim]])), "ribs: every rim node of a rib is a surface node")
        ok &= check(bool(np.all(~on_surface[rib_nodes[~d_rim]])), "ribs: no interior rib node is a surface node")
        # Every interior node written is used by a rib face.
        ok &= check(len(np.unique(rib_faces[rib_faces >= len(verts)])) == len(nodes) - len(verts), "ribs: every interior node written belongs to a rib face")
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
        # The ribs' edges are shared: every edge of a disc belongs to two of
        # its faces or lies on the rim (or a chord).
        e = np.sort(np.concatenate([rib_faces[:, [0, 1]], rib_faces[:, [1, 2]], rib_faces[:, [2, 0]]]), axis=1)
        _, counts = np.unique(e, axis=0, return_counts=True)
        ok &= check(counts.max() <= 4, "ribs: no edge is used by more than four faces (two discs crossing at a chord)")

        # A limited height: the ribs become frames 0.3 deep along the rim.
        job3 = dict(job)
        job3["ribs"] = [dict(r, height=0.3) for r in ribs[:3]]
        job3["outputs"] = {"nodes": "nodes3.bin", "result": "result3.json", "ribs": "ribs3.bin"}
        with open(os.path.join(job_dir, "job.json"), "w", encoding="utf-8") as f:
            json.dump(job3, f, indent=1)
        code = rg.main(["rib_gmsh.py", job_dir])
        with open(os.path.join(job_dir, "result3.json"), "r", encoding="utf-8") as f:
            result3 = json.load(f)
        ok &= check(code == 0 and result3.get("ok"), "ribs (height): the job succeeded: %s" % result3.get("error", ""))
        if result3.get("ok"):
            nodes3 = read_nodes(os.path.join(job_dir, "nodes3.bin"))
            rib_faces3, _ = read_ribs(os.path.join(job_dir, "ribs3.bin"))
            # A 1 x 1 disc less its 0.4 x 0.4 centre, per rib, roughly (the
            # triangles are kept or dropped whole).
            frame = 3.0 * (1.0 - 0.4 * 0.4)
            ok &= check(abs(result3["rib_area"] - frame) < 0.25 * frame, "ribs (height): the frames are the discs less their middles (%.4g vs %.4g)" % (result3["rib_area"], frame))
            ok &= check(len(np.unique(rib_faces3[rib_faces3 >= len(verts)])) == len(nodes3) - len(verts), "ribs (height): the dropped middles' nodes are not written (%d interior nodes)" % (len(nodes3) - len(verts)))

        # A plane that meets no edge: a note, not an error.
        job4 = dict(job)
        job4["ribs"] = [{"id": 3, "normal": [0.0, 1.0, 0.0], "offset": 0.123, "height": 0.0}]
        job4["outputs"] = {"nodes": "nodes4.bin", "result": "result4.json", "ribs": "ribs4.bin"}
        with open(os.path.join(job_dir, "job.json"), "w", encoding="utf-8") as f:
            json.dump(job4, f, indent=1)
        code = rg.main(["rib_gmsh.py", job_dir])
        with open(os.path.join(job_dir, "result4.json"), "r", encoding="utf-8") as f:
            result4 = json.load(f)
        ok &= check(code == 0 and result4.get("ok") and result4["rib_faces"] == 0 and len(result4["rib_notes"]) == 1,
                    "ribs (uncut plane): no faces and one note (%s)" % "; ".join(result4.get("rib_notes", [])))
    finally:
        if keep:
            print("kept %s" % keep)
        else:
            shutil.rmtree(job_dir, ignore_errors=True)
    print("RESULT: %s" % ("all checks passed" if ok else "some checks FAILED"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
