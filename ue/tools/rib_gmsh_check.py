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

Reserved volumes: a 0.5 x 4 x 0.5 box bay inside the body (its grid rows
on the rib planes, so no cut is needed either) with the rib at y = 0 and
the spar at x = 0 through both: the rib area is the sections less the
bay's, no rib face centroid lies in the bay's section, the bay's rim nodes
are written after the surface's and the spar and rib share them. Then the
same bay against a rib it misses (no hole, the full disc), a bay wider
than the body (crosses the rim: a note, the full disc) and a bay larger
than the body (encloses the disc: a note, no faces).

Sections in several pieces (2026-11-09, the port boom that lost its
frames): a U-shaped prism (``cells_prism``: the 3 x 3 unit cells less the
middle column's upper two, 1 high) with a rib normal to y at y = 2 that
cuts the two legs separately -- two 1 x 1 discs, BOTH meshed -- and a spar
normal to z at z = 0.5 (one U-shaped disc of area 7) crossing both legs,
sharing a chord with each. Then a ring (the 3 x 3 cells less the middle
one) with the spar alone: one disc with a real hole, area 8, no face in
the hole.
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


def cells_prism(keep, nz, dz=1.0):
    """The closed, outward surface of the prism over the unit cells `keep`
    (a set of (i, j) column/row indices, cell (i, j) = [i, i+1] x [j, j+1])
    extruded in z through nz layers of height dz. Grid vertices only, so a
    plane on a grid line needs no cut."""
    ids, verts, faces = {}, [], []

    def vid(i, j, k):
        key = (i, j, k)
        if key not in ids:
            ids[key] = len(verts)
            verts.append((float(i), float(j), k * dz))
        return ids[key]

    for (i, j) in keep:
        # The bottom cap faces -z, the top +z.
        faces.append((vid(i, j, 0), vid(i, j + 1, 0), vid(i + 1, j + 1, 0)))
        faces.append((vid(i, j, 0), vid(i + 1, j + 1, 0), vid(i + 1, j, 0)))
        faces.append((vid(i, j, nz), vid(i + 1, j, nz), vid(i + 1, j + 1, nz)))
        faces.append((vid(i, j, nz), vid(i + 1, j + 1, nz), vid(i, j + 1, nz)))
        # The sides: each cell edge, taken counter-clockwise around the
        # cell, that no kept neighbour shares; (edge x z) points outward.
        for a, b, nb in (((i, j), (i + 1, j), (i, j - 1)), ((i + 1, j), (i + 1, j + 1), (i + 1, j)),
                         ((i + 1, j + 1), (i, j + 1), (i, j + 1)), ((i, j + 1), (i, j), (i - 1, j))):
            if nb in keep:
                continue
            for k in range(nz):
                a0, b0, b1, a1 = vid(a[0], a[1], k), vid(b[0], b[1], k), vid(b[0], b[1], k + 1), vid(a[0], a[1], k + 1)
                faces.append((a0, b0, b1))
                faces.append((a0, b1, a1))
    return np.array(verts, dtype=np.float64), np.array(faces, dtype=np.int32)


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

        # A reserved volume: a 0.5 x 4 x 0.5 bay in the middle of the body
        # (2 x 8 x 2 cells: its vertices lie on y = 0 and x = 0, so the
        # planes below need no cut), the rib at y = 0 and the spar at x = 0
        # through both.
        def run_holes(tag, bay_dims, bay_cells, rib_list):
            bv, bf = sfc.box_shell(*bay_dims, *bay_cells)
            tg.write_tri(os.path.join(job_dir, "hole_%s.tri" % tag), bv, bf)
            jobh = dict(job)
            jobh["holes"] = ["hole_%s.tri" % tag]
            jobh["ribs"] = rib_list
            jobh["outputs"] = {"nodes": "nodes_%s.bin" % tag, "result": "result_%s.json" % tag, "ribs": "ribs_%s.bin" % tag}
            with open(os.path.join(job_dir, "job.json"), "w", encoding="utf-8") as f:
                json.dump(jobh, f, indent=1)
            code = rg.main(["rib_gmsh.py", job_dir])
            with open(os.path.join(job_dir, "result_%s.json" % tag), "r", encoding="utf-8") as f:
                res = json.load(f)
            return code, res

        bay = (0.5, 4.0, 0.5)
        code, result5 = run_holes("bay", bay, (2, 8, 2),
                                  [{"id": 1, "normal": [0.0, 1.0, 0.0], "offset": 0.0, "height": 0.0},
                                   {"id": 2, "normal": [1.0, 0.0, 0.0], "offset": 0.0, "height": 0.0}])
        ok &= check(code == 0 and result5.get("ok"), "ribs (bay): the job succeeded: %s" % result5.get("error", ""))
        if result5.get("ok"):
            nodes5 = read_nodes(os.path.join(job_dir, "nodes_bay.bin"))
            rib_faces5, rib_ids5 = read_ribs(os.path.join(job_dir, "ribs_bay.bin"))
            ok &= check(result5["holes"] == 2 and result5["rib_notes"] == [], "ribs (bay): two hole loops used (one per disc), no notes (%s)" % "; ".join(result5["rib_notes"]))
            # The rib: 1 x 1 less the bay's 0.5 x 0.5; the spar: 10 x 1 less 4 x 0.5.
            rib_expected = 1.0 - 0.5 * 0.5
            spar_expected = 10.0 - 4.0 * 0.5
            rib_got = tg.surface_area(nodes5, rib_faces5[rib_ids5 == 1])
            spar_got = tg.surface_area(nodes5, rib_faces5[rib_ids5 == 2])
            ok &= check(abs(rib_got - rib_expected) < 0.01 * rib_expected, "ribs (bay): the rib's area is the section less the bay's, %.4g (got %.6g)" % (rib_expected, rib_got))
            ok &= check(abs(spar_got - spar_expected) < 0.01 * spar_expected, "ribs (bay): the spar's area is the web less the bay's, %.4g (got %.6g)" % (spar_expected, spar_got))
            ok &= check(abs(result5["rib_area"] - rib_expected - spar_expected) < 0.01 * (rib_expected + spar_expected), "ribs (bay): result rib_area agrees (%.6g)" % result5["rib_area"])
            # No face centroid inside the bay's section (|x| < 0.25, |y| < 2, |z| < 0.25).
            cent = nodes5[rib_faces5].mean(axis=1)
            in_bay = (np.abs(cent[:, 0]) < 0.25 - 1e-9) & (np.abs(cent[:, 1]) < 2.0 - 1e-9) & (np.abs(cent[:, 2]) < 0.25 - 1e-9)
            ok &= check(not in_bay.any(), "ribs (bay): no rib face centroid lies in the bay's section (%d do)" % int(in_bay.sum()))
            # The bay's rim nodes (on the bay's surface) are written after the
            # surface's nodes, and none of them is a surface node.
            rib_nodes5 = np.unique(rib_faces5)
            p = nodes5[rib_nodes5]
            on_bay = (np.abs(np.abs(p[:, 0]) - 0.25) < 1e-9) & (np.abs(p[:, 2]) <= 0.25 + 1e-9) & (np.abs(p[:, 1]) <= 2.0 + 1e-9)
            on_bay |= (np.abs(np.abs(p[:, 2]) - 0.25) < 1e-9) & (np.abs(p[:, 0]) <= 0.25 + 1e-9) & (np.abs(p[:, 1]) <= 2.0 + 1e-9)
            on_bay |= (np.abs(np.abs(p[:, 1]) - 2.0) < 1e-9) & (np.abs(p[:, 0]) <= 0.25 + 1e-9) & (np.abs(p[:, 2]) <= 0.25 + 1e-9)
            ok &= check(int(on_bay.sum()) > 0 and bool(np.all(rib_nodes5[on_bay] >= len(verts))), "ribs (bay): the %d bay rim nodes come after the %d surface nodes" % (int(on_bay.sum()), len(verts)))
            ok &= check(len(nodes5) >= len(verts) and np.allclose(nodes5[:len(verts)], verts, atol=1e-12), "ribs (bay): the first nodes are still the surface's vertices in order")
            ok &= check(len(np.unique(rib_faces5[rib_faces5 >= len(verts)])) == len(nodes5) - len(verts), "ribs (bay): every interior node written belongs to a rib face")
            # The spar and the rib share the bay's rim vertices on both planes
            # ((0, 0, +-0.25)) and their chord nodes outside the bay.
            spar_nodes = set(np.unique(rib_faces5[rib_ids5 == 2]).tolist())
            rib_only = set(np.unique(rib_faces5[rib_ids5 == 1]).tolist())
            shared5 = [n for n in spar_nodes & rib_only if abs(nodes5[n][0]) < 1e-9 and abs(nodes5[n][1]) < 1e-9]
            corner = [n for n in shared5 if abs(abs(nodes5[n][2]) - 0.25) < 1e-9]
            ok &= check(len(corner) == 2 and len(shared5) >= 4, "ribs (bay): the spar and the rib share the bay's rim vertices on their chord (%d shared, %d on the bay)" % (len(shared5), len(corner)))
            keyed = np.round(nodes5[rib_nodes5] * 1e9).astype(np.int64)
            ok &= check(len(np.unique(keyed, axis=0)) == len(rib_nodes5), "ribs (bay): no two rib nodes coincide")

        # The bay misses the plane (the rib at y = -2.5 is clear of y in [-2, 2]).
        code, result6 = run_holes("miss", bay, (2, 8, 2), [{"id": 1, "normal": [0.0, 1.0, 0.0], "offset": -2.5, "height": 0.0}])
        ok &= check(code == 0 and result6.get("ok") and result6["holes"] == 0 and result6["rib_notes"] == [] and abs(result6["rib_area"] - 1.0) < 1e-6,
                    "ribs (bay missed): no hole, no note, the full disc (area %.6g, %d holes)" % (result6.get("rib_area", -1.0), result6.get("holes", -1)))

        # The bay crosses the body's boundary (2 wide against a 1 wide body): a note, the full disc.
        code, result7 = run_holes("cross", (2.0, 4.0, 0.5), (2, 8, 2), [{"id": 1, "normal": [0.0, 1.0, 0.0], "offset": 0.0, "height": 0.0}])
        ok &= check(code == 0 and result7.get("ok") and result7["holes"] == 0 and len(result7["rib_notes"]) == 1 and "crosses the rib's boundary" in result7["rib_notes"][0]
                    and abs(result7["rib_area"] - 1.0) < 1e-6,
                    "ribs (bay crossing): no hole, one note, the full disc (area %.6g; %s)" % (result7.get("rib_area", -1.0), "; ".join(result7.get("rib_notes", []))))

        # The bay encloses the whole disc: a note, no faces.
        code, result8 = run_holes("enclose", (3.0, 4.0, 3.0), (2, 8, 2), [{"id": 1, "normal": [0.0, 1.0, 0.0], "offset": 0.0, "height": 0.0}])
        ok &= check(code == 0 and result8.get("ok") and result8["holes"] == 0 and result8["rib_faces"] == 0 and len(result8["rib_notes"]) == 1,
                    "ribs (bay enclosing): no faces, one note (%s)" % "; ".join(result8.get("rib_notes", [])))

        # A section in two pieces: the U prism, the rib at y = 2 through
        # both legs, the spar at z = 0.5 through the whole U.
        def run_prism(tag, keep, rib_list):
            pv, pf = cells_prism(keep, 2, 0.5)
            tg.check_closed(pv, pf, "the %s prism" % tag)
            tg.write_tri(os.path.join(job_dir, "prism_%s.tri" % tag), pv, pf)
            jobp = {"outer": "prism_%s.tri" % tag, "ribs": rib_list, "rib_size": 0.25, "plane_tolerance": 0.0,
                    "outputs": {"nodes": "nodes_%s.bin" % tag, "result": "result_%s.json" % tag, "ribs": "ribs_%s.bin" % tag}}
            with open(os.path.join(job_dir, "job.json"), "w", encoding="utf-8") as f:
                json.dump(jobp, f, indent=1)
            code = rg.main(["rib_gmsh.py", job_dir])
            with open(os.path.join(job_dir, "result_%s.json" % tag), "r", encoding="utf-8") as f:
                res = json.load(f)
            return code, res, pv

        u_cells = {(i, j) for i in range(3) for j in range(3)} - {(1, 1), (1, 2)}
        code, result9, uv = run_prism("u", u_cells, [{"id": 1, "normal": [0.0, 1.0, 0.0], "offset": 2.0, "height": 0.0},
                                                      {"id": 2, "normal": [0.0, 0.0, 1.0], "offset": 0.5, "height": 0.0}])
        ok &= check(code == 0 and result9.get("ok"), "ribs (two pieces): the job succeeded: %s" % result9.get("error", ""))
        if result9.get("ok"):
            nodes9 = read_nodes(os.path.join(job_dir, "nodes_u.bin"))
            rib_faces9, rib_ids9 = read_ribs(os.path.join(job_dir, "ribs_u.bin"))
            ok &= check(result9["rib_notes"] == [], "ribs (two pieces): no notes (%s)" % "; ".join(result9["rib_notes"]))
            rib_got = tg.surface_area(nodes9, rib_faces9[rib_ids9 == 1])
            spar_got = tg.surface_area(nodes9, rib_faces9[rib_ids9 == 2])
            ok &= check(abs(rib_got - 2.0) < 1e-6, "ribs (two pieces): the rib is BOTH legs' discs, area 2 (got %.6g)" % rib_got)
            ok &= check(abs(spar_got - 7.0) < 1e-6, "ribs (two pieces): the spar is the U's section, area 7 (got %.6g)" % spar_got)
            cent = nodes9[rib_faces9[rib_ids9 == 1]].mean(axis=1)
            left, right = int((cent[:, 0] < 1.0).sum()), int((cent[:, 0] > 2.0).sum())
            ok &= check(left > 0 and right > 0 and len(cent) == left + right, "ribs (two pieces): rib faces in both legs (%d left, %d right)" % (left, right))
            # The spar and the rib share the chord nodes in each leg (y = 2,
            # z = 0.5; each chord 1 long at size 0.25: 5 nodes).
            spar_nodes = set(np.unique(rib_faces9[rib_ids9 == 2]).tolist())
            rib_nodes9 = set(np.unique(rib_faces9[rib_ids9 == 1]).tolist())
            on_chords = [n for n in spar_nodes & rib_nodes9 if abs(nodes9[n][1] - 2.0) < 1e-9 and abs(nodes9[n][2] - 0.5) < 1e-9]
            in_left = [n for n in on_chords if nodes9[n][0] < 1.0 + 1e-9]
            in_right = [n for n in on_chords if nodes9[n][0] > 2.0 - 1e-9]
            ok &= check(len(in_left) >= 5 and len(in_right) >= 5 and len(on_chords) == len(in_left) + len(in_right),
                        "ribs (two pieces): the spar shares a chord with each leg's disc (%d + %d nodes)" % (len(in_left), len(in_right)))
            all_rib_nodes = np.unique(rib_faces9)
            keyed = np.round(nodes9[all_rib_nodes] * 1e9).astype(np.int64)
            ok &= check(len(np.unique(keyed, axis=0)) == len(all_rib_nodes), "ribs (two pieces): no two rib nodes coincide")
            ok &= check(len(np.unique(rib_faces9[rib_faces9 >= len(uv)])) == len(nodes9) - len(uv), "ribs (two pieces): every interior node written belongs to a rib face")

        # A section with a real hole: the ring prism and the spar alone.
        ring_cells = {(i, j) for i in range(3) for j in range(3)} - {(1, 1)}
        code, result10, rv = run_prism("ring", ring_cells, [{"id": 1, "normal": [0.0, 0.0, 1.0], "offset": 0.5, "height": 0.0}])
        ok &= check(code == 0 and result10.get("ok") and result10["rib_notes"] == [], "ribs (ring): the job succeeded without notes: %s %s" % (result10.get("error", ""), "; ".join(result10.get("rib_notes", []))))
        if result10.get("ok"):
            nodes10 = read_nodes(os.path.join(job_dir, "nodes_ring.bin"))
            rib_faces10, _ = read_ribs(os.path.join(job_dir, "ribs_ring.bin"))
            ok &= check(abs(result10["rib_area"] - 8.0) < 1e-6, "ribs (ring): one disc with a hole, area 8 (got %.6g)" % result10["rib_area"])
            cent = nodes10[rib_faces10].mean(axis=1)
            in_hole = (np.abs(cent[:, 0] - 1.5) < 0.5 - 1e-9) & (np.abs(cent[:, 1] - 1.5) < 0.5 - 1e-9)
            ok &= check(not in_hole.any(), "ribs (ring): no rib face in the hole (%d do)" % int(in_hole.sum()))
    finally:
        if keep:
            print("kept %s" % keep)
        else:
            shutil.rmtree(job_dir, ignore_errors=True)
    print("RESULT: %s" % ("all checks passed" if ok else "some checks FAILED"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
