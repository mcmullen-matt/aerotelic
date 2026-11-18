"""Minimum-mass thickness sizing on TACS's adjoint gradients (2026-11-18).

    python tacs_thickness_opt.py <job directory> [--out <dir>] [--kinds rib[,skin]]
                                 [--ks-limit 1.0] [--t-min 0.0003] [--t-max 0.01]
                                 [--max-iter 40] [--ftol 1e-5] [--fd-check <dv index> [--fd-step 1e-5]]

The job directory is one the application wrote for a load case (run it
with AEROTELIC_FEA_KEEP=1 and the directory stays in %TEMP%): nodes.bin,
faces.bin, ribs.bin, pressure.bin, job.json. Every TACS component of the
chosen kinds (default: the generated layers -- ribs, spars, frames,
stringers -- which the job calls "rib" cards) is a design variable, its
shell thickness; the skins stay as drawn unless "skin" is asked for.

    minimise   structural mass m(t)
    subject to KS_c(t) <= ks_limit   for every component c (the skins too),
               t_min <= t_i <= t_max

KS_c is TACS's KSFailure aggregate (ksWeight 100) of the von Mises index
sigma_vm / allowable over the component's elements -- an upper bound on
the component's peak index, within ln(N)/100 of it -- and both m and
every KS_c come with their gradients from structural_fea_tacs.py's
adjoint (result "sensitivities"), one TACS solve and one adjoint solve
per iterate whatever the number of variables. The optimiser is SciPy's
SLSQP on the thicknesses in millimetres (so the Hessian scale is sane).

What the adjoint gradient leaves out, and why it is accepted here: the
load case is flown free, so the inertial load M(t) (g - a0 - alpha x r)
depends on the design through the mass matrix and the rigid-body
accelerations; the script hands that load to TACS as a fixed vector, so
d KS / dt lacks the term -psi^T (dM/dt) a. For the generated layers --
a fifth of this model's mass, their inertia a fraction of the aero load
-- the term is second order; --fd-check measures it on one variable
(central difference against the adjoint, all functions) so the claim is
checked, not assumed.

Writes into --out (default <job>/opt): history.json (every evaluation:
thicknesses, mass, KS per component, SLSQP's status), summary.txt, and
thicknesses.yaml -- the sized thicknesses by component label, to carry
back into the project's structure specs by hand (they are Structure-tab
fields of each body: ribs.thickness, spars[].thickness, frames,
stringers). The job directory itself is not touched: each evaluation
runs in a working copy with its own job.json and VTK.
"""

import json
import os
import shutil
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import structural_fea_tacs as st  # noqa: E402

try:
    from scipy.optimize import minimize
except ImportError:  # pragma: no cover
    minimize = None


def _log(msg):
    print(msg, flush=True)


class Sizing:
    def __init__(self, job_dir, out_dir, kinds, ks_limit, t_min, t_max):
        self.job_dir = os.path.abspath(job_dir)
        self.out_dir = os.path.abspath(out_dir)
        self.work_dir = os.path.join(self.out_dir, "work")
        self.kinds = kinds
        self.ks_limit = ks_limit
        self.t_min, self.t_max = t_min, t_max
        os.makedirs(self.out_dir, exist_ok=True)
        if os.path.isdir(self.work_dir):
            shutil.rmtree(self.work_dir)
        shutil.copytree(self.job_dir, self.work_dir, ignore=shutil.ignore_patterns("opt", "*.vtk"))
        with open(os.path.join(self.work_dir, "job.json"), "r", encoding="utf-8-sig") as f:
            self.job = json.load(f)
        # No modes, no project VTK: the solve and the adjoint alone.
        self.job["solver"] = dict(self.job.get("solver") or {}, modes=0, backend="tacs")
        self.job["outputs"] = {"vtk": os.path.join(self.work_dir, "iterate.vtk"), "result": "result.json"}
        # The design variables: the baseline run names the components in
        # TACS's order (one per region card of the kind), with their index.
        self.history = []
        self.cache = {}
        self.best = None
        self.dvs = []
        base = self.evaluate(None, label="baseline")
        self.dv_table = base["design_variables"]
        self.dvs = [d for d in self.dv_table if d["kind"] in self.kinds]
        if not self.dvs:
            raise SystemExit("no component of kind %s in this job" % ", ".join(self.kinds))
        self.t0 = np.array([d["thickness"] for d in self.dvs])
        self.components = [f["component"] for f in base["ks"]]
        _log("design variables (%d of %d components):" % (len(self.dvs), len(self.dv_table)))
        for d in self.dvs:
            _log("  dv %2d  %-48s  %.4g mm" % (d["dv"], d["component"], 1e3 * d["thickness"]))
        _log("constraints: KS <= %g on %d components" % (ks_limit, len(self.components)))

    # ---- one TACS solve + adjoint at a set of thicknesses ----
    def _set_thicknesses(self, t):
        regions = {r["id"]: r for r in self.job["regions"]}
        for d, ti in zip(self.dvs, t):
            card = regions[d["region"]]["skin" if d["kind"] == "skin" else "rib"]
            card["thickness"] = float(ti)

    def evaluate(self, t, label=None):
        key = None if t is None else tuple(np.round(np.asarray(t, dtype=np.float64), 12))
        if key in self.cache:
            return self.cache[key]
        if t is not None:
            self._set_thicknesses(t)
        with open(os.path.join(self.work_dir, "job.json"), "w", encoding="utf-8") as f:
            json.dump(self.job, f, indent=1)
        t_start = time.time()
        r = st.run_job(self.work_dir)
        sens = r.get("sensitivities") or {}
        funcs = sens.get("functions") or []
        mass = next(f for f in funcs if f["name"] == "mass")
        ks = [f for f in funcs if f["name"] != "mass"]
        out = {
            "label": label, "thicknesses": None if t is None else [float(v) for v in t], "seconds": time.time() - t_start,
            "mass": float(mass["value"]), "mass_grad": np.array(mass["gradient"], dtype=np.float64),
            "ks": [{"component": f["component"], "value": float(f["value"]), "grad": np.array(f["gradient"], dtype=np.float64)} for f in ks],
            "design_variables": sens.get("design_variables") or [],
            "margins": {e["label"]: e.get("margin") for e in r.get("per_material", [])},
            "balanced": bool(r.get("balanced")), "warnings": list(r.get("warnings") or []),
            "max_von_mises": float(r.get("max_von_mises", 0.0)),
        }
        self.cache[key] = out
        self.history.append({k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in out.items() if k not in ("ks", "design_variables")}
                            | {"ks": [{"component": e["component"], "value": e["value"]} for e in out["ks"]]})
        worst = max(out["ks"], key=lambda e: e["value"])
        # The best iterate that meets the constraints (to a tenth of a
        # percent), whatever SLSQP ends on: its last point can sit a hair
        # over the limit when the line search stalls on the gradient
        # terms the adjoint leaves out (the inertial load's).
        if t is not None and worst["value"] <= self.ks_limit * (1.0 + 1e-3) and (self.best is None or out["mass"] < self.best["mass"]):
            self.best = out
        self._flush_history()
        _log("  eval %3d %-10s mass %.4f kg  worst KS %.4f (%s)  %.1f s%s" % (
            len(self.history), label or "", out["mass"], worst["value"], worst["component"].replace("BoolOp 1 / ", ""), out["seconds"],
            "" if out["balanced"] else "  NOT BALANCED"))
        return out

    def _flush_history(self):
        # After every evaluation, so a run cut short still leaves its trail.
        with open(os.path.join(self.out_dir, "history.json"), "w", encoding="utf-8") as f:
            json.dump({"job": self.job_dir, "ks_limit": self.ks_limit, "design_variables": self.dvs, "history": self.history,
                       "best_feasible": None if self.best is None else {"mass": self.best["mass"], "thicknesses": self.best["thicknesses"]}}, f, indent=1)

    # ---- SLSQP in millimetres ----
    def _dv_cols(self):
        return np.array([d["dv"] for d in self.dvs], dtype=int)

    def objective(self, x_mm):
        e = self.evaluate(x_mm * 1e-3)
        return e["mass"]

    def objective_grad(self, x_mm):
        e = self.evaluate(x_mm * 1e-3)
        return e["mass_grad"][self._dv_cols()] * 1e-3

    def constraints(self, x_mm):
        e = self.evaluate(x_mm * 1e-3)
        return np.array([self.ks_limit - k["value"] for k in e["ks"]])

    def constraints_jac(self, x_mm):
        e = self.evaluate(x_mm * 1e-3)
        cols = self._dv_cols()
        return np.array([-k["grad"][cols] * 1e-3 for k in e["ks"]])

    def run(self, max_iter, ftol):
        if minimize is None:
            raise SystemExit("scipy is needed: micromamba install -n tacs scipy")
        x0 = self.t0 * 1e3
        bounds = [(self.t_min * 1e3, self.t_max * 1e3)] * len(x0)
        _log("SLSQP from %s mm" % ", ".join("%.3g" % v for v in x0))
        res = minimize(self.objective, x0, jac=self.objective_grad, method="SLSQP", bounds=bounds,
                       constraints=[{"type": "ineq", "fun": self.constraints, "jac": self.constraints_jac}],
                       options={"maxiter": max_iter, "ftol": ftol, "disp": True})
        final = self.evaluate(res.x * 1e-3, label="final")
        worst = max(final["ks"], key=lambda e: e["value"])["value"]
        if self.best is not None and (worst > self.ks_limit * (1.0 + 1e-3) or self.best["mass"] < final["mass"]):
            _log("SLSQP's last point has worst KS %.4f at %.4f kg; reporting the best feasible iterate instead (%.4f kg)" % (worst, final["mass"], self.best["mass"]))
            final = self.evaluate(np.array(self.best["thicknesses"]), label="best feasible")
        return res, final

    # ---- the adjoint against a central difference on one variable ----
    def fd_check(self, dv_index, step):
        base = self.evaluate(self.t0, label="fd base")
        k = [i for i, d in enumerate(self.dvs) if d["dv"] == dv_index]
        if not k:
            raise SystemExit("dv %d is not a design variable here" % dv_index)
        k = k[0]
        tp, tm = self.t0.copy(), self.t0.copy()
        tp[k] += step
        tm[k] -= step
        p = self.evaluate(tp, label="fd +")
        m = self.evaluate(tm, label="fd -")
        lines = ["finite difference on dv %d (%s), step %.3g m:" % (dv_index, self.dvs[k]["component"], step),
                 "  %-50s %14s %14s %9s" % ("function", "adjoint", "central FD", "ratio")]
        fd_mass = (p["mass"] - m["mass"]) / (2 * step)
        lines.append("  %-50s %14.6g %14.6g %9.4f" % ("mass", base["mass_grad"][dv_index], fd_mass, base["mass_grad"][dv_index] / fd_mass if fd_mass else float("nan")))
        for bk, pk, mk in zip(base["ks"], p["ks"], m["ks"]):
            fd = (pk["value"] - mk["value"]) / (2 * step)
            adj = bk["grad"][dv_index]
            if abs(fd) < 1e-9 and abs(adj) < 1e-9:
                continue
            lines.append("  %-50s %14.6g %14.6g %9.4f" % ("KS " + bk["component"].replace("BoolOp 1 / ", ""), adj, fd, adj / fd if fd else float("nan")))
        text = "\n".join(lines)
        _log(text)
        with open(os.path.join(self.out_dir, "fd_check_dv%d.txt" % dv_index), "w", encoding="utf-8") as f:
            f.write(text + "\n")

    # ---- reports ----
    def write(self, res, final):
        base = self.history[0]
        lines = ["TACS thickness sizing on %s" % self.job_dir,
                 "SLSQP: %s (status %d, %d iterations, %d evaluations)" % (res.message, res.status, res.nit, len(self.history)),
                 "mass %.4f kg -> %.4f kg (%+.2f%%); KS limit %g" % (base["mass"], final["mass"], 100 * (final["mass"] / base["mass"] - 1), self.ks_limit),
                 "", "%-50s %10s %10s" % ("design variable", "from mm", "to mm")]
        for d, t_from, t_to in zip(self.dvs, self.t0, final["thicknesses"]):
            lines.append("%-50s %10.3f %10.3f" % (d["component"].replace("BoolOp 1 / ", ""), 1e3 * t_from, 1e3 * t_to))
        lines += ["", "%-50s %10s %10s %10s" % ("component", "KS from", "KS to", "margin to")]
        for k0, k1 in zip(self.history[0]["ks"], final["ks"]):
            margin = final["margins"].get(k1["component"])
            lines.append("%-50s %10.4f %10.4f %10s" % (k1["component"].replace("BoolOp 1 / ", ""), k0["value"], k1["value"], "" if margin is None else "%+.3f" % margin))
        if final["warnings"]:
            lines += ["", "warnings at the optimum: " + "; ".join(final["warnings"])]
        text = "\n".join(lines)
        _log(text)
        with open(os.path.join(self.out_dir, "summary.txt"), "w", encoding="utf-8") as f:
            f.write(text + "\n")
        self._flush_history()
        with open(os.path.join(self.out_dir, "history.json"), "r+", encoding="utf-8") as f:
            data = json.load(f)
            data["slsqp"] = {"status": int(res.status), "message": str(res.message), "nit": int(res.nit)}
            data["final"] = {"mass": final["mass"], "thicknesses": final["thicknesses"], "label": final["label"]}
            f.seek(0)
            json.dump(data, f, indent=1)
            f.truncate()
        with open(os.path.join(self.out_dir, "thicknesses.yaml"), "w", encoding="utf-8") as f:
            f.write("# sized thicknesses (m) by component, from tacs_thickness_opt.py\n")
            for d, t_to in zip(self.dvs, final["thicknesses"]):
                f.write("%s: %.6g\n" % (json.dumps(d["component"]), t_to))


def main(argv):
    args = list(argv[1:])
    if not args:
        print(__doc__)
        return 2
    job_dir = args.pop(0)
    out_dir, kinds, ks_limit, t_min, t_max, max_iter, fd, fd_step, ftol = None, ["rib"], 1.0, 0.0003, 0.01, 40, None, 1e-5, 1e-5
    while args:
        a = args.pop(0)
        if a == "--out":
            out_dir = args.pop(0)
        elif a == "--kinds":
            kinds = args.pop(0).split(",")
        elif a == "--ks-limit":
            ks_limit = float(args.pop(0))
        elif a == "--t-min":
            t_min = float(args.pop(0))
        elif a == "--t-max":
            t_max = float(args.pop(0))
        elif a == "--max-iter":
            max_iter = int(args.pop(0))
        elif a == "--fd-check":
            fd = int(args.pop(0))
        elif a == "--fd-step":
            fd_step = float(args.pop(0))
        elif a == "--ftol":
            ftol = float(args.pop(0))
        else:
            raise SystemExit("unknown option %s" % a)
    s = Sizing(job_dir, out_dir or os.path.join(job_dir, "opt"), kinds, ks_limit, t_min, t_max)
    if fd is not None:
        s.fd_check(fd, fd_step)
        return 0
    res, final = s.run(max_iter, ftol)
    s.write(res, final)
    return 0 if res.success else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
