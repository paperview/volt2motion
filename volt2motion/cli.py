"""Simulate a robot joint from battery voltage to arm motion.

  volt2motion init [folder]               starter design, parts table and sample board
  volt2motion design [design.json]        phase 1: check a design in simulation
  volt2motion import-kicad <board>        fill in driver values from a KiCad board
  volt2motion calibrate [design.json]     phase 2: fit the model to logs from the machine
  volt2motion trajectory <kind>           export a trajectory for the robot's firmware to play
  volt2motion params [filter]             list every design parameter with its unit and meaning

Every command writes its results to ./out unless you pass --out.
Run "volt2motion <command> --help" for the options of each one.
"""
from __future__ import annotations

import argparse
import json
import math
import shutil
import sys
from importlib.metadata import PackageNotFoundError, version
from pathlib import Path

# Heavy imports (MuJoCo, scipy, sklearn, matplotlib) happen inside each command,
# so "volt2motion --help" stays fast.


def _version() -> str:
    try:
        return version("volt2motion")
    except PackageNotFoundError:
        return "unknown (not installed)"


def _load_design(path: str | None):
    from .params import JointDesign

    if path is None:
        print("Using the built-in baseline design (run 'volt2motion init' for an editable copy).")
        return JointDesign()
    p = Path(path)
    if not p.exists():
        raise SystemExit(f"error: design file not found: {p}")
    try:
        return JointDesign.load(p)
    except (ValueError, TypeError) as e:
        raise SystemExit(f"error: {e}")


def _apply_overrides(design, items: list[str]):
    if not items:
        return design
    try:
        return design.updated(_parse_overrides(items))
    except KeyError as e:
        raise SystemExit(f"error: {e.args[0]}")


def _out_dir(path: str) -> Path:
    out = Path(path)
    out.mkdir(parents=True, exist_ok=True)
    return out


def _parse_overrides(items: list[str]) -> dict:
    changes = {}
    for item in items:
        key, sep, raw = item.partition("=")
        if not sep:
            raise SystemExit(f"error: --set needs KEY=VALUE, got '{item}'")
        try:
            changes[key] = json.loads(raw)
        except json.JSONDecodeError:
            changes[key] = raw
    return changes


def _row(name: str, *vals: str) -> None:
    print(f"  {name:<34}" + "".join(f"{v:>14}" for v in vals))


# -- init ------------------------------------------------------------------------------

def cmd_init(args: argparse.Namespace) -> None:
    from . import sample_board
    from .kicad_import import EXAMPLE_PARTS
    from .links import DOCS_URL
    from .params import JointDesign

    folder = Path(args.folder)
    folder.mkdir(parents=True, exist_ok=True)

    def place(path: Path, write) -> None:
        if path.exists() and not args.force:
            print(f"  kept     {path} (already there; --force overwrites)")
            return
        write(path)
        print(f"  wrote    {path}")

    print(f"Setting up {folder.resolve()}")
    place(folder / "baseline.json", lambda p: JointDesign().save(p))
    place(folder / "parts.json", lambda p: shutil.copyfile(EXAMPLE_PARTS, p))
    place(folder / "sample_driver", lambda p: sample_board.write(p))
    where = "" if folder.resolve() == Path.cwd() else f"  cd {args.folder}\n"
    print(f"""
Next:
{where}  volt2motion design baseline.json                check the baseline joint
  volt2motion import-kicad sample_driver --run    import the sample board and check it
  volt2motion calibrate --virtual                 try phase 2 on a simulated machine

baseline.json holds every parameter. Edit it, or try changes with --set.
parts.json holds made-up example MOSFETs. Add your real parts from their datasheets.
Guides: {DOCS_URL}""")


# -- design ----------------------------------------------------------------------------

def cmd_design(args: argparse.Namespace) -> None:
    design = _apply_overrides(_load_design(args.design), args.set)
    run_design_check(design, _out_dir(args.out), args.cycles, args.move_time, args.shift_minutes)


def run_design_check(design, out: Path, cycles: int = 4, move_time: float = 0.9, shift_minutes: float = 20.0) -> None:
    from .plots import design_plot
    from .sim import print_summary, simulate, summarize
    from .trajectory import pick_and_place

    cycle = pick_and_place(move_time=move_time, cycles=cycles)
    log = simulate(design, cycle, fidelity="full")
    print_summary(summarize(log, design), f"Pick-and-place, full detail: {design.name}")
    design_plot(log, design, out / f"{design.name}_cycle.png", f"{design.name}: pick-and-place, full detail")

    if shift_minutes > 0:
        n = max(1, int(shift_minutes * 60 / (cycle.duration / cycles)))
        shift = pick_and_place(move_time=move_time, cycles=n)
        print(f"\nSimulating a {shift_minutes:.0f} minute shift ({n} cycles) at fast detail...")
        slog = simulate(design, shift, fidelity="fast", log_every=10, verbose=True)
        print_summary(summarize(slog, design), f"Shift, fast detail: {design.name}")
        design_plot(slog, design, out / f"{design.name}_shift.png", f"{design.name}: {shift_minutes:.0f} min shift")
    print(f"\nPlots saved in {out}")


# -- import-kicad ----------------------------------------------------------------------

def cmd_import_kicad(args: argparse.Namespace) -> None:
    from .kicad_import import EXAMPLE_PARTS, analyze, load_parts_table, load_project, print_report

    board_path = Path(args.board)
    if not board_path.exists():
        raise SystemExit(f"error: not found: {board_path}")
    if args.parts:
        parts_path = Path(args.parts)
    elif Path("parts.json").exists():
        parts_path = Path("parts.json")
    else:
        parts_path = EXAMPLE_PARTS
        print("No parts.json here, so using the bundled example parts table (made-up parts).")
    if not parts_path.exists():
        raise SystemExit(f"error: parts table not found: {parts_path}")

    base = _load_design(args.base)
    name = args.name or (board_path.stem if board_path.is_file() else board_path.resolve().name)
    try:
        board = load_project(board_path)
    except (FileNotFoundError, ValueError) as e:
        raise SystemExit(f"error: {e}")
    try:
        parts = load_parts_table(parts_path)
    except ValueError as e:
        raise SystemExit(f"error: {e}")
    design, report = analyze(board, parts, base, args.convection, name=name)
    print_report(report)

    out = _out_dir(args.out)
    path = out / f"{name}.json"
    design.save(path)
    print(f"\nWrote {path}")
    if args.run:
        print()
        run_design_check(design, out)
    else:
        print(f"Next: volt2motion design {path}")


# -- calibrate -------------------------------------------------------------------------

def cmd_calibrate(args: argparse.Namespace) -> None:
    if args.log and not args.validation_log:
        args.parser.error("--validation-log is required with --log")

    from . import calibrate as C
    from . import virtual_hardware as V
    from .logio import load_csv, save_csv
    from .plots import calibration_plot
    from .sim import simulate
    from .trajectory import SampledTrajectory, calibration_routine, pick_and_place

    design = _load_design(args.design)
    out = _out_dir(args.out)

    # 1. Get logs
    if args.virtual:
        print("Recording logs from the virtual machine (synthetic, not real hardware)...")
        validation_traj = pick_and_place(cycles=4)
        cal_log = V.record(design, calibration_routine(args.cal_seconds), seed=100)
        val_log = V.record(design, validation_traj, seed=200)
        save_csv(cal_log, out / "virtual_calibration_log.csv")
        save_csv(val_log, out / "virtual_validation_log.csv")
        truth = V.hidden_truth(design)
    else:
        try:
            cal_log, val_log = load_csv(args.log), load_csv(args.validation_log)
        except (FileNotFoundError, KeyError, ValueError) as e:
            raise SystemExit(f"error reading logs: {e}")
        # Replay whatever job the validation run did, from its logged setpoints.
        validation_traj = SampledTrajectory.from_log(val_log)
        truth = None

    # 2. Physics fit
    elec = C.fit_electrical(cal_log, design)
    mech = C.fit_mechanical(cal_log, design, elec)
    problems = C.fit_problems(elec, mech)
    if problems:
        print("\nThe fit came out unphysical, which usually means a sign or unit mistake in the log:")
        for line in problems:
            print(f"  - {line}")
        from .links import DOCS_URL

        raise SystemExit(f"error: nothing was saved. Log conventions: {DOCS_URL}/logging.md#conventions")
    calibrated = C.apply_calibration(design, elec, mech)
    calibrated.save(out / f"{calibrated.name}.json")

    print("\nStep 1: physics fit")
    _row("parameter", *(["design", "fitted"] + (["hidden truth"] if truth else [])))
    params = [
        ("winding resistance @20C (mOhm)", "motor.resistance", 1e3, "{:.1f}"),
        ("magnet flux linkage @20C (mWb)", "motor.flux_linkage", 1e3, "{:.3f}"),
        ("inductance (uH)", "motor.lq", 1e6, "{:.0f}"),
        ("gearbox Coulomb friction (Nm)", "gearbox.coulomb", 1, "{:.2f}"),
        ("gearbox viscous (Nm s/rad)", "gearbox.viscous", 1, "{:.2f}"),
        ("gearbox efficiency", "gearbox.efficiency", 1, "{:.3f}"),
    ]
    for label, key, scale, fmt in params:
        sec, leaf = key.split(".")
        vals = [getattr(getattr(d, sec), leaf) * scale for d in (design, calibrated) + ((truth,) if truth else ())]
        _row(label, *(fmt.format(v) for v in vals))
    gap = abs(mech.kt_free_fit / mech.kt_electrical - 1)
    print(f"  Torque constant check: {mech.kt_electrical:.4f} Nm/A from back-EMF, "
          f"{mech.kt_free_fit:.4f} Nm/A from motion alone ({100 * gap:.1f}% apart).")
    if gap > 0.08:
        print("  Over 8% apart. Usual causes: link or payload masses that do not match the real arm,")
        print("  the current sensor gain, or dq scaling (about 37% off if the firmware logs")
        print("  power-invariant currents and voltages, 22% if only the currents).")
    if truth:
        print("  Coulomb friction comes out above the hidden value because the fit also absorbs")
        print("  breakaway and position-dependent friction, which the physics model leaves out.")

    # 3. Learned residual
    print("\nStep 2: learned residual on top of calibrated physics")
    residual = C.ResidualModel().fit([cal_log], calibrated)
    residual.save(out / "residual_model.npz")
    _row("model", "torque error on held-out run (Nm rms)")
    for k, v in {
        "design (phase 1)": C.torque_prediction_rmse(val_log, design),
        "calibrated physics": C.torque_prediction_rmse(val_log, calibrated),
        "calibrated physics + residual": C.torque_prediction_rmse(val_log, calibrated, residual),
    }.items():
        _row(k, f"{v:.2f}")

    # 4. Does the simulator now reproduce the machine?
    print("\nStep 3: simulate the held-out run with each model and compare to the log")
    sims = {
        "design model": simulate(design, validation_traj, firmware=design, seed=1),
        "calibrated": simulate(calibrated, validation_traj, firmware=design, seed=1),
        "calibrated + residual": simulate(calibrated, validation_traj, firmware=design, residual=residual, seed=1),
    }
    _row("simulation", "tracking diff", "current diff")
    _row("", "(deg rms)", "(A rms)")
    for name, log in sims.items():
        cmp = C.compare_traces(log, val_log)
        _row(name, f'{cmp["tracking_err_diff_deg"]:.4f}', f'{cmp["current_diff_a"]:.2f}')
    calibration_plot(val_log, sims, out / "calibration_check.png",
                     "Held-out pick-and-place: hardware log vs simulation")

    # 5. Hone the firmware
    print("\nStep 4: hone the firmware with the calibrated model")
    if args.virtual:
        honed = V.record(design, validation_traj, firmware=calibrated, seed=200)
        before, after = C.tracking_rms_deg(val_log), C.tracking_rms_deg(honed)
        print(f"  Tracking error on the machine, design firmware:     {before:.4f} deg rms")
        print(f"  Tracking error on the machine, calibrated firmware: {after:.4f} deg rms "
              f"({100 * (1 - after / before):.0f}% lower)")
    else:
        print(f"  Flash the feedforward values from {calibrated.name}.json, rerun the job,")
        print("  log it, and rerun this command with the new log to confirm the improvement.")
    print(f"\nSaved in {out}: {calibrated.name}.json, residual_model.npz, calibration_check.png")


# -- trajectory ------------------------------------------------------------------------

def cmd_trajectory(args: argparse.Namespace) -> None:
    from .logio import save_csv
    from .trajectory import calibration_routine, pick_and_place, to_samples

    if args.kind == "calibration":
        if args.q_min >= args.q_max:
            raise SystemExit("error: --q-min must be below --q-max")
        traj = calibration_routine(args.seconds, seed=args.seed, q_min=args.q_min, q_max=args.q_max)
        default_name = "calibration_trajectory.csv"
    else:
        traj = pick_and_place(args.q_pick, args.q_place, args.move_time, args.dwell, args.cycles)
        default_name = "pick_and_place_trajectory.csv"
    samples = to_samples(traj, args.rate)
    path = Path(args.output or Path(_out_dir(args.out)) / default_name)
    path.parent.mkdir(parents=True, exist_ok=True)
    save_csv(samples, path, columns=["t", "q_ref", "qd_ref", "qdd_ref", "payload"])
    q = samples["q_ref"]
    print(f"Wrote {path}: {traj.duration:.1f} s at {args.rate:.0f} Hz, {len(q)} rows")
    print(f"  angle range {q.min():+.3f} to {q.max():+.3f} rad "
          f"({math.degrees(q.min()):+.1f} to {math.degrees(q.max()):+.1f} deg, 0 = horizontal)")
    print(f"  fastest {abs(samples['qd_ref']).max():.2f} rad/s; starts at {q[0]:+.3f} rad "
          f"with the gripper {'closed' if samples['payload'][0] else 'open'}")
    print("Move the arm to the start angle slowly before playing it. Check the range fits your machine.")


# -- params ----------------------------------------------------------------------------

def cmd_params(args: argparse.Namespace) -> None:
    from .params import parameter_reference, section_docs

    rows = parameter_reference()
    if args.markdown:
        sys.stdout.write(params_markdown(rows, section_docs()))
        return
    values = _load_design(args.design).to_dict() if args.design else None
    shown = [r for r in rows if not args.filter or args.filter.lower() in r[0].lower()]
    if not shown:
        raise SystemExit(f"error: no parameter matches '{args.filter}'")
    w = max(len(r[0]) for r in shown)
    label = "value" if values else "default"
    print(f"{'parameter':<{w}}  {label:>12}  {'unit':<11}  meaning")
    for key, unit, default, doc in shown:
        value = default
        if values:
            node = values
            for part in key.split("."):
                node = node[part]
            value = node
        print(f"{key:<{w}}  {_fmt(value):>12}  {unit:<11}  {doc}")


def _fmt(v) -> str:
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, (int, str)):
        return str(v)
    if isinstance(v, list):
        return "[]" if not v else f"[{len(v)} values]"
    return "0" if v == 0 else (f"{v:.4g}" if 1e-3 <= abs(v) < 1e5 else f"{v:.3g}")


def params_markdown(rows, sections: dict[str, str]) -> str:
    lines = ["# Design parameters", "",
             "Every key in a design file, with its unit and default. Generated by",
             "`volt2motion params --markdown`, so do not edit by hand.", "",
             "See [the design file guide](design-file.md) for where each value comes from.", ""]
    current = None
    for key, unit, default, doc in rows:
        section = key.split(".")[0] if "." in key else ""
        if section != current:
            current = section
            title = section or "top level"
            lines += ["", f"## {title}", ""]
            if sections.get(section):
                lines += [sections[section], ""]
            lines += ["| Key | Default | Unit | Meaning |", "|---|---|---|---|"]
        lines.append(f"| `{key}` | {_fmt(default)} | {unit} | {doc} |")
    return "\n".join(lines).replace("\n\n\n", "\n\n").lstrip() + "\n"


# -- argument parsing ------------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    raw = argparse.RawDescriptionHelpFormatter
    ap = argparse.ArgumentParser(prog="volt2motion", description=__doc__, formatter_class=raw)
    ap.add_argument("--version", action="version", version=f"volt2motion {_version()}")
    sub = ap.add_subparsers(dest="command", metavar="<command>")

    p = sub.add_parser("init", formatter_class=raw, help="set up a folder with a starter design, parts table and sample board",
                       description="Write baseline.json (every design parameter), parts.json (datasheet values\n"
                                   "for the KiCad importer) and a sample KiCad driver board into a folder.")
    p.add_argument("folder", nargs="?", default=".", help="where to write them (default: current folder)")
    p.add_argument("--force", action="store_true", help="overwrite files that already exist")
    p.set_defaults(func=cmd_init)

    p = sub.add_parser("design", formatter_class=raw, help="phase 1: check a design in simulation",
                       description="Simulate a design and check torque, voltage headroom, bus voltage and heat.\n"
                                   "Runs a few pick-and-place cycles at full detail, then a long shift at fast detail.",
                       epilog="examples:\n"
                              "  volt2motion design baseline.json\n"
                              "  volt2motion design baseline.json --set arm.payload_mass=8 --set gearbox.ratio=80\n"
                              "  volt2motion design out/sample_driver.json --shift-minutes 60")
    p.add_argument("design", nargs="?", help="design file (default: the built-in baseline)")
    p.add_argument("--set", action="append", default=[], metavar="KEY=VALUE",
                   help="override a parameter, e.g. gearbox.ratio=60 (repeatable)")
    p.add_argument("--cycles", type=int, default=4, help="pick-and-place cycles at full detail (default: 4)")
    p.add_argument("--move-time", type=float, default=0.9, help="seconds per pick or place move (default: 0.9)")
    p.add_argument("--shift-minutes", type=float, default=20.0, help="length of the shift run, 0 skips it (default: 20)")
    p.add_argument("--out", default="out", help="folder for plots (default: out)")
    p.set_defaults(func=cmd_design)

    p = sub.add_parser("import-kicad", formatter_class=raw, help="fill in driver values from a KiCad board",
                       description="Read a motor driver's .kicad_pcb (and .kicad_sch if present) and fill in the\n"
                                   "driver, bus and MOSFET cooling parameters of a design. Writes out/<name>.json.",
                       epilog="examples:\n"
                              "  volt2motion import-kicad sample_driver --run\n"
                              "  volt2motion import-kicad ~/kicad/my_driver --base baseline.json\n"
                              "  volt2motion import-kicad my_driver.kicad_pcb --convection 40   (with a fan)")
    p.add_argument("board", help="KiCad project folder, or a .kicad_pcb file")
    p.add_argument("--base", help="design supplying the motor, gearbox, arm and firmware values (default: built-in baseline)")
    p.add_argument("--parts", help="parts table with MOSFET datasheet values (default: ./parts.json, else the bundled example)")
    p.add_argument("--convection", type=float, default=15.0,
                   help="heat transfer to air in W/m^2K: about 15 in still air, 40-80 with a fan (default: 15)")
    p.add_argument("--name", help="design name (default: the board's folder or file name)")
    p.add_argument("--run", action="store_true", help="run the design check on the result")
    p.add_argument("--out", default="out", help="folder for the design file and plots (default: out)")
    p.set_defaults(func=cmd_import_kicad)

    p = sub.add_parser("calibrate", formatter_class=raw, help="phase 2: fit the model to logs from the built machine",
                       description="Fit the physics model to logs from the built joint, train a small learned\n"
                                   "correction on what the physics misses, and check both on a held-out run.\n\n"
                                   "With real hardware, log two runs at 1 kHz with these columns:\n"
                                   "  t, q_ref, q, qd, payload, iq_ref, iq, id, vd, vq, vbus, t_winding\n"
                                   "  1. the calibration routine (export it with 'volt2motion trajectory calibration')\n"
                                   "  2. any normal job, held out for validation\n\n"
                                   "Before hardware exists, --virtual runs the whole loop on a synthetic machine.",
                       epilog="examples:\n"
                              "  volt2motion calibrate --virtual\n"
                              "  volt2motion calibrate baseline.json --log cal.csv --validation-log pick.csv")
    p.add_argument("design", nargs="?", help="design the machine was built from (default: the built-in baseline)")
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument("--virtual", action="store_true", help="use the synthetic stand-in machine")
    src.add_argument("--log", help="CSV from the calibration routine on real hardware")
    p.add_argument("--validation-log", help="CSV from a normal job, held out for validation (needed with --log)")
    p.add_argument("--cal-seconds", type=float, default=90.0, help="length of the virtual calibration run (default: 90)")
    p.add_argument("--out", default="out", help="folder for fitted files and plots (default: out)")
    p.set_defaults(func=cmd_calibrate, parser=p)

    p = sub.add_parser("trajectory", formatter_class=raw, help="export a trajectory for the robot's firmware to play",
                       description="Write a trajectory as a CSV of setpoints (t, q_ref, qd_ref, qdd_ref, payload)\n"
                                   "for the robot's firmware to play back while logging.\n\n"
                                   "  calibration     the system identification routine for 'volt2motion calibrate --log'\n"
                                   "  pick-and-place  a repeated job, useful as the held-out validation run\n\n"
                                   "Angles are in radians at the joint output, 0 = horizontal, positive lifts.",
                       epilog="examples:\n"
                              "  volt2motion trajectory calibration --q-min -0.5 --q-max 1.0\n"
                              "  volt2motion trajectory pick-and-place --cycles 10 -o job.csv")
    p.add_argument("kind", choices=["calibration", "pick-and-place"])
    p.add_argument("-o", "--output", help="CSV file to write (default: out/<kind>_trajectory.csv)")
    p.add_argument("--out", default="out", help="folder for the default output file (default: out)")
    p.add_argument("--rate", type=float, default=1000.0, help="samples per second (default: 1000)")
    g = p.add_argument_group("calibration options")
    g.add_argument("--seconds", type=float, default=90.0, help="length (default: 90)")
    g.add_argument("--q-min", type=float, default=-0.8, help="lowest angle in rad (default: -0.8)")
    g.add_argument("--q-max", type=float, default=1.3, help="highest angle in rad (default: 1.3)")
    g.add_argument("--seed", type=int, default=1, help="random seed for the move sequence (default: 1)")
    g = p.add_argument_group("pick-and-place options")
    g.add_argument("--q-pick", type=float, default=-0.5, help="pick angle in rad (default: -0.5)")
    g.add_argument("--q-place", type=float, default=1.0, help="place angle in rad (default: 1.0)")
    g.add_argument("--move-time", type=float, default=0.9, help="seconds per move (default: 0.9)")
    g.add_argument("--dwell", type=float, default=0.4, help="seconds to grip or release (default: 0.4)")
    g.add_argument("--cycles", type=int, default=4, help="pick-and-place cycles (default: 4)")
    p.set_defaults(func=cmd_trajectory)

    p = sub.add_parser("params", formatter_class=raw, help="list every design parameter with its unit and meaning",
                       description="List the design parameters: key, default, unit and meaning.",
                       epilog="examples:\n"
                              "  volt2motion params\n"
                              "  volt2motion params gearbox\n"
                              "  volt2motion params --design out/sample_driver.json driver")
    p.add_argument("filter", nargs="?", help="only show keys containing this text, e.g. motor or tempco")
    p.add_argument("--design", help="show this design file's values instead of the defaults")
    p.add_argument("--markdown", action="store_true", help="print the full reference as Markdown")
    p.set_defaults(func=cmd_params)
    return ap


def main(argv: list[str] | None = None) -> None:
    parser = build_parser()
    args = parser.parse_args(argv)
    if not getattr(args, "func", None):
        parser.print_help()
        sys.exit(2)
    args.func(args)


if __name__ == "__main__":
    main()
