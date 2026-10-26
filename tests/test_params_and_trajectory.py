import json
from dataclasses import fields
from pathlib import Path

import numpy as np
import pytest

from volt2motion import JointDesign, pick_and_place
from volt2motion.cli import main, params_markdown
from volt2motion.params import parameter_reference, section_docs
from volt2motion.trajectory import SampledTrajectory, calibration_routine, to_samples

DOCS = Path(__file__).resolve().parents[1] / "docs"


def test_every_parameter_has_unit_and_description():
    rows = parameter_reference()
    n_fields = sum(len(fields(getattr(JointDesign(), f.name))) if not f.metadata else 1
                   for f in fields(JointDesign))
    assert len(rows) == n_fields
    assert all(doc for _, _, _, doc in rows)


def test_parameter_reference_doc_is_up_to_date():
    expected = params_markdown(parameter_reference(), section_docs())
    assert (DOCS / "parameters.md").read_text() == expected, \
        "regenerate with: volt2motion params --markdown > docs/parameters.md"


@pytest.mark.parametrize("data, message", [
    ({"gearbox": {"ratoi": 80}}, "Did you mean 'gearbox.ratio'"),
    ({"gerbox": {}}, "Did you mean 'gearbox'"),
    ({"gearbox": 5}, "should be a JSON object"),
])
def test_design_file_typos_are_errors(data, message):
    with pytest.raises(ValueError, match=message):
        JointDesign.from_dict(data)


def test_underscore_keys_are_comments():
    d = JointDesign.from_dict({"_note": "rev B", "gearbox": {"_why": "quieter", "ratio": 60}})
    assert d.gearbox.ratio == 60


def test_bad_json_names_the_line(tmp_path):
    path = tmp_path / "d.json"
    path.write_text('{\n  "gearbox": {"ratio": 60,}\n}')
    with pytest.raises(ValueError, match="line 2"):
        JointDesign.load(path)


def test_sampled_trajectory_replays_a_job():
    traj = pick_and_place(cycles=2)
    s = to_samples(traj)
    replay = SampledTrajectory(s["t"], np.round(s["q_ref"], 6), s["payload"])
    for t in np.arange(0.01, traj.duration - 0.02, 0.013):
        q, qd, qdd, pay = traj(t)
        rq, rqd, rqdd, rpay = replay(t)
        assert rq == pytest.approx(q, abs=1e-5)
        assert rqd == pytest.approx(qd, abs=2e-3)
        assert rqdd == pytest.approx(qdd, abs=0.3)  # 3% of the 10 rad/s^2 peak, at move starts


def test_sampled_trajectory_prefers_logged_setpoints():
    s = to_samples(calibration_routine(20))
    replay = SampledTrajectory(s["t"], s["q_ref"], s["payload"], s["qd_ref"], s["qdd_ref"])
    i = len(s["t"]) // 2
    assert replay(s["t"][i])[1] == pytest.approx(s["qd_ref"][i])


def test_trajectory_command_respects_range(tmp_path, capsys):
    out = tmp_path / "cal.csv"
    main(["trajectory", "calibration", "--seconds", "20", "--q-min", "-0.3", "--q-max", "0.6", "-o", str(out)])
    data = np.genfromtxt(out, delimiter=",", names=True)
    assert list(data.dtype.names) == ["t", "q_ref", "qd_ref", "qdd_ref", "payload"]
    assert data["q_ref"].min() >= -0.3 and data["q_ref"].max() <= 0.6
    assert np.diff(data["t"]) == pytest.approx(1e-3)


def test_params_command_shows_design_values(tmp_path, capsys):
    path = tmp_path / "d.json"
    JointDesign().updated({"gearbox.ratio": 77}).save(path)
    main(["params", "gearbox.ratio", "--design", str(path)])
    assert "77" in capsys.readouterr().out


@pytest.fixture(scope="module")
def short_cal_log():
    from volt2motion import virtual_hardware as V
    return V.record(JointDesign(), calibration_routine(30), seed=100)


@pytest.mark.parametrize("flip, expected", [
    ([], None),
    (["iq", "iq_ref", "vq", "vd"], "opposite sign to the joint angle"),
    (["iq", "iq_ref"], "opposite sign to the voltage"),
    (["q", "q_ref", "qd"], "angle direction looks reversed"),
])
def test_sign_mistakes_in_logs_are_caught(short_cal_log, flip, expected):
    from volt2motion import calibrate as C
    log = {k: (-v if k in flip else v) for k, v in short_cal_log.items()}
    elec = C.fit_electrical(log, JointDesign())
    problems = C.fit_problems(elec, C.fit_mechanical(log, JointDesign(), elec))
    if expected is None:
        assert problems == []
    else:
        assert expected in problems[0]
