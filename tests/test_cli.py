import json

import pytest

from volt2motion.cli import main


@pytest.fixture
def workspace(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    main(["init"])
    return tmp_path


def test_init_writes_starter_files(workspace, capsys):
    assert (workspace / "baseline.json").exists()
    assert "mosfets" in json.loads((workspace / "parts.json").read_text())
    assert (workspace / "sample_driver" / "sample_driver.kicad_pcb").exists()
    # A second run keeps what is there.
    (workspace / "baseline.json").write_text("{}")
    main(["init"])
    assert (workspace / "baseline.json").read_text() == "{}"
    assert "kept" in capsys.readouterr().out


def test_import_kicad_writes_design(workspace):
    main(["import-kicad", "sample_driver"])
    design = json.loads((workspace / "out" / "sample_driver.json").read_text())
    assert design["battery"]["brake_resistance"] == 10
    # A .kicad_pcb file works as well as its folder.
    main(["import-kicad", "sample_driver/sample_driver.kicad_pcb", "--name", "by_file"])
    assert (workspace / "out" / "by_file.json").exists()


def test_design_runs_with_overrides(workspace, capsys):
    main(["design", "baseline.json", "--set", "gearbox.ratio=60", "--cycles", "1", "--shift-minutes", "0"])
    assert "Pick-and-place, full detail" in capsys.readouterr().out
    assert (workspace / "out" / "baseline_cycle.png").exists()


@pytest.mark.parametrize("argv, message", [
    (["design", "missing.json"], "design file not found"),
    (["import-kicad", "nowhere"], "not found"),
    (["design", "--set", "gearbox.ratio"], "needs KEY=VALUE"),
])
def test_clear_errors(workspace, argv, message):
    with pytest.raises(SystemExit, match=message):
        main(argv)


def test_calibrate_log_needs_validation_log(workspace, capsys):
    with pytest.raises(SystemExit):
        main(["calibrate", "--log", "cal.csv"])
    assert "--validation-log is required" in capsys.readouterr().err
