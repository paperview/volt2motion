import numpy as np
import pytest
from sklearn.neural_network import MLPRegressor
from sklearn.preprocessing import StandardScaler

from volt2motion.calibrate import RESIDUAL_LAGS, ResidualModel, _features, _lagged


@pytest.fixture
def model_and_log():
    rng = np.random.default_rng(0)
    n = 400
    log = {"q": rng.uniform(-1, 1, n), "qd": rng.uniform(-2, 2, n), "iq": rng.uniform(-20, 20, n)}
    X = _features(log["q"], log["qd"], log["iq"], [_lagged(log["iq"], lag) for lag in RESIDUAL_LAGS])
    y = np.sin(X[:, 0]) + 0.1 * X[:, 2]
    m = ResidualModel(hidden=(8, 8))
    m.scaler = StandardScaler().fit(X)
    m.net = MLPRegressor(hidden_layer_sizes=(8, 8), activation="tanh", max_iter=200, random_state=0).fit(m.scaler.transform(X), y)
    m.clip = 5.0
    m._cache()
    return m, log, X


def test_numpy_forward_pass_matches_sklearn(model_and_log):
    m, log, X = model_and_log
    expected = np.clip(m.net.predict(m.scaler.transform(X)), -m.clip, m.clip)
    gate = np.clip((np.abs(log["qd"]) - 0.02) / 0.03, 0, 1)
    assert m.predict(log) == pytest.approx(expected * gate, abs=1e-9)


def test_save_and_load_round_trip(model_and_log, tmp_path):
    m, log, _ = model_and_log
    path = tmp_path / "residual.npz"
    m.save(path)
    loaded = ResidualModel.load(path)
    assert loaded.predict(log) == pytest.approx(m.predict(log), abs=1e-12)
    assert loaded.predict_one(0.3, 1.0, 5.0, (4.0, 3.0)) == pytest.approx(m.predict_one(0.3, 1.0, 5.0, (4.0, 3.0)))
