"""Unit and parity tests for ONNX policy export and dual-inference consistency.

Run with: PYTHONPATH=python pytest tests/test_onnx_export.py
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import onnx
import onnxruntime as ort
import pytest
import torch

from export_onnx import NavigationActor, export_actor_to_onnx
from verify_onnx_parity import verify_dual_parity

ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "models/navigation/nav_policy.zip"
ONNX_PATH = ROOT / "models/navigation/nav_policy.onnx"
MANIFEST_PATH = ROOT / "models/navigation/manifest.json"
NORMALIZATION_PATH = ROOT / "models/navigation/normalization.json"


@pytest.fixture(scope="module")
def onnx_model_file(tmp_path_factory):
    """Ensure ONNX model is exported to a test directory."""
    if not MODEL_PATH.is_file():
        pytest.skip("Trained policy nav_policy.zip is not present")
    out_dir = tmp_path_factory.mktemp("onnx_test")
    out_file = out_dir / "nav_policy.onnx"
    export_actor_to_onnx(MODEL_PATH, out_file)
    return out_file


def test_exported_onnx_passes_onnx_checker(onnx_model_file):
    model = onnx.load(str(onnx_model_file))
    onnx.checker.check_model(model)

    graph = model.graph
    assert len(graph.input) == 1
    assert graph.input[0].name == "observation"
    assert len(graph.output) == 1
    assert graph.output[0].name == "action"

    # Verify input shape: [batch_size, 87]
    input_shape = [dim.dim_value or dim.dim_param for dim in graph.input[0].type.tensor_type.shape.dim]
    assert input_shape[-1] == 87

    # Verify output shape: [batch_size, 3]
    output_shape = [dim.dim_value or dim.dim_param for dim in graph.output[0].type.tensor_type.shape.dim]
    assert output_shape[-1] == 3


def test_dual_inference_parity_satisfies_numerical_tolerance(onnx_model_file):
    metrics = verify_dual_parity(
        MODEL_PATH,
        onnx_model_file,
        num_samples=1_000,
        seed=123,
        tolerance=1e-5,
    )
    assert metrics["max_abs_error"] < 1e-5
    assert metrics["dynamic_batch_max_error"] < 1e-5


@pytest.mark.parametrize("batch_size", [1, 2, 7, 16, 64])
def test_dynamic_batching_matches_pytorch(onnx_model_file, batch_size):
    from stable_baselines3 import PPO

    model = PPO.load(str(MODEL_PATH), device="cpu")
    actor = NavigationActor(model.policy)
    actor.eval()

    session = ort.InferenceSession(str(onnx_model_file), providers=["CPUExecutionProvider"])
    inputs = torch.randn(batch_size, 87, dtype=torch.float32)

    with torch.no_grad():
        pt_out = actor(inputs).numpy()

    onnx_out = session.run(None, {"observation": inputs.numpy()})[0]

    assert onnx_out.shape == (batch_size, 3)
    np.testing.assert_allclose(pt_out, onnx_out, atol=1e-5)


def test_onnx_actions_are_strictly_bounded(onnx_model_file):
    session = ort.InferenceSession(str(onnx_model_file), providers=["CPUExecutionProvider"])
    # Extreme inputs to test clamping
    extreme_inputs = np.random.uniform(-100.0, 100.0, size=(500, 87)).astype(np.float32)
    onnx_out = session.run(None, {"observation": extreme_inputs})[0]

    assert np.all(onnx_out >= -1.0)
    assert np.all(onnx_out <= 1.0)


def test_manifest_and_normalization_files_are_consistent():
    assert MANIFEST_PATH.is_file(), "manifest.json must exist"
    assert NORMALIZATION_PATH.is_file(), "normalization.json must exist"

    manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
    assert manifest["observation_dim"] == 87
    assert manifest["action_dim"] == 3
    assert manifest["action_frame"] == "body"
    assert manifest["action_limits"]["forward_mps"] == 15.0
    assert manifest["action_limits"]["left_mps"] == 15.0
    assert manifest["action_limits"]["up_mps"] == 5.0
    assert manifest["normalization_file"] == "normalization.json"

    norm_data = json.loads(NORMALIZATION_PATH.read_text(encoding="utf-8"))
    assert len(norm_data["mean"]) == 87
    assert len(norm_data["var"]) == 87
    assert norm_data["epsilon"] > 0.0
    assert norm_data["clip_obs"] > 0.0
