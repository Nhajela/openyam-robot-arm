"""Per-site limits (armctl/site_limits.json) can only tighten the URDF ∩ DimOS limits."""
import json
import os
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
PROBE = "from armctl import envelope as e; import json; print(json.dumps(e.URDF_LIMITS))"


def _limits(site: dict | None, tmp_path) -> dict:
    env = dict(os.environ)
    if site is None:
        env["ARMCTL_SITE_LIMITS"] = ""
    else:
        f = tmp_path / "site.json"
        f.write_text(json.dumps({"limits": site}))
        env["ARMCTL_SITE_LIMITS"] = str(f)
    out = subprocess.run([sys.executable, "-c", PROBE], cwd=REPO, env=env, capture_output=True, text=True, check=True)
    return {j: tuple(v) for j, v in json.loads(out.stdout).items()}


def test_site_limits_tighten(tmp_path):
    base = _limits(None, tmp_path)
    site = _limits({"j1": [-1.8, 1.5708]}, tmp_path)
    assert site["j1"] == (-1.8, base["j1"][1])
    assert {j: v for j, v in site.items() if j != "j1"} == {j: v for j, v in base.items() if j != "j1"}


def test_site_limits_never_loosen(tmp_path):
    base = _limits(None, tmp_path)
    assert _limits({"j1": [-9.0, 9.0], "j6": [-9.0, 9.0]}, tmp_path) == base


def test_lab_site_file_is_valid():
    d = json.loads((REPO / "armctl" / "site_limits.json").read_text())
    assert d["why"] and all(lo < hi for lo, hi in d["limits"].values())
