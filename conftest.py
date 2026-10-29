import os

# Tests must not depend on where the arm is mounted today: no per-site limits (armctl/site_limits.json).
os.environ.setdefault("ARMCTL_SITE_LIMITS", "")
# ...nor on this arm's measured kinematic correction (armctl/kin_corrections.json): synthetic worlds
# render with the nominal FK. Tests that want a correction pass one explicitly.
os.environ.setdefault("ARMCTL_KIN_CORRECTIONS", "off")
