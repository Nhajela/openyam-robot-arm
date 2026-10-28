import os

# Tests must not depend on where the arm is mounted today: no per-site limits (armctl/site_limits.json).
os.environ.setdefault("ARMCTL_SITE_LIMITS", "")
