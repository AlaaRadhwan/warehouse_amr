#!/usr/bin/env python3
"""Temporary Nav2 params file for one run: the repo nav2_params.yaml with a few anchored single-line value overrides.
Nothing in src/ is modified. Each override is  ANCHOR|KEY|VALUE : the first line matching KEY (not commented out) AFTER the line that
contains ANCHOR is rewritten; the script asserts the anchor and key exist. A unified diff vs the repo file is written next to OUT.
usage: make_params.py OUT.yaml "FollowPath:|angular_dist_threshold|3.2" ["general_goal_checker:|xy_goal_tolerance|0.15" ...]
"""
import difflib, re, sys
from pathlib import Path
REPO = Path(__file__).resolve().parents[3]
src = (REPO / "src/amr_navigation/config/nav2_params.yaml").read_text().splitlines(keepends=True)
new = list(src)
for spec in sys.argv[2:]:
    anchor, key, val = spec.split("|")
    ai = next(i for i, l in enumerate(new) if anchor in l and not l.lstrip().startswith("#"))
    pat = re.compile(rf"^(\s*{re.escape(key)}:\s*)\S+(.*)$")
    ki = next(i for i in range(ai + 1, len(new)) if pat.match(new[i]) and not new[i].lstrip().startswith("#"))
    new[ki] = pat.sub(lambda m: f"{m.group(1)}{val}{m.group(2)}", new[ki].rstrip("\n")) + "\n"
out = Path(sys.argv[1]); out.write_text("".join(new))
diff = "".join(difflib.unified_diff(src, new, "repo nav2_params.yaml", str(out), n=0))
out.with_suffix(".diff").write_text(diff); print(diff or "(no override)")
