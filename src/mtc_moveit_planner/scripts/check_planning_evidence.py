#!/usr/bin/env python3
"""Classify planning-tool logs without inferring a collision from NO_PLAN.

Only a completed successful tool run proves a positive plan. A rejected plan
does not identify the obstacle, attached tool, IK, or timeout as its cause.
OpenAI Codex assisted this evidence-only integration helper.
"""

import argparse
from pathlib import Path
import re

from check_launch_shutdown import shutdown_errors


def classify(text, collision_probe=False):
    successes = re.findall(r"\]: (?:SEQUENCE )?PLAN OK(?:\b|:)", text)
    failures = re.findall(r"\]: (?:SEQUENCE )?PLAN FAILED(?: \[([A-Z_]+)\])?", text)
    if successes and failures:
        return False, "conflicting planning conclusions; inspect raw log"
    if successes:
        if len(successes) != 1 or shutdown_errors(text):
            return False, "success marker without a single clean tool completion"
        if collision_probe:
            return False, "unexpected planning success; collision probe did not reject"
        return True, "planning success and clean tool completion"
    if failures:
        reasons = ",".join(reason or "unspecified" for reason in failures)
        detail = f"planning rejection observed [{reasons}]"
        if collision_probe:
            detail += "; collision cause UNVERIFIED (requires independent collision evidence)"
        return False, detail
    if "failed to apply planning scene obstacles" in text:
        return False, "NO_PLANNING_RESULT: planning scene initialization failed"
    return False, "NO_PLANNING_RESULT: no tool conclusion; inspect raw log and process exit"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    parser.add_argument("--collision-probe", action="store_true")
    args = parser.parse_args()
    try:
        passed, detail = classify(args.log.read_text(encoding="utf-8"), args.collision_probe)
    except (OSError, UnicodeError) as exc:
        passed, detail = False, f"UNREADABLE_EVIDENCE: {exc}"
    print(detail)
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
