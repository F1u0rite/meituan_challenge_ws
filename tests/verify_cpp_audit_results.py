#!/usr/bin/env python3
"""Offline audit: compare baseline test identities to actual GTest result XML.

Run after colcon test, with --build-base selecting a fresh build directory.
No ROS graph, hardware, or network access. Failure/missing/skipped cases fail.
"""
import argparse
from collections import Counter
from pathlib import Path
import re
import subprocess
import xml.etree.ElementTree as ET



def declared_cases(source):
    """Expand the existing trajectory validation macro into registered identities."""
    pattern = re.compile(r"\bTEST(?:_F)?\s*\(\s*(\w+)\s*,\s*(\w+)\s*\)")
    cases = {f"{suite}.{case}" for suite, case in pattern.findall(source)
             if (suite, case) != ("TrajectoryValidator", "Name")}
    cases.update("TrajectoryValidator." + case for case in
                 re.findall(r"^INVALID_TEST\((\w+),", source, re.M))
    return cases


def expected_results(root):
    """Use the package's registered GTests, retaining the mtc_*_cpp audit scope."""
    return {
        f"{cmake.parent.name}/test_results/{cmake.parent.name}/{name}.gtest.xml"
        for cmake in (root / "src").glob("mtc_*_cpp/CMakeLists.txt")
        for name in re.findall(r"ament_add_gtest\(\s*(test_\w+)", cmake.read_text())
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-base", type=Path, default=Path("build"))
    parser.add_argument("--baseline", default="e2c48088b7c58fe34615d636f88da85b2f859ee6")
    parser.add_argument("--baseline-count", type=int, default=290,
                        help="Expected baseline identities; use 257 for the original migration HEAD")
    args = parser.parse_args()
    files = subprocess.check_output(
        ["git", "ls-tree", "-r", "--name-only", args.baseline, "src"], text=True
    ).splitlines()
    baseline = set()
    for filename in files:
        if not re.fullmatch(r"src/mtc_\w+_cpp/test/test_\w+\.cpp", filename):
            continue
        source = subprocess.check_output(["git", "show", f"{args.baseline}:{filename}"], text=True)
        baseline.update(declared_cases(source))
    if len(baseline) != args.baseline_count:
        raise SystemExit(f"FAIL: unexpected baseline inventory: {len(baseline)}")
    actual = Counter()
    by_package = Counter()
    bad = []
    xmls = list(args.build_base.glob("mtc_*_cpp/test_results/**/*.gtest.xml"))
    for filename in xmls:
        root = ET.parse(filename).getroot()
        for case in root.iter("testcase"):
            # ament prefixes classname with the ROS package after execution.
            suite = case.attrib["classname"].split(".")[-1]
            identity = f"{suite}.{case.attrib['name']}"
            actual[identity] += 1
            by_package[filename.relative_to(args.build_base).parts[0]] += 1
            if (case.find("failure") is not None or case.find("error") is not None or
                    case.find("skipped") is not None or case.get("status") != "run" or
                    case.get("result") != "completed"):
                bad.append(identity)
    current = set()
    for filename in Path("src").glob("mtc_*_cpp/test/test_*.cpp"):
        current.update(declared_cases(filename.read_text()))
    missing = sorted((baseline | current) - actual.keys())
    unexpected = sorted(actual.keys() - current)
    duplicate = sorted(k for k, count in actual.items() if count != 1)
    expected_xml = expected_results(Path.cwd())
    actual_xml = {str(path.relative_to(args.build_base)) for path in xmls}
    missing_xml = sorted(expected_xml - actual_xml)
    unexpected_xml = sorted(actual_xml - expected_xml)
    if missing or unexpected or duplicate or bad or missing_xml or unexpected_xml or not expected_xml:
        raise SystemExit(f"FAIL: missing={missing}; unexpected={unexpected}; duplicates={duplicate}; "
                         f"bad={bad}; executables={len(xmls)}; missing_xml={missing_xml}; unexpected_xml={unexpected_xml}")
    print(f"PASS: original {len(baseline)}/{args.baseline_count} executed; "
          f"{len(actual) - len(baseline)} new cases executed")
    print(f"PASS: {len(actual)} distinct cases in {len(xmls)} GTest executables; no failure/error/skip")
    for package, count in sorted(by_package.items()):
        print(f"  {package}: {count}")
    print("New executed cases:")
    for identity in sorted(actual.keys() - baseline):
        print(f"  {identity}")


if __name__ == "__main__":
    main()
