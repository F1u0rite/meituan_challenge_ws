#!/usr/bin/env python3
"""核对 Phase 2B 原424项身份与全部 C++（含 Planner）GTest 实际执行结果。"""
import argparse
from collections import Counter
from pathlib import Path
import re
import subprocess
import xml.etree.ElementTree as ET


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build-base', type=Path, default=Path('build'))
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[3]
    build = args.build_base if args.build_base.is_absolute() else root / args.build_base
    git = lambda *a: subprocess.check_output(['git', *a], cwd=root, text=True)
    baseline = set()
    pattern = re.compile(r'\bTEST(?:_F)?\s*\(\s*(\w+)\s*,\s*(\w+)\s*\)')
    base = 'a04ac68'
    for name in git('ls-tree', '-r', '--name-only', base, 'src').splitlines():
        if re.fullmatch(r'src/mtc_\w+_cpp/test/test_\w+\.cpp', name):
            baseline.update(f'{s}.{c}' for s, c in pattern.findall(git('show', f'{base}:{name}')))
    assert len(baseline) == 358, len(baseline)
    for name in git('ls-tree', '-r', '--name-only', 'b47b5fe', 'src/mtc_trajectory_execution_cpp/test').splitlines():
        if name.endswith('.cpp'):
            source = git('show', f'b47b5fe:{name}')
            baseline.update(f'{s}.{c}' for s, c in pattern.findall(source) if c != 'Name')
            baseline.update('TrajectoryValidator.' + c for c in re.findall(r'^INVALID_TEST\((\w+),', source, re.M))
    assert len(baseline) == 424, len(baseline)
    actual = Counter()
    inventory = set()
    by_package = Counter()
    packages = list((root / 'src').glob('mtc_*_cpp')) + [root / 'src/mtc_moveit_planner']
    xmls = [p for pkg in packages for p in (build / pkg.name / 'test_results').rglob('*.gtest.xml')]
    expected_executables = 0
    for pkg in packages:
        cmake = pkg / 'CMakeLists.txt'
        if not cmake.exists():
            continue
        names = re.findall(r'ament_add_gtest\(\s*(test_\w+)', cmake.read_text())
        expected_executables += len(names)
        for name in names:
            # Listing registers tests but never runs fixtures or enables a backend.
            output = subprocess.check_output([str(build / pkg.name / name), '--gtest_list_tests'],
                                             text=True, timeout=15)
            suite = None
            for line in output.splitlines():
                if re.fullmatch(r'\w+\.', line):
                    suite = line[:-1]
                elif suite and re.fullmatch(r'  \w+', line):
                    key = f'{suite}.{line.strip()}'
                    assert key not in inventory, f'duplicate registration {key}'
                    inventory.add(key)
    for filename in xmls:
        for case in ET.parse(filename).getroot().iter('testcase'):
            key = f"{case.attrib['classname'].split('.')[-1]}.{case.attrib['name']}"
            assert case.find('failure') is None and case.find('error') is None, key
            assert case.find('skipped') is None and case.get('status') == 'run', key
            assert case.get('result') == 'completed', key
            actual[key] += 1
            by_package[filename.relative_to(build).parts[0]] += 1
    assert len(xmls) == expected_executables, (len(xmls), expected_executables)
    assert baseline <= actual.keys(), sorted(baseline - actual.keys())
    assert inventory == actual.keys(), (sorted(inventory - actual.keys()), sorted(actual.keys() - inventory))
    assert all(v == 1 for v in actual.values()), actual
    print(f'PASS: 原358项 Core/Action + 原66项 Phase 2B 身份全部保留并执行；新增及合入{len(inventory - baseline)}项')
    print(f'PASS: {len(actual)}项独立GTest / {len(xmls)}个可执行文件；FAIL 0 / SKIP 0')
    for package, count in sorted(by_package.items()):
        print(f'  {package}: {count} PASS')


if __name__ == '__main__':
    main()
