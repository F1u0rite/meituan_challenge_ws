#!/usr/bin/env python3
"""Exercise both audit CLIs against copied results, including corrupt evidence.

Run after a successful colcon test. Never edits the original XML or runs ROS.
OpenAI Codex assisted; existing test identities and failure gates are retained.
"""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build-base', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    args.output_dir.mkdir(parents=True, exist_ok=False)
    sources = sorted(args.build_base.glob('mtc_*_cpp/test_results/**/*.gtest.xml'))
    assert sources, 'No GTest results in audit scope'
    scenarios = ['valid', 'missing_xml', 'renamed_xml', 'unexpected_xml', 'missing_case',
                 'duplicate_case', 'unexpected_case', 'failure', 'error', 'skipped',
                 'not_run', 'not_completed']
    rows = []
    for scenario in scenarios:
        with tempfile.TemporaryDirectory(prefix='cpp-audit-') as tmp:
            build = Path(tmp)
            for source in sources:
                target = build / source.relative_to(args.build_base)
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(source, target)
            target = build / sources[0].relative_to(args.build_base)
            if scenario == 'missing_xml':
                target.unlink()
            elif scenario == 'renamed_xml':
                # Same test identities and XML count, but wrong registration path.
                target.rename(target.with_name('test_unregistered.gtest.xml'))
            elif scenario == 'unexpected_xml':
                target.with_name('test_extra.gtest.xml').write_text('<testsuites/>\n')
            elif scenario != 'valid':
                tree = ET.parse(target)
                case = next(tree.getroot().iter('testcase'))
                parent = next(p for p in tree.getroot().iter() if case in list(p))
                if scenario == 'missing_case':
                    parent.remove(case)
                elif scenario == 'duplicate_case':
                    parent.append(deepcopy(case))
                elif scenario == 'unexpected_case':
                    case.set('name', 'UnknownCaseFromStaleBuild')
                elif scenario in ['failure', 'error', 'skipped']:
                    ET.SubElement(case, scenario).text = 'intentional audit regression input'
                elif scenario == 'not_run':
                    case.set('status', 'notrun')
                elif scenario == 'not_completed':
                    case.set('result', 'suppressed')
                tree.write(target, encoding='utf-8', xml_declaration=True)
            commands = {
                'legacy': [sys.executable, str(root / 'tests/verify_cpp_audit_results.py'),
                           '--baseline', '1d940f7c5f12561fed6641457c0b4b630a282b05', '--baseline-count', '318'],
                'action': [sys.executable, str(root / 'src/mtc_ros2_action_cpp/test/verify_results.py')],
            }
            for label, argv in commands.items():
                argv += ['--build-base', str(build)]
                q = subprocess.run(argv, cwd=root, capture_output=True, text=True, timeout=30)
                (args.output_dir / f'{scenario}-{label}.log').write_text(q.stdout + q.stderr)
                row = dict(scenario=scenario, verifier=label, argv=argv, exit_code=q.returncode,
                           expected_success=scenario == 'valid', passed=(q.returncode == 0) == (scenario == 'valid'))
                rows.append(row)
    (args.output_dir / 'summary.json').write_text(json.dumps(rows, indent=2) + '\n')
    assert all(row['passed'] for row in rows), [r for r in rows if not r['passed']]
    print(f'PASS: {len(rows)} verifier outcomes; valid results accepted, corrupt/incomplete results rejected')


if __name__ == '__main__':
    main()
