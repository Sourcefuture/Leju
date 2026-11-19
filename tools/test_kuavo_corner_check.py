#!/usr/bin/env python3
"""Exercise local point inspection without recording confirmations in user state."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def main():
    checks = []
    with tempfile.TemporaryDirectory(prefix='kuavo-point-check-test-') as temporary:
        base = Path(temporary)
        scene, state = base / 'scene/scene.xml', base / 'state'
        command = [sys.executable, str(ROOT / 'tools/kuavo_corner_check.py'),
                   '--scene', str(scene), '--state-dir', str(state)]

        def run(*args, code=0):
            proc = subprocess.run(command + list(args), cwd=ROOT, capture_output=True,
                                  text=True, timeout=180, env=os.environ.copy())
            if proc.returncode != code:
                raise AssertionError(f'{args}: exit={proc.returncode}\n{proc.stdout}\n{proc.stderr}')
            index = proc.stdout.find('{')
            return json.loads(proc.stdout[index:]) if index >= 0 else {}

        run('prepare')
        assert not (state / 'manual_calibration.json').exists()
        for point in ['R1', 'R2', 'R3', 'R4', 'A', 'B']:
            before = json.loads((state / 'latest_state.json').read_text())['arm14_rad']
            result = run('preview', point)
            assert result['status'] == 'snapshot_shown', result
            assert result['position_error_mm'] <= 3 and result['orientation_error_deg'] <= 1
            after = json.loads((state / 'latest_state.json').read_text())['arm14_rad']
            other = slice(7, 14) if result['arm'] == 'left' else slice(0, 7)
            np.testing.assert_allclose(np.array(before)[other], np.array(after)[other], atol=1e-12)
            assert not result['operator_confirmed'] and not result['path_checked']
            checks.append({'point': point, 'position_error_mm': result['position_error_mm'],
                           'endpoint_collisions': result['collisions'], 'other_arm_unchanged': True})
            if result['collisions']:
                assert not result['endpoint_collision_free']
                run('confirm', point, '--confirm', code=1)
                assert not (state / 'manual_calibration.json').exists()
                run('reset')
        for point in ['C', 'D']:
            before = (state / 'latest_state.json').read_bytes()
            result = run('preview', point, code=2)
            assert result['state_unchanged'] and result['status'] == 'rejected'
            assert (state / 'latest_state.json').read_bytes() == before
        checks.append('unreachable C/D leave latest state byte-identical')
        # A and B were the last successful previews, so both hands are at
        # their near corners even after rejected C/D requests.
        before = (state / 'latest_state.json').read_bytes()
        archive = base / 'AB_archive'
        run('archive-ab', '--output', str(archive))
        frozen = json.loads((archive / 'AB_pose.json').read_text())
        np.testing.assert_allclose(frozen['arm14_rad'], json.loads(before)['arm14_rad'], atol=1e-12)
        assert (archive / 'source_latest_state.json').read_bytes() == before
        assert (state / 'latest_state.json').read_bytes() == before
        assert not frozen['path_checked'] and not frozen['real_robot_validated']
        run('archive-ab', '--output', str(archive), code=1)
        checks.append('AB archive freezes actual shared pose without moving and refuses overwrite')
        run('preview', 'R1')
        run('archive-ab', '--output', str(base / 'not_AB'), code=1)
        assert not (base / 'not_AB').exists()
        before = (state / 'latest_state.json').read_bytes()
        result = run('jog', '--arm', 'left', '--delta', '.002', '0', '0')
        assert result['status'] == 'planned'
        assert (state / 'latest_state.json').read_bytes() == before
        assert not (state / 'history.jsonl').exists()
        run('jog', '--arm', 'left', '--delta', '.002', '0', '0', '--execute')
        after = run('status')['inspection']
        assert 1 < after['position_error_mm'] < 3 and not after['collisions']
        assert (state / 'history.jsonl').is_file()
        run('jog', '--arm', 'left', '--delta', '-.002', '0', '0', '--execute')
        checks.append('jog dry run does not move; execute records checked path and current state')
        run('confirm', 'R1', code=1)
        assert not (state / 'manual_calibration.json').exists()
        run('confirm', 'R1', '--confirm')
        manual = json.loads((state / 'manual_calibration.json').read_text())
        assert manual['references']['R1']['operator_confirmed']
        assert len(manual['references']['R1']['arm14_rad']) == 14
        checks.append('confirmation requires explicit flag and records 14D in temporary test state only')
        run('prepare', code=1)
        checks.append('prepare refuses to overwrite an existing session')
    report = {'passed': True, 'checks': checks,
              'scope': 'real Kuavo model in temporary local simulation; no user confirmations or robot access'}
    output = ROOT / 'outputs/kuavo_sim/manual_corner_check/command_validation.json'
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
