#!/usr/bin/env python3
"""Backward-compatible CLI for stateful Cartesian arm steps.

Planning is the default. Add ``--execute`` to publish the generated 14-joint
target. The implementation lives in :mod:`kuavo_arm_controller` so every
operation uses fresh feedback and shares the same persistent state history.
"""

import argparse
import json

from kuavo_arm_controller import (
    ARM_SLICES,
    DEFAULT_MUJOCO_STATE_DIR,
    DEFAULT_REAL_STATE_DIR,
    DIRECTIONS,
    KuavoArmController,
)


def main():
    parser = argparse.ArgumentParser(
        description="Stateful Cartesian step to Kuavo arm14 joint target"
    )
    parser.add_argument("--arm", choices=("left", "right"), required=True)
    parser.add_argument("--direction", choices=tuple(DIRECTIONS), required=True)
    parser.add_argument("--step-m", type=float, default=0.02)
    parser.add_argument("--duration-s", type=float, default=1.0)
    parser.add_argument("--robot", choices=tuple(ARM_SLICES), default="5")
    parser.add_argument("--backend", choices=("real", "mujoco"), default="real")
    parser.add_argument("--scene")
    parser.add_argument("--reset-sim", action="store_true")
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--max-error-mm", type=float, default=5.0)
    parser.add_argument(
        "--state-dir", default=DEFAULT_REAL_STATE_DIR
    )
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()

    if args.backend == "mujoco":
        if args.robot != "5":
            raise SystemExit("the MuJoCo backend currently targets robot=5 only")
        from kuavo_mujoco_controller import DEFAULT_SCENE, MuJoCoKuavoArmController

        state_dir = (
            DEFAULT_MUJOCO_STATE_DIR
            if args.state_dir == DEFAULT_REAL_STATE_DIR
            else args.state_dir
        )
        controller = MuJoCoKuavoArmController(
            scene=args.scene or DEFAULT_SCENE,
            state_dir=state_dir,
            max_position_error_mm=args.max_error_mm,
            resume=not args.reset_sim,
        )
    else:
        controller = KuavoArmController(
            robot=args.robot,
            state_dir=args.state_dir,
            feedback_timeout=args.timeout,
            max_position_error_mm=args.max_error_mm,
        )
    result = controller.move_relative(
        arm=args.arm,
        direction=args.direction,
        step_m=args.step_m,
        duration_s=args.duration_s,
        execute=args.execute,
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
