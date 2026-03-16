#!/usr/bin/env python3
"""
Python example: trajectory optimization for a quadruped using TOWR.

This uses the built-in quadruped robot model exposed by the current bindings
and saves the sampled result to quadruped_result.json.
"""

import json
import os
import sys

from towr_bootstrap import load_towr

try:
    towr = load_towr()
    import numpy as np
except ImportError as e:
    print(f"Error importing TOWR bindings: {e}")
    sys.exit(1)


def _to_serializable(value):
    if isinstance(value, dict):
        return {key: _to_serializable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_to_serializable(item) for item in value]
    if hasattr(value, "tolist"):
        return _to_serializable(value.tolist())
    if hasattr(value, "item"):
        try:
            return value.item()
        except Exception:
            pass
    return value


def main():
    print("=" * 60)
    print("TOWR Quadruped Trajectory Optimization Example (Python)")
    print("=" * 60)

    formulation = towr.NlpFormulation()

    print("\n1. Setting terrain...")
    formulation.terrain = towr.FlatGround(height=0.0)
    print("   ✓ Terrain set to flat ground")

    print("\n2. Setting robot model...")
    formulation.model = towr.RobotModel(towr.RobotType.Anymal)
    print("   ✓ Robot: Anymal")
    print(f"   ✓ Mass: {formulation.model.mass():.3f} kg")
    print(f"   ✓ End effectors: {formulation.model.num_ee()}")
    nominal_stance = np.asarray(formulation.model.nominal_stance(), dtype=float)
    print(f"   ✓ Nominal stance: {nominal_stance}")

    print("\n3. Setting initial and goal states...")
    nominal_base_height = float(-np.mean(nominal_stance[:, 2]))
    base_start = np.array([0.0, 0.0, nominal_base_height])
    base_goal = np.array([0.8, 0.0, nominal_base_height])
    formulation.initial_base.lin.p = base_start
    formulation.initial_base.lin.v = np.zeros(3)
    formulation.final_base.lin.p = base_goal
    formulation.final_base.lin.v = np.zeros(3)

    formulation.initial_ee_W = [base_start + foot for foot in nominal_stance]
    print(f"   ✓ Initial base: {formulation.initial_base.lin.p}")
    print(f"   ✓ Goal base:    {formulation.final_base.lin.p}")
    print(f"   ✓ Initial feet: {formulation.initial_ee_W}")

    print("\n4. Setting stand-trot-stand gait timings...")
    # formulation.params.ee_phase_durations = [
    #     [0.8, 0.3, 0.5],
    #     [0.3, 0.3, 0.7],
    #     [0.3, 0.3, 0.7],
    #     [0.8, 0.3, 0.5],
    # ]

    formulation.params.ee_phase_durations = [
    [0.5, 0.2, 0.3, 0.2, 0.3, 0.2, 0.3],
    [0.2, 0.2, 0.3, 0.2, 0.3, 0.2, 0.6],
    [0.2, 0.2, 0.3, 0.2, 0.3, 0.2, 0.6],
    [0.5, 0.2, 0.3, 0.2, 0.3, 0.2, 0.3],
    ]

    formulation.params.ee_in_contact_at_start = [True, True, True, True]

    # Optional: jointly optimize phase durations (gait timing).
    # formulation.params.bound_phase_duration = (0.15, 1.20)
    # formulation.params.optimize_phase_durations()

    print(f"   ✓ Phase durations leg 0: {formulation.params.ee_phase_durations[0]}")
    print(f"   ✓ Contact at start: {formulation.params.ee_in_contact_at_start}")

    print("\n" + "=" * 60)
    print("Formulation setup complete!")
    print("=" * 60)

    if not towr.has_ipopt_solver():
        print("\nIPOPT solver support is not available in towr_cpp.")
        sys.exit(1)

    print("\n5. Solving with IPOPT...")
    result = towr.solve_nlp(
        formulation,
        max_cpu_time=30.0,
        jacobian_approximation="exact",
        sample_dt=0.02,
        print_nlp=False,
    )
    sampled = result["sampled"]
    print(f"   ✓ Iterations: {result['iterations']}")
    print(f"   ✓ Wall clock time: {result['solver_total_wallclock_time']:.3f} s")
    print(f"   ✓ Total motion time: {sampled['total_time']:.3f} s")
    print(f"   ✓ Sample count: {len(sampled['times'])}")
    print(f"   ✓ First base sample: {sampled['base_linear'][0]}")
    print(f"   ✓ Last base sample:  {sampled['base_linear'][-1]}")

    output_path = os.path.join(os.path.dirname(__file__), "quadruped_result.json")
    with open(output_path, "w", encoding="utf-8") as output_file:
        json.dump(_to_serializable(result), output_file, indent=2)
    print(f"   ✓ Saved result: {output_path}")
    print("=" * 60)


if __name__ == "__main__":
    main()