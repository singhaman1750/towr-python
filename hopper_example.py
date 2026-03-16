#!/usr/bin/env python3
"""
Python example: Trajectory optimization for a hopper robot using TOWR.
Equivalent to the C++ hopper_example.cc but using Python bindings.
"""

import json
import sys
import os

from towr_bootstrap import load_towr

try:
    towr = load_towr()
    import numpy as np
except ImportError as e:
    print(f"Error importing TOWR bindings: {e}")
    print("\nMake sure to build the Python bindings first:")
    print("  cd towr")
    print("  mkdir build")
    print("  cd build")
    print("  cmake ..")
    print("  cmake --build . --config Release")
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
    """Create and setup a TOWR trajectory optimization problem."""
    
    print("=" * 60)
    print("TOWR Hopper Trajectory Optimization Example (Python)")
    print("=" * 60)
    
    # Create the NLP formulation
    formulation = towr.NlpFormulation()
    
    # Set terrain to flat ground at height 0.0
    print("\n1. Setting terrain (flat ground)...")
    formulation.terrain = towr.FlatGround(height=0.0)
    print("   ✓ Terrain set to flat ground")
    
    # Set robot model to monoped (hopper)
    print("\n2. Setting robot model (Monoped - Hopper)...")
    formulation.model = towr.RobotModel(towr.RobotType.Monoped)
    print(f"   ✓ Robot: Monoped")
    print(f"   ✓ Mass: {formulation.model.mass():.3f} kg")
    print(f"   ✓ End effectors: {formulation.model.num_ee()}")
    print(f"   ✓ Max deviation from nominal: {formulation.model.max_dev_from_nominal()}")
    
    # Set initial state
    print("\n3. Setting initial state...")
    formulation.initial_base.lin.p = np.array([0.0, 0.0, 0.5])
    formulation.initial_base.lin.v = np.array([0.0, 0.0, 0.0])
    formulation.initial_ee_W = [np.zeros(3)]
    print(f"   ✓ Initial position: {formulation.initial_base.lin.p}")
    print(f"   ✓ Initial velocity: {formulation.initial_base.lin.v}")
    
    # Set goal state
    print("\n4. Setting goal state...")
    formulation.final_base.lin.p = np.array([1.0, 0.0, 0.5])
    formulation.final_base.lin.v = np.array([0.0, 0.0, 0.0])
    print(f"   ✓ Goal position: {formulation.final_base.lin.p}")
    print(f"   ✓ Goal velocity: {formulation.final_base.lin.v}")
    
    # Set phase durations for stance/swing pattern
    # Pattern: ____-----_____-----_____-----_____
    # (stance for 0.4s, swing for 0.2s, repeat)
    print("\n5. Setting motion phases (stance/swing)...")
    formulation.params.ee_phase_durations = [[0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.2]]
    formulation.params.ee_in_contact_at_start = [True]
    print(f"   ✓ Phase durations: {formulation.params.ee_phase_durations[0]}")
    print(f"   ✓ Starts in contact: {formulation.params.ee_in_contact_at_start[0]}")
    
    print("\n" + "=" * 60)
    print("Formulation setup complete!")
    print("=" * 60)

    if towr.has_ipopt_solver():
        print("\n6. Solving with IPOPT...")
        result = towr.solve_nlp(
            formulation,
            max_cpu_time=20.0,
            jacobian_approximation="exact",
            sample_dt=0.01,
            print_nlp=False,
        )
        sampled = result["sampled"]
        print(f"   ✓ Iterations: {result['iterations']}")
        print(f"   ✓ Wall clock time: {result['solver_total_wallclock_time']:.3f} s")
        print(f"   ✓ Total motion time: {sampled['total_time']:.3f} s")
        print(f"   ✓ Sample count: {len(sampled['times'])}")
        print(f"   ✓ First base sample: {sampled['base_linear'][0]}")
        print(f"   ✓ Last base sample:  {sampled['base_linear'][-1]}")

        output_path = os.path.join(os.path.dirname(__file__), "hopper_result.json")
        with open(output_path, "w", encoding="utf-8") as output_file:
            json.dump(_to_serializable(result), output_file, indent=2)
        print(f"   ✓ Saved result: {output_path}")
    else:
        print("\n6. Solver status...")
        print("   ! IPOPT solver support is not compiled into towr_cpp.")
        print("   ! You can construct formulations from Python, but not solve them yet.")
        print("   ! Rebuild ifopt with BUILD_IPOPT=ON and rebuild towr_cpp to enable solve_nlp().")

    print("=" * 60)


if __name__ == "__main__":
    main()
