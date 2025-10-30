# visualization_integration.py - UPDATED WITH METRICS TRACKING

"""
Fixed visualization integration with proper collision detection and metrics display
"""

from visualization import SimulationVisualizer
from scenario_generator_challenging import ChallengeScenarioBuilder
from simulator import USV, SimulationConfig
from planners import ImprovedAPF, AEGAfi
import matplotlib.pyplot as plt
from pathlib import Path
import math
import numpy as np
import time


def visualize_single_trial(scenario='dynamic', difficulty='medium', method='APF'):
    """
    Run and visualize a single trial with metrics.
    Metrics displayed on plot: path length, efficiency, planning time, steps.

    Usage:
        visualize_single_trial('maze', 'medium', 'APF')
        visualize_single_trial('dynamic', 'hard', 'AEGAfi')
    """
    print(f"\n{'='*60}")
    print(f"Running: {method} on {scenario} ({difficulty})")
    print(f"{'='*60}")

    viz = SimulationVisualizer()
    start, goal, static_obs, dynamic_obs = ChallengeScenarioBuilder.create_scenario(
        scenario, difficulty
    )

    config = SimulationConfig()
    usv = USV(start[0], start[1], theta=0.0, config=config)
    all_obs = static_obs + dynamic_obs

    plan_start_time = time.time()

    if method == 'APF':
        planner = ImprovedAPF()
        trajectory = [usv.get_position()]

        for step in range(500):
            # Update dynamic obstacles
            for dobs in dynamic_obs:
                dobs.step(0.1)

            # Compute control and move
            throttle, yaw_cmd = planner.compute_control(usv, goal, all_obs, 0.1)
            usv.set_control(throttle, yaw_cmd)
            usv.step(0.1)
            trajectory.append(usv.get_position())

            # Check if reached goal
            if usv.distance_to(goal[0], goal[1]) < config.goal_threshold:
                print(f"✓ Goal reached in {step} steps")
                break

        planning_time = time.time() - plan_start_time
        steps = len(trajectory)

    elif method == 'AEGAfi':
        planner = AEGAfi(bounds=[(-1.0, 13.0), (-4.0, 4.0)], pop_size=100, generations=200)
        print("  Planning path with AEGAfi...")

        global_path = planner.plan(start, goal, all_obs, timeout=8.0)
        planning_time = time.time() - plan_start_time
        print(f"  Planned path with {len(global_path)} waypoints in {planning_time:.3f}s")

        trajectory = [usv.get_position()]
        current_idx = 1

        exec_start_time = time.time()

        for step in range(500):
            # Update dynamic obstacles
            for dobs in dynamic_obs:
                dobs.step(0.1)

            if current_idx >= len(global_path):
                current_idx = len(global_path) - 1

            target = global_path[current_idx]
            dx = target[0] - usv.x
            dy = target[1] - usv.y
            dist = math.hypot(dx, dy)

            desired_heading = math.atan2(dy, dx)
            heading_error = USV._angle_diff(desired_heading, usv.theta)
            yaw_cmd = np.clip(heading_error * 1.5 / 0.1, 
                            -config.max_yaw_rate, config.max_yaw_rate)
            throttle = 1.0 if dist > 0.6 else 0.3

            usv.set_control(throttle, yaw_cmd)
            usv.step(0.1)
            trajectory.append(usv.get_position())

            if dist < 0.6 and current_idx < len(global_path) - 1:
                current_idx += 1

            if usv.distance_to(goal[0], goal[1]) < config.goal_threshold:
                print(f"✓ Goal reached in {step} steps")
                break

        planning_time += (time.time() - exec_start_time)
        steps = len(trajectory)

    print(f"  Trajectory length: {len(trajectory)} points")
    print(f"  Total time: {planning_time:.3f}s")

    # Visualize with metrics

    # DEBUG: Check trajectory
    print(f"\n[DEBUG] Trajectory info:")
    print(f"  Points: {len(trajectory)}")
    if len(trajectory) > 1:
        print(f"  Start: ({trajectory[0][0]:.2f}, {trajectory[0][1]:.2f})")
        print(f"  End: ({trajectory[-1][0]:.2f}, {trajectory[-1][1]:.2f})")
        displacement = math.hypot(trajectory[-1][0]-trajectory[0][0], trajectory[-1][1]-trajectory[0][1])
        print(f"  Displacement: {displacement:.3f}m")
        if displacement < 0.01:
            print(f"  ⚠️  WARNING: USV barely moved!")

        viz.plot_scenario(
        start, goal, static_obs, dynamic_obs, trajectory,
        title=f"{method} - {scenario.capitalize()} ({difficulty})",
        save_path=f"trial_{method}_{scenario}_{difficulty}.png",
        planning_time=planning_time,
        steps=steps,
        show_metrics=True
    )
    plt.show()


def create_comparison_visualization(scenario='maze', difficulty='medium'):
    """
    Compare APF and AEGAfi side-by-side with metrics displayed on each plot.
    Tracks: path length, efficiency, planning time, steps taken.
    """
    print(f"\n{'='*60}")
    print(f"Comparing APF vs AEGAfi on {scenario} ({difficulty})")
    print(f"{'='*60}")

    viz = SimulationVisualizer()
    config = SimulationConfig()

    # Run APF
    print("\nRunning APF...")
    apf_start = time.time()
    start, goal, static_obs, dynamic_obs = ChallengeScenarioBuilder.create_scenario(
        scenario, difficulty
    )

    usv_apf = USV(start[0], start[1], theta=0.0, config=config)
    planner_apf = ImprovedAPF()
    apf_trajectory = [usv_apf.get_position()]
    all_obs = static_obs + dynamic_obs
    apf_steps = 0

    for step in range(500):
        for dobs in dynamic_obs:
            dobs.step(0.1)
        throttle, yaw_cmd = planner_apf.compute_control(usv_apf, goal, all_obs, 0.1)
        usv_apf.set_control(throttle, yaw_cmd)
        usv_apf.step(0.1)
        apf_trajectory.append(usv_apf.get_position())
        apf_steps = step
        if usv_apf.distance_to(goal[0], goal[1]) < config.goal_threshold:
            print(f"  APF: Goal reached in {step} steps")
            break

    apf_time = time.time() - apf_start
    print(f"  APF time: {apf_time:.3f}s")

    # Reset and run AEGAfi
    print("\nRunning AEGAfi...")
    ga_start = time.time()
    start, goal, static_obs, dynamic_obs = ChallengeScenarioBuilder.create_scenario(
        scenario, difficulty
    )

    bounds = [(-1.0, 13.0), (-4.0, 4.0)]
    planner_ga = AEGAfi(bounds, pop_size=100, generations=200)
    global_path = planner_ga.plan(start, goal, static_obs + dynamic_obs, timeout=8.0)
    print(f"  GA: Planned path with {len(global_path)} waypoints")

    usv_ga = USV(start[0], start[1], theta=0.0, config=config)
    ga_trajectory = [usv_ga.get_position()]
    current_idx = 1
    ga_steps = 0

    for step in range(500):
        for dobs in dynamic_obs:
            dobs.step(0.1)

        if current_idx >= len(global_path):
            current_idx = len(global_path) - 1

        target = global_path[current_idx]
        dx = target[0] - usv_ga.x
        dy = target[1] - usv_ga.y
        dist = math.hypot(dx, dy)

        desired_heading = math.atan2(dy, dx)
        heading_error = USV._angle_diff(desired_heading, usv_ga.theta)
        yaw_cmd = np.clip(heading_error * 1.5 / 0.1,
                        -config.max_yaw_rate, config.max_yaw_rate)
        throttle = 1.0 if dist > 0.6 else 0.3

        usv_ga.set_control(throttle, yaw_cmd)
        usv_ga.step(0.1)
        ga_trajectory.append(usv_ga.get_position())
        ga_steps = step

        if dist < 0.6 and current_idx < len(global_path) - 1:
            current_idx += 1
        if usv_ga.distance_to(goal[0], goal[1]) < config.goal_threshold:
            print(f"  GA: Goal reached in {step} steps")
            break

    ga_time = time.time() - ga_start
    print(f"  GA time: {ga_time:.3f}s")

    # Create comparison with metrics
    viz.plot_comparison(
        scenario, difficulty, start, goal, static_obs, dynamic_obs,
        apf_trajectory, ga_trajectory,
        apf_time=apf_time,
        ga_time=ga_time,
        apf_steps=apf_steps,
        ga_steps=ga_steps,
        save_path=f"comparison_{scenario}_{difficulty}.png"
    )
    plt.show()


def test_all_scenarios():
    """Run all scenarios for both APF and AEGAfi, log metrics, and save comparison plots."""
    print(f"\n{'='*60}")
    print("Testing all scenarios and difficulties for APF & AEGAfi")
    print(f"{'='*60}\n")

    scenarios = [
    'open', 'channel', 'clutter', 'dynamic', 'maze',
    'colregs_headon', 'colregs_crossing', 'colregs_overtaking']
    difficulties = ['easy', 'medium', 'hard', 'extreme']

    results_dir = Path("comparison_results")
    results_dir.mkdir(exist_ok=True)
    config = SimulationConfig()
    viz = SimulationVisualizer()

    for scenario in scenarios:
        for difficulty in difficulties:
            print(f"\n{'-'*60}")
            print(f"Scenario: {scenario} | Difficulty: {difficulty}")
            print(f"{'-'*60}")

            # --- Run APF ---
            try:
                start, goal, static_obs, dynamic_obs = ChallengeScenarioBuilder.create_scenario(
                    scenario, difficulty
                )
                usv_apf = USV(start[0], start[1], theta=0.0, config=config)
                planner_apf = ImprovedAPF()
                all_obs = static_obs + dynamic_obs
                apf_trajectory = [usv_apf.get_position()]
                apf_success, apf_collision = False, False

                apf_start = time.time()
                for step in range(500):
                    for dobs in dynamic_obs:
                        dobs.step(0.1)
                    throttle, yaw_cmd = planner_apf.compute_control(usv_apf, goal, all_obs, 0.1)
                    usv_apf.set_control(throttle, yaw_cmd)
                    usv_apf.step(0.1)
                    apf_trajectory.append(usv_apf.get_position())
                    collided, _ = usv_apf.check_collision(all_obs)
                    if collided:
                        apf_collision = True
                        break
                    if usv_apf.distance_to(goal[0], goal[1]) < config.goal_threshold:
                        apf_success = True
                        break
                apf_time = time.time() - apf_start
                apf_status = "COLLISION" if apf_collision else ("SUCCESS" if apf_success else "TIMEOUT")

            except Exception as e:
                apf_status, apf_time, apf_trajectory = f"ERROR: {e}", 0, []

            # --- Run AEGAfi ---
            try:
                start, goal, static_obs, dynamic_obs = ChallengeScenarioBuilder.create_scenario(
                    scenario, difficulty
                )
                planner_ga = AEGAfi(bounds=[(-1.0, 13.0), (-4.0, 4.0)], pop_size=100, generations=200)
                usv_ga = USV(start[0], start[1], theta=0.0, config=config)
                global_path = planner_ga.plan(start, goal, static_obs + dynamic_obs, timeout=8.0)

                ga_trajectory = [usv_ga.get_position()]
                current_idx = 1
                ga_success, ga_collision = False, False

                ga_start = time.time()
                for step in range(500):
                    for dobs in dynamic_obs:
                        dobs.step(0.1)
                    if current_idx >= len(global_path):
                        current_idx = len(global_path) - 1
                    target = global_path[current_idx]
                    dx, dy = target[0] - usv_ga.x, target[1] - usv_ga.y
                    dist = math.hypot(dx, dy)
                    desired_heading = math.atan2(dy, dx)
                    heading_error = USV._angle_diff(desired_heading, usv_ga.theta)
                    yaw_cmd = np.clip(heading_error * 1.5 / 0.1, -config.max_yaw_rate, config.max_yaw_rate)
                    throttle = 1.0 if dist > 0.6 else 0.3
                    usv_ga.set_control(throttle, yaw_cmd)
                    usv_ga.step(0.1)
                    ga_trajectory.append(usv_ga.get_position())
                    collided, _ = usv_ga.check_collision(static_obs + dynamic_obs)
                    if collided:
                        ga_collision = True
                        break
                    if dist < 0.6 and current_idx < len(global_path) - 1:
                        current_idx += 1
                    if usv_ga.distance_to(goal[0], goal[1]) < config.goal_threshold:
                        ga_success = True
                        break
                ga_time = time.time() - ga_start
                ga_status = "COLLISION" if ga_collision else ("SUCCESS" if ga_success else "TIMEOUT")

            except Exception as e:
                ga_status, ga_time, ga_trajectory = f"ERROR: {e}", 0, []

            # --- Save Comparison Plot ---
            viz.plot_comparison(
                scenario, difficulty, start, goal, static_obs, dynamic_obs,
                apf_trajectory, ga_trajectory,
                apf_time=apf_time,
                ga_time=ga_time,
                save_path=results_dir / f"{scenario}_{difficulty}_comparison.png"
            )
            plt.close()

            print(f"APF: {apf_status:10s} | Time: {apf_time:.2f}s | Steps: {len(apf_trajectory)}")
            print(f"GA : {ga_status:10s} | Time: {ga_time:.2f}s | Steps: {len(ga_trajectory)}")


if __name__ == "__main__":
    print("Maritime Path Planning Visualization - WITH METRICS")
    print("="*60)
    print()
    print("QUICK START:")
    print()
    print("  1. Test maze scenario:")
    print("     python -c \"from visualization_corrected_integration import visualize_single_trial; visualize_single_trial('maze', 'medium', 'APF')\"")
    print()
    print("  2. Compare APF vs AEGAfi (with metrics):")
    print("     python -c \"from visualization_corrected_integration import create_comparison_visualization; create_comparison_visualization('maze', 'medium')\"")
    print()
    print("  3. Test all scenarios:")
    print("     python -c \"from visualization_corrected_integration import test_all_scenarios; test_all_scenarios()\"")
    print()
    print("METRICS SHOWN ON PLOTS:")
    print("  • Path Length (m)")
    print("  • Efficiency (% of straight line)")
    print("  • Distance to Goal (m)")
    print("  • Planning Time (s)")
    print("  • Steps Taken")