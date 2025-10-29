import os
import csv
import time
import numpy as np
import math
from pathlib import Path
from typing import Dict, List, Tuple
from datetime import datetime

from tqdm import tqdm

from simulator import USV, SimulationConfig, Obstacle, DynamicObstacle, calculate_path_length
from planners_colregs_aware import ImprovedAPFColREGs
from colregs import COLREGsChecker

class CommonOceanRunner:
    """
    Run algorithms against CommonOcean scenarios
    Note: Requires commonoceanio installed: pip install commonoceanio
    """

    def __init__(self, config: SimulationConfig, max_steps: int = 500, dt: float = 0.1):
        self.config = config
        self.max_steps = max_steps
        self.dt = dt
        
        try:
            from commonoceanio import ScenarioFileReader
            self.scenario_reader = ScenarioFileReader()
            self.commonocean_available = True
        except ImportError:
            print("WARNING: commonoceanio not installed. Install with: pip install commonoceanio")
            self.commonocean_available = False

    def load_scenario_from_xml(self, xml_path: str) -> Tuple[Tuple[float, float], Tuple[float, float], List[Obstacle], List[DynamicObstacle]]:
        if not self.commonocean_available:
            raise ImportError("commonoceanio not installed")

        from commonoceanio import ScenarioFileReader
        reader = ScenarioFileReader()
        scenario = reader.open(xml_path)

        start = tuple(scenario.start_poses[0][:2]) if hasattr(scenario, 'start_poses') else (0, 0)
        goal = tuple(scenario.goal_poses[0][:2]) if hasattr(scenario, 'goal_poses') else (10, 0)

        obstacles = []
        dynamic_obs = []
        if hasattr(scenario, 'obstacles'):
            for i, obs in enumerate(scenario.obstacles):
                obstacles.append(Obstacle(obs.center_x, obs.center_y, obs.radius, name=f"co_{i}"))
        if hasattr(scenario, 'dynamic_obstacles'):
            for i, dobs in enumerate(scenario.dynamic_obstacles):
                dynamic_obs.append(DynamicObstacle(dobs.center_x, dobs.center_y, dobs.radius,
                                                   vx=dobs.velocity_x, vy=dobs.velocity_y, name=f"co_dyn_{i}"))

        return start, goal, obstacles, dynamic_obs

    def create_synthetic_commonocean_scenario(self, scenario_type: str = "open") -> Tuple[Tuple[float, float], Tuple[float, float], List[Obstacle], List[DynamicObstacle]]:
        np.random.seed(42)

        if scenario_type == "open":
            start = (0, 0)
            goal = (20, 0)
            obstacles = [
                Obstacle(5, 2, 0.8),
                Obstacle(10, -3, 1.0),
                Obstacle(15, 2.5, 0.9)
            ]
            dynamic = [
                DynamicObstacle(3, 3, 0.3, vx=0.1, vy=-0.05)  # moving vessel to produce encounters
            ]

        elif scenario_type == "channel":
            start = (0, 0)
            goal = (20, 0)
            obstacles = []
            for x in np.linspace(1, 19, 18):
                obstacles.append(Obstacle(x, 2.5, 0.3))
                obstacles.append(Obstacle(x, -2.5, 0.3))
            dynamic = []

        elif scenario_type == "harbor":
            start = (0, 0)
            goal = (15, -8)
            obstacles = [
                Obstacle(5, 2, 1.2),
                Obstacle(8, -1, 1.0),
                Obstacle(12, 3, 0.8),
                Obstacle(14, -3, 1.1)
            ]
            dynamic = [
                DynamicObstacle(4, -2, 0.4, vx=0.05, vy=0.1)
            ]
        else:
            raise ValueError(f"Unknown scenario type: {scenario_type}")

        return start, goal, obstacles, dynamic

    def run_apf_colregs_trial(self, scenario_name: str, scenario_type: str,
                             trial_id: int, seed: int = None) -> Dict:
        if seed is not None:
            np.random.seed(seed)

        try:
            if self.commonocean_available:
                # This will now look for "USA_NYM-1_20190613_T-1.xml" and "USA_MEC-1_20190115_T-13.xml"
                start, goal, obstacles, dynamic_obstacles = self.load_scenario_from_xml(f"{scenario_name}.xml")
                print(f"Loaded CommonOcean XML scenario: {scenario_name}.xml")
            else:
                raise FileNotFoundError
        except Exception as e:
            print(f"Warning: Failed to load CommonOcean XML '{scenario_name}.xml' ({e}), using synthetic scenario instead.")
            start, goal, obstacles, dynamic_obstacles = self.create_synthetic_commonocean_scenario(scenario_type)

        usv = USV(start[0], start[1], theta=0.0, config=self.config)
        planner = ImprovedAPFColREGs()
        colregs_checker = COLREGsChecker()

        trajectory = [usv.get_position()]
        collided = False
        reached = False
        total_time = 0.0
        colregs_violations = 0

        for step in range(self.max_steps):
            # Update dynamic obstacles
            for dobs in dynamic_obstacles:
                dobs.step(self.dt)

            # Compute control with dynamic obstacles (fully utilize COLREGs-aware planner)
            t0 = time.time()
            throttle, yaw_cmd = planner.compute_control(
                usv, goal, obstacles, dynamic_obstacles, self.dt
            )
            total_time += time.time() - t0

            usv.set_control(throttle, yaw_cmd)
            usv.step(self.dt)
            trajectory.append(usv.get_position())

            # Collision check
            for obs in obstacles:
                if obs.distance_to_point(usv.x, usv.y) <= self.config.collision_buffer:
                    collided = True
                    break
            if collided:
                break

            # COLREGs violation detection between usv and dynamic obstacles
            for dobs in dynamic_obstacles:
                violation, violation_type = colregs_checker.check_colregs_violation(
                    (usv.x, usv.y), usv.theta, usv.v,
                    yaw_cmd,
                    (dobs.x, dobs.y), 
                    dobs.get_heading(),
                    dobs.get_speed()
                )
                if violation:
                    colregs_violations += 1

            if usv.distance_to(goal[0], goal[1]) < self.config.goal_threshold:
                reached = True
                break

        path_len = calculate_path_length(trajectory)

        return {
            'scenario': scenario_name,
            'scenario_type': scenario_type,
            'trial_id': trial_id,
            'planner': 'APF_COLREGs',
            'success': reached,
            'collision': collided,
            'steps': len(trajectory),
            'path_length': path_len,
            'planning_cpu_time': total_time,
            'colregs_violations': colregs_violations,
            'colregs_corrections': planner.metrics.colregs_corrections,
            'colregs_encounters': planner.metrics.encounters_detected,
            'num_obstacles': len(obstacles) + len(dynamic_obstacles),
        }

class CommonOceanExperimentRunner:
    def __init__(self, output_dir: str = 'results_commonocean'):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)
        self.config = SimulationConfig()
        self.runner = CommonOceanRunner(self.config)
        self.all_results: List[Dict] = []

    def run_experiments(self, trials_per_scenario: int = 10) -> List[Dict]:

        # --- UPDATED SCENARIOS ---
        scenarios = [
            ("USA_NYM-1_20190613_T-1", "open"),
            ("USA_MEC-1_20190115_T-13", "harbor"),
        ]
        # -------------------------

        total_trials = len(scenarios) * trials_per_scenario
        pbar = tqdm(total=total_trials, desc="CommonOcean Trials")

        for scenario_name, scenario_type in scenarios:
            print(f"\n{'='*60}")
            print(f"Scenario: {scenario_name} ({scenario_type})")
            print(f"{'='*60}")

            trial_results = []

            for trial_id in range(trials_per_scenario):
                result = self.runner.run_apf_colregs_trial(
                    scenario_name, scenario_type, trial_id, seed=trial_id * 1000
                )
                trial_results.append(result)
                self.all_results.append(result)
                pbar.update(1)

            success_rate = np.mean([r['success'] for r in trial_results])
            collision_rate = np.mean([r['collision'] for r in trial_results])
            avg_colregs_corrections = np.mean([r['colregs_corrections'] for r in trial_results])
            avg_colregs_violations = np.mean([r['colregs_violations'] for r in trial_results])

            print(f"Success: {success_rate*100:.1f}% | Collision: {collision_rate*100:.1f}%")
            print(f"Avg COLREGs Corrections: {avg_colregs_corrections:.1f} | Avg COLREGs Violations Detected: {avg_colregs_violations:.1f}")

            self._save_trial_csv(trial_results, scenario_name, '_'.join(['APF_COLREGs', scenario_name]))

        pbar.close()
        self._save_summary_csv()
        self._save_report()

        return self.all_results

    def _save_trial_csv(self, results: List[Dict], scenario_name: str, filename_prefix: str):
        filename = self.output_dir / f"{filename_prefix}_trials.csv"
        fieldnames = list(results[0].keys())

        with open(filename, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(results)

    def _save_summary_csv(self):
        if not self.all_results:
            return

        summary = []
        for scenario in sorted(set(r['scenario'] for r in self.all_results)):
            subset = [r for r in self.all_results if r['scenario'] == scenario]

            success_rate = np.mean([r['success'] for r in subset])
            collision_rate = np.mean([r['collision'] for r in subset])
            avg_colregs_correct = np.mean([r['colregs_corrections'] for r in subset])
            avg_colregs_viol = np.mean([r['colregs_violations'] for r in subset])

            summary.append({
                'scenario': scenario,
                'trials': len(subset),
                'success_rate': success_rate,
                'collision_rate': collision_rate,
                'avg_colregs_corrections': avg_colregs_correct,
                'avg_colregs_violations': avg_colregs_viol,
            })

        with open(self.output_dir / 'summary.csv', 'w', newline='') as f:
            if summary:
                writer = csv.DictWriter(f, fieldnames=summary[0].keys())
                writer.writeheader()
                writer.writerows(summary)

    def _save_report(self):
        if not self.all_results:
            return

        report = []
        report.append("="*80)
        report.append("COMMONOCEAN MARITIME PATH PLANNING WITH COLREGS REPORT")
        report.append(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        report.append("="*80)
        report.append("")
        report.append("Algorithm: Improved APF with COLREGs Compliance")
        report.append("Benchmarks: CommonOcean (with dynamic vessels and COLREGs checking)")
        report.append("")

        for scenario in sorted(set(r['scenario'] for r in self.all_results)):
            subset = [r for r in self.all_results if r['scenario'] == scenario]

            report.append(f"Scenario: {scenario}")
            report.append("-" * 60)

            success_rate = np.mean([r['success'] for r in subset])
            collision_rate = np.mean([r['collision'] for r in subset])
            successful = [r for r in subset if r['success']]

            report.append(f"  Success Rate: {success_rate*100:.1f}% ({sum(r['success'] for r in subset)}/{len(subset)})")
            report.append(f"  Collision Rate: {collision_rate*100:.1f}%")

            if successful:
                avg_path = np.mean([r['path_length'] for r in successful])
                report.append(f"  Avg Path Length (successful): {avg_path:.2f}m")

            colregs_corrections = np.mean([r['colregs_corrections'] for r in subset])
            colregs_violations = np.mean([r['colregs_violations'] for r in subset])

            report.append(f"  COLREGs Corrections Applied: {colregs_corrections:.1f}")
            report.append(f"  COLREGs Violations Detected: {colregs_violations:.1f}")
            report.append("")

        with open(self.output_dir / 'report.txt', 'w') as f:
            f.write("\n".join(report))

        print("\n" + "\n".join(report))


if __name__ == '__main__':
    print("="*80)
    print("CommonOcean Maritime Path Planning with COLREGs Validation")
    print("="*80)
    print()

    runner = CommonOceanExperimentRunner(output_dir='results_commonocean')
    results = runner.run_experiments(trials_per_scenario=10)

    print("\n" + "="*80)
    print("VALIDATION COMPLETE!")
    print("Results saved to 'results_commonocean/' directory")
    print("="*80)