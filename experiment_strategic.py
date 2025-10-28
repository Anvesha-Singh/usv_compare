# experiment_strategic.py
# Run comprehensive tests with difficulty levels to expose algorithm weaknesses

import os
import csv
import time
import random
import numpy as np
import math
from datetime import datetime
from typing import Dict, List, Tuple
from pathlib import Path

import matplotlib.pyplot as plt
from tqdm import tqdm

from scenario_generator_challenging import ChallengeScenarioBuilder
from simulator import Obstacle, DynamicObstacle, USV, SimulationConfig
from simulator import calculate_path_length, calculate_path_smoothness, calculate_clearance, calculate_heading_stability
from planners import ImprovedAPF, AEGAfi


class StrategicTrialRunner:
    """Run trials strategically focused on algorithm differences"""
    
    def __init__(self, config: SimulationConfig, max_steps: int = 500, dt: float = 0.1):
        self.config = config
        self.max_steps = max_steps
        self.dt = dt
        
    def run_online_apf_trial(self, scenario_name: str, difficulty: str, 
                            trial_id: int, seed: int = None) -> Dict:
        """Run APF trial"""
        if seed is not None:
            random.seed(seed)
            np.random.seed(seed)
        
        start, goal, static_obs, dynamic_obs = ChallengeScenarioBuilder.create_scenario(
            scenario_name, difficulty)
        all_obs = static_obs + dynamic_obs
        
        usv = USV(start[0], start[1], theta=0.0, config=self.config)
        planner = ImprovedAPF()
        
        trajectory = [usv.get_position()]
        collided = False
        reached = False
        total_time = 0.0
        collision_obs = None
        
        for step in range(self.max_steps):
            for dobs in dynamic_obs:
                dobs.step(self.dt)
            
            t0 = time.time()
            throttle, yaw_cmd = planner.compute_control(usv, goal, all_obs, self.dt)
            total_time += time.time() - t0
            
            usv.set_control(throttle, yaw_cmd)
            usv.step(self.dt)
            trajectory.append(usv.get_position())
            
            collided, collision_obs = usv.check_collision(all_obs)
            if collided:
                break
            
            if usv.distance_to(goal[0], goal[1]) < self.config.goal_threshold:
                reached = True
                break
        
        path_len = calculate_path_length(trajectory)
        smoothness = calculate_path_smoothness(trajectory)
        clearance = calculate_clearance(trajectory, static_obs)
        
        return {
            'mode': 'Online_APF',
            'scenario': scenario_name,
            'difficulty': difficulty,
            'trial_id': trial_id,
            'success': reached,
            'collision': collided,
            'steps': len(trajectory),
            'path_length': path_len,
            'path_smoothness': smoothness,
            'min_clearance': clearance,
            'planning_cpu_time': total_time,
            'num_obstacles': len(static_obs) + len(dynamic_obs),
        }

    def run_offline_aegafi_trial(self, scenario_name: str, difficulty: str, 
                                trial_id: int, seed: int = None) -> Dict:
        """Run AEGAfi trial"""
        if seed is not None:
            random.seed(seed)
            np.random.seed(seed)
        
        start, goal, static_obs, dynamic_obs = ChallengeScenarioBuilder.create_scenario(
            scenario_name, difficulty)
        all_obs = static_obs + dynamic_obs
        
        usv = USV(start[0], start[1], theta=0.0, config=self.config)
        
        bounds = [(-1.0, 13.0), (-4.0, 4.0)]
        planner = AEGAfi(bounds, pop_size=100, generations=200, elite_frac=0.25)
        
        t0 = time.time()
        global_path = planner.plan(start, goal, all_obs, timeout=8.0)
        planning_time = time.time() - t0
        
        trajectory = [usv.get_position()]
        collided = False
        reached = False
        execution_time = 0.0
        current_idx = 1
        
        for step in range(self.max_steps):
            for dobs in dynamic_obs:
                dobs.step(self.dt)
            
            if current_idx >= len(global_path):
                current_idx = len(global_path) - 1
            
            target = global_path[current_idx]
            dx = target[0] - usv.x
            dy = target[1] - usv.y
            dist = math.hypot(dx, dy)
            
            desired_heading = math.atan2(dy, dx)
            heading_error = USV._angle_diff(desired_heading, usv.theta)
            yaw_cmd = np.clip(heading_error * 1.5 / self.dt, 
                             -self.config.max_yaw_rate, self.config.max_yaw_rate)
            throttle = 1.0 if dist > 0.6 else 0.3
            
            t0 = time.time()
            usv.set_control(throttle, yaw_cmd)
            usv.step(self.dt)
            execution_time += time.time() - t0
            trajectory.append(usv.get_position())
            
            if dist < 0.6 and current_idx < len(global_path) - 1:
                current_idx += 1
            
            collided, _ = usv.check_collision(all_obs)
            if collided:
                break
            
            if usv.distance_to(goal[0], goal[1]) < self.config.goal_threshold:
                reached = True
                break
        
        path_len = calculate_path_length(trajectory)
        smoothness = calculate_path_smoothness(trajectory)
        clearance = calculate_clearance(trajectory, static_obs)
        
        return {
            'mode': 'Offline_AEGAfi',
            'scenario': scenario_name,
            'difficulty': difficulty,
            'trial_id': trial_id,
            'success': reached,
            'collision': collided,
            'steps': len(trajectory),
            'path_length': path_len,
            'path_smoothness': smoothness,
            'min_clearance': clearance,
            'planning_cpu_time': planning_time,
            'execution_cpu_time': execution_time,
            'num_obstacles': len(static_obs) + len(dynamic_obs),
        }


class StrategicExperimentRunner:
    """Run strategic trials based on difficulty matrix"""
    
    def __init__(self, output_dir: str = 'results_strategic'):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)
        self.config = SimulationConfig()
        self.trial_runner = StrategicTrialRunner(self.config)
        self.all_results: List[Dict] = []
        
    def run_experiments(self) -> List[Dict]:
        """Run with strategic difficulty matrix"""
        
        # Matrix designed to expose algorithm differences
        test_matrix = {
            'open': {
                'easy': 3,
                'medium': 5,
                'hard': 10,     
                'extreme': 5
            },
            'channel': {
                'easy': 3,
                'medium': 3,
                'hard': 8,       
                'extreme': 6
            },
            'clutter': {
                'easy': 3,
                'medium': 8,    
                'hard': 5,
                'extreme': 3
            },
            'dynamic': {
                'easy': 3,
                'medium': 5,
                'hard': 8,      
                'extreme': 5
            },
            'maze': {
                'easy': 3,
                'medium': 8,     
                'hard': 5,
                'extreme': 3
            }
        }
        
        modes = ['Online_APF', 'Offline_AEGAfi']
        
        # Calculate total trials
        total_trials = sum(
            sum(v.values()) * len(modes)
            for v in test_matrix.values()
        )
        
        pbar = tqdm(total=total_trials, desc="Strategic Trials")
        
        for scenario, difficulties in test_matrix.items():
            for difficulty, num_trials in difficulties.items():
                print(f"\n{'='*60}")
                print(f"{scenario.upper()} - {difficulty.upper()}")
                print(f"{'='*60}")
                
                for mode in modes:
                    trial_results = []
                    
                    for trial_id in range(num_trials):
                        if mode == 'Online_APF':
                            result = self.trial_runner.run_online_apf_trial(
                                scenario, difficulty, trial_id, seed=trial_id * 1000)
                        else:
                            result = self.trial_runner.run_offline_aegafi_trial(
                                scenario, difficulty, trial_id, seed=trial_id * 1000)
                        
                        trial_results.append(result)
                        self.all_results.append(result)
                        pbar.update(1)
                    
                    # Print summary for this batch
                    success_rate = np.mean([r['success'] for r in trial_results])
                    avg_steps = np.mean([r['steps'] for r in trial_results])
                    print(f"  {mode:20} | Success: {success_rate*100:5.1f}% | Avg Steps: {avg_steps:6.1f}")
                    
                    # Save trial CSV
                    self._save_trial_csv(trial_results, scenario, difficulty, mode)
        
        pbar.close()
        
        self._save_summary_csv()
        self._save_comparison_report()
        
        return self.all_results

    def _save_trial_csv(self, results: List[Dict], scenario: str, difficulty: str, mode: str):
        """Save trial data"""
        if not results:
            return
        
        filename = self.output_dir / f"{mode}_{scenario}_{difficulty}_trials.csv"
        fieldnames = list(results[0].keys())
        
        with open(filename, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(results)

    def _save_summary_csv(self):
        """Save aggregate summary"""
        if not self.all_results:
            return
        
        summary_data = []
        
        for scenario in sorted(set(r['scenario'] for r in self.all_results)):
            for difficulty in sorted(set(r['difficulty'] for r in self.all_results if r['scenario'] == scenario)):
                for mode in sorted(set(r['mode'] for r in self.all_results)):
                    subset = [r for r in self.all_results 
                             if r['scenario'] == scenario and r['difficulty'] == difficulty and r['mode'] == mode]
                    
                    if not subset:
                        continue
                    
                    success_rate = np.mean([r['success'] for r in subset])
                    collision_rate = np.mean([r['collision'] for r in subset])
                    
                    successful = [r for r in subset if r['success']]
                    avg_path = np.mean([r['path_length'] for r in successful]) if successful else 0.0
                    avg_steps = np.mean([r['steps'] for r in subset])
                    
                    summary_data.append({
                        'scenario': scenario,
                        'difficulty': difficulty,
                        'mode': mode,
                        'trials': len(subset),
                        'success_rate': success_rate,
                        'collision_rate': collision_rate,
                        'avg_path_length': avg_path,
                        'avg_steps': avg_steps,
                        'num_obstacles': subset[0]['num_obstacles'],
                    })
        
        with open(self.output_dir / 'summary.csv', 'w', newline='') as f:
            if summary_data:
                writer = csv.DictWriter(f, fieldnames=summary_data[0].keys())
                writer.writeheader()
                writer.writerows(summary_data)

    def _save_comparison_report(self):
        """Save detailed report"""
        if not self.all_results:
            return
        
        report = []
        report.append("="*80)
        report.append("STRATEGIC MARITIME PATH PLANNING COMPARISON")
        report.append("Testing Algorithm Limits with Difficulty Progression")
        report.append(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        report.append("="*80)
        report.append("")
        
        scenarios = sorted(set(r['scenario'] for r in self.all_results))
        
        for scenario in scenarios:
            report.append(f"\n{'='*80}")
            report.append(f"SCENARIO: {scenario.upper()}")
            report.append(f"{'='*80}\n")
            
            difficulties = sorted(set(r['difficulty'] for r in self.all_results 
                                     if r['scenario'] == scenario))
            
            for difficulty in difficulties:
                report.append(f"{difficulty.upper()}:")
                report.append("-" * 60)
                
                for mode in ['Online_APF', 'Offline_AEGAfi']:
                    subset = [r for r in self.all_results 
                             if r['scenario'] == scenario and r['difficulty'] == difficulty and r['mode'] == mode]
                    
                    if not subset:
                        continue
                    
                    success_rate = np.mean([r['success'] for r in subset])
                    collision_rate = np.mean([r['collision'] for r in subset])
                    successful = [r for r in subset if r['success']]
                    
                    report.append(f"  {mode}:")
                    report.append(f"    Success: {success_rate*100:5.1f}% ({sum(r['success'] for r in subset)}/{len(subset)})")
                    report.append(f"    Collision: {collision_rate*100:5.1f}%")
                    
                    if successful:
                        avg_steps = np.mean([r['steps'] for r in successful])
                        report.append(f"    Avg Steps (successful): {avg_steps:.1f}")
                
                report.append("")
        
        with open(self.output_dir / 'comparison_report.txt', 'w') as f:
            f.write("\n".join(report))
        
        print("\n" + "\n".join(report))


if __name__ == '__main__':
    print("Starting Strategic Maritime Path Planning Experiments...")
    print("="*80)
    print("Testing Algorithm Limits with Difficulty Progression")
    print("="*80)
    print()
    
    runner = StrategicExperimentRunner(output_dir='results_strategic')
    results = runner.run_experiments()
    
    print("\n" + "="*80)
    print("EXPERIMENTS COMPLETE!")
    print("Results saved to 'results_strategic/' directory")
    print("="*80)