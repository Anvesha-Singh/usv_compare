# experiment_comprehensive.py
# Comprehensive experiment with three modes: Online APF, Offline AEGAfi, and Hybrid

import os
import csv
import time
import random
import json
import numpy as np
import math
from datetime import datetime
from typing import Dict, List, Tuple
from pathlib import Path

import matplotlib.pyplot as plt
from tqdm import tqdm

from simulator import Obstacle, DynamicObstacle, USV, SimulationConfig
from simulator import calculate_path_length, calculate_path_smoothness, calculate_clearance, calculate_heading_stability
from planners import ImprovedAPF, AEGAfi


class ScenarioBuilder:
    """Helper to create standardized test scenarios"""
    
    @staticmethod
    def create_scenario(name: str) -> Tuple[Tuple[float, float], Tuple[float, float], 
                                           List[Obstacle], List[DynamicObstacle]]:
        """Create standardized maritime scenarios"""
        if name == 'open':
            start = (0.0, 0.0)
            goal = (12.0, 0.0)
            static_obs = [
                Obstacle(4.0, 1.5, 0.7, name="obs1"),
                Obstacle(7.0, -1.2, 0.9, name="obs2")
            ]
            dynamic_obs = []
            
        elif name == 'channel':
            start = (0.0, 0.0)
            goal = (12.0, 0.0)
            static_obs = []
            for x in np.linspace(1, 11, 20):
                static_obs.append(Obstacle(x, 2.0, 0.25, name=f"wall_top_{x:.1f}"))
                static_obs.append(Obstacle(x, -2.0, 0.25, name=f"wall_bot_{x:.1f}"))
            dynamic_obs = []
            
        elif name == 'clutter':
            # FIXED: Solvable clutter scenario
            start = (0.0, 0.0)
            goal = (12.0, 0.0)
            static_obs = []
            random.seed(42)
            for i in range(12):  # Reduced from 25
                x = random.uniform(2, 10)
                y = random.uniform(-2.5, 2.5)
                r = random.uniform(0.2, 0.4)  # Smaller obstacles
                static_obs.append(Obstacle(x, y, r, name=f"clutter_{i}"))
            dynamic_obs = []
            
        elif name == 'dynamic':
            start = (0.0, 0.0)
            goal = (12.0, 0.0)
            static_obs = [Obstacle(6.0, 1.5, 0.6, name="static_obs")]
            dynamic_obs = [
                DynamicObstacle(5.0, -1.5, 0.4, vx=0.12, vy=0.05, name="dynamic_obs1")
            ]
        else:
            raise ValueError(f"Unknown scenario: {name}")
        
        return start, goal, static_obs, dynamic_obs


class TrialRunner:
    """Runs individual trials and collects metrics"""
    
    def __init__(self, config: SimulationConfig, max_steps: int = 500, dt: float = 0.1):
        self.config = config
        self.max_steps = max_steps
        self.dt = dt
        
    def run_online_apf_trial(self, scenario_name: str, trial_id: int, seed: int = None) -> Dict:
        """
        Run APF in ONLINE mode (real-time reactive)
        - No global planning
        - Pure reactive control
        """
        if seed is not None:
            random.seed(seed)
            np.random.seed(seed)
        
        start, goal, static_obs, dynamic_obs = ScenarioBuilder.create_scenario(scenario_name)
        all_obs = static_obs + dynamic_obs
        
        usv = USV(start[0], start[1], theta=0.0, config=self.config)
        planner = ImprovedAPF()
        
        trajectory = [usv.get_position()]
        collided = False
        reached = False
        total_planning_time = 0.0
        collision_obs = None
        
        for step in range(self.max_steps):
            # Update dynamic obstacles
            for dobs in dynamic_obs:
                dobs.step(self.dt)
            
            # Compute control
            t0 = time.time()
            throttle, yaw_cmd = planner.compute_control(usv, goal, all_obs, self.dt)
            total_planning_time += time.time() - t0
            
            # Execute
            usv.set_control(throttle, yaw_cmd)
            usv.step(self.dt)
            trajectory.append(usv.get_position())
            
            # Check collision
            collided, collision_obs = usv.check_collision(all_obs)
            if collided:
                break
            
            # Check goal
            if usv.distance_to(goal[0], goal[1]) < self.config.goal_threshold:
                reached = True
                break
        
        # Calculate metrics
        path_len = calculate_path_length(trajectory)
        smoothness = calculate_path_smoothness(trajectory)
        clearance = calculate_clearance(trajectory, static_obs)
        heading_stability = calculate_heading_stability(usv.heading_history)
        
        trial_result = {
            'mode': 'Online_APF',
            'scenario': scenario_name,
            'trial_id': trial_id,
            'success': reached,
            'collision': collided,
            'steps': len(trajectory),
            'path_length': path_len,
            'path_smoothness': smoothness,
            'min_clearance': clearance,
            'heading_stability': heading_stability,
            'planning_cpu_time': total_planning_time,
            'avg_planning_time_per_step': total_planning_time / len(trajectory) if len(trajectory) > 0 else 0,
            'max_speed': max(usv.velocity_history) if usv.velocity_history else 0.0,
            'avg_speed': np.mean(usv.velocity_history) if usv.velocity_history else 0.0,
        }
        
        return trial_result

    def run_offline_aegafi_trial(self, scenario_name: str, trial_id: int, seed: int = None) -> Dict:
        """
        Run AEGAfi in OFFLINE mode (global planning ONLY)
        - Generates one optimal path at start
        - Measures planning time separately from execution
        - Simple waypoint following (no reactive avoidance)
        """
        if seed is not None:
            random.seed(seed)
            np.random.seed(seed)
        
        start, goal, static_obs, dynamic_obs = ScenarioBuilder.create_scenario(scenario_name)
        all_obs = static_obs + dynamic_obs
        
        usv = USV(start[0], start[1], theta=0.0, config=self.config)
        
        # OFFLINE: Plan once at start with generous timeout
        bounds = [(-1.0, 13.0), (-4.0, 4.0)]
        planner = AEGAfi(bounds, pop_size=100, generations=200, elite_frac=0.25)
        
        t0 = time.time()
        global_path = planner.plan(start, goal, all_obs, timeout=20.0)  # 20 second timeout for offline
        planning_time = time.time() - t0
        
        trajectory = [usv.get_position()]
        collided = False
        reached = False
        execution_time = 0.0
        collision_obs = None
        current_waypoint_idx = 1
        
        # Execute planned path
        for step in range(self.max_steps):
            # Update dynamic obstacles (even though path was precomputed)
            for dobs in dynamic_obs:
                dobs.step(self.dt)
            
            # Simple waypoint following (no reactive control)
            if current_waypoint_idx >= len(global_path):
                current_waypoint_idx = len(global_path) - 1
            
            target = global_path[current_waypoint_idx]
            dx = target[0] - usv.x
            dy = target[1] - usv.y
            dist_to_target = math.hypot(dx, dy)
            
            # Heading control
            desired_heading = math.atan2(dy, dx)
            heading_error = USV._angle_diff(desired_heading, usv.theta)
            yaw_cmd = np.clip(heading_error * 1.5 / self.dt, 
                             -self.config.max_yaw_rate, self.config.max_yaw_rate)
            
            # Throttle
            throttle = 1.0 if dist_to_target > 0.6 else 0.3
            
            t0 = time.time()
            usv.set_control(throttle, yaw_cmd)
            usv.step(self.dt)
            execution_time += time.time() - t0
            trajectory.append(usv.get_position())
            
            # Advance waypoint if close
            if dist_to_target < 0.6 and current_waypoint_idx < len(global_path) - 1:
                current_waypoint_idx += 1
            
            # Check collision
            collided, collision_obs = usv.check_collision(all_obs)
            if collided:
                break
            
            # Check goal
            if usv.distance_to(goal[0], goal[1]) < self.config.goal_threshold:
                reached = True
                break
        
        # Calculate metrics
        path_len = calculate_path_length(trajectory)
        smoothness = calculate_path_smoothness(trajectory)
        clearance = calculate_clearance(trajectory, static_obs)
        heading_stability = calculate_heading_stability(usv.heading_history)
        
        trial_result = {
            'mode': 'Offline_AEGAfi',
            'scenario': scenario_name,
            'trial_id': trial_id,
            'success': reached,
            'collision': collided,
            'steps': len(trajectory),
            'path_length': path_len,
            'path_smoothness': smoothness,
            'min_clearance': clearance,
            'heading_stability': heading_stability,
            'planning_cpu_time': planning_time,  # Separated from execution
            'execution_cpu_time': execution_time,
            'total_cpu_time': planning_time + execution_time,
            'avg_planning_time_per_step': planning_time / len(trajectory) if len(trajectory) > 0 else 0,
            'max_speed': max(usv.velocity_history) if usv.velocity_history else 0.0,
            'avg_speed': np.mean(usv.velocity_history) if usv.velocity_history else 0.0,
            'ga_generations': planner.metrics.generation_count,
        }
        
        return trial_result

    def run_hybrid_trial(self, scenario_name: str, trial_id: int, seed: int = None) -> Dict:
        """
        Run HYBRID mode (Offline AEGAfi + Online APF)
        - AEGAfi generates global path offline
        - APF provides local reactive control during execution
        - Combines strengths of both
        """
        if seed is not None:
            random.seed(seed)
            np.random.seed(seed)
        
        start, goal, static_obs, dynamic_obs = ScenarioBuilder.create_scenario(scenario_name)
        all_obs = static_obs + dynamic_obs
        
        usv = USV(start[0], start[1], theta=0.0, config=self.config)
        
        # Phase 1: Offline global planning with AEGAfi
        bounds = [(-1.0, 13.0), (-4.0, 4.0)]
        global_planner = AEGAfi(bounds, pop_size=100, generations=200, elite_frac=0.25)
        
        t0 = time.time()
        global_path = global_planner.plan(start, goal, all_obs, timeout=20.0)
        planning_time = time.time() - t0
        
        # Phase 2: Online execution with APF local control
        local_planner = ImprovedAPF()
        trajectory = [usv.get_position()]
        collided = False
        reached = False
        execution_time = 0.0
        collision_obs = None
        
        for step in range(self.max_steps):
            # Update dynamic obstacles
            for dobs in dynamic_obs:
                dobs.step(self.dt)
            
            # Use APF with global path as guidance
            # Target the next waypoint on the global path
            closest_idx = 0
            closest_dist = float('inf')
            for i, waypoint in enumerate(global_path):
                dist = math.hypot(usv.x - waypoint[0], usv.y - waypoint[1])
                if dist < closest_dist:
                    closest_dist = dist
                    closest_idx = i
            
            # Use waypoint ahead on path as APF target
            target_idx = min(closest_idx + 2, len(global_path) - 1)
            path_target = global_path[target_idx]
            
            # APF control toward path target
            t0 = time.time()
            throttle, yaw_cmd = local_planner.compute_control(usv, path_target, all_obs, self.dt)
            execution_time += time.time() - t0
            
            # Execute
            usv.set_control(throttle, yaw_cmd)
            usv.step(self.dt)
            trajectory.append(usv.get_position())
            
            # Check collision
            collided, collision_obs = usv.check_collision(all_obs)
            if collided:
                break
            
            # Check goal
            if usv.distance_to(goal[0], goal[1]) < self.config.goal_threshold:
                reached = True
                break
        
        # Calculate metrics
        path_len = calculate_path_length(trajectory)
        smoothness = calculate_path_smoothness(trajectory)
        clearance = calculate_clearance(trajectory, static_obs)
        heading_stability = calculate_heading_stability(usv.heading_history)
        
        trial_result = {
            'mode': 'Hybrid',
            'scenario': scenario_name,
            'trial_id': trial_id,
            'success': reached,
            'collision': collided,
            'steps': len(trajectory),
            'path_length': path_len,
            'path_smoothness': smoothness,
            'min_clearance': clearance,
            'heading_stability': heading_stability,
            'planning_cpu_time': planning_time,
            'execution_cpu_time': execution_time,
            'total_cpu_time': planning_time + execution_time,
            'avg_planning_time_per_step': (planning_time + execution_time) / len(trajectory) if len(trajectory) > 0 else 0,
            'max_speed': max(usv.velocity_history) if usv.velocity_history else 0.0,
            'avg_speed': np.mean(usv.velocity_history) if usv.velocity_history else 0.0,
        }
        
        return trial_result


class ComprehensiveExperimentRunner:
    """Run all three modes comprehensively"""
    
    def __init__(self, output_dir: str = 'results'):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)
        self.config = SimulationConfig()
        self.trial_runner = TrialRunner(self.config)
        self.all_results: List[Dict] = []
        
    def run_all_experiments(self, scenarios: List[str], trials: int = 30) -> List[Dict]:
        """Run all three modes for all scenarios"""
        modes = ['Online_APF', 'Offline_AEGAfi', 'Hybrid']
        
        total_runs = len(scenarios) * len(modes) * trials
        pbar = tqdm(total=total_runs, desc="All Experiments")
        
        for scenario in scenarios:
            for mode in modes:
                print(f"\n{'='*60}")
                print(f"Scenario: {scenario.upper()} | Mode: {mode}")
                print(f"{'='*60}")
                
                trial_results = []
                
                for trial_id in range(trials):
                    if mode == 'Online_APF':
                        result = self.trial_runner.run_online_apf_trial(scenario, trial_id, seed=trial_id * 1000)
                    elif mode == 'Offline_AEGAfi':
                        result = self.trial_runner.run_offline_aegafi_trial(scenario, trial_id, seed=trial_id * 1000)
                    else:  # Hybrid
                        result = self.trial_runner.run_hybrid_trial(scenario, trial_id, seed=trial_id * 1000)
                    
                    trial_results.append(result)
                    self.all_results.append(result)
                    pbar.update(1)
                
                # Save per-scenario results
                self._save_trial_csv(trial_results, scenario, mode)
                self._print_summary(trial_results)
        
        pbar.close()
        
        # Save overall summary
        self._save_summary_csv()
        self._save_comparison_report()
        
        return self.all_results

    def _save_trial_csv(self, results: List[Dict], scenario: str, mode: str):
        """Save trial-level results"""
        if not results:
            return
        
        filename = self.output_dir / f"{mode}_{scenario}_trials.csv"
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
        
        scenarios = sorted(set(r['scenario'] for r in self.all_results))
        modes = sorted(set(r['mode'] for r in self.all_results))
        
        for scenario in scenarios:
            for mode in modes:
                subset = [r for r in self.all_results 
                         if r['scenario'] == scenario and r['mode'] == mode]
                
                if not subset:
                    continue
                
                success_rate = np.mean([r['success'] for r in subset])
                collision_rate = np.mean([r['collision'] for r in subset])
                
                path_lengths = [r['path_length'] for r in subset if r['success']]
                avg_path_length = np.mean(path_lengths) if path_lengths else 0.0
                std_path_length = np.std(path_lengths) if path_lengths else 0.0
                
                smoothness_vals = [r['path_smoothness'] for r in subset if r['success']]
                avg_smoothness = np.mean(smoothness_vals) if smoothness_vals else 0.0
                
                clearance_vals = [r['min_clearance'] for r in subset]
                avg_clearance = np.mean(clearance_vals)
                
                if 'planning_cpu_time' in subset[0]:
                    planning_times = [r['planning_cpu_time'] for r in subset]
                    avg_planning = np.mean(planning_times)
                else:
                    avg_planning = 0.0
                
                if 'execution_cpu_time' in subset[0]:
                    exec_times = [r['execution_cpu_time'] for r in subset]
                    avg_exec = np.mean(exec_times)
                    total_times = [r['total_cpu_time'] for r in subset]
                    avg_total = np.mean(total_times)
                else:
                    avg_exec = 0.0
                    avg_total = avg_planning
                
                steps = [r['steps'] for r in subset]
                avg_steps = np.mean(steps)
                
                summary_data.append({
                    'scenario': scenario,
                    'mode': mode,
                    'trials': len(subset),
                    'success_rate': success_rate,
                    'collision_rate': collision_rate,
                    'avg_path_length': avg_path_length,
                    'std_path_length': std_path_length,
                    'avg_path_smoothness': avg_smoothness,
                    'avg_min_clearance': avg_clearance,
                    'avg_planning_cpu_time': avg_planning,
                    'avg_execution_cpu_time': avg_exec,
                    'avg_total_cpu_time': avg_total,
                    'avg_steps': avg_steps,
                })
        
        filename = self.output_dir / 'summary.csv'
        with open(filename, 'w', newline='') as f:
            if summary_data:
                writer = csv.DictWriter(f, fieldnames=summary_data[0].keys())
                writer.writeheader()
                writer.writerows(summary_data)

    def _save_comparison_report(self):
        """Save detailed comparison report"""
        if not self.all_results:
            return
        
        report = []
        report.append("="*80)
        report.append("COMPREHENSIVE MARITIME PATH PLANNING COMPARISON")
        report.append("Three Modes: Online APF vs Offline AEGAfi vs Hybrid")
        report.append(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        report.append("="*80)
        report.append("")
        
        scenarios = sorted(set(r['scenario'] for r in self.all_results))
        
        for scenario in scenarios:
            report.append(f"\n{'='*80}")
            report.append(f"SCENARIO: {scenario.upper()}")
            report.append(f"{'='*80}\n")
            
            modes_to_check = ['Online_APF', 'Offline_AEGAfi', 'Hybrid']
            
            for mode in modes_to_check:
                results = [r for r in self.all_results 
                          if r['scenario'] == scenario and r['mode'] == mode]
                
                if not results:
                    continue
                
                report.append(f"{mode}:")
                report.append("-" * 60)
                
                success_rate = np.mean([r['success'] for r in results])
                collision_rate = np.mean([r['collision'] for r in results])
                
                successful = [r for r in results if r['success']]
                if successful:
                    path_lengths = [r['path_length'] for r in successful]
                    smoothness = [r['path_smoothness'] for r in successful]
                    
                    report.append(f"  Success Rate:          {success_rate*100:.1f}%")
                    report.append(f"  Collision Rate:        {collision_rate*100:.1f}%")
                    report.append(f"  Avg Path Length:       {np.mean(path_lengths):.3f}m (±{np.std(path_lengths):.3f})")
                    report.append(f"  Avg Path Smoothness:   {np.mean(smoothness):.3f} rad (±{np.std(smoothness):.3f})")
                else:
                    report.append(f"  Success Rate:          {success_rate*100:.1f}%")
                    report.append(f"  Collision Rate:        {collision_rate*100:.1f}%")
                
                if 'planning_cpu_time' in results[0]:
                    planning_times = [r['planning_cpu_time'] for r in results]
                    report.append(f"  Avg Planning Time:     {np.mean(planning_times):.4f}s (±{np.std(planning_times):.4f})")
                
                if 'execution_cpu_time' in results[0]:
                    exec_times = [r['execution_cpu_time'] for r in results]
                    report.append(f"  Avg Execution Time:    {np.mean(exec_times):.4f}s (±{np.std(exec_times):.4f})")
                    total_times = [r['total_cpu_time'] for r in results]
                    report.append(f"  Avg Total Time:        {np.mean(total_times):.4f}s (±{np.std(total_times):.4f})")
                
                clearances = [r['min_clearance'] for r in results]
                steps = [r['steps'] for r in results]
                
                report.append(f"  Avg Min Clearance:     {np.mean(clearances):.3f}m (±{np.std(clearances):.3f})")
                report.append(f"  Avg Steps:             {np.mean(steps):.1f} (±{np.std(steps):.1f})")
                report.append("")
        
        # Summary comparison table
        report.append(f"\n{'='*80}")
        report.append("SUMMARY COMPARISON TABLE")
        report.append(f"{'='*80}\n")
        
        report.append("Success Rates by Scenario and Mode:")
        report.append("-" * 80)
        
        scenarios = sorted(set(r['scenario'] for r in self.all_results))
        modes_to_check = ['Online_APF', 'Offline_AEGAfi', 'Hybrid']
        
        report.append(f"{'Scenario':<15} {'Online APF':<15} {'Offline AEGAfi':<20} {'Hybrid':<15}")
        report.append("-" * 80)
        
        for scenario in scenarios:
            row = [scenario]
            for mode in modes_to_check:
                subset = [r for r in self.all_results 
                         if r['scenario'] == scenario and r['mode'] == mode]
                if subset:
                    success_rate = np.mean([r['success'] for r in subset])
                    row.append(f"{success_rate*100:>6.1f}%")
                else:
                    row.append("N/A")
            report.append(f"{row[0]:<15} {row[1]:<15} {row[2]:<20} {row[3]:<15}")
        
        report_text = "\n".join(report)
        
        with open(self.output_dir / 'comparison_report.txt', 'w') as f:
            f.write(report_text)
        
        print(report_text)

    def _print_summary(self, results: List[Dict]):
        """Print summary statistics"""
        success_rate = np.mean([r['success'] for r in results])
        collision_rate = np.mean([r['collision'] for r in results])
        
        successful = [r for r in results if r['success']]
        if successful:
            avg_len = np.mean([r['path_length'] for r in successful])
            print(f"Success: {success_rate*100:.1f}% | Collision: {collision_rate*100:.1f}% | Avg Path: {avg_len:.3f}m")
        else:
            print(f"Success: {success_rate*100:.1f}% | Collision: {collision_rate*100:.1f}%")
        
        if 'planning_cpu_time' in results[0]:
            planning_times = [r['planning_cpu_time'] for r in results]
            print(f"Avg Planning Time: {np.mean(planning_times):.4f}s")
        
        if 'execution_cpu_time' in results[0]:
            exec_times = [r['execution_cpu_time'] for r in results]
            print(f"Avg Execution Time: {np.mean(exec_times):.4f}s")


if __name__ == '__main__':
    print("Starting Comprehensive Maritime Path Planning Experiments...")
    print("="*80)
    print("Three Modes:")
    print("  1. Online APF: Real-time reactive control (no planning)")
    print("  2. Offline AEGAfi: Global path planning upfront (no reactive control)")
    print("  3. Hybrid: Offline planning + online reactive control")
    print("="*80)
    print()
    
    runner = ComprehensiveExperimentRunner(output_dir='results')
    scenarios = ['open', 'channel', 'clutter', 'dynamic']
    results = runner.run_all_experiments(scenarios, trials=30)
    
    print("\n" + "="*80)
    print("EXPERIMENTS COMPLETE!")
    print("Results saved to 'results/' directory:")
    print("  - summary.csv: Aggregate statistics")
    print("  - comparison_report.txt: Detailed analysis")
    print("  - [mode]_[scenario]_trials.csv: Per-trial data")
    print("="*80)