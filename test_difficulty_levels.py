# Quick test to see algorithm failures at different difficulties

from scenario_generator_challenging import ChallengeScenarioBuilder
from simulator import USV, SimulationConfig
from planners import ImprovedAPF, AEGAfi
import numpy as np

config = SimulationConfig()

print("="*80)
print("DIFFICULTY LEVEL TESTING - Finding Algorithm Limits")
print("="*80)

scenarios = ['open', 'channel', 'clutter', 'dynamic', 'maze', 'crowded']
difficulties = ['easy', 'medium', 'hard', 'extreme']

for scenario in scenarios:
    print(f"\n{scenario.upper()}:")
    print("-" * 60)
    
    for difficulty in difficulties:
        # Create scenario
        start, goal, static_obs, dynamic_obs = ChallengeScenarioBuilder.create_scenario(scenario, difficulty)
        all_obs = static_obs + dynamic_obs
        
        # Test APF (single trial)
        apf_results = []
        for trial in range(3):
            usv = USV(start[0], start[1], config=config)
            planner = ImprovedAPF()
            
            for step in range(500):
                for dobs in dynamic_obs:
                    dobs.step(0.1)
                
                throttle, yaw_cmd = planner.compute_control(usv, goal, all_obs, 0.1)
                usv.set_control(throttle, yaw_cmd)
                usv.step(0.1)
                
                collided, _ = usv.check_collision(all_obs)
                if collided:
                    apf_results.append('COLLISION')
                    break
                
                if usv.distance_to(goal[0], goal[1]) < 0.6:
                    apf_results.append('SUCCESS')
                    break
            else:
                apf_results.append('TIMEOUT')
        
        # Test AEGAfi (single trial, offline)
        usv = USV(start[0], start[1], config=config)
        bounds = [(-1.0, 13.0), (-4.0, 4.0)]
        planner = AEGAfi(bounds, pop_size=100, generations=200)
        
        import time
        t0 = time.time()
        path = planner.plan(start, goal, all_obs, timeout=5.0)
        planning_time = time.time() - t0
        
        # Follow path
        aegafi_result = None
        current_idx = 1
        for step in range(500):
            if current_idx >= len(path):
                current_idx = len(path) - 1
            
            target = path[current_idx]
            dx, dy = target[0] - usv.x, target[1] - usv.y
            desired_heading = np.arctan2(dy, dx)
            heading_error = USV._angle_diff(desired_heading, usv.theta)
            yaw_cmd = np.clip(heading_error * 1.5 / 0.1, -config.max_yaw_rate, config.max_yaw_rate)
            throttle = 1.0 if np.hypot(dx, dy) > 0.6 else 0.3
            
            usv.set_control(throttle, yaw_cmd)
            usv.step(0.1)
            
            if np.hypot(dx, dy) < 0.6 and current_idx < len(path) - 1:
                current_idx += 1
            
            collided, _ = usv.check_collision(all_obs)
            if collided:
                aegafi_result = 'COLLISION'
                break
            
            if usv.distance_to(goal[0], goal[1]) < 0.6:
                aegafi_result = 'SUCCESS'
                break
        else:
            aegafi_result = 'TIMEOUT'
        
        apf_success = apf_results.count('SUCCESS')
        print(f"  {difficulty.upper():8} | APF: {apf_success}/3 | AEGAfi: {aegafi_result} (plan: {planning_time:.2f}s)")

print("\n" + "="*80)