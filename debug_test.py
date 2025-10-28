# debug_test.py
# Quick diagnostic to identify why APF isn't moving

import math
import numpy as np
from simulator import USV, Obstacle, SimulationConfig
from planners import ImprovedAPF

# Create simple test scenario
config = SimulationConfig()
usv = USV(0.0, 0.0, theta=0.0, config=config)
goal = (5.0, 0.0)
obstacles = [Obstacle(2.5, 1.0, 0.5)]

planner = ImprovedAPF()

print("="*60)
print("DEBUG: APF Movement Test")
print("="*60)
print(f"Start position: {usv.get_position()}")
print(f"Goal position: {goal}")
print(f"Initial velocity: {usv.get_velocity()}")
print()

# Run 20 steps manually and print debug info
for step in range(20):
    # Get control from planner
    throttle, yaw_cmd = planner.compute_control(usv, goal, obstacles, config.dt)
    
    # Print debug info
    print(f"Step {step:2d}:")
    print(f"  Throttle: {throttle:7.4f} | Yaw cmd: {yaw_cmd:7.4f} rad")
    print(f"  Pos: ({usv.x:6.3f}, {usv.y:6.3f}) | Heading: {math.degrees(usv.theta):6.1f}°")
    print(f"  Velocity: {usv.v:6.4f} m/s | v_cmd: {usv.v_cmd:6.4f}")
    
    # Apply control
    usv.set_control(throttle, yaw_cmd)
    print(f"  After set_control - v_cmd: {usv.v_cmd:6.4f} | omega_cmd: {usv.omega_cmd:6.4f}")
    
    # Step simulation
    usv.step(config.dt)
    print(f"  After step - v: {usv.v:6.4f} | Position: ({usv.x:6.3f}, {usv.y:6.3f})")
    print()
    
    # Check if reached goal
    dist = usv.distance_to(goal[0], goal[1])
    if dist < 0.6:
        print(f"GOAL REACHED! Distance: {dist:.3f}m")
        break

print("="*60)
print(f"Final position: {usv.get_position()}")
print(f"Final velocity: {usv.v:.4f} m/s")
print(f"Distance to goal: {usv.distance_to(goal[0], goal[1]):.3f} m")
print(f"Total steps: {len(usv.position_history)}")
print("="*60)