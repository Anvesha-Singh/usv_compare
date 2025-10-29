# planners_colregs_aware.py
# Updated planners with COLREGs compliance

import math
import time
import random
import numpy as np
from typing import List, Tuple, Optional, Dict
from dataclasses import dataclass, field

from colregs import COLREGsChecker, EncounterType


@dataclass
class APFMetrics:
    """Metrics for Improved APF algorithm - UPDATED WITH COLREGs"""
    planning_times: List[float] = field(default_factory=list)
    attractive_forces: List[float] = field(default_factory=list)
    repulsive_forces: List[float] = field(default_factory=list)
    total_forces: List[float] = field(default_factory=list)
    stuck_events: int = 0
    escape_perturbations: int = 0
    force_computation_count: int = 0
    # NEW: COLREGs metrics
    colregs_violations: int = 0
    colregs_corrections: int = 0
    encounters_detected: int = 0


class ImprovedAPFColREGs:
    """
    Improved APF with COLREGs awareness
    Adds COLREGs-compliant evasive force to standard APF
    """
    
    def __init__(self, params: Optional[Dict] = None):
        default_params = {
            'k_att': 2.5,
            'k_rep': 1.5,
            'd0': 1.2,
            'escape_strength': 1.0,
            'goal_threshold': 0.5,
            'max_yaw_rate': math.radians(30),
            'max_speed': 1.2,
            'stuck_window': 15,
            'stuck_threshold': 0.10,
            'repulsion_inside_penalty': 30.0,
            # NEW: COLREGs parameters
            'colregs_force_magnitude': 1.5,  # Strength of COLREGs evasion force
            'colregs_activation_distance': 3.0,  # Distance at which COLREGs force activates
        }
        self.params = default_params if params is None else {**default_params, **params}
        self.position_history: List[Tuple[float, float]] = []
        self.metrics = APFMetrics()
        self.colregs_checker = COLREGsChecker()

    def attractive_force(self, pos: Tuple[float, float], goal: Tuple[float, float]) -> Tuple[float, float]:
        """Calculate attractive force toward goal"""
        dx = goal[0] - pos[0]
        dy = goal[1] - pos[1]
        dist = math.hypot(dx, dy) + 1e-6
        
        fx = self.params['k_att'] * dx
        fy = self.params['k_att'] * dy
        
        return fx, fy

    def repulsive_force(self, pos: Tuple[float, float], obstacles: List) -> Tuple[float, float]:
        """Calculate repulsive forces from obstacles"""
        fx = 0.0
        fy = 0.0
        
        for obs in obstacles:
            dx = pos[0] - obs.x
            dy = pos[1] - obs.y
            dist = math.hypot(dx, dy) - obs.r
            
            if dist < self.params['d0']:
                if dist <= 0:
                    push_magnitude = self.params['repulsion_inside_penalty']
                    unit_dist = math.hypot(dx, dy) + 1e-6
                    fx += (dx / unit_dist) * push_magnitude
                    fy += (dy / unit_dist) * push_magnitude
                else:
                    rep_magnitude = self.params['k_rep'] / (dist + 0.1)
                    unit_dist = math.hypot(dx, dy) + 1e-6
                    fx += (dx / unit_dist) * rep_magnitude
                    fy += (dy / unit_dist) * rep_magnitude
        
        return fx, fy

    def colregs_force(self, usv_pos: Tuple[float, float], 
                     usv_heading: float, usv_speed: float,
                     dynamic_obstacles: List) -> Tuple[float, float]:
        """
        Calculate COLREGs-compliant evasion force
        Detects collision encounters and applies appropriate evasion force
        """
        fx_colregs = 0.0
        fy_colregs = 0.0
        
        for dobs in dynamic_obstacles:
            # Get distance to dynamic obstacle
            dx = dobs.x - usv_pos[0]
            dy = dobs.y - usv_pos[1]
            distance = math.hypot(dx, dy)
            
            # Only apply COLREGs force if close enough
            if distance < self.params['colregs_activation_distance']:

                # Classify encounter
                encounter = self.colregs_checker.classify_encounter(
                    usv_pos, usv_heading, usv_speed,
                    (dobs.x, dobs.y), 
                    dobs.get_heading(),  
                    dobs.get_speed()    
                )
                
                if encounter != EncounterType.SAFE:
                    self.metrics.encounters_detected += 1
                    
                    # Get COLREGs-compliant heading
                    colregs_heading = self.colregs_checker.get_colregs_correction_heading(
                        usv_pos, 
                        usv_heading, 
                        (dobs.x, dobs.y), 
                        encounter  
                    )
                    
                    # Convert heading to force vector
                    colregs_force_mag = self.params['colregs_force_magnitude']
                    fx_colregs += colregs_force_mag * math.cos(colregs_heading)
                    fy_colregs += colregs_force_mag * math.sin(colregs_heading)
                    self.metrics.colregs_corrections += 1
        
        return fx_colregs, fy_colregs

    def detect_stuck(self) -> bool:
        """Detect if stuck in local minimum"""
        if len(self.position_history) < self.params['stuck_window']:
            return False
        
        recent = self.position_history[-self.params['stuck_window']:]
        movement = 0.0
        for i in range(1, len(recent)):
            movement += math.hypot(recent[i][0] - recent[i-1][0],
                                  recent[i][1] - recent[i-1][1])
        
        is_stuck = movement < self.params['stuck_threshold']
        if is_stuck:
            self.metrics.stuck_events += 1
        return is_stuck

    def escape_perturbation(self, fx: float, fy: float) -> Tuple[float, float]:
        """Add perpendicular perturbation to escape local minimum"""
        angle = math.atan2(fy, fx)
        perp_angle = angle + (math.pi / 2) * (1 if random.random() < 0.5 else -1)
        
        perp_fx = self.params['escape_strength'] * math.cos(perp_angle)
        perp_fy = self.params['escape_strength'] * math.sin(perp_angle)
        
        self.metrics.escape_perturbations += 1
        return perp_fx, perp_fy

    def compute_control(self, usv, goal: Tuple[float, float], 
                       obstacles: List, dynamic_obstacles: List, dt: float) -> Tuple[float, float]:
        """
        Compute control commands using COLREGs-aware APF
        
        Args:
            usv: USV object
            goal: Goal position
            obstacles: Static obstacles
            dynamic_obstacles: Dynamic obstacles (moving vessels)
            dt: Time step
        """
        t0 = time.time()
        
        pos = (usv.x, usv.y)
        self.position_history.append(pos)
        if len(self.position_history) > 200:
            self.position_history.pop(0)
        
        # Calculate standard APF forces
        fax, fay = self.attractive_force(pos, goal)
        frx, fry = self.repulsive_force(pos, obstacles)
        
        self.metrics.attractive_forces.append(math.hypot(fax, fay))
        self.metrics.repulsive_forces.append(math.hypot(frx, fry))
        
        # Calculate COLREGs force (NEW)
        fcx, fcy = self.colregs_force(pos, usv.theta, usv.v, dynamic_obstacles)
        
        # Combine all forces
        fx = fax + frx + fcx
        fy = fay + fry + fcy
        
        # Escape from local minimum if stuck
        if self.detect_stuck():
            perp_fx, perp_fy = self.escape_perturbation(fx, fy)
            fx += perp_fx
            fy += perp_fy
        
        total_force = math.hypot(fx, fy)
        self.metrics.total_forces.append(total_force)
        
        # Convert force to control commands
        desired_heading = math.atan2(fy, fx)
        heading_error = self._angle_diff(desired_heading, usv.theta)
        
        yaw_cmd = np.clip(heading_error * 1.5 / dt, 
                         -self.params['max_yaw_rate'], 
                         self.params['max_yaw_rate'])
        
        dist_to_goal = math.hypot(goal[0] - pos[0], goal[1] - pos[1])
        base_throttle = np.clip(total_force / 3.0, 0.2, 1.0)
        
        if dist_to_goal < 2.0:
            base_throttle *= (dist_to_goal / 2.0)
        
        throttle = base_throttle
        
        self.metrics.planning_times.append(time.time() - t0)
        
        return throttle, yaw_cmd

    @staticmethod
    def _angle_diff(a: float, b: float) -> float:
        """Calculate shortest angle difference"""
        d = a - b
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        return d

    def reset(self):
        """Reset planner state"""
        self.position_history = []
        self.metrics = APFMetrics()


# NOTE: AEGAfi COLREGs integration would require updating the fitness function
# to include COLREGs_Violation_Penalty as shown in your Phase 1 description.
# The fitness function would become:
#
# fitness = (w1 * path_length + 
#            w2 * collision_penalty + 
#            w3 * smoothness + 
#            w4 * colregs_violation_penalty)
#
# Where colregs_violation_penalty checks each waypoint pair for COLREGs violations.
