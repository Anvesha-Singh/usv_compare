# planners.py - WITH COLREGS COMPLIANCE (FIXED)

import math
import time
import random
import numpy as np
from typing import List, Tuple, Optional, Dict
from dataclasses import dataclass, field


# ==================== COLREGS UTILITIES ====================

def get_relative_bearing(own_x, own_y, own_heading, other_x, other_y):
    """Calculate relative bearing from own ship to other ship in degrees (0-360)"""
    dx = other_x - own_x
    dy = other_y - own_y
    abs_bearing = math.atan2(dy, dx)
    rel_bearing = abs_bearing - own_heading
    # Normalize to 0-360
    bearing_deg = math.degrees(rel_bearing)
    return (bearing_deg + 360) % 360


def colregs_encounter_type(own_x, own_y, own_heading, own_speed, 
                           other_x, other_y, other_vx, other_vy):
    """
    Determine COLREGs encounter type based on relative position and courses.

    Args:
        own_x, own_y: Own ship position
        own_heading: Own ship heading (radians)
        own_speed: Own ship speed (scalar)
        other_x, other_y: Other ship position
        other_vx, other_vy: Other ship velocity components

    Returns: 'head-on', 'crossing-starboard', 'crossing-port', 'overtaking', or 'undefined'
    """
    rel_bearing = get_relative_bearing(own_x, own_y, own_heading, other_x, other_y)

    # Calculate own course from heading and speed
    own_vx = own_speed * math.cos(own_heading)
    own_vy = own_speed * math.sin(own_heading)

    # Calculate course angles
    own_course = math.degrees(math.atan2(own_vy, own_vx)) % 360
    other_course = math.degrees(math.atan2(other_vy, other_vx)) % 360
    course_diff = abs(own_course - other_course)
    if course_diff > 180:
        course_diff = 360 - course_diff

    # Head-on: reciprocal courses (150-210 deg difference) and bearing ahead (330-30 deg)
    if course_diff > 150 and (rel_bearing < 30 or rel_bearing > 330):
        return 'head-on'

    # Overtaking: approaching from stern sector (135-225 deg)
    if 135 <= rel_bearing <= 225:
        return 'overtaking'

    # Crossing from starboard (0-112.5 deg)
    if 0 < rel_bearing < 112.5:
        return 'crossing-starboard'

    # Crossing from port (247.5-360 deg)
    if 247.5 < rel_bearing < 360:
        return 'crossing-port'

    return 'undefined'


def check_colregs_compliance(own_x, own_y, own_heading, own_speed,
                             dynamic_obstacles: List) -> Dict:
    """
    Check COLREGs compliance for current state against dynamic obstacles.

    Args:
        own_x, own_y: Own ship position
        own_heading: Own ship heading (radians)
        own_speed: Own ship speed (scalar)
        dynamic_obstacles: List of dynamic obstacles with vx, vy attributes

    Returns dictionary with encounter information and compliance status.
    """
    encounters = []
    total_violations = 0

    for obs in dynamic_obstacles:
        if not hasattr(obs, 'vx') or not hasattr(obs, 'vy'):
            continue

        # Skip if obstacle is stationary
        if abs(obs.vx) < 0.01 and abs(obs.vy) < 0.01:
            continue

        encounter_type = colregs_encounter_type(
            own_x, own_y, own_heading, own_speed,
            obs.x, obs.y, obs.vx, obs.vy
        )

        if encounter_type != 'undefined':
            # Simple compliance check: are we maintaining safe distance?
            dist = math.hypot(own_x - obs.x, own_y - obs.y)
            safe_distance = 2.0  # meters

            compliant = dist > safe_distance
            if not compliant:
                total_violations += 1

            encounters.append({
                'obstacle': obs,
                'type': encounter_type,
                'distance': dist,
                'compliant': compliant,
                'relative_bearing': get_relative_bearing(own_x, own_y, own_heading, obs.x, obs.y)
            })

    return {
        'encounters': encounters,
        'total_encounters': len(encounters),
        'violations': total_violations,
        'compliant': total_violations == 0
    }


# ==================== METRICS CLASSES ====================

@dataclass
class APFMetrics:
    """Metrics for Improved APF algorithm"""
    planning_times: List[float] = field(default_factory=list)
    attractive_forces: List[float] = field(default_factory=list)
    repulsive_forces: List[float] = field(default_factory=list)
    total_forces: List[float] = field(default_factory=list)
    stuck_events: int = 0
    escape_perturbations: int = 0
    force_computation_count: int = 0
    # COLREGs metrics
    colregs_encounters: List[Dict] = field(default_factory=list)
    colregs_violations: int = 0


@dataclass
class GAMetrics:
    """Metrics for AEGAfi algorithm"""
    generation_fitnesses: List[float] = field(default_factory=list)
    best_fitness_per_gen: List[float] = field(default_factory=list)
    population_diversity: List[float] = field(default_factory=list)
    mutation_rates: List[float] = field(default_factory=list)
    elite_fractions: List[float] = field(default_factory=list)
    replan_times: List[float] = field(default_factory=list)
    generation_count: int = 0
    # COLREGs metrics
    colregs_encounters: List[Dict] = field(default_factory=list)
    colregs_violations: int = 0


# ==================== APF PLANNER ====================

class ImprovedAPF:
    """
    Improved Artificial Potential Field planner with COLREGs awareness
    """

    def __init__(self, params: Optional[Dict] = None):
        """Initialize APF with better tuned parameters"""
        default_params = {
            'k_att': 2.5,
            'k_rep': 1.5,
            'd0': 1.2,
            'escape_strength': 1.2,
            'goal_threshold': 0.5,
            'max_yaw_rate': math.radians(30),
            'max_speed': 1.2,
            'stuck_window': 15,
            'stuck_threshold': 0.10,
            'repulsion_inside_penalty': 30.0,
        }

        self.params = default_params if params is None else {**default_params, **params}
        self.position_history: List[Tuple[float, float]] = []
        self.metrics = APFMetrics()

    def attractive_force(self, pos: Tuple[float, float], goal: Tuple[float, float]) -> Tuple[float, float]:
        """Calculate attractive force toward goal"""
        dx = goal[0] - pos[0]
        dy = goal[1] - pos[1]
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
                       obstacles: List, dt: float) -> Tuple[float, float]:
        """
        Compute control commands using corrected APF with COLREGs checking

        Args:
            usv: USV object with attributes: x, y, theta, v (speed)
            goal: Goal position (x, y)
            obstacles: List of obstacles
            dt: Timestep
        """
        t0 = time.time()
        pos = (usv.x, usv.y)
        self.position_history.append(pos)

        if len(self.position_history) > 200:
            self.position_history.pop(0)

        # Check COLREGs compliance - use usv.v (speed) instead of vx, vy
        dynamic_obs = [obs for obs in obstacles if hasattr(obs, 'vx') and hasattr(obs, 'vy')]
        if dynamic_obs:
            colregs_status = check_colregs_compliance(
                usv.x, usv.y, usv.theta, usv.v, dynamic_obs
            )
            self.metrics.colregs_encounters.append(colregs_status)
            if not colregs_status['compliant']:
                self.metrics.colregs_violations += 1

        # Calculate forces
        fax, fay = self.attractive_force(pos, goal)
        frx, fry = self.repulsive_force(pos, obstacles)

        self.metrics.attractive_forces.append(math.hypot(fax, fay))
        self.metrics.repulsive_forces.append(math.hypot(frx, fry))
        self.metrics.force_computation_count += 1

        # Combine forces
        fx = fax + frx
        fy = fay + fry

        # Escape from local minimum if stuck
        if self.detect_stuck():
            perp_fx, perp_fy = self.escape_perturbation(fx, fy)
            fx += perp_fx
            fy += perp_fy

        total_force = math.hypot(fx, fy)
        self.metrics.total_forces.append(total_force)

        # Calculate desired heading from net force
        desired_heading = math.atan2(fy, fx)

        # Calculate heading error
        heading_error = self._angle_diff(desired_heading, usv.theta)

        # Yaw rate command
        yaw_cmd = np.clip(heading_error * 1.5 / dt,
                         -self.params['max_yaw_rate'],
                         self.params['max_yaw_rate'])

        # Throttle
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


# ==================== GA PLANNER ====================

class AEGAfi:
    """
    Adaptive-Elite Genetic Algorithm with Fuzzy Inference and COLREGs awareness
    """

    def __init__(self, bounds: List[Tuple[float, float]],
                 pop_size: int = 40,
                 generations: int = 60,
                 elite_frac: float = 0.2):
        self.bounds = bounds
        self.pop_size = pop_size
        self.generations = generations
        self.base_elite_frac = elite_frac
        self.metrics = GAMetrics()
        self.waypoint_count = 6

    def initialize_population(self, start: Tuple[float, float],
                            goal: Tuple[float, float]) -> List[List[Tuple[float, float]]]:
        """Initialize population"""
        population = []
        for _ in range(self.pop_size):
            waypoints = []
            for _ in range(self.waypoint_count):
                x = random.uniform(self.bounds[0][0], self.bounds[0][1])
                y = random.uniform(self.bounds[1][0], self.bounds[1][1])
                waypoints.append((x, y))
            population.append(waypoints)
        return population

    def decode_path(self, individual: List[Tuple[float, float]],
                   start: Tuple[float, float],
                   goal: Tuple[float, float]) -> List[Tuple[float, float]]:
        """Convert chromosome to path"""
        return [start] + individual + [goal]

    def evaluate_fitness(self, individual: List[Tuple[float, float]],
                        start: Tuple[float, float],
                        goal: Tuple[float, float],
                        obstacles: List) -> float:
        """Multi-objective fitness"""
        path = self.decode_path(individual, start, goal)

        # Path length
        path_length = 0.0
        for i in range(1, len(path)):
            path_length += math.hypot(path[i][0] - path[i-1][0],
                                    path[i][1] - path[i-1][1])

        # Collision penalty
        collision_penalty = 0.0
        for i in range(len(path) - 1):
            segment_len = max(1, int(math.hypot(path[i+1][0] - path[i][0],
                                              path[i+1][1] - path[i][1]) * 5))
            for s in range(segment_len + 1):
                t = s / (segment_len + 1e-6)
                px = path[i][0] * (1 - t) + path[i+1][0] * t
                py = path[i][1] * (1 - t) + path[i+1][1] * t

                for obs in obstacles:
                    dist = obs.distance_to_point(px, py)
                    if dist < 0:
                        collision_penalty += 1000.0 + abs(dist) * 2000.0
                    elif dist < 0.4:
                        collision_penalty += (0.4 - dist) * 500.0

        # Smoothness
        smoothness = 0.0
        for i in range(1, len(path) - 1):
            v1 = (path[i][0] - path[i-1][0], path[i][1] - path[i-1][1])
            v2 = (path[i+1][0] - path[i][0], path[i+1][1] - path[i][1])
            mag1 = math.hypot(v1[0], v1[1]) + 1e-6
            mag2 = math.hypot(v2[0], v2[1]) + 1e-6
            dot = (v1[0] * v2[0] + v1[1] * v2[1]) / (mag1 * mag2)
            dot = np.clip(dot, -1.0, 1.0)
            smoothness += abs(math.acos(dot))

        fitness = 1.0 * path_length + 1.0 * collision_penalty + 0.3 * smoothness
        return fitness

    def population_diversity(self, population: List[List[Tuple[float, float]]]) -> float:
        """Calculate diversity"""
        centroids = []
        for ind in population:
            cx = sum(p[0] for p in ind) / len(ind)
            cy = sum(p[1] for p in ind) / len(ind)
            centroids.append((cx, cy))

        if len(centroids) < 2:
            return 0.0

        total_dist = 0.0
        count = 0
        for i in range(len(centroids)):
            for j in range(i + 1, len(centroids)):
                total_dist += math.hypot(centroids[i][0] - centroids[j][0],
                                       centroids[i][1] - centroids[j][1])
                count += 1

        return total_dist / (count + 1e-6)

    def fuzzy_adaptation(self, diversity: float, improvement: float,
                        generation: int, total_generations: int) -> Tuple[float, float]:
        """Fuzzy parameter adaptation"""
        if diversity < 0.3:
            diversity_level = 'low'
        elif diversity > 0.7:
            diversity_level = 'high'
        else:
            diversity_level = 'medium'

        if improvement < 0.001:
            improvement_level = 'low'
        elif improvement > 0.01:
            improvement_level = 'high'
        else:
            improvement_level = 'medium'

        progress = generation / (total_generations + 1e-6)
        if progress < 0.3:
            phase = 'early'
        elif progress > 0.7:
            phase = 'late'
        else:
            phase = 'middle'

        mut_rate = 0.08
        elite_frac = self.base_elite_frac

        if diversity_level == 'low':
            mut_rate = 0.2
            elite_frac *= 0.8

        if improvement_level == 'low':
            mut_rate = min(0.4, mut_rate * 1.5)
            elite_frac = max(0.05, elite_frac * 0.6)

        if phase == 'early' and improvement_level == 'high':
            mut_rate = 0.05

        if phase == 'late':
            mut_rate = max(0.03, mut_rate * 0.8)

        return mut_rate, elite_frac

    def selection(self, population: List[List[Tuple[float, float]]],
                 fitnesses: List[float],
                 elite_frac: float) -> Tuple[List[List[Tuple[float, float]]],
                                            List[List[Tuple[float, float]]]]:
        """Elite selection"""
        indices = sorted(range(len(fitnesses)), key=lambda i: fitnesses[i])
        n_elite = max(1, int(len(population) * elite_frac))
        elites = [population[i] for i in indices[:n_elite]]

        selected = [list(ind) for ind in elites]
        while len(selected) < len(population):
            a, b = random.sample(range(len(population)), 2)
            if fitnesses[a] < fitnesses[b]:
                selected.append([tuple(p) for p in population[a]])
            else:
                selected.append([tuple(p) for p in population[b]])

        return selected, elites

    def crossover(self, parent1: List[Tuple[float, float]],
                 parent2: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """Crossover"""
        if len(parent1) < 2:
            return list(parent1)

        if random.random() < 0.5:
            pt = random.randint(1, len(parent1) - 1)
            child = parent1[:pt] + parent2[pt:]
        else:
            child = [((parent1[i][0] + parent2[i][0]) / 2,
                     (parent1[i][1] + parent2[i][1]) / 2)
                    for i in range(len(parent1))]

        return child

    def mutate(self, individual: List[Tuple[float, float]],
              mut_rate: float) -> List[Tuple[float, float]]:
        """Mutation"""
        mutated = list(individual)
        for i in range(len(mutated)):
            if random.random() < mut_rate:
                x = mutated[i][0] + random.gauss(0, (self.bounds[0][1] - self.bounds[0][0]) * 0.05)
                y = mutated[i][1] + random.gauss(0, (self.bounds[1][1] - self.bounds[1][0]) * 0.05)
                x = np.clip(x, self.bounds[0][0], self.bounds[0][1])
                y = np.clip(y, self.bounds[1][0], self.bounds[1][1])
                mutated[i] = (x, y)

        return mutated

    def plan(self, start: Tuple[float, float],
            goal: Tuple[float, float],
            obstacles: List,
            timeout: float = 4.0) -> List[Tuple[float, float]]:
        """GA evolution loop"""
        self.metrics = GAMetrics()
        population = self.initialize_population(start, goal)

        best_path = None
        best_fitness = float('inf')
        prev_mean_fitness = float('inf')

        t_start = time.time()

        for generation in range(self.generations):
            t_gen_start = time.time()

            fitnesses = [self.evaluate_fitness(ind, start, goal, obstacles)
                        for ind in population]

            for i, fit in enumerate(fitnesses):
                if fit < best_fitness:
                    best_fitness = fit
                    best_path = self.decode_path(population[i], start, goal)

            diversity = self.population_diversity(population)
            mean_fitness = np.mean(fitnesses)
            improvement = prev_mean_fitness - mean_fitness
            prev_mean_fitness = mean_fitness

            self.metrics.generation_fitnesses.append(mean_fitness)
            self.metrics.best_fitness_per_gen.append(best_fitness)
            self.metrics.population_diversity.append(diversity)

            mut_rate, elite_frac = self.fuzzy_adaptation(diversity, improvement,
                                                        generation, self.generations)

            self.metrics.mutation_rates.append(mut_rate)
            self.metrics.elite_fractions.append(elite_frac)

            selected, elites = self.selection(population, fitnesses, elite_frac)

            new_pop = [list(ind) for ind in elites]
            while len(new_pop) < len(population):
                parent1, parent2 = random.sample(selected, 2)
                child = self.crossover(parent1, parent2)
                child = self.mutate(child, mut_rate)
                new_pop.append(child)

            population = new_pop

            self.metrics.generation_count += 1
            self.metrics.replan_times.append(time.time() - t_gen_start)

            if time.time() - t_start > timeout:
                break

        return best_path if best_path else [start, goal]

    def reset(self):
        """Reset"""
        self.metrics = GAMetrics()