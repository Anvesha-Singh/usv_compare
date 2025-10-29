# visualization.py - ENHANCED WITH METRICS

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import numpy as np
from typing import List, Tuple, Optional, Dict
import math
import time

class SimulationVisualizer:
    """Visualize maritime path planning simulations with collision detection and metrics"""

    def __init__(self, figsize=(14, 9)):
        self.figsize = figsize

    def _check_collision_points(self, trajectory: List[Tuple[float, float]], 
                               obstacles: List) -> Tuple[bool, int]:
        """Check if trajectory passes through obstacles"""
        for i, pos in enumerate(trajectory):
            for obs in obstacles:
                dist = math.hypot(pos[0] - obs.x, pos[1] - obs.y)
                if dist < obs.r:
                    return True, i
        return False, -1

    def _calculate_metrics(self, trajectory: List[Tuple[float, float]], 
                          start: Tuple[float, float],
                          goal: Tuple[float, float],
                          planning_time: float = 0.0,
                          steps: int = 0) -> Dict:
        """Calculate trajectory metrics"""
        metrics = {}

        # Path length
        path_length = 0.0
        for i in range(len(trajectory) - 1):
            dx = trajectory[i+1][0] - trajectory[i][0]
            dy = trajectory[i+1][1] - trajectory[i][1]
            path_length += math.hypot(dx, dy)
        metrics['path_length'] = path_length

        # Distance to goal
        if trajectory:
            final_pos = trajectory[-1]
            dist_to_goal = math.hypot(final_pos[0] - goal[0], final_pos[1] - goal[1])
            metrics['dist_to_goal'] = dist_to_goal
        else:
            metrics['dist_to_goal'] = math.hypot(start[0] - goal[0], start[1] - goal[1])

        # Straight line distance
        straight_line = math.hypot(goal[0] - start[0], goal[1] - start[1])
        metrics['straight_line'] = straight_line

        # Efficiency ratio
        if straight_line > 0:
            metrics['efficiency'] = (straight_line / path_length) * 100 if path_length > 0 else 0
        else:
            metrics['efficiency'] = 0

        # Timing metrics
        metrics['planning_time'] = planning_time
        metrics['steps'] = steps

        return metrics

    def plot_scenario(self, start: Tuple[float, float], 
                     goal: Tuple[float, float],
                     static_obs: List,
                     dynamic_obs: List = None,
                     trajectory: List[Tuple[float, float]] = None,
                     title: str = "Maritime Path Planning",
                     save_path: Optional[str] = None,
                     planning_time: float = 0.0,
                     steps: int = 0,
                     show_metrics: bool = True):
        """
        Plot scenario with trajectory and metrics displayed on plot
        """
        if dynamic_obs is None:
            dynamic_obs = []

        # Calculate metrics
        metrics = self._calculate_metrics(trajectory or [], start, goal, planning_time, steps)

        fig, ax = plt.subplots(figsize=self.figsize)

        # Plot static obstacles
        for obs in static_obs:
            circle = plt.Circle((obs.x, obs.y), obs.r, 
                              color='gray', alpha=0.7, 
                              label='Static Obstacle' if obs == static_obs[0] else '')
            ax.add_patch(circle)

        # Plot dynamic obstacles
        for obs in dynamic_obs:
            circle = plt.Circle((obs.x, obs.y), obs.r, 
                              color='orange', alpha=0.6,
                              label='Dynamic Obstacle' if obs == dynamic_obs[0] else '')
            ax.add_patch(circle)

            if hasattr(obs, 'vx') and hasattr(obs, 'vy'):
                if abs(obs.vx) > 0.01 or abs(obs.vy) > 0.01:
                    ax.arrow(obs.x, obs.y, obs.vx*2, obs.vy*2,
                            head_width=0.15, head_length=0.1, 
                            fc='darkorange', ec='darkorange', alpha=0.8)

        # Plot start and goal
        ax.plot(start[0], start[1], 'go', markersize=15, 
               markeredgewidth=2, markeredgecolor='darkgreen',
               label='Start', zorder=10)
        ax.plot(goal[0], goal[1], 'r*', markersize=20,
               markeredgewidth=2, markeredgecolor='darkred',
               label='Goal', zorder=10)

        # Check for collisions and plot trajectory
        collision_detected = False
        collision_idx = -1
        collision_color = 'blue'
        status_text = "SUCCESS"

        if trajectory and len(trajectory) > 1:
            all_obs = static_obs + dynamic_obs
            collision_detected, collision_idx = self._check_collision_points(trajectory, all_obs)

            if collision_detected:
                collision_color = 'red'
                status_text = f"COLLISION (Step {collision_idx})"

            if collision_detected:
                # Split trajectory at collision
                valid_x = [p[0] for p in trajectory[:collision_idx+1]]
                valid_y = [p[1] for p in trajectory[:collision_idx+1]]
                ax.plot(valid_x, valid_y, 'b-', linewidth=2.5, 
                       alpha=0.8, label='Valid Path')

                invalid_x = [p[0] for p in trajectory[collision_idx:]]
                invalid_y = [p[1] for p in trajectory[collision_idx:]]
                ax.plot(invalid_x, invalid_y, 'r--', linewidth=2, 
                       alpha=0.6, label='Invalid (Through Obstacle)')

                collision_pos = trajectory[collision_idx]
                ax.plot(collision_pos[0], collision_pos[1], 'rx', 
                       markersize=15, markeredgewidth=3, label='Collision')
            else:
                traj_x = [p[0] for p in trajectory]
                traj_y = [p[1] for p in trajectory]
                ax.plot(traj_x, traj_y, collision_color, linewidth=2.5, 
                       alpha=0.8, label='Trajectory')

            # Add direction arrows
            step_size = max(1, len(trajectory) // 10)
            for i in range(0, len(trajectory)-1, step_size):
                if collision_detected and i >= collision_idx:
                    break

                dx = trajectory[i+1][0] - trajectory[i][0]
                dy = trajectory[i+1][1] - trajectory[i][1]
                if abs(dx) > 0.01 or abs(dy) > 0.01:
                    ax.arrow(trajectory[i][0], trajectory[i][1], 
                            dx*0.5, dy*0.5,
                            head_width=0.1, head_length=0.08,
                            fc=collision_color, ec=collision_color, alpha=0.5)

        ax.set_xlabel('X Position (m)', fontsize=12, fontweight='bold')
        ax.set_ylabel('Y Position (m)', fontsize=12, fontweight='bold')
        ax.set_title(f"{title} - {status_text}", fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper right', fontsize=10)
        ax.set_aspect('equal')

        # Set bounds
        all_x = [start[0], goal[0]]
        all_y = [start[1], goal[1]]
        for obs in static_obs + dynamic_obs:
            all_x.extend([obs.x - obs.r, obs.x + obs.r])
            all_y.extend([obs.y - obs.r, obs.y + obs.r])
        if trajectory:
            all_x.extend([p[0] for p in trajectory])
            all_y.extend([p[1] for p in trajectory])

        margin = 1.0
        ax.set_xlim(min(all_x) - margin, max(all_x) + margin)
        ax.set_ylim(min(all_y) - margin, max(all_y) + margin)

        # Add metrics text box if requested
        if show_metrics and trajectory:
            metrics_text = self._format_metrics_text(metrics)
            props = dict(boxstyle='round', facecolor='wheat', alpha=0.9)
            ax.text(0.02, 0.98, metrics_text, transform=ax.transAxes, 
                   fontsize=10, verticalalignment='top', bbox=props, 
                   family='monospace')

        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"✓ Figure saved to {save_path}")

        return fig, ax

    def _format_metrics_text(self, metrics: Dict) -> str:
        """Format metrics for display"""
        text = "━━━ METRICS ━━━\n"
        text += f"Path Length:    {metrics['path_length']:.2f} m\n"
        text += f"Straight Line:  {metrics['straight_line']:.2f} m\n"
        text += f"Efficiency:     {metrics['efficiency']:.1f}%\n"
        text += f"Distance to Goal: {metrics['dist_to_goal']:.2f} m\n"
        if metrics['planning_time'] > 0:
            text += f"Planning Time:  {metrics['planning_time']:.3f} s\n"
        if metrics['steps'] > 0:
            text += f"Steps Taken:    {metrics['steps']}\n"
        return text

    def plot_comparison(self, scenario_name: str, difficulty: str,
                       start: Tuple[float, float],
                       goal: Tuple[float, float],
                       static_obs: List,
                       dynamic_obs: List,
                       apf_trajectory: List[Tuple[float, float]],
                       ga_trajectory: List[Tuple[float, float]],
                       apf_time: float = 0.0,
                       ga_time: float = 0.0,
                       apf_steps: int = 0,
                       ga_steps: int = 0,
                       save_path: Optional[str] = None):
        """
        Plot side-by-side comparison with metrics
        """
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(18, 8))

        all_obs = static_obs + dynamic_obs
        apf_collision, apf_collision_idx = self._check_collision_points(apf_trajectory, all_obs)
        ga_collision, ga_collision_idx = self._check_collision_points(ga_trajectory, all_obs)

        # Calculate metrics for both
        apf_metrics = self._calculate_metrics(apf_trajectory, start, goal, apf_time, apf_steps)
        ga_metrics = self._calculate_metrics(ga_trajectory, start, goal, ga_time, ga_steps)

        for ax, trajectory, method, color, collision, collision_idx, metrics in [
            (ax1, apf_trajectory, 'Online APF', 'blue', apf_collision, apf_collision_idx, apf_metrics),
            (ax2, ga_trajectory, 'Offline AEGAfi', 'purple', ga_collision, ga_collision_idx, ga_metrics)
        ]:
            # Plot obstacles
            for obs in static_obs:
                circle = plt.Circle((obs.x, obs.y), obs.r, 
                                  color='gray', alpha=0.7)
                ax.add_patch(circle)

            for obs in dynamic_obs:
                circle = plt.Circle((obs.x, obs.y), obs.r, 
                                  color='orange', alpha=0.6)
                ax.add_patch(circle)

            # Start and goal
            ax.plot(start[0], start[1], 'go', markersize=12, 
                   markeredgewidth=2, markeredgecolor='darkgreen')
            ax.plot(goal[0], goal[1], 'r*', markersize=18,
                   markeredgewidth=2, markeredgecolor='darkred')

            # Trajectory
            if trajectory and len(trajectory) > 1:
                if collision:
                    traj_x = [p[0] for p in trajectory[:collision_idx+1]]
                    traj_y = [p[1] for p in trajectory[:collision_idx+1]]
                    ax.plot(traj_x, traj_y, color=color, linewidth=2.5, alpha=0.9)

                    traj_x = [p[0] for p in trajectory[collision_idx:]]
                    traj_y = [p[1] for p in trajectory[collision_idx:]]
                    ax.plot(traj_x, traj_y, 'r--', linewidth=2, alpha=0.6)

                    collision_pos = trajectory[collision_idx]
                    ax.plot(collision_pos[0], collision_pos[1], 'rx', 
                           markersize=12, markeredgewidth=3)
                else:
                    traj_x = [p[0] for p in trajectory]
                    traj_y = [p[1] for p in trajectory]
                    ax.plot(traj_x, traj_y, color=color, linewidth=2.5, alpha=0.9)

            status = "COLLISION" if collision else "SUCCESS"
            ax.set_xlabel('X Position (m)', fontsize=11, fontweight='bold')
            ax.set_ylabel('Y Position (m)', fontsize=11, fontweight='bold')
            ax.set_title(f'{method} - {status}', fontsize=13, fontweight='bold')
            ax.grid(True, alpha=0.3)
            ax.set_aspect('equal')
            ax.set_xlim(-1, 13)
            ax.set_ylim(-4, 4)

            # Add metrics text box
            metrics_text = self._format_metrics_text(metrics)
            props = dict(boxstyle='round', facecolor='wheat', alpha=0.85)
            ax.text(0.02, 0.98, metrics_text, transform=ax.transAxes, 
                   fontsize=9, verticalalignment='top', bbox=props, 
                   family='monospace')

        fig.suptitle(f'{scenario_name.capitalize()} - {difficulty.capitalize()} Difficulty', 
                    fontsize=15, fontweight='bold')
        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"✓ Comparison saved to {save_path}")

        return fig

    def plot_metrics(self, results: List[dict], 
                    save_path: Optional[str] = None):
        """Plot performance metrics comparison"""
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))

        apf_results = [r for r in results if r.get('mode') == 'Online_APF']
        ga_results = [r for r in results if r.get('mode') == 'Offline_AEGAfi']

        # 1. Success Rate
        ax = axes[0, 0]
        difficulties = ['easy', 'medium', 'hard', 'extreme']
        apf_success = []
        ga_success = []

        for d in difficulties:
            apf_d = [r for r in apf_results if r.get('difficulty') == d]
            ga_d = [r for r in ga_results if r.get('difficulty') == d]

            apf_success.append(np.mean([r.get('success', False) for r in apf_d]) * 100 if apf_d else 0)
            ga_success.append(np.mean([r.get('success', False) for r in ga_d]) * 100 if ga_d else 0)

        x = np.arange(len(difficulties))
        width = 0.35
        ax.bar(x - width/2, apf_success, width, label='APF', color='blue', alpha=0.7)
        ax.bar(x + width/2, ga_success, width, label='AEGAfi', color='purple', alpha=0.7)
        ax.set_xlabel('Difficulty', fontsize=11)
        ax.set_ylabel('Success Rate (%)', fontsize=11)
        ax.set_title('Success Rate vs Difficulty', fontweight='bold')
        ax.set_xticks(x)
        ax.set_xticklabels(difficulties)
        ax.legend()
        ax.grid(axis='y', alpha=0.3)
        ax.set_ylim([0, 105])

        # 2. Path Length
        ax = axes[0, 1]
        apf_lengths = [r.get('path_length', 0) for r in apf_results if r.get('success')]
        ga_lengths = [r.get('path_length', 0) for r in ga_results if r.get('success')]

        data_to_plot = []
        labels = []
        if apf_lengths:
            data_to_plot.append(apf_lengths)
            labels.append('APF')
        if ga_lengths:
            data_to_plot.append(ga_lengths)
            labels.append('AEGAfi')

        if data_to_plot:
            ax.boxplot(data_to_plot, labels=labels)
        ax.set_ylabel('Path Length (m)', fontsize=11)
        ax.set_title('Path Length Distribution', fontweight='bold')
        ax.grid(axis='y', alpha=0.3)

        # 3. Planning Time
        ax = axes[1, 0]
        apf_times = [r.get('planning_cpu_time', 0) * 1000 for r in apf_results]
        ga_times = [r.get('planning_cpu_time', 0) * 1000 for r in ga_results]

        data_to_plot = []
        labels = []
        if apf_times:
            data_to_plot.append(apf_times)
            labels.append('APF')
        if ga_times:
            data_to_plot.append(ga_times)
            labels.append('AEGAfi')

        if data_to_plot:
            ax.boxplot(data_to_plot, labels=labels)
        ax.set_ylabel('Planning Time (ms)', fontsize=11)
        ax.set_title('Computational Time', fontweight='bold')
        ax.grid(axis='y', alpha=0.3)
        ax.set_yscale('log')

        # 4. Steps to Goal
        ax = axes[1, 1]
        apf_steps = [r.get('steps', 0) for r in apf_results if r.get('success')]
        ga_steps = [r.get('steps', 0) for r in ga_results if r.get('success')]

        data_to_plot = []
        labels = []
        if apf_steps:
            data_to_plot.append(apf_steps)
            labels.append('APF')
        if ga_steps:
            data_to_plot.append(ga_steps)
            labels.append('AEGAfi')

        if data_to_plot:
            ax.boxplot(data_to_plot, labels=labels)
        ax.set_ylabel('Steps to Goal', fontsize=11)
        ax.set_title('Efficiency (Steps)', fontweight='bold')
        ax.grid(axis='y', alpha=0.3)

        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"✓ Metrics plot saved to {save_path}")

        return fig


if __name__ == "__main__":
    print("Enhanced visualization module loaded successfully!")
    print("\nNew features:")
    print("  ✓ Collision detection")
    print("  ✓ Real-time metrics display on plots")
    print("  ✓ Path length, efficiency, and timing metrics")