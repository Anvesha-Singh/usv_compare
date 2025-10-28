# Maritime Path Planning Comparison: Improved APF vs AEGAfi

Complete implementation of two maritime path planning algorithms with comprehensive comparison framework, Stonefish-ready architecture.

## Files Overview

### `simulator.py` - 2D USV Kinematic Simulator
- **SimulationConfig**: Centralized configuration (easily portable to Stonefish)
- **Obstacle**: Static circular obstacles (Stonefish-compatible)
- **DynamicObstacle**: Moving obstacles with velocity
- **USV**: Kinematic unicycle model with state tracking
- **Metrics functions**: Path length, smoothness, clearance, heading stability

**Stonefish Portability**: USV state can be directly mapped to Stonefish messages. Obstacle representation matches Stonefish collision geometry.

### `planners.py` - Path Planning Algorithms
- **ImprovedAPF**: Local reactive planner with stuck detection and escape
  - APFMetrics: Logs forces, stuck events, perturbations
  - Comprehensive force calculations and control conversion
  
- **AEGAfi**: Global genetic algorithm with fuzzy adaptation
  - GAMetrics: Logs fitness, diversity, mutation rates, replan times
  - Full fuzzy inference system for parameter adaptation
  - Multi-objective fitness (length, smoothness, collision avoidance)

**Stonefish Portability**: Both planners return control commands (throttle, yaw_rate) that directly interface with vehicle actuators.

### `experiment.py` - Experiment Runner & Analysis
- **ScenarioBuilder**: 4 standardized maritime scenarios
  - Open: Few sparse obstacles
  - Channel: Narrow passage with walls
  - Clutter: Dense random obstacles
  - Dynamic: Moving obstacles (collision avoidance)
  
- **TrialRunner**: Executes individual trials with full metric collection
- **ExperimentRunner**: Orchestrates all experiments with statistical analysis
  - Saves per-trial CSV data
  - Generates summary statistics
  - Produces statistical comparison report
  - Implements independent t-tests

**Output**:
- `summary.csv`: Aggregate metrics by scenario/planner
- `comparison_report.txt`: Detailed analysis with statistical tests
- `[planner]_[scenario]_trials.csv`: Individual trial results

## Metrics Collected

### Navigation Performance
- **Success Rate**: Percentage of trials reaching goal
- **Collision Rate**: Percentage of trials hitting obstacles
- **Path Length**: Total distance traveled (m)
- **Path Smoothness**: Sum of heading changes (rad)
- **Minimum Clearance**: Closest approach to obstacles (m)
- **Heading Stability**: Variance in heading changes

### Computational Performance
- **Planning CPU Time**: Total computation time per trial (s)
- **Avg Time Per Step**: Planning time normalized by steps
- **Steps To Goal**: Number of simulation steps taken
- **Average Speed**: Mean velocity during trial (m/s)

### Algorithm-Specific Metrics

**Improved APF**:
- Stuck Events: Number of times stuck detector triggered
- Escape Perturbations: Number of escapes from local minima
- Attractive Force Mean: Average goal-directed force
- Repulsive Force Mean: Average obstacle avoidance force

**AEGAfi**:
- Replan Count: Number of times path regenerated
- Average GA Generations: Evolutions per replan
- Average Population Diversity: Genetic diversity metric
- Average Mutation Rate: Fuzzy-adapted mutation probability

## How to Run

### Prerequisites
```bash
# Activate virtual environment
cd ~/usv_compare
python3 -m venv usv-env
pip install numpy matplotlib scipy pandas tqdm
source usv-env/bin/activate
```

### Run Full Experiment Suite
```bash
python3 experiment_strategic.py
```

This will:
1. Run 30 trials × 2 algorithms × 4 scenarios = 240 total trials
2. Display progress with tqdm progress bar
3. Print summary statistics after each scenario
4. Generate all output files in `results/` directory
5. Produce comprehensive comparison report

**Expected Runtime**: ~10-15 minutes on modern laptop (CPU-only, no GPU)

### Run Subset (Quick Testing)
Modify `experiment.py` last line:
```python
results = runner.run_all_experiments(['open', 'dynamic'], trials=10)  # Fast test
```

## Output Files

After running, check `results_strategic/` directory:

```
results_strategic/
├── summary.csv                          # Main aggregate statistics
├── comparison_report.txt                # Full analysis and t-tests
├── apf_open_trials.csv                  # APF individual trials (open scenario)
├── aegafi_open_trials.csv               # AEGAfi individual trials (open scenario)
├── apf_channel_trials.csv
├── aegafi_channel_trials.csv
├── apf_clutter_trials.csv
├── aegafi_clutter_trials.csv
├── apf_dynamic_trials.csv
└── aegafi_dynamic_trials.csv
```

### Reading Results

**summary.csv columns**:
- scenario, planner, trials
- success_rate, collision_rate
- avg_path_length, std_path_length
- avg_path_smoothness
- avg_min_clearance
- avg_planning_cpu_time
- avg_steps

**comparison_report.txt**:
- Per-scenario performance summary for each planner
- Statistical t-tests comparing algorithms
- p-values indicate significant differences (p < 0.05)

## Stonefish Migration Path

This codebase is architected for seamless Stonefish integration:

### Phase 1: Simulator Replacement (Minimal Changes)
1. **Keep**: `planners.py` (unchanged)
2. **Replace**: `simulator.py` with Stonefish ROS2 interface
   - USV class wraps Stonefish vehicle topics
   - Obstacle class represents Stonefish static objects
   - State updates from `/vehicle/state` topic
   - Control commands to `/vehicle/thruster_command` topic

### Phase 2: Configuration Parity
Stonefish scenario XML automatically maps to:
- SimulationConfig (physics parameters)
- Obstacle list (from scenario objects)
- Start/goal positions

### Phase 3: ROS2 Node Structure
```
├── stonefish_sim_node.py      (Stonefish simulator wrapper)
├── planner_node.py             (APF or AEGAfi)
├── experiment_runner_ros.py    (Updated experiment loop)
└── config/
    ├── stonefish_scenarios/
    │   ├── open.xml
    │   ├── channel.xml
    │   ├── clutter.xml
    │   └── dynamic.xml
```

### Porting Checklist
- [ ] Install Stonefish and stonefish_ros2
- [ ] Create ROS2 package wrapper
- [ ] Map USV state topics → USV.get_state()
- [ ] Map control command topics ← (throttle, yaw_rate)
- [ ] Update obstacle detection (from `/sensor/sonar` or static environment)
- [ ] Run `experiment.py` with Stonefish backend
- [ ] Compare results between 2D simulator and Stonefish

**Expected Result**: Similar trajectory shapes with differences in:
- Speed profiles (hydrodynamic effects)
- Turn quality (realistic inertia)
- Execution accuracy (sensor noise in Stonefish)

## Algorithm Details

### Improved APF Parameters
Modify in `planners.py` ImprovedAPF.__init__:
```python
'k_att': 1.0,               # Increase for faster goal seeking
'k_rep': 5.0,               # Increase for stronger obstacle avoidance
'd0': 2.5,                  # Obstacle influence distance
'escape_strength': 0.6,     # Local minima escape magnitude
'stuck_window': 8,          # Timesteps to detect stuck (↑ for patience)
'stuck_threshold': 0.05,    # Movement threshold to detect stuck
```

### AEGAfi Parameters
Modify in `experiment.py` run_aegafi_trial:
```python
bounds = [(-1.0, 13.0), (-4.0, 4.0)]    # Search space
pop_size = 40                            # Population size
generations = 60                         # Max generations
elite_frac = 0.2                         # Elite preservation
```

## Comparison Interpretation

### Success Rate
- APF: Limited by local minima in cluttered/narrow scenarios
- AEGAfi: Global optimization, higher success but slower

### Path Length
- APF: Direct paths when not stuck, suboptimal when escaping
- AEGAfi: Smoother, near-optimal paths (longer computation)

### Planning Time
- APF: Fast per-step (~0.001s), constant overhead
- AEGAfi: Slower (~0.5-1.5s per replan), but infrequent

### Path Smoothness
- APF: May have sharp turns during escape perturbations
- AEGAfi: Generally smoother due to waypoint optimization

### Minimum Clearance
- APF: Can get dangerously close to obstacles (reactive)
- AEGAfi: Maintains safer distances (global perspective)

## Reproducibility

### Random Seeds
All trials use deterministic seeding:
```python
seed = trial_id * 1000  # Enables reproducible scenarios
random.seed(seed)
np.random.seed(seed)
```

### Environment Specifications
Results logged in:
- Experiment timestamp in comparison_report.txt
- Python/NumPy versions used

### Re-running Specific Trial
```python
# In experiment.py, modify run_all_experiments to run single config:
runner = ExperimentRunner()
scenario = 'clutter'
aegafi = AEGAfi(bounds=[(-1.0, 13.0), (-4.0, 4.0)])
result = runner.trial_runner.run_aegafi_trial(aegafi, scenario, trial_id=0, seed=0)
```

## Common Issues

### Issue: "ModuleNotFoundError: No module named scipy"
**Solution**: scipy was used implicitly in stats functions
```bash
pip install scipy
```

### Issue: Empty results or crashes
**Solution**: Check scenario creation in ScenarioBuilder
- Verify goal is reachable from start
- Ensure obstacles don't block entire path

### Issue: Extremely long computation time
**Solution**: Reduce trials for quick validation
```python
results = runner.run_all_experiments(['open'], trials=5)
```

## Citation & References

This implementation is based on:

**Improved APF**:
- Reference: [MDPI 2025] "An improved artificial potential field method for multi-AGV..."
- Key improvement: Local minima escape via perturbation

**AEGAfi**:
- Reference: [Ocean Engineering 2023, vol 280] Zhao et al. "Global-local hierarchical path planning scheme..."
- Key features: Adaptive elite selection + fuzzy parameter control

**Maritime Benchmarks**:
- CommonOcean framework (compatible scenario format)
- COLREGs-aware collision detection considerations

## Next Steps

1. **Analyze Results**: Check `summary.csv` and `comparison_report.txt`
2. **Tune Hyperparameters**: Adjust k_att, k_rep, pop_size based on results
3. **Stonefish Integration**: Use porting checklist above
4. **COLREGs Enhancement**: Add maritime rule-specific penalties (future work)
5. **Real-world Validation**: Compare against recorded vessel trajectories

## License & Usage

This is a research/educational implementation. Use for:
- Comparison studies
- Academic research
- Stonefish integration research
- Maritime autonomy education

Modification friendly - adapt metrics, scenarios, and algorithms as needed for your research.
