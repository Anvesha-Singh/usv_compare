## Strategic Maritime Path Planning: Key Findings

### CHANNEL
- **Easy/Medium:** Both APF and AEGAfi succeed (100%), no collisions.
- **Hard:** APF succeeds (100%), AEGAfi drops to 62.5% (often collides).
- **Extreme:** Both fail (0% success).

### CLUTTER
- **Easy:** Both succeed (100%).
- **Medium/Hard/Extreme:** Both fail—dense obstacles lead to consistent collisions.

### DYNAMIC
- **Easy/Medium:** Both succeed (100%).
- **Hard:** APF fails, AEGAfi achieves 75% (AEGAfi copes better with moving obstacles).
- **Extreme:** Both fail.

### MAZE
- **Easy:** Both succeed.
- **Medium:** AEGAfi succeeds (75%), APF fails (0%)—AEGAfi navigates complex layouts better.
- **Hard/Extreme:** Both mostly fail.

### OPEN
- **Easy/Medium:** Both succeed (100%).
- **Hard:** APF fails (0%), AEGAfi 60% success.
- **Extreme:** Both fail.

***

### Summary

- **APF:** Best in open and simple channels, but fails in cluttered or complex layouts.
- **AEGAfi:** Handles mazes and complex paths better, but struggles most in very dense or hard open setups.
- **Both algorithms fail** at extreme difficulty and very crowded environments.
- **Increasing scenario difficulty quickly exposes planner limits.**

---