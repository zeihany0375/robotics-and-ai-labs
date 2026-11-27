Lab 3 Reinforcement Learning
Student ID 0076

This folder contains the MATLAB experiment scripts, the supplied warehouse
model and pretrained agent, the modified noise model, and the measured outputs.
The tests ran in MATLAB R2024b Update 5 with Simulink, Reinforcement Learning
Toolbox, and Deep Learning Toolbox.

To run the experiments, extract the archive and make its folder the MATLAB
Current Folder. Run runAll. The individual entry points are runToy, runRobot,
runNoise, plotEvidence, and runChecks. The robot and noise sweeps reuse the
provided checkpoints when present. For a fresh simulation run, move the
evidence folder to a separate location before running runAll.

Evidence includes the following files.

  toy_sweep.csv           Each seed's first correct and first strict episode.
  toy_values.csv          Averaged learned tables, policies, and margins.
  grid.csv                Outcomes, times, and clearance for 240 fixed starts.
  baseline.csv            The live script's 50-run random benchmark replay.
  mirror_actor_grid.csv   Every paired actor query and absolute error.
  mirror_pairs.csv        Outcomes and timings for the 24 mirrored pairs.
  robustness.csv          Per-start results for all model mismatch settings.
  noise.csv               Per-start results for all sensor noise settings.
  collision_probe.csv     Raw collision and done times for a colliding start.
  noise_checks.csv        A check of the matched random disturbance sequences.

The MAT files contain trajectories, per-case checkpoints and simulation
diagnostics. The PNGs are the original MATLAB figures used in the report.
whrobot_noise.slx contains the controller-only position noise wiring.
