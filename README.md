# Planar 3R Manipulator Simulation

Planar 3R robotic arm simulation with analytical kinematics, workspace analysis,
and trajectory tracking.

## Overview

This repository contains a planar three-degree-of-freedom robotic arm
simulation implemented with direct geometric kinematics. It demonstrates
forward and inverse kinematics, reachable workspace analysis, and
proportional joint-space tracking.

## Report

The analytical report is available at:

[planar-3r-kinematics-report.pdf](./planar-3r-kinematics-report.pdf)

The report covers:

- Geometric modeling and forward kinematics
- Analytical inverse kinematics
- Jacobian matrix
- Singularity analysis
- Workspace analysis
- Proportional kinematic control

## Visuals

Path tracking simulation:

![Path tracking](./media/3DOF_2D_Path_Tracking.png)

Workspace visualization:

![Workspace analysis](./media/3DOF_2D_Workspace_Analysis.png)

## Run

Install the project dependencies:

```bash
pip install -r requirements.txt
```

Run the simulations from the repository root:

```bash
python src/3DOF_2D_Simulation_Path.py
python src/3DOF_2D_Simulation_Workspace.py
```

## Repository Layout

- `src/`: simulation scripts
- `media/`: generated figures used by the documentation
- `planar-3r-kinematics-report.pdf`: project report
