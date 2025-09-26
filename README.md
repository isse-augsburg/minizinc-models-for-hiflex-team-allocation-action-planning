# MiniZinc Models for Team Allocation and Action Planning with Example Data Files

## Overview

This repository contains MiniZinc models and example data files for solving the **team allocation** and **action planning** problems in a flexible assembly cell with multiple robots.

They are used in the **HiFlex** system and published as supplementary material for the paper: 
*Assembly Planning and Team Formation in a Flexible Multi-Robot Production Cell*.

For this publication, the models have been restructured and commented to facilitate understanding as supplementary material.
A few variables have been renamed and minor presentation changes were made for readability.
The constraints and optimization criteria are identical to those used in the experiments, and we verified that with the provided example data, the models reproduce the same solutions.

The example data files were automatically generated in our system and represent the first team allocation at the start of the simplified setting presented in Section 7.1 of the paper, with 2 tasks and a total of 4 assembly steps.
In accordance with the models, we also changed some variable names to conform to the names used in the paper, while the actual values remain completely unchanged.

Because the data is automatically generated, it does not contain comments or human-interpretable identifiers.
Transformation between MiniZinc’s integer indices and the actual IDs in our system (e.g., robot names, part IDs) is handled by a ROS node that works as a wrapper for the MiniZinc solver.
The integers in the data files therefore carry no semantic meaning on their own.

Data is split across multiple files (`cell.dzn`, `robots.dzn`, `parts.dzn`, …) to reflect the modular structure of the system and provide some structure for the data.
- Some information is static (e.g. cell layout) and reused across allocations.
- Other information is dynamic (e.g. current robot/part status)

All data files are required for solving the problems.
The file `team_allocation_result.dzn` contains the solver output of the team allocation model and is then used as additional input for the action planning model.

## Contents

- `models`: MiniZinc model files with detailed comments.
    - `team_allocation.mzn`: Team allocation model (presented in Section 6.1 of the paper).
    - `action_planning.mzn`: Action planning model (mentioned in Section 6.2).
- `example_data`: Example data files.
    - `simple_setting_first_step`: Example data for the first allocation of the simplified setting presented in Section 7.1.
        - `cell.dzn`: Static description of the assembly cell (positions, mobility classes, distances).
        - `blocked_positions.dzn`: Positions currently blocked (e.g., by ongoing steps).
        - `robots.dzn`: Number of robots and their mobility classes.
        - `robot_status.dzn`: Current robot positions and availability.
        - `skills.dzn`: Information about the skills of the robots.
        - `skill_status.dzn`: Current skill status.
        - `parts.dzn`: Parts in the cell (current/potential holders, handlers).
        - `steps_and_tasks.dzn`: Assembly steps, tasks, requirements (skills, working positions, parts).
        - `robot_costs.dzn`: Robot allocation and movement costs.
        - `skill_costs.dzn`: Skill usage costs.
        - `params.dzn`: Various parameters for the model execution and optimization (e.g., flags for additional constraints, optimization aspects, additional cost factors, the maximum number of states for the action planning).
        - `team_allocation_result.dzn`: Example solver output for team allocation (used as input to action planning).

## Reading the Models

While the data files are not directly human-interpretable, the models use semantically meaningful names and a structured layout.
MiniZinc’s set/array comprehensions and declarative constraints using predicate logic make it possible to understand the models without prior MiniZinc experience.

We recommend reading Section 6 of the paper first to understand the overall problem and approach.

### Structure

The models are structured into sections, with comments at the start of each section.
Namely, the structure of both models is as follows:
- Inputs: Declaration of input variables and simple derived variables (e.g., index sets for robots, parts, skills etc.) with subsection like in Section 6.1 of the paper.
- Decision Variables: Variables to be solved, plus helper variables derived from them.
- Constraints: The constraints of the model.
For the team allocation model, we have added the numbering of the constraints from Section 6.1.
- Optimization: Inputs and calculation of the objective function.
- Definition of maximization task and search hints for the solver.

### Mapping Model to Paper

We have taken various measures in order to simplify understanding the team allocation model and its relation to the mathematical formulation of the CSOP in the paper:
- [Structure](#structure) similar to the order of the presentation in Section 6.1 of the paper.
- A mapping of variable names between the model and the paper at the start of the `team_allocation.mzn` file.
- Comments highlighting connections to what is presented in the paper, but also explaining differences [see below](#differences-between-the-minizinc-models-and-the-mathematical-formulations-in-the-paper).

The action planning model is less polished, but included for transparency, despite not being a big focus of the paper.

### Differences between the MiniZinc Models and the Mathematical Formulations in the Paper

As discussed in the paper, the mathematical model was presented at a higher level of abstraction for readability. The MiniZinc models include implementation details and extensions. These differences arise from the need to balance focus and clarity in the paper with practical implementation details in MiniZinc

All differences between the MiniZinc models and the mathematical formulations in the paper are highlighted in the comments in the `team_allocation.mzn` file using the tag `EXT` (for "extension").

The main differences are:
- **Robot availability** split into active, passive, and moving flags (vs. simple available/unavailable).
- **Skills** modeled with more detail:
    - Skills have several additional properties (e.g., active/passive) left out in the paper for simplicity.
    - min/max reach per skill (instead of per robot).
    - Skills have a maximum capacity for holding parts, which is not considered in the paper for simplicity.
- **Parts** is not just **held by** a robot, but also has a specific **skill** of that robot assigned as its current holder. This allows robots to have different skills for holding different parts with different properties (e.g. capacity). This is probably the most significant difference, but we found the presentation in the paper more intuitive and easier to understand with the level of detail presented there, and consider the assignment of a specific skill to hold a part as an implementation detail, which can easily be abstracted away (as it is of course trivial to derive the robot to which the skill belongs).
- **Working positions**: model allows multiple per step, though experiments only use one.
- Model could assign **multiple assembly steps** at once, but this is disabled by a constraint limiting the number of allocated steps to 1 (creating the single-step assumption presented in the paper).
- **Additional constraints** omitted from the paper for brevity: Either because they are usually fulfilled anyways in the cases we present in our evaluation; or because they are extensions improving the practical quality of results, but not conceptually necessary for specifying the problem.
- The optimization is slightly more complex than presented in the paper, but follows the same principles: reward important steps, penalize costly allocations, moves, and transfers.

Further details can be found in the comments in the `team_allocation.mzn` file.

## Executing the Models

### Requirements

- **MiniZinc 2.9.3**: Download and install from [official website](https://www.minizinc.org/software.html)
- **Chuffed Solver**: Usually included in MiniZinc binaries.
Must be selected as the solver in the MiniZinc IDE (usually **not** the default) or specified in the command line (`--solver chuffed`).
More information in the [Chuffed GitHub repository](https://github.com/chuffed/chuffed).
- Enabled **Free Search** solver option for performance
    - CLI: `-f`
    - IDE: Check the "Free Search" option in the Solver configuration.

To execute the models with the example data, you can use the MiniZinc IDE or the command line interface.

### Execution Using the Command Line

- Depending on your operating system, you may need to add the MiniZinc binary directory to your system's PATH environment variable.
- Open a terminal and navigate to the directory containing this `README.md` file (and the MiniZinc model and data files)

#### Team Allocation

Paste the following command in the terminal:
```
minizinc --solver chuffed -i -f models/team_allocation.mzn example_data/simple_setting_first_step/cell.dzn example_data/simple_setting_first_step/blocked_positions.dzn example_data/simple_setting_first_step/robots.dzn example_data/simple_setting_first_step/robot_status.dzn example_data/simple_setting_first_step/skills.dzn example_data/simple_setting_first_step/skill_status.dzn example_data/simple_setting_first_step/parts.dzn example_data/simple_setting_first_step/steps_and_tasks.dzn example_data/simple_setting_first_step/robot_costs.dzn example_data/simple_setting_first_step/skill_costs.dzn example_data/simple_setting_first_step/params.dzn
```

#### Action Planning

Paste the following command in the terminal:
```
minizinc --solver chuffed -i -f models/action_planning.mzn example_data/simple_setting_first_step/cell.dzn example_data/simple_setting_first_step/blocked_positions.dzn example_data/simple_setting_first_step/robots.dzn example_data/simple_setting_first_step/robot_status.dzn example_data/simple_setting_first_step/skills.dzn example_data/simple_setting_first_step/skill_status.dzn example_data/simple_setting_first_step/parts.dzn example_data/simple_setting_first_step/steps_and_tasks.dzn example_data/simple_setting_first_step/robot_costs.dzn example_data/simple_setting_first_step/skill_costs.dzn example_data/simple_setting_first_step/params.dzn example_data/simple_setting_first_step/team_allocation_result.dzn
```

#### Notes

- You may remove the flag `-i` if you do not want to see (non-optimal) intermediate solutions.
- If you remove the flag `-f`, the solver will not use the Free Search option, which may result in significantly longer solving times.
- If you want to proceed with your solution, instead of the provided one, you can copy the output and replace the contents of the `team_allocation_result.dzn` file with it or make a new data file with the output.
Then, in the action planning, replace the `team_allocation_result.dzn` file with your custom file (do not use both, as that will cause errors)

### Execution with the MiniZinc IDE

- Open all model (`*.mzn`) files in the [models](./models) directory and all data files (`*.dzn`) in the [example_data/simple_setting_first_step](./example_data/simple_setting_first_step) directory. You can do this via the **Open** dialog in the IDE.
- **IMPORTANT**: Click **Show configuration editor** and select the **Chuffed** solver in the dropdown menu on top. Also, look for the **Free Search** option and enable it.
Do not change any of the other options.

#### Team Allocation

- Select the tab of the model file (**`team_allocation.mzn`**).
- Click the **Run** button to execute the model.
- There should be a pop-up window allowing you to select the data files you want to use.
Potentially, you have to click on the tab "Select data file" in the pop-up window, if this is not already selected by default.
- Select **all** data files **except `team_allocation_result.dzn`**.
- Click **OK** to confirm your selection.

#### Action Planning

Repeat the same steps as in the [Team Allocation Model Execution](#team-allocation-model-execution) section, with the following changes:
- Select the tab of the model file **`action_planning.mzn`** instead of `team_allocation.mzn`.
- In the pop-up window for selecting data files, select **all** data files **including `team_allocation_result.dzn`** (or select the data file you created with your own solution instead; do not use both).

#### Common Issues when Using the MiniZinc IDE

- If after clicking "Run" the pop-up window does not appear, make sure, you have opened the data files.
- If you get an error message instead of the expected output, check if you have selected the Chuffed solver (not Gecode) and selected the correct data files.
- If the solver takes a very long time, check if you have enabled the "Free Search" option in the solver configurations.

### Expected Behavior and Output

#### Team Allocation

- Optimal solution indicated by `==========`.
- Expected time: at most a few seconds
- Objective value of optimal solutions: **2611**
- Multiple optimal solutions exist (as we have 4 assignable steps with identical requirements), so the exact result may not match the provided `team_allocation_result.dzn` file, but should look similar and have the same objective value.


#### Action Planning

- The optimal solution can again be recognized by the double line (`==========`) at the end of the output.
- Expected time: longer than team allocation, usually still under a minute (may depend on hardware, OS, and solver version).
- Objective value of optimal solutions: **2686**.
- The raw solution is not directly human-interpretable; action plans are reconstructed by the ROS wrapper (not included here).

## Contact

Primary contact: [Christian Lehner](mailto:lehner@isse.de)
Other contacts: [Hella Ponsar](mailto:ponsar@isse.de), [Oliver Kosak](mailto:kosak@isse.de)

Institute for Software & Systems Engineering, University of Augsburg,
Universitätsstraße 6a, 86159 Augsburg, Germany.

If you encounter issues with the model execution, please provide:
- Error description
- MiniZinc + Chuffed version
- Hardware + OS

We welcome feedback and suggestions for improving the supplementary material or its documentation.

We can provide further material (e.g., additional example data files) upon request.
