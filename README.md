# MiniZinc Models for Team Allocation and Action Planning with Example Data Files

- For the mathematical formulation of the team allocation problem CSOP, please see the [Mathematical Description of the CSOP for Team Allocation](#mathematical-description-of-the-csop-for-team-allocation) section.
- If you want to quickly execute the models with the provided example data, please check the [Requirements] and then past ethe instructions for [Execution Using the Command Line](#execution-using-the-command-line). You can check your results against the ones we expect using the [Expected Behavior and Output](#expected-behavior-and-output) section.
- If you want an overview of the files in this repository, please see the [Contents](#contents) section.

For more details, more customized execution instructions, and further explanations, please read the full README below.

- [MiniZinc Models for Team Allocation and Action Planning with Example Data Files](#minizinc-models-for-team-allocation-and-action-planning-with-example-data-files)
  - [Overview](#overview)
  - [Contents](#contents)
  - [Understanding the Models](#understanding-the-models)
    - [Structure](#structure)
    - [Mathematical Description of the CSOP for Team Allocation](#mathematical-description-of-the-csop-for-team-allocation)
      - [General Notes](#general-notes)
      - [Rendering](#rendering)
      - [Input Variables](#input-variables)
        - [Positions \& Mobility Classes](#positions--mobility-classes)
        - [Robots \& Skills](#robots--skills)
        - [Parts](#parts)
        - [Assignable Assembly Steps](#assignable-assembly-steps)
      - [Decision Variables](#decision-variables)
      - [Constraints](#constraints)
      - [Optimization Criteria](#optimization-criteria)
      - [Differences between the MiniZinc Models and the Mathematical Formulations](#differences-between-the-minizinc-models-and-the-mathematical-formulations)
  - [Executing the Models](#executing-the-models)
    - [Requirements](#requirements)
    - [Execution Using the Command Line](#execution-using-the-command-line)
      - [Team Allocation](#team-allocation)
      - [Action Planning](#action-planning)
      - [Notes](#notes)
    - [Execution with the MiniZinc IDE](#execution-with-the-minizinc-ide)
      - [Team Allocation](#team-allocation-1)
      - [Action Planning](#action-planning-1)
      - [Common Issues when Using the MiniZinc IDE](#common-issues-when-using-the-minizinc-ide)
    - [Expected Behavior and Output](#expected-behavior-and-output)
      - [Team Allocation](#team-allocation-2)
      - [Action Planning](#action-planning-2)
  - [Contact](#contact)


## Overview

This repository contains MiniZinc models and example data files for solving the **team allocation** and **action planning** problems in a flexible assembly cell with multiple robots.

They are used in the **HiFlex** system and published as supplementary material for the paper: 
*Online Team Formation in a Flexible Multi-Robot Production Cell*.
This repository is intended to enable detailed inspection and reproduction of the model which was explained in more general terms in the paper.

For this publication, the models have been restructured and commented to facilitate understanding as supplementary material.
A few variables have been renamed and minor presentation changes were made for readability.
The constraints and optimization criteria are identical to those used in the experiments, and we verified that with the provided example data, the models reproduce the same solutions.

The example data files were automatically generated in our system and represent the first team allocation at the start of the *simplified setting* presented in *Section 5* of the paper, with 2 tasks and a total of 4 assembly steps.
In accordance with the models, we also changed some variable names to conform to the names used in our mathematical model, while the actual values remain completely unchanged.

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
    - `team_allocation.mzn`: Team allocation model (cf. Section 4.1 of the paper).
    - `action_planning.mzn`: Action planning model (cf. Section 4.2 of the paper).
- `example_data`: Example data files.
    - `simple_setting_first_step`: Example data for the first allocation of the simplified setting presented in Section 5 of the paper. Currently, this is the only provided example.
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

## Understanding the Models

While the data files are not directly human-interpretable, the models use semantically meaningful names and a structured layout.
MiniZinc’s set/array comprehensions and declarative constraints using predicate logic make it possible to understand the models without prior MiniZinc experience.

We recommend reading *Section 4* of the paper first to understand the overall problem and approach.

### Structure

The models are structured into sections, with comments at the start of each section.
Namely, the structure of both models is as follows:
- Inputs: Declaration of input variables and simple derived variables (e.g., index sets for robots, parts, skills etc.)
- Decision Variables: Variables to be solved, plus helper variables derived from them.
- Constraints: The constraints of the model.
For the team allocation model, we have added the numbering of the constraints from Section 4.1.
- Optimization: Inputs and calculation of the objective function.
- Definition of maximization task and search hints for the solver.

### Mathematical Description of the CSOP for Team Allocation

#### General Notes

In order to make the models easier to understand, we have included a mathematical description of the CSOP.
This description is simplified compared to the MiniZinc models in order to focus on the core aspects of the problem.
Still, we have taken various measures in order to simplify understanding the team allocation model and its relation to the mathematical formulation of the CSOP:
- [Structure](#structure) and order of the various input and decision variables and constraints is similar in the MiniZinc model and the mathematical formulation.
- A mapping of variable names between the mathematical model and the MiniZinc model at the start of the `team_allocation.mzn` file.
- Comments highlighting connections between the mathematical model and the MiniZinc model, but also explaining differences [see below](#differences-between-the-minizinc-models-and-the-mathematical-formulations-in-the-paper).

The action planning model is less polished and has no mathematical description here, but the gist of it can be understood from Section 4.2 of the paper in combination with the model for team allocation.

#### Rendering

This README uses GitHub’s LaTeX math rendering. If you view it somewhere that doesn’t support math, formulas may appear as raw LaTeX.

#### Input Variables

##### Positions & Mobility Classes

- $\mathrm{Pos}$ is a set of relevant positions in the cell. These could, for example, correspond to a certain square in a grid layout for the cell.
- $\mathrm{Pos}_w \subseteq \mathrm{Pos}$ is the set of working positions, i.e., positions that can be assigned as the position where the actual production process should take place.
- $\mathrm{Pos}_{\mathrm{blocked}} \subseteq \mathrm{Pos}$ is the set of positions that are currently blocked from being used, e.g., because another robot is already planned to move there.

As we have different kinds of robots in the cell, which can reach different positions and have different movement distances between positions, we differentiate between multiple mobility classes.
In our example cell, which we introduced in the paper, there is one mobility class for every linear axis and one for the carts that can freely move on the floor, while linear axes are considered as obstacles.
Therefore, in this example, there are four mobility classes.

- The set of mobility classes is denoted by $\mathrm{M}$; the index $m$ indicates a mobility class $m \in \mathrm{M}$.
- For every mobility class, $\mathrm{Pos}^m \subseteq \mathrm{Pos}$ is the set of positions that can be reached by a robot of this mobility class.
- $\mathrm{L} \subseteq \mathrm{M}$ is the subset of linear mobility classes, on which the positions are strictly ordered and robots cannot jump over other robots.
- $o^l : \mathrm{Pos}^l \rightarrow \mathbb{N}$ is a linear ordering of positions for mobility class $l \in \mathrm{L}$.
- $\Delta^l \in \mathbb{N}$ is the minimal required offset two robots must keep from each other.

There are multiple different distances measured between positions $d(pos_1, pos_2)$:

- $d^m : \mathrm{Pos}^m \times \mathrm{Pos}^m \rightarrow \mathbb{N}$ is an approximation of the distance a robot from mobility class $m$ would have to move from one position to the other.
- $d_w : \mathrm{Pos} \times \mathrm{Pos}_w \rightarrow \mathbb{N}$ is the work distance, which practically corresponds to the Euclidean distance.
- $d_{\mathrm{trans}} : \mathrm{Pos} \times \mathrm{Pos} \rightarrow \mathbb{N}$ is the transfer distance.

##### Robots & Skills

- $\mathrm{R}$ is the set of all robots.
- $\mu : \mathrm{R} \rightarrow \mathrm{M}$ is the mobility class of a robot.
- $\mathrm{R}^m := \{ r \in \mathrm{R} \mid \mu(r) = m \}$ is the set of all robots of mobility class $m$.
- $pos_0 : \mathrm{R} \rightarrow \mathrm{Pos}$ is the current position of a robot.
- $\mathrm{avail} : \mathrm{R} \rightarrow \mathbb{B}$ indicates if a robot is currently available.
- $r_{\min}, r_{\max} : \mathrm{R} \rightarrow \mathbb{N}$ are minimum and maximum reachability distances.
- $\mathrm{Sk}$ is the set of all skills.
- $\mathrm{skills} : \mathrm{R} \rightarrow \mathbb{P}(\mathrm{Sk})$ are the current skills of a robot.

##### Parts

- $\mathrm{P}$ is the set of all available parts.
- $h_0 : \mathrm{P} \rightarrow \mathrm{R}$ is the robot that currently holds the part.
- $\mathrm{R}_\mathrm{hold} : \mathrm{P} \rightarrow \mathbb{P}(\mathrm{R})$ are robots that store or keep the part in place.
- $\mathrm{R}_\mathrm{handle} : \mathrm{P} \rightarrow \mathbb{P}(\mathrm{R})$ are robots that can also actively take the part or hand it to another holder.

##### Assignable Assembly Steps

- $\mathrm{T}, \mathrm{S}, \mathrm{Rl}, \mathrm{Req}$ denote tasks, steps, roles, and part requirements.
- $\mathrm{S}^t \subseteq \mathrm{S}$ are the steps of task $t \in \mathrm{T}$ with pairwise disjointness and $\mathrm{S} = \bigcup_{t \in \mathrm{T}} \mathrm{S}^t$.
- $\mathrm{Rl}^s \subseteq \mathrm{Rl}$ are the roles of step $s \in \mathrm{S}$ with pairwise disjointness and $\mathrm{Rl} = \bigcup_{s \in \mathrm{S}} \mathrm{Rl}^s$.
- $\mathrm{Req}^s \subseteq \mathrm{Req}$ are the requirements of step $s \in \mathrm{S}$ with pairwise disjointness and $\mathrm{Req} = \bigcup_{s \in \mathrm{S}} \mathrm{Req}^s$.
- $\mathrm{targetRole} : \mathrm{Req} \rightarrow \mathrm{Rl}$ assigns the role that should hold the part at the start of step execution.
- $\mathrm{possibleSkills} : \mathrm{Rl} \rightarrow \mathbb{P}(\mathrm{Sk})$ are the skills that can fulfill a role.
- $\mathrm{possibleParts} : \mathrm{Req} \rightarrow \mathbb{P}(\mathrm{P})$ are the parts that can fulfill a requirement.

#### Decision Variables

The solver must determine:

- $\hat{s} \in \mathrm{S}$: the step chosen for execution.
- $\hat{pos}_w \in \mathrm{Pos}_w \setminus \mathrm{Pos}_{\mathrm{blocked}}$: the working position.
- $\hat{r} : \mathrm{Rl}^{\hat{s}} \rightarrow \mathrm{R}$: the team of robots.
- $\hat{p} : \mathrm{Req}^{\hat{s}} \rightarrow \mathrm{P}$: the assigned parts.

Additionally, heuristic target positions:

- $\hat{pos}_t : \mathrm{R} \rightarrow \mathrm{Pos}$: a possible target position of each robot.

#### Constraints

1. **No robot or part is chosen twice**

$$
\forall rl_1 \neq rl_2 \in \mathrm{Rl}^{\hat{s}}:\hat{r}(rl_1) \neq \hat{r}(rl_2)
$$

$$
\forall req_1 \neq req_2 \in \mathrm{Req}^{\hat{s}}:\hat{p}(req_1) \neq \hat{p}(req_2)
$$

1. **Chosen robots can fulfill the roles**

$$
\forall rl \in \mathrm{Rl}^{\hat{s}} \exists sk \in \mathrm{possibleSkills}(rl): sk \in \mathrm{skills}(\hat{r}(rl))
$$

3. **Chosen parts fulfill the requirements**

$$
\forall req \in \mathrm{Req}^{\hat{s}}:\hat{p}(req) \in \mathrm{possibleParts}(req)
$$

4. **Only available robots and parts held by available robots**

$$
\forall rl \in \mathrm{Rl}^{\hat{s}}:\mathrm{avail}(\hat{r}(rl))
$$

$$
\forall req \in \mathrm{Req}^{\hat{s}}:\mathrm{avail}(h_0(\hat{p}(req)))
$$

5. **Team members can reach the working position**

$$
\forall rl \in \mathrm{Rl}^{\hat{s}}:r_{\min}(\hat{r}(rl)) \le d_w(\hat{pos}_t(\hat{r}(rl)), \hat{pos}_w)
\le r_{\max}(\hat{r}(rl))
$$

6. **Robots can reach their target position**

$$
\forall r \in \mathrm{R}:\hat{pos}_t(r) \neq pos_0(r) \Rightarrow \hat{pos}_t(r) \in (\mathrm{Pos}^{\mu(r)} \setminus \mathrm{Pos}_{\mathrm{blocked}}) \land \mathrm{avail}(r)
$$

7. **Robots cannot share positions**

$$
\forall r_1 \neq r_2 \in \mathrm{R}: \hat{pos}_t(r_1) \neq \hat{pos}_t(r_2)
$$

8. **Linear ordering and offsets on linear axes**

$$
\forall l \in \mathrm{L}, r_1 \neq r_2 \in \mathrm{R}^l: o^l(pos_0(r_1)) > o^l(pos_0(r_2)) \Rightarrow o^l(\hat{pos}_t(r_1)) > o^l(\hat{pos}_t(r_2)) + \Delta^l
$$

#### Optimization Criteria

For optimization, the solver receives some additional input variables.
In the following description, as well as in our MiniZinc implementation, all rewards and costs are given as integers to deal with the limitations of the solver.
In reality, the system usually uses floating point numbers for these values, which are then multiplied by a constant factor and rounded to convert them to integers.

The reward for the model is given by a simple assignment of an importance value for each step.
- $\mathrm{importance}: \mathrm{S} \to \mathbb{N}$: The importance of a step, to prioritize them, calculated by the production control based on various configurable factors (cf. Section 4.1 in the paper).


The following costs are considered to penalize certain allocations:
- $\mathrm{allocationCost}: \mathrm{R} \to \mathbb{N}$: The cost of assigning a robot to a team. If certain robots are considered more valuable than others, this can be differentiated here.
- $\mathrm{moveCost}: \mathrm{R} \to \mathbb{N}$: The cost of moving a robot, no matter the distance. This is done for representing the difficulties it can cause to precisely measure the new position of a moved robot. In our system, this is trivial for the robots on linear axes but very challenging for the carts, as their internal positioning system is not sufficient for high-precision production processes and thus requires additional camera-based localization. Therefore, this cost is quite high for the carts, while it is negligible for the robots on linear axes.
- $\mathrm{moveDistCost}: \mathrm{R} \to \mathbb{N}$: The cost of moving a robot per distance unit, which is used to represent the time it takes to move the robot.
- $\mathrm{transferCost}: \mathbb{N}$: The constant cost if a part has to be transferred to another robot.
- $\mathrm{transferDistCost}: \mathbb{N}$: The approximated cost of transferring a part over a distance, as given by $d_{\mathrm{trans}}$.

The solver maximizes the following value (with all inputs being treated as constants):

$$
\begin{array}{rl}
\mathrm{value}(\hat{s}, \hat{pos}_w, \hat{r}, \hat{p}, \hat{pos}_t) = & \mathrm{importance}(\hat{s}) \\
&- \sum_{rl \in \mathrm{Rl}^{\hat{s}}} \mathrm{allocationCost}(\hat{r}(rl)) \\
&- \sum_{r \in \mathrm{R}} \mathrm{moved}(r, \hat{pos}_t)\cdot \mathrm{moveCost}(r) \\
&- \sum_{r \in \mathrm{R}} d^{\mu(r)}(pos_0(r), \hat{pos}_t(r))\cdot \mathrm{moveDistCost}(r) \\
&- \sum_{req \in \mathrm{Req}^{\hat{s}}} \mathrm{transferred}(req, \hat{r}, \hat{p})\cdot \mathrm{transferCost} \\
&- \sum_{req \in \mathrm{Req}^{\hat{s}}} \mathrm{transferDist}(req, \hat{r}, \hat{p}, \hat{pos}_t)\cdot \mathrm{transferDistCost}
\end{array}
$$


with auxiliary definitions:

$$
\mathrm{moved}(r, \hat{pos}_t) =
\begin{cases}
1 & \mathrm{if } \hat{pos}_t(r) \neq pos_0(r) \\
0 & \mathrm{otherwise}
\end{cases}
$$

$$
\mathrm{transferred}(req, \hat{r}, \hat{p}) =
\begin{cases}
1 & \mathrm{if } h_0(\hat{p}(req)) \neq \hat{r}(\mathrm{targetRole}(req)) \\
0 & \mathrm{otherwise}
\end{cases}
$$

$$
\mathrm{transferDist}(req, \hat{r}, \hat{p}, \hat{pos}_t) =
\begin{cases}
d_{\mathrm{trans}}(pos_0(h_0(\hat{p}(req))), \hat{pos}_t(\hat{r}(\mathrm{targetRole}(req)))) & \mathrm{if } transferred(req, \hat{r}, \hat{p}) \\
0 & \mathrm{otherwise}
\end{cases}
$$

If the solver does not find an optimal solution within a timeout, it returns the best solution found so far.
If no solution fulfills all constraints, no new step execution is started.


#### Differences between the MiniZinc Models and the Mathematical Formulations

The mathematical model was presented at a higher level of abstraction for readability.
The MiniZinc models include implementation details and extensions.
These differences arise from the need to balance focus and clarity with practical implementation details in MiniZinc

All differences between the MiniZinc models and the mathematical formulations are highlighted in the comments in the `team_allocation.mzn` file using the tag `EXT` (for "extension").

The main differences are:
- **Robot availability** split into active, passive, and moving flags (vs. simple available/unavailable).
- **Skills** modeled with more detail:
    - Skills have several additional properties (e.g., active/passive) left out in the mathematical model for simplicity.
    - min/max reach per skill (instead of per robot).
    - Skills have a maximum capacity for holding parts, which is not considered in the mathematical model for simplicity.
- **Parts** is not just **held by** a robot, but also has a specific **skill** of that robot assigned as its current holder. This allows robots to have different skills for holding different parts with different properties (e.g. capacity). This is probably the most significant difference, but we found the presentation in the mathematical model more intuitive and easier to understand with the level of detail presented there, and consider the assignment of a specific skill to hold a part as an implementation detail, which can easily be abstracted away (as it is of course trivial to derive the robot to which the skill belongs).
- **Working positions**: model allows multiple per step, though experiments only use one.
- Model could assign **multiple assembly steps** at once, but this is disabled by a constraint limiting the number of allocated steps to 1 (creating the single-step assumption presented in the mathematical model).
- **Additional constraints** omitted from the mathematical model for brevity: Either because they are usually fulfilled anyways in the cases we present in our evaluation; or because they are extensions improving the practical quality of results, but not conceptually necessary for specifying the problem.
- The **optimization** is slightly more complex than presented in the mathematical model, but follows the same principles: reward important steps, penalize costly allocations, moves, and transfers.

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

Repeat the same steps as in the section above for the team allocation, with the following changes:
- Select the tab of the model file **`action_planning.mzn`** instead of `team_allocation.mzn`.
- In the pop-up window for selecting data files, select **all** data files **including `team_allocation_result.dzn`** (or select the data file you created with your own solution instead; do not use both).

#### Common Issues when Using the MiniZinc IDE

- If after clicking "Run" the pop-up window does not appear, make sure, you have opened the data files.
- If you get an error message instead of the expected output, check if you have selected the Chuffed solver (not Gecode) and selected the correct data files.
- If the solver takes a very long time, check if you have enabled the "Free Search" option in the solver configurations.

### Expected Behavior and Output

#### Team Allocation

- Optimal solution indicated by `==========`.
- Expected time: typically at most a few seconds
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
