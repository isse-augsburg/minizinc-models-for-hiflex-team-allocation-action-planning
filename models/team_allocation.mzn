% =========================================================
% Team Allocation CSOP
% ---------------------------------------------------------
% This is a the actual model used for the team allocation in our system.
% - Variable names are sometimes changed compared to the paper due to the different presentation format (mathematical formulas in two-column format vs. MiniZinc code).
%   - Where variable names are changed compared to the paper, this is noted at declaration of the variable in [square brackets].
% - The sets of robots (R), parts (P) etc. are represented as set of int with a certain range. Therefore, every object is simply represented by an integer. Our code maps the actual objects to these integers and vice versa.
% - Functions A -> B are represented as arrays with index set A and value set B (array[A] of B).
%   - Similarly, functions A -> P(B) are represented as arrays with index set A and value set P(B) (array[A] of set of B)
%   and functions A x B -> C are represented as arrays with index set A x B and value set C (array[A,B] of C) etc.
% - As mentioned in the paper, some aspects explained there were simplified compared to the actual model here.
%   - Whenever it is possible, to map to sections of the paper, this is done. Extensions compared to the paper model are marked with [EXT] and quickly explained.
% =========================================================

% Paper ↔ Code aliases (quick reference)
%
%   inputs
% ----------------------------------------
%   Pos              ≙ POS
%   Pos_w            ≙ ALLOWEDWORKPOS
%   Pos_blocked      ≙ BLOCKEDPOS
%   M                ≙ MC
%   Pos^m            ≙ mcPos[m]
%   L                ≙ LMC
%   o^l(pos)         ≙ mcLinearOrder[l, pos]
%   Delta^l          ≙ requiredOffset[l]
%   d^m              ≙ mcDist[m, ., .]
%   d_w              ≙ workDist
%   d_trans          ≙ transferDist
%   R                ≙ ROBOT
%   mu               ≙ mobilityClass
%   R^m              ≙ robotsByMobilityClass[m]
%   pos_0            ≙ currentPos
%   avail            ≙ robotAvailableActive, robotAvailablePassive, robotAvailableMoving
%   r_min, r_max     ≙ minReach, maxReach
%   Sk               ≙ SKILL
%   skills           ≙ inverse: skillToRobot
%   P                ≙ PART
%   h_0              ≙ initialHolder
%   R_hold           ≙ potentialHolders
%   R_handle         ≙ potentialHandlers
%   T,S,Rl,Req       ≙ TASK, STEP, ROLE, REQ
%   S^t              ≙ taskSteps[t]
%   Rl^s             ≙ stepToRoles[s]
%   Req^s            ≙ stepToReqs[s]
%   targetRole       ≙ targetRole
%   possibleSkills   ≙ possibleSkills
%   possibleParts    ≙ possibleParts
%
%   decision variables
% ----------------------------------------
%   \hat{s}          ≙ assignedSteps (card = 1)
%   \hat{r}          ≙ executingRobots (indirectly via decision variable executingSkill)
%   \hat{p}          ≙ chosenPart
%   \\hat{pos_w}     ≙ workPos (card = 1)
%   \hat{pos_t}      ≙ targetPos

include "globals.mzn";

% ----------------------------------------
% ----------------------------------------
% input data (if there is no "=" they must be directly specified in a data file, otherwise they are derived from other input data)
% ----------------------------------------
% ----------------------------------------

% ----------------------------------------
% domains (for allowing restrictions of the values for better performance with some solvers; here: no restrictions)
% ----------------------------------------
set of int: VAL = 0..;                          % values and costs for optimization concerns (including sums of multiple such values/costs)
set of int: DIST = -1..;                        % distance measures, -1 for unreachable
set of int: LINEARORDER = -1..;                 % Indices for positions on linear axes, -1 for not on the axis
set of int: REACH = 0..;                        % min/max reach distances of the robots
set of int: CAPACITY = 0..;                     % capacity of skills to hold parts

% ----------------------------------------
% Positions & mobility classes
% ----------------------------------------

int: positions;                                 % number of positions
set of int: POS = 1..positions;                 % set of all positions [Pos]
set of POS: ALLOWEDWORKPOS;                     % set of possible working positions [Pos_w]
set of POS: BLOCKEDPOS;                         % set of positions that are blocked and cannot be used [Pos_blocked]
set of POS: FREEWORKPOS = ALLOWEDWORKPOS diff BLOCKEDPOS; % set of positions that are not blocked and can be used as working positions


int: mobilityClasses;                           % number of mobility classes
set of int: MC = 1..mobilityClasses;            % set of mobility classes [M]
int: linearMobilityClasses;                     % number of mobility classes that move on linear axes (e.g., robots on rails); these will be given the first indices 1..linearMobilityClasses
set of MC: LMC = 1..linearMobilityClasses;      % set of mobility classes that move on linear axes [L]
array[LMC,POS] of LINEARORDER: mcLinearOrder;   % linear order of positions for linear mobility classes [o^l]
array[LMC] of LINEARORDER: requiredOffset;      % minimal required offset between two robots of the same linear mobility class [Delta^l]

array[MC,POS,POS] of DIST: mcDist;              % (approximate) moving distance between positions for each mobility class [d^m]
array[MC] of set of POS: mcPos = [{pos | pos in POS where mcDist[mc, pos, pos] = 0}| mc in MC]; % positions reachable by robots of mobility class m [Pos^m]
array[POS,POS] of DIST: workDist;               % distance between positions used for reachability calculations [d_w]
array[POS,POS] of DIST: transferDist;           % distance between positions used for approximating costs for part transfers [d_trans]

% ----------------------------------------
% Robots and skills
% ----------------------------------------

int: noRobots;                                      % number of robots
set of int: ROBOT = 1..noRobots;                    % set of robots [R]
array[ROBOT] of MC: mobilityClass;                  % mobility class of each robot [mu]
array[MC] of set of ROBOT: robotsByMobilityClass =  [{rob | rob in ROBOT where mobilityClass[rob]=mc}|mc in MC]; % robots of each mobility class [R^m]
array[ROBOT] of POS: currentPos;                    % position of each robot before the allocation [pos_0] 
% [EXT, avail] availability of a robot; here differentiated by availability for active skill execution (robot has to move), passive skill execution (robot is used, e.g. for storing parts), or moving the robot to another position
% In the paper simplified to a simple available/unavailable distinction for each robot.
array[ROBOT] of bool: robotAvailableActive;
array[ROBOT] of bool: robotAvailablePassive;
array[ROBOT] of bool: robotAvailableMoving;

% [EXT] The skill concept is more complex than presented in the paper. The main differences are:
% - Every instance of a skill by a robot is represented as a separate skill in the model with its own properties/costs etc., so if two robots can screw there are two screwing skills in the model, one for each robot.
%   - This means, that unlike in the paper, it is easier to map skills to robots (as every skill has exactly one robot) instead of the other way around, which we chose for the paper for easier readability.
%   - It also means getting the available skills for a role is a bit more complex and requires some preprocessing, as skills are not just strings with no properties as could be assumed from the paper.
% - Skills have some additional properties left out in the paper for simplicity (active/passive, transfer skill or not, internal successor skills)
% - minReach/maxReach can be set for every skill of a robot, allowing different reaches for different skills (does not need to be used, but is possible); in the paper this is a property of the robot, as this is more natural without a detailed explanation of the skill concept
int: skills;                                        % number of skills
set of int: SKILL = 1..skills;                      % set of skills [Sk]
array[SKILL] of bool: currentSkill;                 % [EXT] whether the skill is currently active or only available by a tool change

array[SKILL] of ROBOT: skillToRobot;                % robot that has the skill [inverse of skills]
array[SKILL] of REACH: minReach;                    % min reach of the skill on the robot [EXT, r_min]
array[SKILL] of REACH: maxReach;                    % max reach of the skill on the robot [EXT, r_max]
array[SKILL] of CAPACITY: freeCapacity;             % [EXT] remaining capacity of a skill to hold more parts (0 if it cannot hold parts or is already full)
array[SKILL] of bool: activeSkill;                  % [EXT] Skills can be active (robot has to move to execute the skill) or passive (robot does not actually have to do anything, e.g., for storing parts).
array[SKILL] of bool: transferSkill;                % [EXT] Skills can be transfer skills (can be involved in part transfers to other robots) or not (can only be involved in moving parts from one skill of the same robot to another)
array[SKILL] of set of SKILL: internalSuccessorSkills; % [EXT] successor skills on the same robot that can be used to transfer parts without involving another robot (e.g., a TrayStoringSkill and a PickAndPlaceSkill of a robotic arm)

% helper variables
array[SKILL] of set of SKILL: transitiveSuccessors = [transitiveSuccessors(sk, {})|sk in SKILL];
array[SKILL] of bool: skillAvailable = [currentSkill[sk] /\ ((activeSkill[sk] /\ robotAvailableActive[skillToRobot[sk]]) \/ ((not activeSkill[sk]) /\ robotAvailablePassive[skillToRobot[sk]])) | sk in SKILL];
set of SKILL: availableSkills = {sk | sk in SKILL where skillAvailable[sk]};


% ----------------------------------------
% Parts
% [EXT] Every part is controlled using a particular skill of a robot, while in the paper only the robots are specified for simplicity.
% Using the skill concept to represent part holding allows some flexibility by allowing a robot that can hold different parts in different ways, potentially witch different capacity (e.g., an industrial robot can grip a part with its end-effector or store it on a tray that moves with it).
% It also makes the model more complex and might be an unintuitive extension of the skill concept
% It as so far proven to be useful in our implementation, but is still open for discussion, weighing pros and cons.
% For the paper, we decided to keep the simpler representation using only robots.
% The model here can be easily adapted to the simpler representation, which has also been done in some tests, therefore, we believe it is not dishonest to present the model used in the paper as quasi-equivalent to the one we actually use right now (seen here).
% ----------------------------------------

int: partsInCell;                                   % number of parts in the cell (counting only those relevant for the team allocation problem)
set of int: PART = 1..partsInCell;                  % set of parts in the cell [P]
array[PART] of SKILL: initialHolder;                % skill used for holding the part at the robot that currently holds the part [in the paper, only h_0 is specified instead of a particular skill [EXT, h_0]
array[PART] of set of SKILL: potentialHolders;      % skills that can be used for holding the part [EXT, R_hold], not including handlers, which can also be used for holding
array[PART] of set of SKILL: potentialHandlers;     % skills that can be used for handling the part, i.e., changing its orientation etc. [EXT, R_handle]

% helper variables
array[PART] of set of SKILL: availableHolders = [potentialHolders[p] intersect availableSkills| p in PART];
array[PART] of set of SKILL: availableHandlers = [potentialHandlers[p] intersect availableSkills| p in PART];
array[PART] of set of SKILL: allAvailableSkills = [availableHolders[p] union availableHandlers[p]| p in PART];  % all skills that can hold the part (including handlers)
array[PART] of set of SKILL: allAvailableTransferSkills = [{sk | sk in allAvailableSkills[p] where transferSkill[sk]}| p in PART];

% [EXT] only allow using parts if the skill currently holding them is available and there is an available skill that allows the part to be transferred to another robot if needed
% This is a restriction that reduces the chance of a solution that cannot actually be executed while not placing a serious restriction on usable solutions in practice
% It was included due to experiences in practical tests but could be removed if desired
array[PART] of bool: partTransferable = [exists(sk in transitiveSuccessors[initialHolder[p]] intersect allAvailableTransferSkills[p])(true)| p in PART];
set of PART: availableParts = {p | p in PART where skillAvailable[initialHolder[p]] /\ partTransferable[p]};

% [EXT] Some (small) parts may freely be moved to free potential holders to make space at their current holder, while other (large) ones should only be moved if absolutely necessary
% This is not necessary for the theoretical model we presented, but this additional variable allows us to constrain the system, so that some larger parts will never be handed over from their current holder to just free up a cart for other parts
% Therefore, a skill that has no free capacity can only be used for holding a part during the skill execution if it currently holds a part which it is allowed to move
% Constraints of such nature are considered implementation details and may be changed/added/removed as needed for practical reasons and not part of the general model we developed.
% In the future, we wish to allow the ability to add such additional constraints on demand, as specific practical scenarios might require similar practical restrictions which are not universal enough to be part of the general model.
array[PART] of bool: mayBeTransferredToFreeHolder;

% ----------------------------------------
% Assignable Assembly Steps
% ----------------------------------------

int: tasks;                                         % number of tasks with assignable steps
set of int: TASK = 1..tasks;                        % set of tasks [T]
int: stepsToAssign;                                 % number of assignable steps
set of int: STEP = 1..stepsToAssign;                % set of assignable steps [S]
int: roles;                                         % number of roles of all steps combined; every step has distinct roles for the MiniZinc model; the code allows defining roles used for multiple steps, but these will be copied for the MiniZinc input
set of int: ROLE = 1..roles;                        % set of roles [Rl]
int: requiredParts;                                 % number of required parts of all steps combined; every step has distinct required parts for the MiniZinc model; the code allows defining required parts used for multiple steps, but these will be copied for the MiniZinc input
set of int: REQ =  1..requiredParts;                % set of required parts [Req]
int: workPositions;                                 % [EXT] number of work positions that must be assigned if a certain step is chosen; currently we always have exactly one work position per step which every participating robot must be able to reach; the MiniZinc model would allow multiple work positions for a step, each of which might only need to be reachable by a subset of the robots, but this is not used in our actual system so far and is an extension that might be useful for more complex assembly steps
set of int: WORKPOS = 1..workPositions;             % [EXT] set of work positions that must be assigned if a certain step is chosen [see above]

array[TASK] of set of STEP: taskSteps;              % steps belonging to each task [S^t]
array[STEP] of set of ROLE: stepToRoles;            % roles for each step [Rl^s]
array[STEP] of set of REQ: stepToReqs;              % required parts for each step [Req^s]
array[ROLE] of set of WORKPOS: roleToWorkPos;       % [EXT] work positions that must be reachable by the robot executing the role if the step is chosen
array[STEP] of set of WORKPOS: stepToWorkPos = [{wp| rl in stepToRoles[t], wp in roleToWorkPos[rl]}|t in STEP]; % [EXT] work positions that must be assigned if the step is chosen; currently always exactly one per step

array[REQ] of ROLE: targetRole;                     % role that holds the part at the beginning of the step execution [targetRole]
array[ROLE] of set of SKILL: possibleSkills;        % skills that can be used for executing the role [possibleSkills]
array[REQ] of set of PART: possibleParts;           % parts that can be used for the required part [possibleParts]

% ----------------------------------------
% ----------------------------------------
% decision variables
% [EXT] model allows assigning multiple steps at once, if they do not interfere; we used this extension in some tests, but for our current implementation and all experiments mentioned in the paper we presented we only allow one step to be assigned, as presented in the paper
% In order to allow this restriction to be lifted easily, the data structures used are more generic and an extra constraint is used to restrict to at most one assigned step 
% Some of the other variable domains also are also more generic then in the paper on first glance, but will be restricted in the following section "data consistency constraints"
% ----------------------------------------
% ----------------------------------------

array[TASK] of var opt STEP: assignedStepForTask;   % assigned steps; always at most one per task; currently at most one per team allocation (see constraint below) [\hat{s}]
array[ROLE] of var opt SKILL: executingSkill;       % skill used for executing the role [\hat{r}] [EXT] In the paper, the robot is chosen directly instead of the appropriate skill, that does not really matter in practice
array[REQ] of var opt PART: chosenPart;             % part used for the required part [\hat{p}]
array[WORKPOS] of var opt FREEWORKPOS: workPos;     % working position assigned for the step [\hat{pos}_w]
array[ROBOT] of var POS: targetPos;                 % target position for each robot when the step can start [\hat{pos}_t]

% ----------------------------------------
% Data-consistency constraints.
% In the paper, these are implicit via function domains; MiniZinc needs them as explicit constraints.
% ----------------------------------------

% consistency of the "assignedStepForTask" variable and definition of the set of assigned steps
constraint forall(t in TASK)(assignedStepForTask[t] = <> \/ assignedStepForTask[t] in taskSteps[t]);
var set of STEP: assignedSteps = {s | s in STEP where s in assignedStepForTask};

% Paper models choosing a single step → enforce at most one assigned step here.
constraint card(assignedSteps) <= 1;

% definition of the actual domains for the other decision variables (cannot be directly set as the array index set in MiniZinc, as they are not automatically ranges, i.e., sequences of directly consecutive integers, as is required for index sets; therefore the domain restriction is done by constraints)
var set of ROLE: allocatedRoles = {rl| t in assignedSteps, rl in stepToRoles[t]};           % [Rl^\hat{s}]
var set of REQ: allocatedReqs = {req| t in assignedSteps, req in stepToReqs[t]};            % [Req^\hat{s}]
var set of WORKPOS: allocatedWorkPos = {wp| t in assignedSteps, wp in stepToWorkPos[t]};    % [EXT] Work positions that need to be assigned for the chosen step (currently always exactly one, see see Pos_w comment)


% ensuring that allocations take place for all role, required parts, work positions of the chosen step and not for others; conforming to the domains of decision variables presented in the paper
constraint forall(rl in ROLE)(rl in allocatedRoles <-> occurs(executingSkill[rl]));
constraint forall(rl in REQ)(rl in allocatedReqs <-> occurs(chosenPart[rl]));
constraint forall(wp in WORKPOS)(wp in allocatedWorkPos <-> occurs(workPos[wp]));

% ----------------------------------------
% helper variables depending on decision variables
% ----------------------------------------

array[REQ] of var opt SKILL: targetSkill = array1d(REQ, [if req in allocatedReqs then executingSkill[targetRole[req]] else <> endif | req in REQ]); % skill that must hold the part at the beginning of the step execution

array[ROLE] of var opt ROBOT: executingRobots = [if occurs(executingSkill[rl]) then skillToRobot[deopt(executingSkill[rl])] else <> endif| rl in ROLE]; % exactly [\hat{r}] in the paper
array[STEP] of var set of ROBOT: teamForStep = [if s in assignedSteps then {deopt(executingRobots[rl]) | rl in stepToRoles[s]} else {} endif  | s in STEP];
array[ROBOT] of var bool: movedRobot = [targetPos[rob] != currentPos[rob]| rob in ROBOT];  % [moved]
array[ROLE] of var opt POS: roleRobotPos = [if rl in allocatedRoles then targetPos[executingRobots[rl]] else <> endif| rl in ROLE]; % target position of the robot that executes the role

% ----------------------------------------
% ----------------------------------------
% functions
% ----------------------------------------
% ----------------------------------------

% Implementation detail regarding local transfers between skills of the same robot
function set of SKILL: transitiveSuccessors(SKILL: root, set of SKILL: visited) = {root} union {sk | sk in SKILL where exists(s0 in (internalSuccessorSkills[root] diff visited))(sk in transitiveSuccessors(s0, visited union {root}))};

% ----------------------------------------
% ----------------------------------------
% constraints
% ----------------------------------------
% ----------------------------------------

% [(1) No robot or part is chosen twice]
constraint all_different(executingRobots);
constraint all_different(chosenPart);

% [(2) Chosen robots can fulfill the roles]
% [EXT] here: chosen skills can fulfill the roles; equivalent in practice
constraint forall(rl in allocatedRoles)(executingSkill[rl] in possibleSkills[rl]);

% [(3) Chosen parts fulfill the requirements]
constraint forall(req in allocatedReqs)(chosenPart[req] in possibleParts[req]);

% [(4) Only available robots and parts currently held by available robots can be chosen]
constraint forall(rl in allocatedRoles)(executingSkill[rl] in availableSkills);
constraint forall(req in allocatedReqs)(chosenPart[req] in availableParts);

% [(5) Team members can reach the working position]
constraint forall(rl in allocatedRoles, wp in roleToWorkPos[rl])(minReach[executingSkill[rl]] <= workDist[roleRobotPos[rl], workPos[wp]]);
constraint forall(rl in allocatedRoles, wp in roleToWorkPos[rl])(workDist[roleRobotPos[rl], workPos[wp]] <= maxReach[executingSkill[rl]]);

% [(6) Robots can reach their target position and only available robots can move]
% constraint forall(d in moveDistByRobot)(d >= 0); TODO: delete this or the next line?
constraint forall(rob in ROBOT)(movedRobot[rob] -> targetPos[rob] in mcPos[mobilityClass[rob]]);
constraint forall(rob in ROBOT)(movedRobot[rob] -> not (targetPos[rob] in BLOCKEDPOS));
constraint forall(rob in ROBOT)(movedRobot[rob] -> robotAvailableMoving[rob]);

% [(7) Robots cannot share positions]
constraint all_different(targetPos);

% [(8) Robots on linear axes respect ordering and required offsets]
constraint forall(lmc in LMC, rob1 in robotsByMobilityClass[lmc], rob2 in robotsByMobilityClass[lmc])(mcLinearOrder[lmc, currentPos[rob1]] > mcLinearOrder[lmc, currentPos[rob2]] -> mcLinearOrder[lmc, targetPos[rob1]] > mcLinearOrder[lmc, targetPos[rob2]] + requiredOffset[lmc]);

% [EXT] work position cannot be set at position of robot not participating at the work there, as it could block the execution of the step
% This was simply left out of the paper for brevity, as it is usually fulfilled anyways if there needs to be a cart exactly at the work position (because of constraint 7.), which is the case in all examples we presented in the paper
% As this is not a permanent and fixed constraint for our system, we still need the constraint for the MiniZinc model to work in all possible cases
constraint forall(s in STEP, wp in stepToWorkPos[s], rob in ROBOT)(targetPos[rob]=workPos[wp] -> rob in teamForStep[s]);

% [EXT] parts must be held by a transfer skill or have a local successor that is a transfer skill
% This rules out allocating parts that cannot be transferred to another robot if needed
constraint forall(req in allocatedReqs)(exists(sk in allAvailableTransferSkills[deopt(chosenPart[req])])(targetSkill[req] in transitiveSuccessors[sk]));

% [EXT] don't move parts to free holders for which this is not allowed
constraint forall(sk in targetSkill)(freeCapacity[sk] = 0 -> exists(p in PART where initialHolder[p]=sk)(p in chosenPart \/ (p in availableParts /\ mayBeTransferredToFreeHolder[p])));

% ----------------------------------------
% ----------------------------------------
% Optimization
% ----------------------------------------
% ----------------------------------------

% ----------------------------------------
% input variables for optimization
% ----------------------------------------

% [EXT] The optimization is slightly more involved than presented in the paper
% As this is only a heuristic anyways and is part of continuous development, we decided to present our base considerations for an optimization function
% in the paper and leave out some of the more complex aspects which came into place during development and are not fixed permanently.
% The idea presented in the paper (punishing aspects which will likely lead to delays or problems during execution) is still fully applicable and many aspects can be mapped despite the
% formula not being exactly the same.

array[STEP] of VAL: importance; % importance of steps [importance]

array[ROBOT] of VAL: robotAllocationCost;   % cost for assigning a robot to the team [allocationCost]
array[SKILL] of VAL: assignmentCost;        % [EXT] additional cost for using a particular skill of a robot
array[SKILL] of VAL: totalSkillAssignmentCost = [assignmentCost[sk] + robotAllocationCost[skillToRobot[sk]]| sk in SKILL];
array[SKILL] of VAL: transferCost;          % [EXT] cost, if a part has to be transferred to this skill, i.e., if the skill must hold the part at the beginning of the step execution while it is not the initial holder of the part


array[ROBOT] of VAL: robotMoveCost;         % cost for moving a robot, regardless of distance [moveCost]
array[ROBOT] of VAL: robotMoveDistanceCost; % cost per distance unit for moving a robot [moveDistanceCost]

int: adjustmentPenalty;                     % [EXT] additional cost if the part is not held by a robot that can handle it and possibly change its orientation etc.
int: transferPenalty;                       % Additional cost if a part has to be transferred to another robot before the step execution can start [transferCost]
int: transferDistPenalty;                   % Additional cost per approximated distance unit if a part has to be transferred to another robot before the step execution can start [transferDistCost]

% [EXT] boolean flags to turn on/off certain optimization aspects; by default, all are true; in the paper, we just assume all are true and did not mention them
bool: optimizeAssignedSkills;
bool: optimizeTransfers;
bool: optimizeMovementNum;
bool: optimizeMovementDist;
bool: applyUnknownTransferPenalties;

% ----------------------------------------
% Calculation of the value of a team allocation result
% - This was somewhat simplified for the presentation in the paper. Approximate mapping:
%   + importanceSum                                 ≙ importance(ŝ)
%   - sum(assignmentCostArray)                      ≙ Σ allocationCost(r̂(rl)) and skill-level extension [EXT]
%   - sum(robotMoveCostArray)                       ≙ Σ moved(r) * moveCost(r)
%   - sum(robotMoveDistanceCostArray)               ≙ Σ d^μ(r)(pos0, pos_t) * moveDistCost(r)
%   - sum(transferCostArray)                        ≙ Σ transferred(req) * transferCost
%   - sum(unknownTransferPenalty)                   ≙ Σ transferred(req)*transferDistCost + adjustmentPenalty [EXT]
% ----------------------------------------

% helper variables for calculating the total cost
array[REQ] of var bool: adjustmentNeeded = array1d(REQ, [req in allocatedReqs /\ let {var PART: p = deopt(chosenPart[req]), var SKILL: sk = deopt(targetSkill[req])} in (not (sk in transitiveSuccessors[initialHolder[p]] /\ sk in potentialHandlers[p])) | req in REQ]);
array[REQ] of var bool: transferred = array1d(REQ, [req in allocatedReqs /\ let {var PART: p = deopt(chosenPart[req]), var SKILL: sk = deopt(targetSkill[req])} in (not (skillToRobot[initialHolder[p]] = skillToRobot[sk])) | req in REQ]);
array[REQ] of var DIST: requiredTransferDist = array1d(REQ, [if transferred[req] then let {var PART: p = deopt(chosenPart[req]), var SKILL: sk = deopt(targetSkill[req])} in (transferDist[currentPos[skillToRobot[initialHolder[p]]], targetPos[skillToRobot[sk]]]) else 0 endif | req in REQ]);
array[ROBOT] of var DIST: moveDistByRobot = [mcDist[mobilityClass[rob], currentPos[rob], targetPos[rob]]| rob in ROBOT];

% total importance of assigned steps (currently at most one step, so this is just the importance of that step or 0 if no step is assigned)
array[STEP] of var VAL: importanceArray = [if s in assignedSteps then importance[s] else 0 endif | s in STEP];
var VAL: importanceSum = sum(importanceArray);

% different cost aspects
array[ROLE] of var VAL: assignmentCostArray = [optimizeAssignedSkills * (if occurs(executingSkill[rl]) then totalSkillAssignmentCost[executingSkill[rl]] else 0 endif) | rl in ROLE];
array[REQ] of var VAL: transferCostArray = [optimizeTransfers * (if transferred[req] then transferCost[targetSkill[req]] else 0 endif) | req in REQ];
array[ROBOT] of var VAL: robotMoveCostArray = [optimizeMovementNum * movedRobot[rob] * robotMoveCost[rob] | rob in ROBOT];
array[ROBOT] of var VAL: robotMoveDistanceCostArray = [optimizeMovementDist * moveDistByRobot[rob] * robotMoveDistanceCost[rob] | rob in ROBOT];
array[REQ] of var VAL: unknownTransferPenalty = array1d(REQ, [applyUnknownTransferPenalties * (adjustmentNeeded[req] * adjustmentPenalty + transferred[req] * transferPenalty + requiredTransferDist[req] * transferDistPenalty) | req in REQ]);

% total costs
var VAL: costSum = sum(assignmentCostArray) + sum(transferCostArray) + sum(robotMoveCostArray) + sum(robotMoveDistanceCostArray) + sum(unknownTransferPenalty);

% total value to maximize
var VAL: value = importanceSum - costSum;

% ----------------------------------------
% ----------------------------------------
% unused input variables; only defined in order to keep the same inputs as for the action planning model
% ----------------------------------------
% ----------------------------------------

bool: allowHandlersToStoreParts; 
int: finalState;

% ----------------------------------------
% ----------------------------------------
% Definition of maximization task and search hints for solver (which speed up time needed by the solver significantly)
% ----------------------------------------
% ----------------------------------------

solve
::seq_search(
[
bool_search(movedRobot, smallest, indomain_min),
bool_search(transferred, smallest, indomain_min),
bool_search(adjustmentNeeded, smallest, indomain_min),
int_search(moveDistByRobot, smallest, indomain_min),
int_search(requiredTransferDist, smallest, indomain_min),
])
maximize value;
