% =========================================================
% Action Planning CSOP
% ---------------------------------------------------------
% This is a the actual model used for the action planning in our system.
% For some general comments about comparison between the mathematical model presented in the paper and the MiniZinc model used in practice, please refer to the comments in the team allocation MiniZinc model.
%
% NOTE: This action planning model is included for transparency.
% It was not described in detail in the paper and is less polished than the team allocation model.
% It nevertheless corresponds to the approach outlined in the paper and is used in our system.
%
% It uses the same input as the team allocation model + the result of the team allocation as additional input.
% From there it creates a sequence of intermediate states between the initial state and the final state where the actual step execution can begin.
% The system can then transform this sequence of states into a semi-ordered collection of actions to be executed.
% =========================================================
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
% In the paper simplified to a simple available/unavailable distinction for each robot
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
array[SKILL] of bool: skillAvailable = [currentSkill[sk] /\ ((activeSkill[sk] /\ robotAvailableActive[skillToRobot[sk]]) \/ ((not activeSkill[sk]) /\ robotAvailablePassive[skillToRobot[sk]])) | sk in SKILL];
set of SKILL: availableSkills = {sk | sk in SKILL where skillAvailable[sk]};

% additional helper variables for action planning
array[SKILL, POS] of set of POS: reachablePositions = array2d(SKILL, POS,[{p | p in POS where workDist[p0, p] in minReach[sk]..maxReach[sk]} |sk in SKILL, p0 in POS]);
array[SKILL] of CAPACITY: maxCapacity = [freeCapacity[sk] + count(initialHolder, sk)|sk in SKILL];
array[ROBOT] of set of SKILL: skillsForRobot = [{sk | sk in SKILL where skillToRobot[sk]=rob /\ skillAvailable[sk]}| rob in ROBOT];

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

% [EXT] Some (small) parts may freely be moved to free potential holders to make space at their current holder, while other (large) ones should only be moved if absolutely necessary
% This is not necessary for the theoretical model we presented, but this additional variable allows us to constrain the system, so that some larger parts will never be handed over from there current holder to just free up a cart for other parts
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
% team allocation result
% [EXT] cf. comments for team allocation MiniZinc model
% ----------------------------------------

array[TASK] of opt STEP: assignedStepForTask;   % assigned steps; always at most one per task; currently at most one per team allocation [\hat{s}]
array[ROLE] of opt SKILL: executingSkill;       % skill used for executing the role [\hat{r}] [EXT] In the paper, the robot is chosen directly instead of the appropriate skill, that does not really matter in practice
array[REQ] of opt PART: chosenPart;             % part used for the required part [\hat{p}]
array[WORKPOS] of opt FREEWORKPOS: workPos;     % working position assigned for the step [pos_w]
array[ROBOT] of POS: targetPos;                 % target position for each robot when the step can start [pos_t]

% ----------------------------------------
% helper variables derived from the team allocation result
% ----------------------------------------
set of STEP: assignedSteps = {s | s in STEP where s in assignedStepForTask};
set of ROLE: allocatedRoles = {rl| t in assignedSteps, rl in stepToRoles[t]};   % [Rl^\hat{s}]
set of REQ: allocatedReqs = {req| t in assignedSteps, req in stepToReqs[t]};    % [Req^\hat{s}]
set of PART: chosenParts = {p | p in PART where p in chosenPart};

array[ROLE] of opt ROBOT: executingRobots = [if occurs(executingSkill[r]) then skillToRobot[deopt(executingSkill[r])] else <> endif| r in ROLE];
set of ROBOT: allocatedRobots = {deopt(executingRobots[r])| r in allocatedRoles};

array[PART] of opt REQ: partToReq = [let {set of REQ: reqs = {req | req in REQ where chosenPart[req] = p}} in (if card(reqs) = 0 then <> else min(reqs) endif)| p in PART];
array[PART] of opt SKILL: targetSkill = [if occurs(partToReq[p]) then executingSkill[targetRole[partToReq[p]]] else <> endif| p in PART];

% ----------------------------------------
% states
% For the action planning the solver creates array of states representing intermediate situations between the initial state and the final state where the actual step execution can begin
% "Final state" therefor, in this context, means the state after all actions to prepare the step but before the actual step execution
% Between any two consecutive states, every robot can move at most once and every part can be transferred at most once
% A state change can therefore combine multiple independent actions
% From the sequence of states, the system then recreates the move and transfer actions taken, recognizes the actual minimum logical dependencies and creates a semi-ordered collection of such actions,which si the actual result of the action planning.
% These latter aspects take place outside of the MiniZinc model and cannot be clearly recognized here.
% MiniZinc is just tasked with creating the sequence of states.
% ----------------------------------------

int: finalState;                                    % number of consecutive states after the initial state considered in the plan [n]
% 0: initial state (given by input); 1..finalState-1: intermediate states (decided by the solver); finalState: final state (restricted by the team allocation result)
set of int: STATE = 1..finalState;                  % set of states after the initial state
set of int: STATEx = 0..finalState;                 % set of states including the initial state

% Setting finalState too low might lead to no solution being found despite the state for the step execution actually being reachable.
% Setting it too high significantly increases the size of the search space and therefore the solving time, eventually to unacceptable levels.
% The value should therefore be chosen carefully, depending on the complexity of the scenario.
% This issue shows the problems of losing a solver meant for traditional CS(O)P problems for such temporal planning problems.
% We did this anyways to create a more uniform model together with the team, allocation, possibly even alloing both to be solved together (which quickly proved too inefficient even for small problems).
% As we mentioned in the paper, we now believe that while this is a good benchmark solution for us, using a more dedicated planner for these types of problems is more sensible.
% Therefore, we already created on a Planning Domain Definition Language (PDDL) representation of the problem and expect a solution using a PDDL planner to be fully available in the near future.
% We plan to present this in a follow-up publication.


% ----------------------------------------
% additional planning parameters
% ----------------------------------------

bool: allowHandlersToStoreParts;                    % [EXT] Whether handling skills (like PickAndPlace of an industrial robot) are allowed to store parts between assembly step executions (true) or only temporarily during the plan execution, with the part having to be transferred to a pure holding skill within the same plan (false)
% This is an optimization that avoids our gripper robots tro be misused as a part storage. As it is not necessary to find legal solutions, it can be switched on/off as desired.

% ----------------------------------------
% ----------------------------------------
% decision variables
% ----------------------------------------
% ----------------------------------------

% sequence of states characterized by the positions of the robots and the holders of the parts
array[STATEx, ROBOT] of var POS: positionAtTime;        % [pos_i]
array[STATEx, PART] of var SKILL: holderAtTime;         % [h_i]

% positions for transfers of parts between robots
set of POS: HANDOVERPOS = {p|p in POS where not exists(lmc in LMC)(p in mcPos[lmc])};
array[STATE, PART] of var opt HANDOVERPOS: handoverPoint;

% a skill can only give OR receive a part during a state change, not both. This makes the constraints as there is a clear order between such transfers.
array[STATE, SKILL] of var bool: mayGive;
array[STATE, SKILL] of var bool: mayReceive;

% For every required part, there must be a final adjustment action responsible for moving the part to the target role in the proper orientation etc.
% This must be a handling skill and not a pure holder
% In our experiments, this is what is responsible for the additional part transfers mentioned in the text, as it is not sufficient if the part is already held by the right cart, as a cart cannot make any adjustments necessary before the skill can start, so a robotic arm must take the cart and put in back
array[REQ] of var opt STATEx: finalAdjustmentState;
array[REQ] of var opt SKILL: finalAdjustmentSkill;

% ----------------------------------------
% helper variables depending on decision variables
% ----------------------------------------

array[PART] of var opt STATE: finalAdjustmentStateForPart = [if occurs(partToReq[p]) then finalAdjustmentState[partToReq[p]] else <> endif| p in PART];
array[PART] of var opt SKILL: finalAdjustmentSkillForPart = [if occurs(partToReq[p]) then  finalAdjustmentSkill[partToReq[p]] else <> endif| p in PART];

% identifying which kind of state changes took place between two consecutive states
array[STATE, ROBOT] of var bool: movedRobot = array2d(STATE, ROBOT,[positionAtTime[i, rob] != positionAtTime[i-1, rob]|i in STATE, rob in ROBOT]);  % whether the robot moved from state i-1 to state i
array[STATEx, PART] of var ROBOT: robotAtTime = array2d(STATEx, PART,[skillToRobot[holderAtTime[i, p]]|i in STATEx, p in PART]);
array[STATE, PART] of var bool: transferedPart = array2d(STATE, PART,[holderAtTime[i, p] != holderAtTime[i-1, p]|i in STATE, p in PART]);           % whether the part was transferred from one skill to another between state i-1 and state i
array[STATE, PART] of var bool: nonLocalTransfer = array2d(STATE, PART,[robotAtTime[i, p] != robotAtTime[i-1, p]|i in STATE, p in PART]);           % whether the part was transferred between two different robots between state i-1 and state i
array[STATE, PART] of var bool: localTransfer = array2d(STATE, PART,[transferedPart[i,p] /\ not nonLocalTransfer[i,p]|i in STATE, p in PART]);      % whether the part was transferred between two skills of the same robot between state i-1 and state i

% parts controlled by a skill in a certain state
array[STATEx, SKILL] of var set of PART: partsOfSkill;
constraint forall(i in STATEx)(int_set_channel(holderAtTime[i, PART], partsOfSkill[i, SKILL]));

array[STATE, ROBOT] of var DIST: movedDist = array2d(STATE, ROBOT,[mcDist[mobilityClass[rob], positionAtTime[i-1, rob], positionAtTime[i, rob]] |i in STATE, rob in ROBOT]);

% ----------------------------------------
% ----------------------------------------
% constraints
% ----------------------------------------
% ----------------------------------------

% ----------------------------------------
% % data consistency
% ----------------------------------------

constraint forall(i in STATE, p in PART)(occurs(handoverPoint[i,p]) <-> nonLocalTransfer[i,p]); % handover points are specified exactly if a part is handed over to another robot

% final adjustment actions are specified exactly if the corresponding part will be used for the assembly step
constraint forall(req in REQ)(occurs(finalAdjustmentState[req]) <-> (req in allocatedReqs));
constraint forall(req in REQ)(occurs(finalAdjustmentSkill[req]) <-> (req in allocatedReqs));

% ----------------------------------------
% initial state is as specified by the input
% ----------------------------------------

constraint forall(rob in ROBOT)(positionAtTime[0, rob] = currentPos[rob]);
constraint forall(p in PART)(holderAtTime[0, p] = initialHolder[p]);

% ----------------------------------------
% team allocation result is realized in the final state (as far as this is required by the system)
% ----------------------------------------

% robots participating in the team will move to their assigned target positions (later skill parameter planning may deviate from this); other robots are irrelevant and must not move to the position specified by the team allocation result
constraint forall(rob in allocatedRobots)(positionAtTime[finalState, rob] = targetPos[rob]);

% parts needed for the step will be held by the skill executing the corresponding role in the final state
constraint forall(p in chosenParts)(holderAtTime[finalState, p] = targetSkill[p]);

% Chosen skills for the step that are not set to hold a part may not hold a part in the final state (prohibiting - for example - an already used gripper from being assigned in a step)
constraint forall(sk in executingSkill)(sk in holderAtTime[finalState, PART] -> sk in targetSkill);

% ----------------------------------------
% moves are possible
% ----------------------------------------

% robots can never move to a position their mobility class cannot reach
constraint forall(i in STATE, rob in ROBOT)(positionAtTime[i, rob] in mcPos[mobilityClass[rob]]);

% robots must be able to directly reach their next position
constraint forall(d in movedDist)(d >= 0);

% robots prohibited from moving can never change their position
constraint forall(rob in ROBOT, i in STATE)(not (robotAvailableMoving[rob]) -> positionAtTime[i, rob] = currentPos[rob]);

% no two robots are ever at the same grid position; as the order of moves within one state change is not defined, they also cannot move to a position that was occupied by another robot in the immediately previous state
constraint forall(i in STATE)(all_different(positionAtTime[i,ROBOT]));
constraint forall(i in STATE, rob in ROBOT)(movedRobot[i,rob] -> not (positionAtTime[i, rob] in positionAtTime[i-1, ROBOT]));

% robots cannot move to blocked positions
constraint forall(i in STATE, rob in ROBOT)(movedRobot[i,rob] -> not (positionAtTime[i, rob] in BLOCKEDPOS));

% robots on linear axes cannot jump over other robots and must leave the required space
% This must hold regardless of the order of moves in one state change, therefore we must check all combinations of previous and new positions
constraint forall(lmc in LMC, rob0 in robotsByMobilityClass[lmc], rob1 in robotsByMobilityClass[lmc])(mcLinearOrder[lmc, currentPos[rob0]] > mcLinearOrder[lmc, currentPos[rob1]] -> forall(i in STATE)(mcLinearOrder[lmc, positionAtTime[i, rob0]] > mcLinearOrder[lmc, positionAtTime[i, rob1]] + requiredOffset[lmc]));
constraint forall(lmc in LMC, rob0 in robotsByMobilityClass[lmc], rob1 in robotsByMobilityClass[lmc])(mcLinearOrder[lmc, currentPos[rob0]] > mcLinearOrder[lmc, currentPos[rob1]] -> forall(i in STATE)(mcLinearOrder[lmc, positionAtTime[i-1, rob0]] > mcLinearOrder[lmc, positionAtTime[i, rob1]] + requiredOffset[lmc]));
constraint forall(lmc in LMC, rob0 in robotsByMobilityClass[lmc], rob1 in robotsByMobilityClass[lmc])(mcLinearOrder[lmc, currentPos[rob0]] > mcLinearOrder[lmc, currentPos[rob1]] -> forall(i in STATE)(mcLinearOrder[lmc, positionAtTime[i, rob0]] > mcLinearOrder[lmc, positionAtTime[i-1, rob1]] + requiredOffset[lmc]));



% ----------------------------------------
% transfers are possible
% ----------------------------------------

% skills participating in transfer must be available able to actually hold the part
constraint forall(i in STATE, p in PART)(transferedPart[i,p] -> holderAtTime[i,p] in allAvailableSkills[p]);
constraint forall(i in STATE, p in PART)(transferedPart[i,p] -> holderAtTime[i-1,p] in allAvailableSkills[p]);

% skill capacity is always respected
constraint forall(i in STATE, sk in SKILL)(count(holderAtTime[i, PART], sk) <= maxCapacity[sk]);

% skill may either give or receive parts in order to avoid cycles where capacity constraints would only be fulfilled if multiple actions occured simultaneosuly, where this might not actually be possible (e.g., two robots "exchange" their gripped parts)
constraint forall(i in STATE, p in PART  where nonLocalTransfer[i,p])(mayGive[i, holderAtTime[i-1,p]]);
constraint forall(i in STATE, p in PART where nonLocalTransfer[i,p])(mayReceive[i, holderAtTime[i,p]]);
constraint forall(i in STATE, sk in SKILL)(not (mayGive[i,sk] /\ mayReceive[i,sk]));

% all transfers are possible based on the skills involved (either both are transfer skills or it is a legal internal transfer within the same robot)
constraint forall(i in STATE, p in PART where localTransfer[i,p])(holderAtTime[i, p] in internalSuccessorSkills[holderAtTime[i-1, p]]);
constraint forall(i in STATE, sk in SKILL, p in (partsOfSkill[i, sk] union partsOfSkill[i-1, sk]) where nonLocalTransfer[i,p])(sk in allAvailableTransferSkills[p]);

% handover point can be reached
constraint forall(i in STATE, rob in ROBOT, sk in skillsForRobot[rob] where transferSkill[sk], p in (partsOfSkill[i, sk] union partsOfSkill[i-1, sk]) where nonLocalTransfer[i,p])(handoverPoint[i,p] in reachablePositions[sk, positionAtTime[i, rob]]);

% handover point is not occupied by a robot other than the old or new holder
constraint forall(i in STATE, p in PART where nonLocalTransfer[i,p])((handoverPoint[i,p] in positionAtTime[i, ROBOT]) -> handoverPoint[i,p] = positionAtTime[i, robotAtTime[i, p]] \/ handoverPoint[i,p] = positionAtTime[i, robotAtTime[i-1, p]]);

% only transfer parts chosen for the step or the ones for which moving them to free holders is allowed
constraint forall(p in PART where not (mayBeTransferredToFreeHolder[p] \/ p in chosenPart), i in STATE)(holderAtTime[i,p] = initialHolder[p]);

% ----------------------------------------
% final adjustment action is set in a correct way
% ----------------------------------------

% final adjustment state is the last state where a part is transferred
constraint forall(p in chosenParts)(finalAdjustmentStateForPart[p] > 0 -> transferedPart[finalAdjustmentStateForPart[p], p]);
constraint forall(i in STATE, p in chosenParts)(i > finalAdjustmentStateForPart[p] -> holderAtTime[i, p] = holderAtTime[finalAdjustmentStateForPart[p], p]);

% final adjustment skill participates in final transfer or is the initial holder if no transfer takes place
constraint forall(p in chosenParts)(finalAdjustmentStateForPart[p] > 0 -> finalAdjustmentSkillForPart[p] = holderAtTime[finalAdjustmentStateForPart[p], p] \/ finalAdjustmentSkillForPart[p] = holderAtTime[finalAdjustmentStateForPart[p] - 1, p]);
constraint forall(p in chosenParts)(finalAdjustmentStateForPart[p] = 0 -> finalAdjustmentSkillForPart[p] = initialHolder[p]);

% final adjustment skill can handle the part
constraint forall(p in chosenParts)(finalAdjustmentSkillForPart[p] in availableHandlers[p]);

% ----------------------------------------
% optional constraints
%----------------------------------------

% if set so, handlers will be prohibited from holding parts that are not used for the step after the last transfer
constraint not allowHandlersToStoreParts -> forall(p in PART where not (p in chosenParts))(not (holderAtTime[finalState, p] in potentialHandlers[p]));

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
array[SKILL] of VAL: transferCost;          % [EXT] cost, if a part has to be transferred to this skill; applied for any transfer


array[ROBOT] of VAL: robotMoveCost;         % cost for moving a robot once, regardless of distance [moveCost]
array[ROBOT] of VAL: robotMoveDistanceCost; % cost per distance unit for moving a robot [moveDistanceCost]

% [EXT] boolean flags to turn on/off certain optimization aspects; by default, all are true; in the paper, we just assume all are true and did not mention them
bool: optimizeAssignedSkills;
bool: optimizeTransfers;
bool: optimizeMovementNum;
bool: optimizeMovementDist;
bool: applyUnknownTransferPenalties;

% ----------------------------------------
% Calculation of the value of an action planning result
% ----------------------------------------

% helper variables for calculating the total cost
array[SKILL] of var STATEx: transfersBySkill = [sum(array2d(STATE, PART,[transferedPart[i, p] /\ holderAtTime[i, p]=sk| i in STATE, p in PART]))| sk in SKILL];
array[ROBOT] of var STATEx: numMovesByRobot = [sum(movedRobot[STATE, rob])| rob in ROBOT];
array[ROBOT] of var 0..: moveDistByRobot = [sum(movedDist[STATE, rob])| rob in ROBOT];

% total importance of assigned steps (currently at most one step, so this is just the importance of that step or 0 if no step is assigned)
array[STEP] of var VAL: importanceArray = [if s in assignedSteps then importance[s] else 0 endif | s in STEP];
var VAL: importanceSum = sum(importanceArray);

% different cost aspects
array[SKILL] of var VAL: assignmentCostArray = [optimizeAssignedSkills * (sk in executingSkill) * totalSkillAssignmentCost[sk] | sk in SKILL];
array[SKILL] of var VAL: transferCostArray = [optimizeTransfers * transfersBySkill[sk] * transferCost[sk]| sk in SKILL];
array[ROBOT] of var VAL: robotMoveCostArray = [optimizeMovementNum * numMovesByRobot[rob] * robotMoveCost[rob] | rob in ROBOT];
array[ROBOT] of var VAL: robotMoveDistanceCostArray = [optimizeMovementDist * moveDistByRobot[rob] * robotMoveDistanceCost[rob] | rob in ROBOT];

array[SKILL] of var VAL: totalSkillCost = [assignmentCostArray[sk] + transferCostArray[sk] | sk in SKILL];
array[ROBOT] of var VAL: totalRobotMovementCost = [robotMoveCostArray[rob] + robotMoveDistanceCostArray[rob]| rob in ROBOT];
array[ROBOT] of var VAL: totalRobotCost = [totalRobotMovementCost[rob] + sum(sk in skillsForRobot[rob])(totalSkillCost[sk])| rob in ROBOT];

% total costs
var VAL: costSum = sum(totalRobotCost);

% total value to maximize
var VAL: value = importanceSum - costSum;

% ----------------------------------------
% ----------------------------------------
% unused input variables; only defined in order to keep the same inputs as for the team allocation model
% ----------------------------------------
% ----------------------------------------
int: adjustmentPenalty;
int: transferPenalty;
int: transferDistPenalty;

% ----------------------------------------
% ----------------------------------------
% Definition of maximization task and search hints for solver (which speed up time needed by the solver significantly)
% ----------------------------------------
% ----------------------------------------

solve
:: seq_search(
[
bool_search(transferedPart, smallest, indomain_min),
bool_search(nonLocalTransfer, smallest, indomain_min),
bool_search(movedRobot, smallest, indomain_min),
int_search(movedDist, smallest, indomain_min),
])
maximize value;
