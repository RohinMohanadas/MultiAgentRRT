classdef Agent
    
%*****************************************************************************************************%
%*                                                                                                   *%
%*   NAME: Agent                                                                                     *%
%*   DESCRIPTION: Class used to model the behavior of the agent performing the task                  *%
%*   AUTHOR: Team 19                                                                                 *%
%*   DATE CREATION:08/10/2016                                                                        *%
%*   LAST MODIFIED:14/10/2016                                                                        *%
%*                                                                                                   *%
%*****************************************************************************************************%
    
    properties
        Position %Current position of the agent (timestep k)
        Bid %Current Bid
        Map %Map used by the agent to campute RRT
        path %Current path
        timeSinceStart %Number of steps from the beginning of the path
        timeToGoal %Number of timesteps from the beginning of the path until the goal
        lastMove %Value of the last move (1 or 0)
        goal %Position of the current goal
        posiblepath %Potential new path
        posiblegoal %Potential new path
        lastPosition %Position in the previous timestep
    end
    
    methods
        
        %%Class Constructor
        function agent = Agent(position,currentMap)
            if nargin == 2
                agent.Position = position;
                agent.Map = currentMap;
                agent.timeSinceStart = 1;
            else
                error('Incorrect Number of Inputs for class Agent')
            end
        end
        
        %% Update the and the goal with posiblepath and posible goal
        function Agent = assignpath(Agent) %Value should be 1 or 0
            Agent.path = Agent.posiblepath;
            Agent.goal = Agent.posiblegoal;
            c = size(Agent.path); Agent.timeToGoal = c(1);
            Agent.timeSinceStart = 1;
            Agent.lastMove = 0;
        end
        
        %%Update the Agent Map with the desired value
        function Agent = updateMap(Agent,tile,value) %Updates the map of the agent with iformation from another agent
            Agent.Map(tile(1),tile(2)) = value;
        end
        
        %%Find the closest tile to paint, calculate a path to reach it and
        %%compute the bid
        function Agent = calculatePath(Agent,otherPath1,otherPath2) %Calculates the path using RRT
            Agent = findClosestTile(Agent);
            r = RRT(Agent.Map , Agent.Position , Agent.posiblegoal);
            r = runRRT(r);
            if r.path_found == true
                Agent.posiblepath = r.path;
                size1 = size(Agent.posiblepath);
                size2 = size(otherPath1);
                size3 = size(otherPath2);
                time = min(size1(1),size2(1));
                for i = 2:time
                    if Agent.posiblepath(i,1) == otherPath1(i,1) && Agent.posiblepath(i,2) == otherPath1(i,2)
                        Agent.posiblepath = [Agent.posiblepath(1:i-1,:); Agent.posiblepath(i-1,:); Agent.posiblepath(i:end,:)];%stops twice in a tile if required
                    end
                end
                time = min(size1(1),size3(1));
                for i = 2:time
                    if Agent.posiblepath(i,1) == otherPath2(i,1) && Agent.posiblepath(i,2) == otherPath2(i,2)
                        Agent.posiblepath = [Agent.posiblepath(1:i-1,:); Agent.posiblepath(i-1,:); Agent.posiblepath(i:end,:)];%stops twice in a tile if required
                    end
                end
                oldcost = length(Agent.path);
                if (isequal(Agent.goal,Agent.Position)) && (Agent.lastMove == 0)
                    Agent.Bid = length(Agent.posiblepath);
                else
                    Agent.Bid = oldcost - length(Agent.posiblepath);
                    if Agent.Bid == 0
                        Agent.Bid = -20;
                    end
                end
                Agent.lastPosition = Agent.Position;
            end
        end
        
        %% Agent finds the closest tiles that needs to be painted
        function Agent = findClosestTile(Agent) %Returns position [row,column] of closest tile. [-1,-1] if there are no more tiles to paint
            [rows,columns] = find(Agent.Map);
            closestDistance = Inf;
            for i=1:length(rows)
                if Agent.Map(rows(i),columns(i)) == 2
                    distance = abs(Agent.Position(1)-rows(i)) + abs(Agent.Position(2)-columns(i));
                    if distance < closestDistance
                        closestDistance = distance;
                        Agent.posiblegoal = [rows(i) columns(i)];
                    end
                end
            end
        end
        
        %% Agent moves one step forward according to the current plan
        function Agent = move(Agent) %returns 1 if the agent moved towards the goal, 0 if he already reached the goal
            Agent.lastMove = 0;
            Agent.lastPosition = Agent.Position;
            
            if Agent.timeSinceStart == Agent.timeToGoal && ~isequal(Agent.lastPosition,Agent.Position)
                Agent.lastMove = 1;
            end
            
            if Agent.timeSinceStart < Agent.timeToGoal
                Agent.Position = Agent.path(2,:);
                Agent.path = Agent.path(2:end,:);
                Agent.lastMove = 1;
                Agent.timeSinceStart = Agent.timeSinceStart + 1;
            end
            
            if Agent.timeSinceStart == 2 && isequal(Agent.lastPosition,Agent.Position)
                Agent.lastMove = 0;
            end
        end
    end
end

