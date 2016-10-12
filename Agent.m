classdef Agent
    
%*****************************************************************************************************%
%*                                                                                                   *%
%*   NAME: Agent                                                                                     *%
%*   DESCRIPTION:                                                                                    *%
%*   AUTHOR:                                                                                         *%
%*   DATE CREATION:08/10/2016                                                                        *%
%*   LAST MODIFIED:10/10/2016                                                                        *%
%*                                                                                                   *%
%*****************************************************************************************************%
    
    properties
        token %Intitialize false for all agents but one
        Position %current position of the agent (timestep k)
        Bid %last PPI calculated
        Map %map used by the agent to campute RRT
        path %current path
        timeSinceStart %number of steps from the beginning of the path 
        timeToGoal %number of timesteps from the beginning of the path until the goal
        lastMove % value of the last move (1 or 0)
        goal %position of the current goal
        posiblepath %potential new path
    end
    
    methods
        %%Class Constructor
        function agent = Agent(token,position,currentMap)
            if nargin == 3
                        agent.token = token;
                        agent.Position = position;
                        agent.Map = currentMap;
                        agent.timeSinceStart = 1;
                        agent.lastMove = 0;
            else
                    error('Incorrect Number of Inputs for class Agent')
            end
        end

        function Agent = updateToken(Agent,value) %Value should be 1 or 0
            Agent.token = value;
        end 
        
        function Agent = assignpath(Agent) %Value should be 1 or 0
            Agent.path = Agent.posiblepath;
            c = size(Agent.path); Agent.timeToGoal = c(1);
            Agent.timeSinceStart = 1;
            %Agent.Bid = - Inf;
        end 
        
        function Agent = updateMap(Agent,tile,value) %Updates the map of the agent with iformation from another agent
            Agent.Map(tile(1),tile(2)) = value;
        end 
        
        function Agent = calculatePath(Agent,otherPath) %Calculates the path using RRT
            Agent = findClosestTile(Agent);
            if isequal(Agent.goal,[-1 -1])
                 Agent.goal = [1,1];%change later
            else
                r = RRT(Agent.Map , Agent.Position , Agent.goal);
                r = runRRT(r);
                Agent.posiblepath = r.path;
                size1 = size(Agent.posiblepath);
                size2 = size(otherPath);
                time = min(size1(1),size2(1));
                for i = 2:time %IMPORTANT THAT THE POSITIONS OF THE PATHS MATCH IN TIME
                   if Agent.posiblepath(i,1) == otherPath(i,1) && Agent.posiblepath(i,2) == otherPath(i,2)
                       Agent.posiblepath = [Agent.posiblepath(1:i-1,:); Agent.posiblepath(i-1,:); Agent.posiblepath(i:end,:)];%stops twice in a tile if required
                   end
                end

                oldcost = length(Agent.path);
                if (isequal(Agent.goal,Agent.Position)) || (Agent.lastMove == 0)
                    Agent.Bid = length(Agent.posiblepath);
                else
                    Agent.Bid = oldcost - length(Agent.posiblepath);
                end
            end
            
            Agent.lastMove = 1;
            
        end 
        
        function Agent = findClosestTile(Agent) %Returns position [row,column] of closest tile. [-1,-1] if there are no more tiles to paint
            [rows,columns] = find(Agent.Map);
            closestDistance = Inf;
            Agent.goal = [-1,-1];
            for i=1:length(rows)
                if Agent.Map(rows(i),columns(i)) == 2
                    distance = abs(Agent.Position(1)-rows(i)) + abs(Agent.Position(2)-columns(i));
                    if distance < closestDistance
                        closestDistance = distance;
                        Agent.goal = [rows(i) columns(i)];
                    end
                end
            end
        end
        
        function Agent = move(Agent) %returns 1 if the agent moved towards the goal, 0 if he already reached the goal
            Agent.lastMove = 0;
            if Agent.timeSinceStart < Agent.timeToGoal
                Agent.Position = Agent.path(2,:);
                Agent.path = Agent.path(2:end,:);
                Agent.lastMove = 1;
                if isequal(Agent.Position,Agent.goal);
                   Agent.lastMove = 0;
                end
                Agent.timeSinceStart = Agent.timeSinceStart + 1;
            end
        end
    end
    
end

