%*****************************************************************************************************%
%*                                                                                                   *%
%*   NAME: AIprojectMain                                                                             *%
%*   DESCRIPTION: Multi-Agent tile painting using RRT                                                *%
%*   AUTHOR: Team 19                                                                                 *%
%*   DATE CREATION:08/10/2016                                                                        *%
%*   LAST MODIFIED:14/10/2016                                                                        *%
%*                                                                                                   *%
%*****************************************************************************************************%

% This main class deals with the followin functions
% 1. Read an 8 bit image and load it into a matrix.
% 2. Modify the image matrix to the form that is suited to RRT processing.
% 3. Initialize the agents.
% 4. Assign Random Starting points to the two agents conforming with the boundaries.
% 5. Decide which agent is going to start the proceedings and call the agent's RRT function with the matrix passed by value

% Define numer of agents
AGENT_COUNT = 3;

% Load Map
MainImage = importdata('Map0.bmp');
GlobalCopy = MainImage(1).cdata;
[Xmax,Ymax] = size(GlobalCopy);
GlobalCopy(GlobalCopy==79) = 5;     %   79	-> 5    (Red to obstacles)
GlobalCopy(GlobalCopy==0) = 2;      %   0	-> 2    (Black cell to picture to be painted)
GlobalCopy(GlobalCopy==255) = 0;    %   255	-> 0    (White to Empty Cell)

valid = false;

for i=1:AGENT_COUNT
    while(valid == false)
        Xinit(i) = ceil((Xmax-1)*rand);
        Yinit(i) = ceil((Ymax-1)*rand);
        Position = [Xinit(i),Yinit(i)];
        if(GlobalCopy(Xinit,Yinit) ~= 5)
            A(i) = Agent(Position,GlobalCopy);
            valid = true;
        end;
    end;
    valid = false;
end;


%AgentMap = zeros(Xmax,Ymax);                                %Map that holds agent's workspace
%To identify the processs start
Counter = 0;
for i=1:AGENT_COUNT
    if(isempty(A(i).path))
        A(i) = calculatePath(A(i),A(mod(i,3)+1).path,A(mod(i+1,3)+1).path);        %its initial, opPosition's path will be empty
        A(i) = assignpath(A(i));
        for j=1:AGENT_COUNT
            A(j) = updateMap(A(j),A(i).goal,3);
        end
    end;
end

obstacledMap = GlobalCopy;
obstacledMap(obstacledMap==2) = 5;

while(~isequal(A(1).Map,obstacledMap) && ~isequal(A(2).Map,obstacledMap) && ~isequal(A(3).Map,obstacledMap))                        %While (the map is not the goal map) iterate
    
    fprintf('timestep:%d , A(1) in position [%d %d] , A(2) in position [%d %d] and A(3) in position [%d %d]\n',Counter,A(1).Position(1),A(1).Position(2),A(2).Position(1),A(2).Position(2),A(3).Position(1),A(3).Position(2));
    fprintf('timestep:%d , A(1) wants to paint [%d %d] , A(2) wants to paint [%d %d] and A(3) wants to paint [%d %d]\n',Counter,A(1).goal(1),A(1).goal(2),A(2).goal(1),A(2).goal(2),A(3).goal(1),A(3).goal(2));
    
    Counter = Counter + 1;  % increase counter
    
    % Call the move function on both the agents and handle the results
    % below. The only place in the while loop where move will be called
    
    
    if(mod(Counter,1)==0) %update the plan of the agent that offers the best bid
        
        % Calculate potential path for all agents. avoid targeting the same goals
        
        for i=1:AGENT_COUNT
            if A(i).Map(A(i).goal(1),A(i).goal(2)) == 3
                for j=1:AGENT_COUNT
                    A(j) = updateMap(A(j),A(i).goal,2);
                end
            end
        end
        
        %verify there are no more 3's in the map
        for i=1:AGENT_COUNT
            for j=1:size(A(i).Map,1)
                for g=1:size(A(i).Map,2)
                    if A(i).Map(j,g)==3
                        A(i).Map(j,g)=2;
                    end
                end
            end
        end
        
        % Both agents calculate a posible new path
        
        A(1) = calculatePath(A(1),A(2).path,A(3).path);
        
        if ~isequal(A(1).Position,A(1).posiblegoal)
            A(2) = updateMap(A(2),A(1).posiblegoal,3);                 %prevent the 2 agents to go to the same goal
            A(2) = calculatePath(A(2),A(1).path,A(3).path);
            A(3) = updateMap(A(3),A(1).posiblegoal,3);
            if ~isequal(A(2).Position,A(2).posiblegoal)
                A(3) = updateMap(A(3),A(2).posiblegoal,3);
                A(3) = calculatePath(A(3),A(1).path,A(2).path);
                A(3) = updateMap(A(3),A(2).posiblegoal,2);
            end
            A(3) = updateMap(A(3),A(1).posiblegoal,2);
            A(2) = updateMap(A(2),A(1).posiblegoal,2);                 %change the map back
        end
        
        
        % The bids are compared and, if there is an averall improvement of
        % the task, one of the agent updated his path
        if A(1).Bid > 0 || A(2).Bid > 0 || A(3).Bid > 0 || A(2).Bid < 100 || A(1).Bid < 100 || A(3).Bid < 100
            if(A(2).Bid > A(1).Bid && A(2).Bid > A(3).Bid) && ~isequal(A(2).Position,A(2).posiblegoal) && ~isequal(A(1).goal,A(2).posiblegoal) && ~isequal(A(3).goal,A(2).posiblegoal)
                fprintf('\nA(2) replans\n\n')
                A(2) = assignpath(A(2));
                A(1) = updateMap(A(1),A(2).goal,3);
                A(2) = updateMap(A(2),A(2).goal,3);
                A(3) = updateMap(A(3),A(2).goal,3);
                
                if ~isequal(A(1).Position,A(1).goal)
                    A(1) = updateMap(A(1),A(1).goal,3);
                    A(2) = updateMap(A(2),A(1).goal,3);
                end
                
                if ~isequal(A(3).Position,A(3).goal)
                    A(1) = updateMap(A(1),A(3).goal,3);
                    A(2) = updateMap(A(2),A(3).goal,3);
                end
                
            elseif (A(1).Bid > A(2).Bid && A(1).Bid > A(3).Bid) && ~isequal(A(1).Position,A(1).posiblegoal) && ~isequal(A(2).goal,A(1).posiblegoal) && ~isequal(A(3).goal,A(1).posiblegoal)
                fprintf('\nA(1) replans\n\n')
                A(1) = assignpath(A(1));
                A(1) = updateMap(A(1),A(1).goal,3);
                A(2) = updateMap(A(2),A(1).goal,3);
                A(3) = updateMap(A(3),A(1).goal,3);
                
                if ~isequal(A(2).Position,A(2).goal)
                    A(1) = updateMap(A(1),A(2).goal,3);
                    A(2) = updateMap(A(2),A(2).goal,3);
                end
                
                if ~isequal(A(3).Position,A(3).goal)
                    A(1) = updateMap(A(1),A(3).goal,3);
                    A(2) = updateMap(A(2),A(3).goal,3);
                end
            elseif (A(3).Bid > A(1).Bid && A(3).Bid > A(2).Bid) && ~isequal(A(3).Position,A(3).posiblegoal) && ~isequal(A(2).goal,A(3).posiblegoal) && ~isequal(A(1).goal,A(3).posiblegoal)
                fprintf('\nA(3) replans\n\n')
                A(3) = assignpath(A(3));
                A(1) = updateMap(A(1),A(3).goal,3);
                A(2) = updateMap(A(2),A(3).goal,3);
                A(3) = updateMap(A(3),A(3).goal,3);
                
                if ~isequal(A(2).Position,A(2).goal)
                    A(1) = updateMap(A(1),A(2).goal,3);
                    A(2) = updateMap(A(2),A(2).goal,3);
                end
                
                if ~isequal(A(1).Position,A(1).goal)
                    A(1) = updateMap(A(1),A(1).goal,3);
                    A(2) = updateMap(A(2),A(1).goal,3);
                end
            end
        end
    end
    
    %Agents move forward
    
    A(1) = move(A(1));
    A(2) = move(A(2));
    A(3) = move(A(3));
    
    %Verify that any agent hit an obstacle and that they are not in the same position
    
    if A(1).Map(A(1).Position(1),A(1).Position(2)) == 5 && A(1).lastMove == 1
        fprintf('\nHit an obstacle!: %d %d by A(1)\n\n',A(1).Position(1),A(1).Position(2));
    end
    if A(2).Map(A(2).Position(1),A(2).Position(2)) == 5 && A(2).lastMove == 1
        fprintf('\nHit an obstacle!: %d %d by A(2)\n\n',A(2).Position(1),A(2).Position(2));
    end
    if A(3).Map(A(3).Position(1),A(3).Position(2)) == 5 && A(3).lastMove == 1
        fprintf('\nHit an obstacle!: %d %d by A(3)\n\n',A(3).Position(1),A(3).Position(2));
    end
    
    if isequal(A(2).Position,A(1).Position) && A(1).lastMove == 1 && A(2).lastMove == 1
        fprintf('\nAgents 1 and 2 crashed!!\n\n')
    end
    
    if isequal(A(3).Position,A(1).Position) && A(1).lastMove == 1 && A(2).lastMove == 1
        fprintf('\nAgents 1 and 3 crashed!!\n\n')
    end
    
    if isequal(A(2).Position,A(3).Position) && A(1).lastMove == 1 && A(2).lastMove == 1
        fprintf('\nAgents 2 and 3 crashed!!\n\n')
    end
    
    if (isequal(A(1).goal,A(1).Position)) && (A(1).Map(A(1).Position(1),A(1).Position(2))~=5) && (A(2).Map(A(1).Position(1),A(1).Position(2))~=5) && A(1).lastMove == 1  % one agent is still on the move.
        
        A(1) = updateMap(A(1),A(1).goal,5);
        A(2) = updateMap(A(2),A(1).goal,5);
        A(3) = updateMap(A(3),A(1).goal,5);
        
        fprintf('A(1) painted a tile : %d,%d\n',A(1).goal(1),A(1).goal(2));
        
        for i=1:AGENT_COUNT
            if(i ~= 1)
                contains = 0;
                for j=1:size(A(i).path,1)
                    if isequal(A(i).path(j,:),A(1).goal)
                        contains = 1;
                    end
                end
                if contains ==1
                    A(i) = calculatePath(A(i),A(mod(i,3)+1).path,A(mod(i+1,3)+1).path);        %its initial, opPosition's path will be empty
                    A(i) = assignpath(A(i));
                    fprintf('\nA(%d) replans\n\n',i);
                    Counter = Counter + 1;
                    for y=1:AGENT_COUNT
                        A(y) = updateMap(A(y),A(i).goal,3);
                    end
                end
            end
        end
                
    end
    
    if (isequal(A(2).goal,A(2).Position)) && (A(2).Map(A(2).Position(1),A(2).Position(2))~=5) && (A(1).Map(A(2).Position(1),A(2).Position(2))~=5) && A(2).lastMove == 1 % one agent is still on the move.
        
        A(1) = updateMap(A(1),A(2).goal,5);
        A(2) = updateMap(A(2),A(2).goal,5);
        A(3) = updateMap(A(3),A(2).goal,5);
        
        fprintf('A(2) painted a tile : %d,%d\n',A(2).goal(1),A(2).goal(2));
        
        for i=1:AGENT_COUNT
            if(i ~= 2)
                contains = 0;
                for j=1:size(A(i).path,1)
                    if isequal(A(i).path(j,:),A(2).goal)
                        contains = 1;
                    end
                end
                if contains ==1
                    A(i) = calculatePath(A(i),A(mod(i,3)+1).path,A(mod(i+1,3)+1).path);        %its initial, opPosition's path will be empty
                    A(i) = assignpath(A(i));
                    fprintf('\nA(%d) replans\n\n',i);
                    Counter = Counter + 1;
                    for y=1:AGENT_COUNT
                        A(y) = updateMap(A(y),A(i).goal,3);
                    end
                end
            end
        end
        
    end
    
    if (isequal(A(3).goal,A(3).Position)) && (A(3).Map(A(3).Position(1),A(3).Position(2))~=5) && A(3).lastMove == 1  % one agent is still on the move.
        
        A(1) = updateMap(A(1),A(3).goal,5);
        A(2) = updateMap(A(2),A(3).goal,5);
        A(3) = updateMap(A(3),A(3).goal,5);
        
        fprintf('A(3) painted a tile : %d,%d\n',A(3).goal(1),A(3).goal(2));
        
        for i=1:AGENT_COUNT
            if(i ~= 3)
                contains = 0;
                for j=1:size(A(i).path,1)
                    if isequal(A(i).path(j,:),A(3).goal)
                        contains = 1;
                    end
                end
                if contains ==1
                    A(i) = calculatePath(A(i),A(mod(i,3)+1).path,A(mod(i+1,3)+1).path);        %its initial, opPosition's path will be empty
                    A(i) = assignpath(A(i));
                    fprintf('\nA(%d) replans\n\n',i);
                    Counter = Counter + 1;
                    for y=1:AGENT_COUNT
                        A(y) = updateMap(A(y),A(i).goal,3);
                    end
                end
            end
        end        
    end
end

fprintf('Time take for the Multi Agent planning :%d\n',Counter);