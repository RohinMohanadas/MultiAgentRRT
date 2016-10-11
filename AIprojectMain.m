% This main class deals with the followin functions
% 1. Read an 8 bit image and load it into a matrix.
% 2. Modify the image matrix to the form that is suited to RRT processing.
% 3. Initialize the agents.
% 4. Assign Random Starting points to the two agents conforming with the boundaries.
% 5. Decide which agent is going to start the proceedings and call the agent's RRT function with the matrix passed by value

AGENT_COUNT = 2;

MainImage = importdata('256TestFile.bmp');
GlobalCopy = MainImage(1).cdata;
[Xmax,Ymax] = size(GlobalCopy);
GlobalCopy(GlobalCopy==79) = 5;     %   79	-> 5    (Red to obstacles)
GlobalCopy(GlobalCopy==0) = 2;      %   0	-> 2    (Black cell to picture to be painted)
GlobalCopy(GlobalCopy==255) = 0;    %   255	-> 0    (White to Empty Cell)

for i = 1:AGENT_COUNT
    Xinit(i) = floor((Xmax-1)*rand);
    Yinit(i) = floor((Ymax-1)*rand);
    if(GlobalCopy(Xinit,Yinit) == 5)%if there are obstacles in the init, re-run rand.
        i = i-1;                    %living dangerously
    end
end

token = zeros(2,1);
token(randi(2)) = 1;
valid = false;
for i=1:AGENT_COUNT
    while(valid == false)
        Xinit(i) = floor((Xmax-1)*rand);
        Yinit(i) = floor((Ymax-1)*rand);
        Position = [Xinit(i),Yinit(i)];
        if(GlobalCopy(Xinit,Yinit) ~= 5)
            A(i) = Agent(token(i),Position,GlobalCopy);
            valid = true;
        end;
    end;
    valid = false;
end;
AgentMap = zeros(Xmax,Ymax);                                %Map that holds agent's workspace
%To identify the processs start
Counter = 0;


if(isempty(A(1).path))
    A(1) = calculatePath(A(1),[]);        %its initial, opPosition's path will be empty
    A(1) = assignpath(A(1));
end;
if(isempty(A(2).path))
    A(2) = calculatePath(A(2),A(1).path);           %its initial,
    A(2) = assignpath(A(2));
end;

obstacledMap = GlobalCopy;
obstacledMap(obstacledMap==2) = 5;

while(~isequal(A(1).Map,obstacledMap) || ~isequal(A(2).Map,obstacledMap))                        %TODO: Implement random start.
    Counter = Counter + 1;
    
    % Both agents have done their initial planning, and have values in path
    % variable.
    if(mod(Counter,100)==0)                                   % Mandatory Planning phase
        A(1) = calculatePath(A(1),A(2).path);               % Both agents plan here and then
        A(2) = calculatePath(A(2),A(1).path);               % their Bids are compared. And the
        if(A(1).Bid > 0 || A(2).Bid > 0)
            if(A(2).Bid > A(1).Bid)
                A(2) = assignpath(A(2));
            else
                A(1) = assignpath(A(1));
            end;
        end;
    else
        
        % Call the move function on both the agents and handle the results
        % below. The only place in the while loop where move will be called
        
        A(1) = move(A(1));
        A(2) = move(A(2));
        
        
        % Check for cases where both the Agents are done with their mini
        % tasks at the same time.
        
        if((isequal(A(1).goal,A(1).Position)) && (isequal(A(2).goal,A(2).Position))) % Both agents have finished their mini task.
            A(1) = updateMap(A(1),A(1).Position);
            A(1) = updateMap(A(1),A(2).Position);
            A(2) = updateMap(A(2),A(1).Position);
            A(2) = updateMap(A(2),A(2).Position);
            fprintf('A(2) painted a tile :%d , %d\n',A(2).goal(1),A(2).goal(2));
            fprintf('A(1) painted a tile :%d , %d\n',A(1).goal(1),A(1).goal(2));
            A(1) = calculatePath(A(1),A(2).path);
            A(2) = calculatePath(A(2),A(1).path);
            if(A(1).Bid > 0 || A(2).Bid > 0)
                if(A(2).Bid > A(1).Bid)
                    A(2) = assignpath(A(2));
                else
                    A(1) = assignpath(A(1));
                end;
            end;
        elseif((~isequal(A(1).goal,A(1).Position)) && (isequal(A(2).goal,A(2).Position)))    % one agent is still on the move.
            A(1) = updateMap(A(1),A(2).Position);
            A(2) = updateMap(A(2),A(2).Position);
            fprintf('A(2) painted a tile :%d , %d\n',A(2).goal(1),A(2).goal(2));
            A(2) = calculatePath(A(2),A(1).path);
            A(2) = assignpath(A(2));
        elseif((isequal(A(1).goal,A(1).Position)) && (~isequal(A(2).goal,A(2).Position)))    % one agent is still on the move.
            A(1) = updateMap(A(1),A(1).Position);
            A(2) = updateMap(A(2),A(1).Position);
            fprintf('A(1) painted a tile :%d , %d\n',A(1).goal(1),A(1).goal(2));
            A(1) = calculatePath(A(1),A(2).path);
            A(1) = assignpath(A(1));
        end;
    end;
end;

fprintf('Time take for the Multi Agent planning :%d',Counter);