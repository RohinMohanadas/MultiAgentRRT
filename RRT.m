classdef RRT
    
%*****************************************************************************************************%
%*                                                                                                   *%
%*   NAME: RRT                                                                                       *%
%*   DESCRIPTION:                                                                                    *%
%*   AUTHOR:                                                                                         *%
%*   DATE CREATION:09/10/2016                                                                        *%
%*   LAST MODIFIED:10/12/2016                                                                        *%
%*                                                                                                   *%
%*****************************************************************************************************%
    
    properties
    initialMap %number of timesteps from the beginning of the path until the goal
    initialPosition % value of the last move (1 or 0)
    goal %position of the current goal
    n %number of rows/columns
    N %number of tiles in the map (N=nxn)
    path %sequence of positions from initialPosition to goal
    finalMap %Map with 1's on the path positions
    end
    
    methods
        %%Class Constructor
        function rrt = RRT(Map,Position,Goal)
            if nargin == 3
                        rrt.initialMap = Map;
                        rrt.initialPosition = Position;
                        rrt.goal = Goal;
                        s = size(rrt.initialMap);
                        rrt.N = s(1)*s(2);
                        rrt.n = s(1);
            else
                    error('Incorrect Number of Inputs for class Agent')
            end
        end
        
        function rrt = runRRT(rrt) %Value should be 1 or 0
            tree = zeros(rrt.N,2);
            parent = zeros(rrt.N,1);
            node_x = zeros(1,rrt.N);
            node_y = zeros(1,rrt.N);
            node_x(1) = rrt.initialPosition(1);
            node_y(1) = rrt.initialPosition(2);
            tree(1,:) = [node_x(1),node_y(1)];
            rrt.initialMap(node_x(1),node_y(1)) = 1;
            loop_count = 0;
            no_path = false;
            
            for i = 2:rrt.N
                is_valid = false;
                while (is_valid == false)
                    if(nnz(~rrt.initialMap) == 0) %all map elements have been filled
                        break;
                    end
                    %find a sample node
                    random_node = randomNode(rrt,node_x,node_y);
                    %Finding the nearest node
                    nearest_node = nearestNode(node_x,node_y,random_node);
                    %get the new node
                    new_node = newNode(rrt,random_node,nearest_node);
                    %check if new node is in collision => rerun that loop
                    is_valid = detectObstacle(rrt.initialMap,new_node,is_valid);
                    %update loop count of this while loop
                    loop_count = loop_count + 1;
                    if(loop_count > rrt.N^2)
                        break;
                    end
                end
                
                if(loop_count > rrt.N^2)
                    disp('Leaving RRT: No path found');
                    %Create some sort of indicator that it will make no
                    %move
                    break;
                end
                
                %add the new nodes to node history
                node_x(i) = new_node(1);
                node_y(i) = new_node(2);
                %fprintf('new node: (%d,%d)\n', new_node(1),new_node(2));

                %update tree
                tree(i,:) = new_node;
                %find the index of the location of the nearest node in tree (find
                %parent)
                for index = 1:i
                    if nearest_node == [node_x(index)',node_y(index)']
                        parent(i) = index;
                    end
                end

                %show entire map with all trees (not necessary)
                rrt.initialMap(new_node(1),new_node(2)) = 1;
                %disp(map);

                %check to see if you have reached goal -- get rrt_path
                if (rrt.initialMap(rrt.goal(1),rrt.goal(2)) == 1)
                    %fprintf('Found goal in %d iterations\n', i);
                    %trace back your path:
                    rrt_path = getPath(tree,parent,i);
                    break; %breaks loop if you have reached the end
                end
                
                %if(loop_count >= N^2)
                if(nnz(~rrt.initialMap) == 0)
                    no_path = true;
                    disp('Leaving RRT: No path found');
                    %Create something that gives indication to do nothing.
                    break;
                end
            end
            
            %create an updated rrt map that shows path
            if(no_path == false && loop_count < rrt.N^2)
                rrt.path=rrt_path;
                %create an updated rrt map that shows path
                rrt.finalMap = getPathMap(rrt,rrt_path);
            end 
        end 
        
        function random_node = randomNode(rrt,node_x,node_y)
            valid = false;
            random_node = randi([1,rrt.n],1,2);
            k = 1;
            while (valid == false)
                if ((random_node(1) == node_x(k) && random_node(2) == node_y(k)) || (rrt.initialMap(random_node(1),random_node(2)) == 5))
                    random_node = randi([1,rrt.n],1,2);
                    k = 1;
                    continue;
                end
                if (k == size(node_x,2))
                    valid = true;
                end
                k = k+1;
            end
            %fprintf('random node: (%d,%d)\n', random_node(1),random_node(2));
        end
        
        function nearest_node = nearestNode(node_x,node_y,random_node)
            if (length(find(node_x)) == 1) %finds the number of non-zero elements
                nearest_node = [node_x(1),node_y(1)];
            else
                nearest_node = [node_x(1),node_y(1)];
                distance = sqrt((random_node(1)-node_x(1))^2 + (random_node(2)-node_y(1))^2);
                for j = 2:length(find(node_x))
                    tmp = sqrt((random_node(1)-node_x(j))^2 + (random_node(2)-node_y(j))^2); %Euclidean distance 
                    if (tmp < distance)
                        nearest_node = [node_x(j),node_y(j)]; %finds the nearest node by searching min
                        distance = tmp;
                    end
                end
            end
            %fprintf('nearest node: (%d,%d)\n', nearest_node(1),nearest_node(2));
        end
        
        function new_node = newNode(rrt,random_node,nearest_node)
            %Agent movements
            up = [-1,0];
            down = [1,0];
            left = [0,-1];
            right = [0,1];
            
            %find the slope from nearest_node to random_node: y_diff/x_diff
            y_diff = random_node(1)-nearest_node(1);
            x_diff = random_node(2)-nearest_node(2);
            if (abs(x_diff) > 0 && (abs(x_diff) < abs(y_diff) || y_diff == 0))%the lesser slope difference is the one that's added
                if (x_diff > 0)
                    new_node = nearest_node+right;
                else
                    new_node = nearest_node+left;
                end
            elseif (abs(y_diff) > 0)
                if (y_diff > 0)
                    new_node = nearest_node+down;
                else
                    new_node = nearest_node+up;
                end
            elseif (abs(x_diff) == abs(y_diff)) %if both are equally likely jsut choose randomly
                if (nearest_node(1) == rrt.n) %if you're at the row border, increase column
                     new_node = [nearest_node(1), nearest_node(2)+1];
                elseif (nearest_node(2) == rrt.n)
                     new_node = [nearest_node(1)+1, nearest_node(2)];
                else
                    choice = randi(2); %choose between options randomly
                    if (choice == 1)
                        if (x_diff > 0)
                            new_node = nearest_node + right;
                        else
                            new_node = nearest_node + left;
                        end
                    else
                        if (y_diff > 0)
                            new_node = nearest_node + down;
                        else
                            new_node = nearest_node + up;
                        end
                    end
                end
            end
        end
        
        function is_valid = detectObstacle(rrt, new_node, is_valid)
            if(rrt.initialMap(new_node(1),new_node(2)) ~= 5)
                is_valid = true;
            end
        end
        
        function rrt_path = getPath(tree,parent,current_index)
            rrt_path = zeros(current_index,2);
            for index = 1:current_index
                if (index == 1)
                    rrt_path(index,:) = tree(current_index,:);
                    ncol = size(rrt_path, 2); %getting rid of the unnecessary zeros
                    rrt_path(rrt_path == 0) = [];
                    rrt_path = reshape(rrt_path, [], ncol);
                    child_of = parent(current_index);
                else
                    if(child_of == 0)
                        break;
                    else
                        rrt_path(index,:) = tree(child_of,:);
                        child_of = parent(child_of);
                    end
                end
            end
                rrt_path = flipud(rrt_path); %flipping the matrix
        end
        
        function rrt_map = getPathMap(rrt,rrt_path)
            rrt_map = zeros(rrt.n);
            for index = 1:size(rrt_path,1)
                rrt_map(rrt_path(index,1),rrt_path(index,2)) = 1;
            end
        end
        
    end
end