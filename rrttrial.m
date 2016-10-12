clear all;
close all;

%Test with dummy map.
MainImage = importdata('256TestFile.bmp');
GlobalCopy = MainImage(1).cdata;
[Xmax,Ymax] = size(GlobalCopy);
GlobalCopy(GlobalCopy==79) = 5;     %   79	-> 5    (Red to obstacles)
GlobalCopy(GlobalCopy==0) = 2;      %   0	-> 2    (Black cell to picture to be painted)
GlobalCopy(GlobalCopy==255) = 0;    %   255	-> 0    (White to Empty Cell)
%

%Debug Area
Ctr = 1;
%

%Agent movements
up = [-1,0];
down = [1,0];
left = [0,-1];
right = [0,1];

%start with empty search tree
n = 100; %this is to be changed.

%create a dynamic node array
%map = zeros(n);
map = GlobalCopy;
%add your initial and final location to the tree
init = [1,1];
final = [Xmax,Ymax];

%get number of nodes & max number of iterations possible
N = Xmax*Ymax;
tree = zeros(N,2);
parent = zeros(N,1);
node_x = zeros(1,N);
node_y = zeros(1,N);
node_x(1) = 1;
node_y(1) = 1;
tree(1,:) = [node_x(1),node_y(1)];
map(node_x(1),node_y(1)) = 1;
i = 1;
while( i <= N)
    i = i+1;
    %find a sample node
    valid = false;
    random_node = randi([1,n],1,2);
    k = 1;
    while (valid == false)
        if ((random_node(1) == node_x(k) && random_node(2) == node_y(k))|| map(random_node(1),random_node(2)) == 5) % added obstacle detection
            random_node = randi([1,n],1,2);
            k = 1;
            continue;
        end
        if (k == size(node_x,2))
            valid = true;
        end
        k = k+1;
    end
    %fprintf('random node: (%d,%d)\n', random_node(1),random_node(2));

    %Finding the nearest node
    if (size(node_x,2) == 1)
        nearest_node = [node_x(1),node_y(1)];
    else
        nearest_node = [node_x(1),node_y(1)];
        distance = sqrt((random_node(1)-node_x(1))^2 + (random_node(2)-node_y(1))^2);
        for j = 2:size(node_x,2)
            tmp = sqrt((random_node(1)-node_x(j))^2 + (random_node(2)-node_y(j))^2); %Euclidean distance 
            if (tmp < distance)
                nearest_node = [node_x(j),node_y(j)]; %finds the nearest node by searching min
                distance = tmp;
            end
        end
    end
    %fprintf('nearest node: (%d,%d)\n', nearest_node(1),nearest_node(2));
    
    %get the new node
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
        if (nearest_node(1) == n) %if you're at the row border, increase column
             new_node = [nearest_node(1), nearest_node(2)+1];
        elseif (nearest_node(2) == n)
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
    Ctr = Ctr+1;
    %Check for obstacles.
    if(map(new_node(1),new_node(2)) == 5)
        fprintf('New node is an obstacle: %d\n', Ctr);
        i = i-1;
        new_node = [0,0];
        continue;
    end;
    
    %add the new nodes to node history
    node_x(i) = new_node(1);
    node_y(i) = new_node(2);
    %fprintf('new node: (%d,%d)\n', new_node(1),new_node(2));
    tree(i,:) = new_node;
    %find the index of the location of the nearest node in tree
    for index = 1:i
        if nearest_node == [node_x(index)',node_y(index)']
            parent(i) = index;
        end
    end
    %add a 1 to show the movement on map
    map(new_node(1),new_node(2)) = 1;
    %disp(map);
    
    %check to see if you have reached goal
    if (map(final(1),final(2)) == 1)
        fprintf('Reached goal in %d iterations\n', i);
        %trace back your path:
        rrt_path = zeros(i,2);
        for index = 1:i
            if (index == 1)
                rrt_path(index,:) = tree(i,:);
                ncol = size(rrt_path, 2); %getting rid of the unnecessary zeros
                rrt_path(rrt_path == 0) = [];
                rrt_path = reshape(rrt_path, [], ncol);
                child_of = parent(i);
            else
                if(child_of == 0)
                    rrt_path = flipud(rrt_path); %flipping the matrix
                    break;
                else
                    rrt_path(index,:) = tree(child_of,:);
                    child_of = parent(child_of);
                end
            end
        end
        break; %breaks loop if you have reached the end
    end
end

%create an updated rrt map that shows path
rrt_map = zeros(n);
for index = 1:size(rrt_path,1)
    rrt_map(rrt_path(index,1),rrt_path(index,2)) = 1;
end

for index = 1:n
    plotmatrix(rrt_path(index,1),rrt_path(index,2));
end;
