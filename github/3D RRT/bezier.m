function rrt_connect_3d
    % Set the start and goal positions
    start_pos = [0, 0, 0];
    goal_pos = [10, 10, 10];
    
    % Define the space boundaries [xmin, xmax; ymin, ymax; zmin, zmax]
    space_boundaries = [-5, 15; -5, 15; -5, 15];
    
    % Define the maximum distance for extending the trees
    max_step = 1.0;
    
    % Initialize the start and goal trees
    start_tree = start_pos;
    goal_tree = goal_pos;
    
    % Maximum number of iterations
    max_iter = 1000;
    
    for i = 1:max_iter
        % Generate a random point in the space
        random_point = [rand_range(space_boundaries(1, :)), ...
                        rand_range(space_boundaries(2, :)), ...
                        rand_range(space_boundaries(3, :))];
                    
        % Extend the start tree towards the random point
        [start_tree, new_point_start] = extend_tree(start_tree, random_point, max_step);
        
        % Extend the goal tree towards the new point from the start tree
        [goal_tree, new_point_goal] = extend_tree(goal_tree, new_point_start, max_step);
        
        % Check if the new points from both trees can be connected
        if norm(new_point_start - new_point_goal) < max_step
            % Path found, construct the path
            path = construct_path(start_tree, goal_tree, new_point_start, new_point_goal);
            disp('Path found!');
            plot_path(start_tree, goal_tree, path);
            return;
        end
    end
    
    disp('No path found.');
end

function val = rand_range(range)
    val = range(1) + (range(2) - range(1)) * rand();
end

function [tree, new_point] = extend_tree(tree, random_point, max_step)
    % Find the nearest node in the tree to the random point
    nearest_node = tree(1, :);
    min_dist = norm(random_point - nearest_node);
    
    for i = 2:size(tree, 1)
        dist = norm(random_point - tree(i, :));
        if dist < min_dist
            min_dist = dist;
            nearest_node = tree(i, :);
        end
    end
    
    % Compute the new point
    direction = (random_point - nearest_node) / norm(random_point - nearest_node);
    new_point = nearest_node + max_step * direction;
    
    % Add the new point to the tree
    tree = [tree; new_point];
end

function path = construct_path(start_tree, goal_tree, new_point_start, new_point_goal)
    % Combine the two trees into a single path
    path = [flipud(start_tree); new_point_start; new_point_goal; goal_tree];
end

function plot_path(start_tree, goal_tree, path)
    figure;
    hold on;
    plot3(start_tree(:, 1), start_tree(:, 2), start_tree(:, 3), 'bo-');
    plot3(goal_tree(:, 1), goal_tree(:, 2), goal_tree(:, 3), 'ro-');
    plot3(path(:, 1), path(:, 2), path(:, 3), 'go-');
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    grid on;
    title('RRT-Connect 3D Path');
    hold off;
end
