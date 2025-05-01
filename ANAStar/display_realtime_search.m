%% Script that displays real time search and movement of an agent and a target

close all
clear all

% Define test config to plot
test_number = 5;
intercept = true;
noise = true;

if intercept
    if noise
        subfolder_name = "intercept_noise";
    else
        subfolder_name = "intercept";
    end
else
    if noise
        subfolder_name = "no_intercept_noise";
    else
        subfolder_name = "no_intercept";
    end
end

% Extract graph and enviornment variables
nodes_raw = csvread('files/nodes.txt');
edges_raw = csvread('files/edges_with_costs.txt');
obstacles_raw = csvread('files/obstacles.txt');

% Extract target path
target_path_raw = csvread(sprintf('files/test%d/target_path_%d.txt', test_number, test_number));

% Extract config information
config_raw = readtable(sprintf('files/test%d/config_%d.txt', test_number, test_number),'Delimiter',':');

% Define output path folder and gif file name
output_path_folder = sprintf('files/test%d/%s/output_paths', test_number, subfolder_name);
n_timesteps = numel(dir(output_path_folder))-2;
gif_filename = sprintf('files/test%d/%s/search_animation_%d_%s.gif', test_number, subfolder_name, test_number, subfolder_name);

% Process node file
nodes = nodes_raw(2:end,2:3);

% Process edge file
edges_raw = edges_raw(2:end,:);
edges = nan(3*size(edges_raw, 1), 2);
edges(1:3:end-2, 1) = nodes(edges_raw(:, 1),1);
edges(2:3:end-1, 1) = nodes(edges_raw(:, 2),1);
edges(1:3:end-2, 2) = nodes(edges_raw(:, 1),2);
edges(2:3:end-1, 2) = nodes(edges_raw(:, 2),2);

% Process the obstacles files
obstacles = obstacles_raw(2:end,:);

% Process the target path file
target_path = target_path_raw(2:end,:);

% Process table file
agent_vel = config_raw{1,2};
target_vel = config_raw{2,2};
planning_timeMS = config_raw{3,2};
agent_startnodeID = config_raw{4,2};
agent_range = config_raw{5,2};

% Circle plotting variables
theta = linspace(0, 2*pi, 100);  % Circle angles

% Initialize image
fig = figure;
for i = 1:size(obstacles,1)
    rectangle("Position",obstacles(i,:),'EdgeColor','r')
    hold on
end
xlim([-50 50])
ylim([-50 50])
title(sprintf('Target Search, Test %d', test_number));

% Add intercept red marker and timestamp counter  ------------
% red x mark that we'll move each frame 
hMarker = plot(nan, nan, 'x', 'Color','r', 'LineWidth',2, 'MarkerSize',8);
% text label for 't = ...' that we'll update each frame
hTime   = text(nan, nan, '', 'FontSize',12, 'FontWeight','bold', 'Color','k');
% ----------------------------------------

% Set current target path index information
current_target_path_idx = 1;
previous_target_path_idx = current_target_path_idx;

% Initialize previous agent path vector
previous_agent_path = [];
agent_moves = 0;

for i = 0:n_timesteps-1
    fprintf("Timestep %d\n",i);

    % Extract the search path for the time step
    try
        path_raw = csvread(sprintf("%s/output_path_t%d.txt", output_path_folder, i));
    catch exception
        disp("No Path found, extracting data with tables.")
        path_raw = readtable(sprintf("%s/output_path_t%d.txt", output_path_folder, i),'Delimiter',',');
        path_raw = table2array(path_raw(1:end-1,1:4));
    end

    % If it's not the first time step, plot the agent's path to the current point 
    if i > 0
        cs = cumsum(flip(path_raw(:,4)));
        am_idx = find(cs > agent_vel, 1, 'first') - 1;
        if isempty(am_idx)
            agent_moves = length(cs) - 1;
        else
            agent_moves = am_idx;
        end
        if agent_moves > 0
            plot(previous_agent_path(end-agent_moves:end,2),previous_agent_path(end-agent_moves:end,3),'g','LineWidth',1.5);
        end
    end  

    % Plot the path the target has covered since the last time step
    plot(target_path(previous_target_path_idx:current_target_path_idx,2), target_path(previous_target_path_idx:current_target_path_idx,3), 'k', 'LineWidth', 1.2)
    
    % If statement exists for setting plot handlers
    if i == 0
        % Plot the ANA* generated trajectory
        h1 = plot(path_raw(:,2), path_raw(:,3), '--b', 'LineWidth', 1.2);

        % Plot the agent's current position as a blue square
        h2 = scatter(path_raw(end,2), path_raw(end,3), 50, 'square', 'b', 'filled');
        
        % Plot the target's current positions as a black square
        h3 = scatter(target_path(current_target_path_idx,2),target_path(current_target_path_idx,3),50, 'square', 'k', 'filled');

        % Plot the search radius around the agent
        circle_x = agent_range * cos(theta) + path_raw(end,2);
        circle_y = agent_range * sin(theta) + path_raw(end,3);
        h4 = plot(circle_x, circle_y, 'b', 'LineWidth', 1);
    else
        % Reset ANA* generated trajectory
        set(h1, 'XData', path_raw(:,2));
        set(h1, 'YData', path_raw(:,3));

        % Reset agent's current positions
        set(h2, 'XData', path_raw(end,2));
        set(h2, 'YData', path_raw(end,3));

        % Reset target's current position
        set(h3, 'XData', target_path(current_target_path_idx,2));
        set(h3, 'YData', target_path(current_target_path_idx,3));

        % Reset agent's search radius
        set(h4, 'XData', agent_range * cos(theta) + path_raw(end,2));
        set(h4, 'YData', agent_range * sin(theta) + path_raw(end,3));
    end

    % -------------------------------------------
    % for intercept only : move the red x to end of generated ANA* path
    if(intercept | noise)
        lastX = path_raw(1,2);
        lastY = path_raw(1,3);
        set(hMarker, 'XData', lastX, 'YData', lastY);
    end
    % position and update the time stamp text
    xl = xlim; yl = ylim;
    tx = xl(1) + 0.02*diff(xl);
    ty = yl(2) - 0.05*diff(yl);
    set(hTime, 'Position', [tx, ty], 'String', sprintf('t = %d', i));
    % --------------------------------------------

    % Export the plot into a GIF format
    % Capture the plot as an image
    frame = getframe(gcf);
    im = frame2im(frame);
    [imind, cm] = rgb2ind(im, 256);

    % Write to the GIF File
    if i == 0
        imwrite(imind, cm, gif_filename, 'gif', 'Loopcount', inf, 'DelayTime', 0.1);
    else
        if i == n_timesteps - 1
            delta_t = 1;
        else
            delta_t = 0.1;
        end
        imwrite(imind, cm, gif_filename, 'gif', 'WriteMode', 'append', 'DelayTime', delta_t);
    end

    % Store previous agent path
    if agent_moves > 0 || i == 0
        previous_agent_path = path_raw;
    end

    % Utilizing cost info, propogate forward to agent's next index
    previous_target_path_idx = current_target_path_idx;
    target_cost = target_vel;
    while true
        target_cost = target_cost - target_path(current_target_path_idx + 1, 4);
        if(target_cost < 0)
            break;
        else
            current_target_path_idx = current_target_path_idx + 1;
        end
    end
end

% Calculate how far the agent moves on the last step, plot its movement
agent_moves = find(cumsum(flip(path_raw(:,4))) > agent_vel, 1, 'first') - 1;
plot(previous_agent_path(end-agent_moves:end,2),previous_agent_path(end-agent_moves:end,3),'g','LineWidth',1.5);

% Plot the target's movement on the last step
plot(target_path(previous_target_path_idx:current_target_path_idx,2), target_path(previous_target_path_idx:current_target_path_idx,3), 'k', 'LineWidth', 1.2)

% Reset agent's current position
set(h2, 'XData', previous_agent_path(end-agent_moves,2));
set(h2, 'YData', previous_agent_path(end-agent_moves,3));

% Reset target's current position
set(h3, 'XData', target_path(current_target_path_idx,2));
set(h3, 'YData', target_path(current_target_path_idx,3));

% Reset agent's search radius
set(h4, 'XData', agent_range * cos(theta) + previous_agent_path(end-agent_moves,2));
set(h4, 'YData', agent_range * sin(theta) + previous_agent_path(end-agent_moves,3));


%% Helper functions

% Function that plots a circle (i.e., an obstacle)
function h = circle(x,y,r)
    hold on
    theta = 0:pi/50:2*pi;
    xunit = x + r*cos(theta);
    yunit = y + r*sin(theta);
    h = plot(xunit, yunit, 'k');
    hold off
end