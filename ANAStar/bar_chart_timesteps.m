%% Script to create bar chart of intercept time results

clear all
close all

% Define categories
categories = {'Greedy Pursuit', 'Greedy Pursuit w/ Noise', 'Intercept Planning', 'Intercept Planning w/ Noise'};
scenarios = {'Scenario 1', 'Scenario 2', 'Scenario 3', 'Scenario 4',...
    'Scenario 5', 'Scenario 6'};

% Timestep data, row = scenario, column corresponds to category
total_timesteps = [
    45, 45, 45, 46;
    122, 123, 122, 122;
    79, 101, 78, 80;
    81, 86, 76, 92;
    150, 105, 103, 123;
    139, 160, 140, 199;
    ];

% Initialize figure
figure;
b = bar(total_timesteps);
grid on

% Set and label bars and legend
set(gca, 'XTickLabel', scenarios, 'FontSize', 14);
legend(categories, 'Location', 'northwest', 'FontSize', 14);
xlabel('Scenario', 'FontSize', 16)
ylabel('Time Steps to Intercept Target', 'FontSize', 16)
title('Comparison of Time to Intercept Target across Scenarios', 'FontSize', 18)

% Add values above each bar
% Loop over each group
for i = 1:length(b)
    % Get bar X and Y data
    x = b(i).XEndPoints;
    y = b(i).YEndPoints;
    labels = string(b(i).YData);
    text(x, y + 0.2, labels, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 14);
end