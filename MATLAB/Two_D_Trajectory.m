%% Parameters - Adjust these!
x_end = 20;          % End of trajectory (X-axis)
amplitude = 3;       % Vertical travel / Peak height
period = 10;          % Distance for one full wave cycle
resolution = 500;    % Sampling points
z_offset = 5;        % Starting Z height (Your [0,0,5] point)

%% Generation Logic
x = linspace(0, x_end, resolution)'; 

% --- 1. Straight Line (Diagonal Ramp) ---
% Starts at z_offset and ends at (z_offset + amplitude)
z_line = linspace(z_offset, z_offset + amplitude, resolution)';

% --- 2. Sine Wave ---
% Starts at 0, goes up to +amplitude, then down.
z_sine = amplitude * sin((2 * pi / period) * x) + z_offset;

% --- 3. Triangle Wave ---
% We use a specific triangle function: 2 * abs( (x/T) - floor(x/T + 0.5) )
% This starts at 0 and peaks at 1. We multiply by amplitude and add offset.
z_triangle = amplitude * (2 * abs((x/period) - floor((x/period) + 0.5))) + z_offset;

%% Export to CSV
results_table = table(x, z_line, z_sine, z_triangle, ...
    'VariableNames', {'X_Position', 'Z_Line', 'Z_Sine', 'Z_Triangle'});

writetable(results_table, 'dual_axis_trajectories.csv');
fprintf('Success! All trajectories start at [0, %0.1f] and exported to CSV.\n', z_offset);

%% Visualization
figure('Color', 'w', 'Name', 'X-Z Trajectories');
hold on;

% Plotting the three paths
plot(x, z_line, 'r--', 'LineWidth', 2, 'DisplayName', 'Straight Line (Diagonal)');
plot(x, z_sine, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Sine Wave');
plot(x, z_triangle, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Triangle Wave');

% Mark the common start point
plot(0, z_offset, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8, 'DisplayName', 'Start [0, 5]');

grid on;
legend('Location', 'best');
xlabel('X Position');
ylabel('Z Position');
title('Trajectories starting at [0, 5] with X and Z movement');
axis equal; % Ensures 1 unit on X looks the same as 1 unit on Z