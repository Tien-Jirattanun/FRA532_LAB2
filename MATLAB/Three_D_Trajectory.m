%% Parameters - Adjust these!
z_start = 5;         % Starting Z height (Start point: [0, 0, 5])
z_end = 20;          % The trajectory ends when Z reaches this value
amplitude = 7.5;       % Radius of the spiral/swing in X-Y plane
period = 10;         % Vertical distance (Z) for one full rotation
resolution = 1500;   % Sampling points

%% Generation Logic
% The independent variable is now Z (Vertical progression)
z = linspace(z_start, z_end, resolution)';
% Relative Z for calculations (starting from 0)
z_rel = z - z_start;

% --- 1. Vertical Straight Line (Diagonal in 3D) ---
% Moves from [0,0,5] to [amplitude, amplitude, z_end]
x_line = linspace(0, amplitude, resolution)';
y_line = linspace(0, amplitude, resolution)';

% --- 2. Vertical Helix Spiral ---
% Spirals upward around the Z-axis. Starts at [0,0,5].
x_helix = amplitude * sin((2 * pi / period) * z_rel);
y_helix = amplitude * (1 - cos((2 * pi / period) * z_rel));

% --- 3. Vertical Möbius-style Line (Figure-8) ---
% Swings in X and Y as it climbs Z.
x_mobius = amplitude * sin((2 * pi / period) * z_rel);
y_mobius = amplitude * sin((4 * pi / period) * z_rel);

%% Export to CSV
% Updated table format for vertical orientation
results_table = table(x_line, y_line, z, x_helix, y_helix, x_mobius, y_mobius, ...
    'VariableNames', {'X_Line', 'Y_Line', 'Z', 'X_Helix', 'Y_Helix', 'X_Mobius', 'Y_Mobius'});

writetable(results_table, 'tri_axis_trajectories_3D.csv');
fprintf('Success! Vertical trajectories exported starting at Z = %d\n', z_start);

%% Visualization
figure('Color', 'w', 'Name', 'Vertical Trajectory Comparison');
hold on; grid on;

% Plotting (Note: Z is the third argument in plot3)
plot3(x_line, y_line, z, 'r-', 'LineWidth', 2, 'DisplayName', 'Vertical Straight Line');
plot3(x_helix, y_helix, z, 'b-', 'LineWidth', 2, 'DisplayName', 'Vertical Helix');
plot3(x_mobius, y_mobius, z, 'g-', 'LineWidth', 2, 'DisplayName', 'Vertical Möbius');

% Mark the common start point [0, 0, 5]
plot3(0, 0, z_start, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8, 'DisplayName', 'Start [0,0,5]');

view(35, 20); 
xlabel('X Position'); ylabel('Y Position'); zlabel('Z Position (Height)');
title('Vertical Trajectories [Progressing along Z]');
legend('Location', 'northeastoutside');
axis equal;