% Clear previous variables and command window
clear;
clc;

%% Robot Definition using D-H Parameters

% Define the D-H parameters for each joint (modified D-H parameters)
th = [0, 0, 0, 0, 0, 0];        % Joint angles (theta)
d = [0, 0, 0, 2850, 0, 0];      % Offsets along previous z to the common normal
a = [0, 500, 1700, 180, 0, 0];  % Lengths of the common normal (a.k.a a, distance from Zi-1 to Zi along Xi)
alp = [0, -pi/2, 0, -pi/2, pi/2, -pi/2];  % Angles about common normal from Zi-1 to Zi

% Define the robot links using the specified D-H parameters
L(1) = Link([th(1), d(1), a(1), alp(1)], 'modified');
L(2) = Link([th(2), d(2), a(2), alp(2)], 'modified');
L(3) = Link([th(3), d(3), a(3), alp(3)], 'modified');
L(4) = Link([th(4), d(4), a(4), alp(4)], 'modified');
L(5) = Link([th(5), d(5), a(5), alp(5)], 'modified');
L(6) = Link([th(6), d(6), a(6), alp(6)], 'modified');

% Create the serial link robot model
robot = SerialLink(L, 'name', '6-DOF Robot');

%% Compute Inverse Kinematics for an Arbitrary End-Effector Pose

% Define the desired end-effector pose (position and orientation)
% Position (x, y, z) in millimeters (ensure it's a 1x3 row vector)
desired_position = [1000, 500, 1500];

% Orientation in terms of Roll-Pitch-Yaw angles (in radians)
% For example, let's set some arbitrary orientation
desired_rpy = [deg2rad(30), deg2rad(45), deg2rad(60)];  % [roll; pitch; yaw]

% Convert Roll-Pitch-Yaw to Homogeneous Transformation Matrix
% Use eul2tr which returns a 4x4 transformation matrix
desired_orientation = eul2tr(desired_rpy(1), desired_rpy(2), desired_rpy(3));

% Create the homogeneous transformation matrix for the desired pose
% Multiply the translation and rotation matrices (both are 4x4)
T_desired = transl(desired_position) * desired_orientation;

% Display the desired transformation matrix
disp('Desired End-Effector Pose:');
disp(T_desired);

%% Inverse Kinematics Calculation

% Initial guess for joint angles (can be zeros or previous pose)
q_initial = zeros(1, 6);

% Use the inverse kinematics function ikcon for better convergence
% Note: ikcon uses numerical methods to find a solution close to q_initial
[q_solution, error] = robot.ikcon(T_desired, q_initial);

% Display the calculated joint angles (in radians)
disp('Calculated Joint Angles (radians):');
disp(q_solution);

% Convert joint angles to degrees for better understanding
q_solution_deg = rad2deg(q_solution);
disp('Calculated Joint Angles (degrees):');
disp(q_solution_deg);

%% Verify the Solution by Forward Kinematics

% Compute the forward kinematics with the calculated joint angles
T_computed = robot.fkine(q_solution);

% Display the computed end-effector pose
disp('Computed End-Effector Pose from Forward Kinematics:');
disp(T_computed);

% Compare the desired and computed end-effector positions
desired_pos_extracted = transl(T_desired);
computed_pos_extracted = transl(T_computed);

position_error = norm(desired_pos_extracted - computed_pos_extracted);
disp(['Position Error (mm): ', num2str(position_error)]);

%% Visualize the Robot Configuration

% Plot the robot in the calculated joint configuration
figure;
robot.plot(q_solution);
title('Robot Configuration for Desired End-Effector Pose');

% Optionally, plot the desired end-effector position
hold on;
plot3(desired_position(1), desired_position(2), desired_position(3), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
legend('Robot Configuration', 'Desired End-Effector Position');
hold off;
