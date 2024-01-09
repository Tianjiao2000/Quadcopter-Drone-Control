% Differential drive robot
clc;
clear all;
close all;

% Simulation parameters
TOTAL_TIME  = 3;
dt          = 0.01;
TIME_SCALE  = 0.1; % slows down simulation if > 1, speeds up if < 1 (and if computation allows...)


% Initialise plot
figure;
ax1 = axes;
hold(ax1,'on');
view(ax1, 3);
axis('equal')
axis([-5 5 -5 5 0 10])
axis('manual')
xlabel('x');
ylabel('y');
ylabel('z');
axis vis3d
grid ON
grid MINOR
ax1.Toolbar.Visible = 'off';
ax1.Interactions = [];

% Initialise Simulation
% drone1 = QuadcopterFreeFall(ax1);
drone1 = QuadcopterRota(ax1);
% drone1 = QuadcopterEquil(ax1);
positions = [];

% Run Simulation
for t = 0:dt:TOTAL_TIME
    tic
    cla
    
    % _______ IMPLEMENT CONTROLLER + SIMULATION PHYSICS HERE ______ %
    drone1.update(t);
    drone1.plot;

    positions = [positions; drone1.getPosition()']; 

    % Plot the trajectory
    plot3(ax1, positions(:,1), positions(:,2), positions(:,3), 'r');
    % _______ IMPLEMENT CONTROLLER + SIMULATION PHYSICS HERE ______ %
    
    
    drawnow nocallbacks limitrate
    pause(TIME_SCALE*dt-toc); 
end

figure;
subplot(3, 1, 1);
pos1 = drone1.posPlot(1,:);
time = 0: 0.01 : TOTAL_TIME ; 
plot(time, pos1);
grid on; % Adds a grid for better visualization
ylabel('X-axis');
xlabel('Time');
title('Position x Over Time');

subplot(3, 1, 2);
pos2 = drone1.posPlot(2,:);
time = 0: 0.01 : TOTAL_TIME ; 
plot(time, pos2);
grid on; % Adds a grid for better visualization
ylabel('Y-axis');
xlabel('Time');
title('Position y Over Time');

subplot(3, 1, 3);
pos3 = drone1.posPlot(3,:);
time = 0: 0.01 : TOTAL_TIME ; 
plot(time, pos3);
grid on; % Adds a grid for better visualization
ylabel('Z-axis');
xlabel('Time');
title('Position z Over Time');

figure;
subplot(3, 1, 1);
roll = drone1.thetaPlot(1, :);
time = 0: 0.01 : TOTAL_TIME ; 
plot(time, roll);
grid on; % Adds a grid for better visualization
ylabel('Roll');
xlabel('Time');
title('Roll Over Time');

subplot(3, 1, 2);
pitch = drone1.thetaPlot(2, :);
time = 0: 0.01 : TOTAL_TIME ; 
plot(time, pitch);
grid on; % Adds a grid for better visualization
ylabel('Pitch');
xlabel('Time');
title('Pitch Over Time');

subplot(3, 1, 3);
yaw = drone1.thetaPlot(3, :);
time = 0: 0.01 : TOTAL_TIME ; 
plot(time, yaw);
grid on; % Adds a grid for better visualization
ylabel('Yaw');
xlabel('Time');
title('Yaw Over Time');