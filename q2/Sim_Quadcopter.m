% Differential drive robot
clc;
clear all;
close all;

% Simulation parameters
TOTAL_TIME  = 3;
dt          = 0.01;
TIME_SCALE  = 0.01; % slows down simulation if > 1, speeds up if < 1 (and if computation allows...)


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
drone1 = Quadcopter(ax1);
drone2 = Quadcopter2(ax1);
positions = [];
positions2 = [];

% Run Simulation
for t = 0:dt:TOTAL_TIME
    tic
    cla
    
    % _______ IMPLEMENT CONTROLLER + SIMULATION PHYSICS HERE ______ %
    drone1.update(t);
    drone2.update(t);
    drone1.plot;
    positions = [positions; drone1.getPosition()']; 
    plot3(ax1, positions(:,1), positions(:,2), positions(:,3), 'r'); 
    drone2.plot;
    positions2 = [positions2; drone2.getPosition()']; 
    plot3(ax1, positions2(:,1), positions2(:,2), positions2(:,3), 'c'); 
    % _______ IMPLEMENT CONTROLLER + SIMULATION PHYSICS HERE ______ %
    
    
    drawnow nocallbacks limitrate
    pause(TIME_SCALE*dt-toc); 
end

figure;
subplot(3, 1, 1);
pos1 = drone1.posPlot(1,:);
pos11 = drone2.posPlot(1,:);
time = 0: dt : TOTAL_TIME ; 
plot(time, pos1);
hold on;
plot(time, pos11);
grid on; % Adds a grid for better visualization
ylabel('X-axis');
xlabel('Time');
title('Position x Over Time');

subplot(3, 1, 2);
pos2 = drone1.posPlot(2,:);
pos22 = drone2.posPlot(2,:);
time = 0: dt : TOTAL_TIME ; 
plot(time, pos2);
hold on;
plot(time, pos22);
grid on; % Adds a grid for better visualization
ylabel('Y-axis');
xlabel('Time');
title('Position y Over Time');

subplot(3, 1, 3);
pos3 = drone1.posPlot(3,:);
pos33 = drone2.posPlot(3,:);
time = 0: dt : TOTAL_TIME ; 
plot(time, pos3);
hold on;
plot(time, pos33);
grid on; % Adds a grid for better visualization
ylabel('Z-axis');
xlabel('Time');
title('Position z Over Time');

figure;
subplot(3, 1, 1);
roll = drone1.thetaPlot(1, :);
roll2 = drone2.thetaPlot(1, :);
time = 0: dt : TOTAL_TIME ; 
plot(time, roll);
hold on;
plot(time, roll2);
grid on; % Adds a grid for better visualization
ylabel('Roll');
xlabel('Time');
title('Roll Over Time');

subplot(3, 1, 2);
pitch = drone1.thetaPlot(2, :);
pitch2 = drone2.thetaPlot(2, :);
time = 0: dt : TOTAL_TIME ; 
plot(time, pitch);
hold on;
plot(time, pitch2);
grid on; % Adds a grid for better visualization
ylabel('Pitch');
xlabel('Time');
title('Pitch Over Time');

subplot(3, 1, 3);
yaw = drone1.thetaPlot(3, :);
yaw2 = drone2.thetaPlot(3, :);
time = 0: dt : TOTAL_TIME ; 
plot(time, yaw);
hold on;
plot(time, yaw2);
grid on; % Adds a grid for better visualization
ylabel('Yaw');
xlabel('Time');
title('Yaw Over Time');