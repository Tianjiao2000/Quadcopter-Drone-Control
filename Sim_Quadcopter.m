% Differential drive robot
clc;
clear all;
close all;

% Simulation parameters
TOTAL_TIME  = 20`;
dt          = 0.5;
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
% drone1 = Quadcopter_T(ax1);

% Run Simulation
for t = 0:dt:TOTAL_TIME
    tic
    cla
    
    % _______ IMPLEMENT CONTROLLER + SIMULATION PHYSICS HERE ______ %
    force = (0.3 * 9.8) / 4;
    input = [force+0.001; force+0.001; force+0.001; force+0.001];
    drone1.update(t);
    % drone1.update(t, input);
    drone1.plot;
    % _______ IMPLEMENT CONTROLLER + SIMULATION PHYSICS HERE ______ %
    
    
    drawnow nocallbacks limitrate
    pause(TIME_SCALE*dt-toc); 
end