classdef Quadcopter < handle
    
    % Define robot fixed parameters
    properties (Access=private, Constant)
        
        %width, length, height offset between centre and rotors
        body = [0.6 0.6 0.0];

        %colours of each component of drone model
        colours = [[.8 .3 .1];[.2 .2 .5];[.8 .1 .3];[.9 .6 .8];[.9 .2 .4]];        

        m = 0.3;
        I = [1, 0, 0; 
            0, 1, 0;
            0, 0, 0.4];
        g = 9.8;
        kd = 0.2;
        k = 1;
        L = 0.25;
        b = 0.2;

        dt = 0.1;
    end
    
    % Define robot variable parameters (incl. state, output, input, etc)
    properties (Access=private)   
        % plotting
        ax (1,1) matlab.graphics.axis.Axes;
        
        % Robot parameters needed for plotting
        pos (3,1) double; % 3D position: x-y-z 
        rot (3,1) double; % 3D orientation: yaw-pitch-roll 

        xdot = zeros(3, 1);
        theta = zeros(3, 1);
        thetadot = zeros(3, 1);
        omega = zeros(3, 1);
        omegadot;

        A = zeros(12, 12);
        B = zeros(12, 4);
    end    

    properties (Access=public)
        posPlot = [];
        thetaPlot = [];
    end

    methods
        % Class constructor
        function obj = Quadcopter(ax)
            obj.ax = ax;
            obj.pos = [0; 0; 1];
            force = (obj.m * obj.g) / (4 * obj.k);
            in = [force; force; force; force];
            [obj.A, obj.B] = linearmodel(obj, in);
        end        
        
        % you can modify this to model quadcopter physics
        function update(obj,t)
            force = (obj.m * obj.g) / (4 * obj.k);
            % small error
            inputa = [force + 0.01; force + 0.01; force + 0.01; force + 0.01];
            % large error
            % inputa = [force+0.05; force+0.05; force+0.01; force+0.01];
            
            state = [obj.pos; obj.xdot; obj.theta; obj.omega];
            result = obj.A * (state) + obj.B * (inputa - [force; force; force; force]);
            
            state = state + obj.dt * result;
            obj.pos = state(1:3);
            obj.xdot = state(4:6);
            obj.theta = state(7:9);
            obj.omega = state(10:12);
            obj.rot = [obj.theta(3); obj.theta(2); obj.theta(1)];

            obj.posPlot = [obj.posPlot, obj.pos];
            obj.thetaPlot = [obj.thetaPlot, obj.theta];
        end

        function pos = getPosition(obj)
            pos = obj.pos;
        end

        function [A,B] = linearmodel(obj, input)
            syms p [3, 1] %x_dot pos
            syms linearv [3, 1] %x xdot linear velocity
            syms angles [3, 1] %theta 
            syms angularv [3, 1] %omega angular velocity
            syms dynamics [12, 1]
            syms force [4, 1]
            state = [p; linearv; angles; angularv];

            % x1_dot = obj.xdot;
            dynamics(1:3) = linearv;

            % x2_dot = [0; 0; -obj.g] + 1 / obj.m * rotation(obj, obj.theta) * obj.k 
            phi = angles(1);
            theta_a = angles(2);
            psi = angles(3);
            R = [cos(psi)*cos(theta_a), cos(psi)*sin(phi)*sin(theta_a) - cos(phi)*sin(psi), sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta_a);
                cos(theta_a)*sin(psi), cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta_a), cos(phi)*sin(psi)*sin(theta_a) - cos(psi)*sin(phi);
                -sin(theta_a), cos(theta_a)*sin(phi), cos(phi)*cos(theta_a)];
            dynamics(4:6)=[0;0;-obj.g]+1/obj.m*R*obj.k*[0;0;force1+force2+force3+force4]-(obj.kd/obj.m)*linearv;

            % x3_dot = omega2thetadot(obj, obj.omega, obj.theta); 
            dynamics(7:9) = ([1, 0 ,-sin(theta_a);
                0, cos(phi), cos(theta_a)*sin(phi);
                0, -sin(phi), cos(theta_a)*cos(phi)]) \ angularv;
            
            % x4_dot
            tau = [obj.L * obj.k * (force(1) - force(3))
                obj.L * obj.k * (force(2) - force(4))
                obj.b * (force(1) - force(2) + force(3) - force(4))];
            dynamics(10:12) = (obj.I) \ (tau - cross(angularv, obj.I * angularv));

            A = jacobian(dynamics, state);
            B = jacobian(dynamics, force);
            
            A = subs(A, force, input);
            A = subs(A, p, obj.pos);
            A = subs(A, linearv, obj.xdot);
            A = subs(A, angles, obj.theta);
            A = subs(A, angularv, obj.omega);
            A = double(A);

            B = subs(B, p, obj.pos);
            B = subs(B, linearv, obj.xdot);
            B = subs(B, angles, obj.theta);
            B = subs(B, angularv, obj.omega);
            B = double(B);
        end
        
        function plot(obj)
            %create middle sphere
            [X Y Z] = sphere(8);
            X = (obj.body(1)/5.).*X + obj.pos(1);
            Y = (obj.body(1)/5.).*Y + obj.pos(2);
            Z = (obj.body(1)/5.).*Z + obj.pos(3);
            s = surf(obj.ax,X,Y,Z);
            set(s,'edgecolor','none','facecolor',obj.colours(1,:));
            
            %create side spheres
            %front, right, back, left
            hOff = obj.body(3)/2;
            Lx = obj.body(1)/2;
            Ly = obj.body(2)/2;
            rotorsPosBody = [...
                0    Ly    0    -Ly;
                Lx    0    -Lx   0;
                hOff hOff hOff hOff];
            rotorsPosInertial = zeros(3,4);
            rot_mat           = eul2rotm(obj.rot.');
            for i = 1:4
                rotorPosBody = rotorsPosBody(:,i);
                rotorsPosInertial(:,i) = rot_mat*rotorPosBody;
                [X Y Z] = sphere(8);
                X = (obj.body(1)/8.).*X + obj.pos(1) + rotorsPosInertial(1,i);
                Y = (obj.body(1)/8.).*Y + obj.pos(2) + rotorsPosInertial(2,i);
                Z = (obj.body(1)/8.).*Z + obj.pos(3) + rotorsPosInertial(3,i);
                s = surf(obj.ax,X,Y,Z);
                set(s,'edgecolor','none','facecolor',obj.colours(i+1,:));
            end            
        end
    end
end













