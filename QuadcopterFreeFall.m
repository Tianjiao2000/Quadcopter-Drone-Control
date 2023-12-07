classdef QuadcopterFreeFall < handle
    
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

        % pos = [10.0 ; 10.0 ; 10.0 ];
        xdot = zeros(3, 1);
        theta = zeros(3, 1);
        thetadot = zeros(3, 1);

        % deviation = 100;
        % thetadot;
        omega = zeros(3, 1);
        omegadot;
    end    

    methods
        % Class constructor
        function obj = QuadcopterFreeFall(ax)
            obj.ax = ax;
            obj.pos = [0, 0, 7];
            % obj.rot = [3; 1; 2];
        end        
        
        % you can modify this to model quadcopter physics
        function update(obj,t)

            % dummy position update
            
            
            % dummy orientation update
            % pitch = 0.3*sin(t*15.2);
            % roll = 0.1*cos(t*33.1 + 0.5);
            % yaw = 2.*pi*sin(t);
            % obj.rot = [yaw;roll;pitch];

            % force = obj.m * obj.g / 4;
            force = 0;

            input = [force; force; force; force];
            % input = [1;1;1;1];
            % i = input(t);
            % ϕ,θ, and ψ, roll pitch yell

            obj.omega = thetadot2omega(obj, obj.thetadot, obj.theta);
            % Compute linear and angular accelerations.
            a = acceleration(obj, input, obj.theta, obj.xdot, obj.m, obj.g, obj.k, obj.kd);
            obj.omegadot = angular_acceleration(obj, input, obj.omega, obj.I, obj.L, obj.b, obj.k);
            obj.omega = obj.omega + obj.dt * obj.omegadot;
            obj.thetadot = omega2thetadot(obj, obj.omega, obj.theta); 
            obj.theta = obj.theta + obj.dt * obj.thetadot; 
            obj.xdot = obj.xdot + obj.dt * a;
            obj.pos = obj.pos + obj.dt * obj.xdot;
            obj.rot = [obj.theta(3); obj.theta(1); obj.theta(2)];
        end

        function omega = thetadot2omega(~, thetadot, theta)
            omega = [1, 0 ,-sin(theta(2));
                0, cos(theta(1)), cos(theta(2))*sin(theta(1));
                0, -sin(theta(1)), cos(theta(2))*cos(theta(1))] * thetadot;
        end

        function thetadot = omega2thetadot(~, omega, theta)
            thetadot = inv([1, 0 ,-sin(theta(2));
                0, cos(theta(1)), cos(theta(2))*sin(theta(1));
                0, -sin(theta(1)), cos(theta(2))*cos(theta(1))]) * omega;
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

        function T = thrust(~, inputs, k)
            T = [0; 0; k * sum(inputs)];
        end

        function tau = torques(~, inputs, L, b, k)
            tau = [L * k * (inputs(1) - inputs(3))
                L * k * (inputs(2) - inputs(4))
                b * (inputs(1) - inputs(2) + inputs(3) - inputs(4))];
        end
        
        function a = acceleration(obj, inputs, angles, xdot, m, g, k, kd) 
            gravity = [0; 0; -g];
            R = rotation(obj, angles);
            T = R * thrust(obj, inputs, k);
            Fd = -kd * xdot;
            a = gravity + 1 / m * T + Fd; 
        end

        function R = rotation(~, angle)
            psi = angle(3);
            theta = angle(2);
            phi = angle(1);
            R = [cos(psi)*cos(theta), cos(psi)*sin(phi)*sin(theta) - cos(phi)*sin(psi), sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta);
                cos(theta)*sin(psi), cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta), cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi);
                -sin(theta), cos(theta)*sin(phi), cos(phi)*cos(theta)];
        end
        
        function omegadot = angular_acceleration(obj, inputs, omega, I, L, b, k) 
            tau = torques(obj, inputs, L, b, k);
            omegadot = inv(I) * (tau - cross(omega, I * omega));
        end
    end
end












