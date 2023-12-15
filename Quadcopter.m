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

        dt = 0.01;
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
        A = zeros(12, 12);
        B = zeros(12, 4);
    end    

    methods
        % Class constructor
        function obj = Quadcopter(ax)
            obj.ax = ax;
            obj.pos = [0, 0, 5];
            force = (obj.m * obj.g) / (4 * obj.k);
            in = [force; force; force; force];
            [obj.A, obj.B] = linearmodel(obj, in);
        end        
        
        % you can modify this to model quadcopter physics
        function update(obj,t)
            % model = linearmodel(obj, input);
            force = (obj.m * obj.g) / (4 * obj.k);
            
            % A = model(1:12);
            % B = model(13:16);
            display(obj.A);
            display(obj.B);
            % input = [force+0.01; force+0.01; force+0.1; force+0.1];
            input = [force+0.001; force+0.001; force+0.001; force+0.001];
            state = [obj.pos; obj.xdot; obj.theta; obj.omega];

            result = obj.A * (state) + obj.B * (input - [force;force;force;force]);
            
            % [obj.pos; obj.xdot; obj.theta; obj.thetadot] = [obj.pos; obj.xdot; obj.theta; obj.thetadot]
            % obj.pos = result(1:3);
            % obj.xdot = result(4:6);
            % obj.theta = result(7:9);
            % obj.theta = result(10:12);
            state = state + obj.dt*result;
            obj.pos = state(1:3);
            obj.xdot = state(4:6);
            obj.theta = state(7:9);
            obj.omega = state(10:12);
            % obj.thetadot 
            

            obj.rot = [obj.theta(3); obj.theta(2); obj.theta(1)];
            display(obj.pos);
            display(obj.rot);
        end

        function [A,B] = linearmodel(obj, input)
            syms p [3, 1] %x_dot pos
            syms linearv [3, 1] %x xdot linear velocity
            syms angles [3, 1] %theta 
            syms angularv [3, 1] %omega angular velocity
            syms dynamics [12, 1]
            syms force [4, 1]
            % state = [obj.pos; obj.xdot; obj.theta; obj.omega];
            % dynamics = [x1_dot; x2_dot; x3_dot; x4_dot];
            state = [p; linearv; angles; angularv];

            % x1_dot = obj.xdot;
            dynamics(1:3) = linearv;

            % x2_dot = [0; 0; -obj.g] + 1 / obj.m * rotation(obj, obj.theta) * obj.k 
            % * [0; 0; obj.k * sum(input)] + 1 / obj.m * (-obj.kd * obj.xdot);
            % phi 1 theta 2 psi 3
            phi = angles(1);
            theta_a = angles(2);
            psi = angles(3);
            R = [cos(psi)*cos(theta_a), cos(psi)*sin(phi)*sin(theta_a) - cos(phi)*sin(psi), sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta_a);
                cos(theta_a)*sin(psi), cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta_a), cos(phi)*sin(psi)*sin(theta_a) - cos(psi)*sin(phi);
                -sin(theta_a), cos(theta_a)*sin(phi), cos(phi)*cos(theta_a)];
            % dynamics(4:6) = [0; 0; -obj.g] + 1 / obj.m * R * [0; 0; obj.k * ...
            %     (force(1) + force(2) + force(3) + force(4))] - 1 / obj.m * obj.kd * linearv;
            dynamics(4:6)=[0;0;-obj.g]+1/obj.m*R*obj.k*[0;0;force1+force2+force3+force4]-(obj.kd/obj.m)*linearv;
            display(dynamics(4:6))

            % x3_dot = omega2thetadot(obj, obj.omega, obj.theta); 
            dynamics(7:9) = inv([1, 0 ,-sin(theta_a);
                0, cos(phi), cos(theta_a)*sin(phi);
                0, -sin(phi), cos(theta_a)*cos(phi)]) * angularv;
            
            tau = [obj.L * obj.k * (force(1) - force(3))
                obj.L * obj.k * (force(2) - force(4))
                obj.b * (force(1) - force(2) + force(3) - force(4))];
            dynamics(10:12) = inv(obj.I) * (tau - cross(angularv, obj.I * angularv));

            % display(dynamics);
            A = jacobian(dynamics, state);
            B = jacobian(dynamics, force);
            % display(A);
            % display(B);
            % af = (obj.m * obj.g) / (4 * obj.k);
            % input = [af;af;af;af];
            
            A = subs(A, force, input);
            A = subs(A, p, obj.pos);
            A = subs(A, linearv, obj.xdot);
            A = subs(A, angles, obj.theta);
            A = subs(A, angularv, obj.omega);
            A = double(A);

            % B = subs(B, force, input);
            B = subs(B, p, obj.pos);
            B = subs(B, linearv, obj.xdot);
            B = subs(B, angles, obj.theta);
            B = subs(B, angularv, obj.omega);
            B = double(B);
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













