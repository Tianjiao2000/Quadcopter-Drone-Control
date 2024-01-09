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
        omegadot = zeros(3, 1);

        Asf = zeros(12, 12);
        Bsf = zeros(12, 4);
        model;
        u = zeros(4, 1);

        target_pos;
        pointer = 1;
        in = zeros(4, 1);
        
        K;
        Ac;
        Bc;
        poles;
        staytime = 0;
        circle;
        points;
        % Gaussian noise parameter
        pos_noise = 0.003;
        theta_noise = 0.001;
        % xdot_noise = 0.002;
        % omega_noise = 0.002;
        % wind force and torque parameter
        wind_force = 0.01;
        wind_torque = 0.005;
        %  observer gain matrix L
        Lso;
        C;
        % e;
    end  

    properties (Access=public)
        posPlot = [];
        thetaPlot = [];
    end

    methods
        % Class constructor
        function obj = Quadcopter(ax)
            obj.ax = ax;
            obj.pos = [0; 0; 0];
            force = (obj.m * obj.g) / (4 * obj.k);
            obj.in = [force; force; force; force];
            model = systemmodel(obj, obj.in);
            obj.Asf = model.A;
            obj.Bsf = model.B;
            % C = model.C;
            % obj.C = [1 0 0 0 0 0 0 0 0 0 0 0;...
            %     0 1 0 0 0 0 0 0 0 0 0 0;...
            %     0 0 1 0 0 0 0 0 0 0 0 0;...
            %     0 0 0 0 0 0 1 0 0 0 0 0;...
            %     0 0 0 0 0 0 0 1 0 0 0 0;...
            %     0 0 0 0 0 0 0 0 1 0 0 0];
            obj.C = eye(size(obj.Ac));
            poles = [0.966 0.959 0.973 0.856 0.822 0.857 0.943 0.960 0.971 0.849 0.835 0.803];
            factor = 10;
            pole_S = poles / factor;
            % poles_S = [0.0966 0.0959 0.0973 0.0856 0.0822 0.0857 0.0943 0.0960 0.0971 0.0849 0.0835 0.0803];
            % poles = [0.1 0.2 0.3 0.4 0.51 0.61 0.71 0.82 0.12 0.23 0.13 0.24];
            % obj.Lso = place(obj.Asf', C', poles)';
            obj.Lso = place(obj.Asf.', obj.C.', pole_S).';
            % obj.Lso = place(obj.Asf.', obj.C.', poles).';
            obj.K = place(obj.Asf, obj.Bsf, poles);
            generateCircular(obj, 50);
            obj.points = [[0 0 5]; obj.circle; [2.5 2.5 2.5]; [2.5 2.5 0]];
            obj.target_pos = obj.points(obj.pointer,:); % target point 
        end        

        function circularTrajectory = generateCircular(obj, numPoints)
            radius = 2.5;
            center_y = 0;
            center_z = 5;
            
            % Generate the trajectory
            theta_circle = linspace(0, 2*pi, numPoints);
            x = zeros(size(theta_circle));
            y = radius * cos(theta_circle) + 0;
            z = radius * sin(theta_circle) + 5;
            p = zeros(3, 1, numPoints);
            circularTrajectory = [x, y, z]';
            circularTrajectory = reshape(circularTrajectory', [], 3);
            obj.circle = circularTrajectory;
        end
        
        % you can modify this to model quadcopter physics
        function update(obj,t)
            % modify pos, theta, xdot and omega to include noise from
            % sensor reading
            % obj.pos = obj.pos + obj.pos_noise * randn(size(obj.pos));
            % obj.theta = obj.theta + obj.theta_noise * randn(size(obj.theta));

            if obj.pos(3) <= 0
                obj.pos(3) = 0;
            end

            % implement the state observer
            state = [obj.pos; obj.xdot; obj.theta; obj.omega];
            D = zeros(size(obj.Bc));
            z = zeros(3,1);
            y = [obj.pos + obj.pos_noise * randn(size(obj.pos)); 
                z; 
                obj.theta + obj.theta_noise * randn(size(obj.theta)); 
                z];
            ye = obj.C * state + D * obj.u;

            state = obj.Asf * state + obj.Bsf * obj.u + obj.Lso * (y - ye);

            e = state - [obj.target_pos'; zeros(9, 1)];

            % e = [obj.pos - obj.target_pos'; state];
            
            % e = (obj.Asf - obj.Lso * obj.C) * e;
            obj.u = - obj.K * e + obj.in;
            obj.u = max(min(obj.u, 1.5), -1.5);
            
            obj.omega = thetadot2omega(obj, obj.thetadot, obj.theta);
            % Compute linear and angular accelerations.
            a = acceleration(obj, obj.u);
            obj.omegadot = angular_acceleration(obj, obj.u);
            obj.omega = obj.omega + obj.dt * obj.omegadot;
            obj.thetadot = omega2thetadot(obj, obj.omega, obj.theta); 
            obj.theta = obj.theta + obj.dt * obj.thetadot; 
            obj.xdot = obj.xdot + obj.dt * a;
            
            if isequal(obj.target_pos,[2.5, 2.5, 0])
                obj.xdot(3) = -0.1;
            elseif sqrt(sum((obj.target_pos' - obj.pos).^2)) < 0.1
                if (sqrt(sum(([0, 0, 5]' - obj.pos).^2)) < 0.1) && (isequal(obj.target_pos,[0, 0, 5]) && (obj.staytime <= 5))
                    obj.staytime = obj.staytime + obj.dt;
                    obj.target_pos = [0, 0, 5];
                    display(obj.staytime);
                elseif (obj.pointer < length(obj.points))
                    obj.pointer = obj.pointer + 1;
                    obj.target_pos = obj.points(obj.pointer,:);
                end
            end

            obj.pos = obj.pos + obj.dt * obj.xdot;

            obj.rot = [obj.theta(3); obj.theta(2); obj.theta(1)];
            display(obj.pos);
            display(obj.target_pos);

            obj.posPlot = [obj.posPlot, obj.pos];
            obj.thetaPlot = [obj.thetaPlot, obj.theta];
        end

        function pos = getPosition(obj)
            pos = obj.pos;
        end

        function model = systemmodel(obj, input)
            syms p [3, 1] %x_dot pos
            syms linearv [3, 1] %x xdot linear velocity
            syms angles [3, 1] %theta 
            syms angularv [3, 1] %omega angular velocity
            syms dynamics [12, 1]
            syms force [4, 1]

            state = [p; linearv; angles; angularv];
            dynamics(1:3) = linearv;

            % phi 1 theta 2 psi 3
            phi = angles(1);
            theta_a = angles(2);
            psi = angles(3);
            R = [cos(psi)*cos(theta_a), cos(psi)*sin(phi)*sin(theta_a) - cos(phi)*sin(psi), sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta_a);
                cos(theta_a)*sin(psi), cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta_a), cos(phi)*sin(psi)*sin(theta_a) - cos(psi)*sin(phi);
                -sin(theta_a), cos(theta_a)*sin(phi), cos(phi)*cos(theta_a)];

            dynamics(4:6)=[0;0;-obj.g]+(1/obj.m)*R*obj.k*[0;0;force1+force2+force3+force4]-(obj.kd*linearv)/obj.m;

            % x3_dot = omega2thetadot(obj, obj.omega, obj.theta); 
            dynamics(7:9) = ([1, 0 ,-sin(theta_a);
                0, cos(phi), cos(theta_a)*sin(phi);
                0, -sin(phi), cos(theta_a)*cos(phi)]) \ angularv;
            
            tau = [obj.L * obj.k * (force(1) - force(3))
                obj.L * obj.k * (force(2) - force(4))
                obj.b * (force(1) - force(2) + force(3) - force(4))];
            dynamics(10:12) = obj.I \ (tau - cross(angularv, obj.I * angularv));

            A = jacobian(dynamics, state);
            B = jacobian(dynamics, force);
            
            A = subs(A, force, input);
            A = subs(A, p, [0;0;5]);
            A = subs(A, linearv, obj.xdot);
            A = subs(A, angles, obj.theta);
            A = subs(A, angularv, obj.omega);
            A = double(A);
            obj.Ac = A;
            
            B = subs(B, p, [0;0;5]);
            B = subs(B, linearv, obj.xdot);
            B = subs(B, angles, obj.theta);
            B = subs(B, angularv, obj.omega);
            B = double(B);
            obj.Bc = B;

            C = [1 1 1 0 0 0 1 1 1 0 0 0];
            D = 0;
            inv_pend_sys = ss(A,B,C,D);
            model = c2d(inv_pend_sys, obj.dt, 'zoh');
        end

        function omega = thetadot2omega(~, thetadot, theta)
            omega = [1, 0 ,-sin(theta(2));
                0, cos(theta(1)), cos(theta(2))*sin(theta(1));
                0, -sin(theta(1)), cos(theta(2))*cos(theta(1))] * thetadot;
        end

        function thetadot = omega2thetadot(~, omega, theta)
            thetadot = [1, 0 ,-sin(theta(2));
                0, cos(theta(1)), cos(theta(2))*sin(theta(1));
                0, -sin(theta(1)), cos(theta(2))*cos(theta(1))] \ omega;
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

        function T = thrust(obj, inputs)
            T = [0; 0; obj.k * sum(inputs)];
        end

        function tau = torques(obj, inputs)
            tau = [obj.L * obj.k * (inputs(1) - inputs(3))
                obj.L * obj.k * (inputs(2) - inputs(4))
                obj.b * (inputs(1) - inputs(2) + inputs(3) - inputs(4))];
        end
        
        function a = acceleration(obj, inputs) 
            gravity = [0; 0; -obj.g];
            R = rotation(obj);
            T = R * thrust(obj, inputs);
            Fd = -obj.kd * obj.xdot;
            wind = obj.wind_force * randn(3, 1);
            a = gravity + 1 / obj.m * T + Fd / obj.m + wind; 
        end

        function R = rotation(obj)
            psi = obj.theta(3);
            the = obj.theta(2); % theta
            phi = obj.theta(1);
            R = [cos(psi)*cos(the), cos(psi)*sin(phi)*sin(the) - cos(phi)*sin(psi), sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(the);
                cos(the)*sin(psi), cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(the), cos(phi)*sin(psi)*sin(the) - cos(psi)*sin(phi);
                -sin(the), cos(the)*sin(phi), cos(phi)*cos(the)];
        end
        
        function omegadot = angular_acceleration(obj, inputs) 
            tau = torques(obj, inputs);
            torque = obj.wind_torque * randn(3, 1);
            omegadot = obj.I \ (tau - cross(obj.omega, obj.I * obj.omega) + torque);
        end

    end
end













