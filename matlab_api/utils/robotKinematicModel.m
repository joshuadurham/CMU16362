classdef robotKinematicModel < handle
    % A convenience class for storing robot physical properties 
    % and performing related kinematic transforms. You can reference the
    % defined constants via the class name (with robotKinematicModel.W2 for
    % example) because they are constant properties and therefore associated
    % with the class rather than any instance. Similiarly, the kinematics
    % routines are also referenced from the class name.
    
    properties(Constant)
        %W  = 9.25*2.54/100;   % NEATO wheel tread in m
        %W2 = 9.25*2.54/2/100; % NEATO 1/2 wheel tread in m
        W  = 0.09;              % Raspbot wheel tread in m
        W2 = 0.045;             % Raspbot 1/2 wheel tread in m
        maxWheelVelocity = 0.5 % max of either wheel in m/sec
        
        %rad = .165;             % NEATO robot body radius is 12.75/2 inches
        rad = 0.06;              % Raspbot robot body radius is 6cm
        %frontOffset = 6*0.0254; % NEATO front surface is 6 inch fwd of axle center
        frontOffset = 2.625*0.0254; % front robot surface is 2-5/8 inch fwd of axle center
        objOffset = 0.015;      % offset from sail face to front of sail
		%laser_l = -0.100;      % Neato laser offset (laser is behind wheels)
        laser_l = -0.000;       % Raspbot laser offset
		laser_rad = 0.04;       % laser housing radius
    end
    
    properties(Access = private)
    end
    
    properties(Access = public)
    end
    
    methods(Static = true)
        
        function [V, w] = vlvrToVw(vl, vr)
        %VLVRTOVW Converts wheel speeds to body linear and angular velocity.
        %
        %   [V, w] = VLVRTOVW(vl, vr) returns the linear velocity V and the
        %   angular velocity w of a body with left wheel speed vl and right
        %   wheel speed vr.
            V = (vr + vl)/2.0;
            w = (vr - vl)/robotKinematicModel.W;
        end
        
        function [vl, vr] = VwTovlvr(V, w)
        %VWTOVLVR Converts body linear and angular velocity to wheel speeds.
        %
        %   [vl, vr] = VWTOVLVR(V, w) returns the left wheel speed vl and
        %   the right wheel speed vr of a body with linear velocity V and
        %   angular velocity w.
            vr = V + robotKinematicModel.W2*w;
            vl = V - robotKinematicModel.W2*w;
        end
        
        function bodyPts = bodyGraph()
            %BODYGRAPH Returns an array of points that can be used to plot the robot
            % body in a window.
            
            % angle arrays
            step = pi/20;
            cir = 0: step: 2.0*pi;

            % circle for laser
            lx = robotKinematicModel.laser_rad*cos(cir);
            ly = robotKinematicModel.laser_rad*sin(cir);

            % body with implicit line to laser circle
            bx = [robotKinematicModel.rad*cos(cir) lx];
            by = [robotKinematicModel.rad*sin(cir) ly];
            
            %create homogeneous points
            bodyPts = [bx ; by ; ones(1,size(bx,2))];
        end
        
        function senToWorld = senToWorld(robPose)
            %SENTOWORLD Finds the sensor pose in the world given the robot 
            % pose in the world.
            %
            %   senPose = SENTOWORLD(robPose) returns the pose senPose of
            %   the sensor in the world given the robot pose in the world,
            %   robPose.
            senToRob = pose(robotKinematicModel.laser_l,0,0);
            senToWorld = robPose.bToA()*senToRob.bToA();
        end
        
        function robToWorld = robToWorld(senPose)
            %ROBTOWORLD Finds the robot pose in the world given the sensor 
            % pose in the world.
            %
            %   robPose = ROBTOWORLD(senPose) returns the pose robPose of
            %   the robot in the world given the sensor pose in the world,
            %   senPose.
            senToRob = pose(robotKinematicModel.laser_l,0,0);
            robToWorld = senPose.bToA()*senToRob.aToB();
        end
        
        function [vl , vr] = limitWheelVelocities(ctrVec)
        %LIMITWHEELVELOCITIES Limits the speed of both wheels.
        %
        %   [limVL, limVR] = LIMITWHEELVELOCITIES(ctrVec) takes a vector of
        %   wheel velocities ctrVec = [vl, vr].  If one of the velocities
        %   is greater than the maximum allowed velocity, both velocities
        %   are scaled relative to the difference between the max velocity
        %   and the supplied velocity.  The function returns the scaled
        %   velocities limVL and limVR.
            vl = ctrVec(1);
            vr = ctrVec(2);
            scale = abs(vr) / robotKinematicModel.maxWheelVelocity;
            if(scale > 1.0)
                vr = vr/scale;
                vl = vl/scale;
            end
            scale = abs(vl) / robotKinematicModel.maxWheelVelocity;
            if(scale > 1.0)
                vr = vr/scale;
                vl = vl/scale;
            end
        end
        
    end
    
    methods(Access = private)
        
    end
            
    methods(Access = public)
        

    end
end