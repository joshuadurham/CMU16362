classdef simRobot < handle
    %   Simulates a differential drive robot.
    %   Accepts left and right wheel commands or encoder readings for a
    %   differentially steered robot and integrates them with respect to time
    %   in the plane. Produces a pose estimate of the form [x y th]. This
    %   class can be configured to work in real time (from the processor
    %   clock) or to work from time tags on sensor data; you choose which
    %   when the object is turned on and you have to stick with that choice
    %   afterwards. This is a design feature to prevent mixing up time
    %   sources. In the case of real-time it is critical that you fire up
    %   the instance right before (and not some time before) you start
    %   feeding it commands or sensor readings. This makes the delays and
    %   time derivatives work correctly. Independently from the nature of
    %   time that is used, the class can be used for estimation if you
    %   drive it with encoder readings or for predictive control if you
    %   drive it with commands. In the latter case, commands are placed in
    %   a FIFO implemented as a sliding vector so that an (interpolated
    %   and) delayed command is the one being simulated. Commands are
    %   assumed to arrive with monotone time tags and it is only possible
    %   to query the latest state. In effect, this object simulates the
    %   data at recieve time and the latest commands provided are treated
    %   as not having arrived yet. Attempting to read past the FIFO on
    %   either end will throw an error in the MATLAB function interp1.
    %   Logging a dedicated number of samples of pose, command, and time is
    %   built-in for convenience if enabled in the constructor.
    
    properties(Constant)
        W  = robotKinematicModel.W;     % wheelTread in m
        W2 = robotKinematicModel.W2;    % 1/2 wheelTread in m
        maxFifo = 100;                  % max length of command FIFOs
        maxLogLen = 10000;              % max length of logging buffers
    end
    
    properties(Access = public)
        s = 0.0;
        pose = [0;0;0];
        V = 0.0; % linear velocity;
        w = 0.0; % angular velocity
        vl = 0.0;
        vr = 0.0;
        startTic = 0;
        lastTime = 0.0;
        started = false;
        doLogging = false;
        doDebug = false;
        tdelay = 0.0;
        realTimeMode = true;
        
        vlHistory = slidingFifoQueue(simRobot.maxFifo);
        vrHistory = slidingFifoQueue(simRobot.maxFifo);
        tHistory  = slidingFifoQueue(simRobot.maxFifo);
    end
    
    properties(Access = public)
        encoders = struct('LatestMessage',struct('Left',0,'Right',0)); % wheel encoders
        logIndex = 1; % index of last logged data point in the arrays
        logArrayT  = zeros(1,simRobot.maxLogLen); % time log array
        logArrayS  = zeros(1,simRobot.maxLogLen); % (signed) distance log array
        logArrayX  = zeros(1,simRobot.maxLogLen); % x log array
        logArrayY  = zeros(1,simRobot.maxLogLen); % y log array
        logArrayTh = zeros(1,simRobot.maxLogLen); % heading log array
        logArrayV  = zeros(1,simRobot.maxLogLen); % linear velocity cmd log array
        logArrayW  = zeros(1,simRobot.maxLogLen); % angular velocity command log array
    end
    
    methods(Static = true)
        
    end
    
    methods(Access = private)
        
        function logData(obj)
        %LOGDATA Stores data in the logging buffers until they are full.
            if(obj.doLogging == false)
                return;
            end
            if(obj.logIndex > 10000)
                fprintf('SimRobot: Logarray Length Exceeded. Logging stopped. \n');
                obj.logIndex = 10000;
            end         
            obj.logArrayT(obj.logIndex)  = obj.lastTime;
            obj.logArrayT(obj.logIndex)  = obj.s;
            obj.logArrayX(obj.logIndex)  = obj.pose(1);
            obj.logArrayY(obj.logIndex)  = obj.pose(2);
            obj.logArrayTh(obj.logIndex) = obj.pose(3);
            obj.logArrayV(obj.logIndex)  = obj.V;
            obj.logArrayW(obj.logIndex)  = obj.w;
            obj.logIndex = obj.logIndex + 1;
            if(obj.doDebug == true)
                %fprintf('Logging t:%f V:%f\n',obj.lastTime,obj.V);
            end
        end   

        function [vlDelay, vrDelay] = delayCommands(obj,vl,vr,timeTag,delay)
        %DELAYCOMMANDS Implements a time delay on the robot commands.
        %
        %   [vlD, vrD] = obj.DELAYCOMMANDS(vl, vr, t, d) implements a delay
        %   d on the left and right wheel velocities vl and vr and returns
        %   the delayed velocities vlD and vrD.  The time tag t is used for
        %   logging and time history.

            obj.tHistory.add(timeTag);
            obj.vlHistory.add(vl);
            obj.vrHistory.add(vr);
            vlDelay = interp1(obj.tHistory.que,obj.vlHistory.que,timeTag-delay);
            if(obj.doDebug == true)
                %fprintf('Adding t:%f V:%f Querying :%f Got:%f\n',timeTag,vl,timeTag-delay,vlDelay);
            end
            vrDelay = interp1(obj.tHistory.que,obj.vrHistory.que,timeTag-delay);

        end
                        
        function updateStateStep(obj,time)
        %UPDATESTATESTEP Updates state based only on the passage of the
        % supplied time step and the active commanded velocities.
        %
        %   obj.UPDATESTATESTEP(t) updates the state using the difference
        %   between the time of the last update and the supplied time t.
            dt = time - obj.lastTime;
            obj.lastTime = time;
            % update encoders
             obj.encoders.LatestMessage.Vector.X = ...
                obj.encoders.LatestMessage.Vector.X + obj.vl*dt;
            obj.encoders.LatestMessage.Vector.Y= ...
                obj.encoders.LatestMessage.Vector.Y + obj.vr*dt;
            %obj.encoders.LatestMessage.Left = ...
                %obj.encoders.LatestMessage.Left + obj.vl*dt*1000.0;
            %obj.encoders.LatestMessage.Right = ...
                %obj.encoders.LatestMessage.Right + obj.vr*dt*1000.0;
            % update pose
            obj.pose(3) = obj.pose(3) + obj.w*dt;
            th = obj.pose(3);
            ds = obj.V * dt;
            obj.s = obj.s + ds;
            obj.pose(1) = obj.pose(1) + ds*cos(th);
            obj.pose(2) = obj.pose(2) + ds*sin(th);
            % Log data
            obj.logData();
        end
        
        function sendVelocityStep(obj,vl,vr,time)
        %SENDVELOCITYSTEP Sends commands to the left and right wheels. Note
        % that the commands are sent through a FIFO so these commands will
        % actually affect the prediction/estimation after the delay has
        % elapsed.  The delay is affected in continuous time with linear
        % interpolation of the command FIFO.
        %
        %   obj.SENDVELOCITYSTEP(vl, vr, t) sends left and right wheel
        %   velocity commands vl and vr to the simulated robot.  The time t
        %   is used as the time tag for the delayCommands function.
            [vld, vrd] = delayCommands(obj,vl,vr,time,obj.tdelay);
            obj.vl = vld;
            obj.vr = vrd;
            obj.V = (vrd+vld)/2.0;
            obj.w = (vrd-vld)/obj.W;
        end
        
        function checkRealTimeMode(obj,mode)
        %CHECKREALTIMEMODE Panics if the wrong time is used in a public
        % method.
        %   obj.CHECKREALTIMEMODE(m) throws an error if the mode of the
        %   simulated robot and the supplied mode m do not match.
            if(obj.realTimeMode ~= mode)
            	err = MException('simRobot:InvalidMode', ...
                    'This Method Cannot be Used in this Mode');
                throw(err);
            end
        end
          
    end
            
    methods(Access = public)
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Constructor and Related Methods
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function obj = simRobot(tdelay,pose,leftEnc,rightEnc,doLogging)
        %SIMROBOT Constructs a simRobot. After construction you must then 
        % "fireUp" the robot RIGHT BEFORE you start using it in real time
        % so that the simulation delay works correctly.
        %
        %   robot = SIMROBOT(tdelay, pose, leftEnc, rightEnc, log)
        %   constructs a simulated robot at coordinate (x,y) with
        %   orientation theta (taken from pose = [x; y; theta]).  Commands
        %   will be delayed by time tdelay.  Initial encoder values for the
        %   left and right wheels are leftEnc and rightEnc, respectively.
        %   Logging is turned on if log = true and off if log = false.
            if  nargin > 0
                obj.tdelay = tdelay;
                obj.pose = pose;
                %need to update for new API
                obj.encoders.LatestMessage.Vector.X  = leftEnc;
                obj.encoders.LatestMessage.Vector.Y = rightEnc;
                %obj.encoders.LatestMessage.Data(1)  = leftEnc;
                %obj.encoders.LatestMessage.Data(2) = rightEnc;
                
                obj.s = (leftEnc+rightEnc)/2.0;
                obj.doLogging = doLogging;
            end
        end
        
        function fireUpForRealTime(obj)
        %FIREUPFORREALTIME Starts the clock and put the robot in real time
        % mode where state updates will be tagged based on true elapsed
        % time. Calls fireUpForSuppliedTime() for the present time.
            obj.startTic = tic();
            time = toc(obj.startTic);
            fireUpForSuppliedTime(obj, time);
            obj.realTimeMode = true;
        end
        
        function fireUpForSuppliedTime(obj, time)
        %FIREUPFORSUPPLIEDTIME Starts the clock and put the robot in
        % simulated time mode where state updates will be tagged based on
        % true supplied time. Save the present state as the start state.
        % Issue and pad the command history with zero commands. Call this
        % right before you want to use the object in real time so that the
        % time taken to set it up does not corrupt the clock.
        %
        %   obj.FIREUPFORSUPPLIEDTIME(t) starts the simulation clock and
        %   supplies the given time t to simulation functions.
            obj.started = true;
            % Initialize the commands to zero velocity for now and 
            % (slightly more than) tdelay before now so that a request for
            % a delayed command at zero produces an answer.
            obj.tHistory.add(time-obj.tdelay-1e-6); % go back extra microsec for safety
            obj.vlHistory.add(0.0);
            obj.vrHistory.add(0.0);
            obj.logData();
            obj.tHistory.add(time);
            obj.vlHistory.add(0.0);
            obj.vrHistory.add(0.0);
            obj.realTimeMode = false;
            obj.logData();
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Control and Estimation Methods
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function sendVelocity(obj,vl,vr)
        %SENDVELOCITY Sends velocity commands to the left and right wheels.
        % Calls sendVelocityForSuppliedTime for the present time.
        %
        %   obj.SENDVELOCITY(vl, vr) commands the simulation to apply
        %   velocity vl to the left wheel and velocity vr to the right
        %   wheel.
            obj.checkRealTimeMode(true); % This is a real time function call
            thisTime = toc(obj.startTic);
            sendVelocityStep(obj,vl,vr,thisTime);
        end
        
        function sendVelocityAtTime(obj,vl,vr,time)
        %SENDVELOCITYATTIME Sends velocity commands to the left and right
        % wheels at a given time. Note that the commands are sent through a
        % FIFO so these commands will actually affect the prediction and
        % estimation after the delay has elapsed. The delay is effected in
        % continuous time with linear interpolation of the command FIFO.
        %
        %   obj.SENDVELOCITYATTIME(vl, vr, t) commands the simulation to
        %   apply velocity vl to the left wheel and velocity vr to the
        %   right wheel at time t.
            obj.checkRealTimeMode(false); % This is a non-real time function call
            sendVelocityStep(obj,vl,vr,time);
        end
        
        function setDebug(obj)
        %SETDEBUG Used to turn on debugging for this instance only. You can
        % insert code and test (obj.debug == true) to activate it.
            obj.doDebug = true;
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Update State Methods
        %
        % The next three methods are used to update the state. Both 
        % updateState() and updateStateFromEncoders() call 
        % updateStateStep(0 and the latter will call the logger to log all 
        % the data.
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
        function updateState(obj)
        %UPDATESTATE Updates state based only on the passage of real time.
            obj.checkRealTimeMode(true); % This is a real time function call
            time = toc(obj.startTic);  % get time difference
            obj.updateStateStep(time);
        end
        
        function updateStateAtTime(obj,time)
        %UPDATESTATEATTIME Updates state based on a supplied time.
        %
        %   obj.UPDATESTATEATTIME(t) updates the simulation state based on
        %   the supplied time t.
            obj.checkRealTimeMode(false); % This is a non-real time function call
            obj.updateStateStep(time);
        end

        function updateStateFromEncodersAtTime(obj,newLeft,newRight,time)
        % UPDATESTATEFROMENCODERSATTIME Sets wheel and body velocities
        % based on encoders and supplied time tag.
        %
        %   obj.UPDATESTATEFROMENCODERSATTIME(newL, newR, t) updates the
        %   simulated wheel and body velocities using encoder values from
        %   the left and right wheels (newL and newR, respectively) and the
        %   supplied time t.
            obj.checkRealTimeMode(false); % This is a non-real time function call
            dt = time-obj.lastTime;
            %Need to change for API
            oldLeft = obj.encoders.LatestMessage.Vector.X;
            oldRight = obj.encoders.LatestMessage.Vector.Y;
            %oldLeft = obj.encoders.LatestMessage.Data(1);
            %oldRight = obj.encoders.LatestMessage.Data(2); 

            %Need to change
            obj.vl = (newLeft - oldLeft)/dt;
            obj.vr = (newRight - oldRight)/dt;        
            %obj.vl = (newLeft - oldLeft)/dt/1000.0;
            %obj.vr = (newRight - oldRight)/dt/1000.0;
            obj.V = (obj.vr+obj.vl)/2.0;
            obj.w = (obj.vr-obj.vl)/obj.W;
            obj.updateStateStep(time);       
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Access Methods
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function [V, w] = getDelayedVw(obj)
        %GETDELAYEDVW Returns the most recent linear and angular velocity
        % commands processed. These are delayed because they went through
        % the velocity command FIFO.
            V = obj.V;
            w = obj.w;
        end
        
        function pose = getPose(obj)
        %GETPOSE Returns the most recent computed pose.
            pose = obj.pose;
        end
        
        function time = getLastTime(obj)
        %GETLASTTIME Returns the most recent computed time.
            time = obj.lastTime;
        end
        
        function dist = getDistance(obj)
        %GETDISTANCE Returns the most recent computed distance.
            dist = obj.s;
        end
            
    end
end