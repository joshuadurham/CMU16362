classdef SimSubscriber < handle
    % Imitates a ROS subscriber for the RaspBot simulator.
	events
        OnMessageReceived % Nofified when message is received by this subscriber
    end
    
    properties
        LatestMessage
        NewMessageFcn
	end
    
    methods
		function obj = SimSubscriber(LatestMessage)
        %SIMSUBSCRIBER Sets the latest message of the simulated ROS node to
        % a given value.
        %
        %   sub = SIMSUBSCRIBER(msg) sets the latest message of sub to msg.
			obj.LatestMessage = LatestMessage;
		end
        function publish(obj)
        %PUBLISH Sends a notification to the ROSCallbackData callback
        % informing it that a message has been 'received'.
			notify(obj, 'OnMessageReceived',ROSCallbackData(obj.LatestMessage));
            % Oct 30,2016, Al Changed 2nd argument from ROSCallbackData(obj.LatestMessage)
            if ~isempty(obj.NewMessageFcn)
                obj.NewMessageFcn(obj,obj.LatestMessage); 
            end
        end
    end 
end