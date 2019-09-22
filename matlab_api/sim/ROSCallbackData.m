classdef ROSCallbackData < event.EventData
    % Simple callback to emulate ROS communication in simulation. 
    
    properties
        LatestMessage
    end
    
    methods
        function obj = ROSCallbackData(LatestMessage)
        %ROSCALLBACKDATA Sets the latest message of the simulated ROS node to
        % a given value.
        %
        %   rcd = ROSCALLBACKDATA(msg) sets the latest message of rcd to msg.
            obj.LatestMessage = LatestMessage;
        end
    end
    
end

