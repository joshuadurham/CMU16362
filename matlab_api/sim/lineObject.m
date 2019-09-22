classdef lineObject < handle
    % A class that provides methods for creating, posing, and plotting a
    % line object for use in maps for simulations
    
    properties(Constant)
		plot_props = {'-', 'LineWidth', 2};
    end
    
    properties(Access = public)
        lines = []; % n x 2
        color = [ 0 0 1 ];
		has_path = false;
		path = [];
		pose = [ 0; 0; 0];
		timer = [];
		cyclic = false;
		h = [];
		line_coords = [];
		id = 0;
    end
            
    methods(Access = public)
		function obj = startTimer(obj)
        %STARTTIMER Starts the timer.
			obj.timer = tic;
		end
		
		function obj = update(obj)
        %UPDATE Updates the pose of the line object.
			if(obj.has_path)
				if(obj.cyclic)
					t = mod(toc(obj.timer), obj.path(end,4));
				else
					t = toc(obj.timer);
				end
				i = find(obj.path(:,4) < t, 1, 'last');
				obj.pose = obj.path(i, 1:3);
			end
			
			%update pose
			obj.line_coords = [...
				obj.pose(1) + cos(obj.pose(3))*obj.lines(:,1) ...
				- sin(obj.pose(3))*obj.lines(:,2), ...
				obj.pose(2) + sin(obj.pose(3))*obj.lines(:,1) ...
				+ cos(obj.pose(3))*obj.lines(:,2)];
        end
        
        function obj = plot(obj)
        %PLOT Draws the line object in a plot.
			if( ~isempty(obj.lines))
				if( ~isempty(obj.h) && ishandle(obj.h))
					set(obj.h, 'XData',obj.line_coords(:,1), ...
						'YData', obj.line_coords(:,2));
                else
                    hold on
					obj.h = plot(obj.line_coords(:,1), obj.line_coords(:,2),...
						obj.plot_props{:}, 'Color', obj.color);
                    hold off
				end
			end
		end
    end
end