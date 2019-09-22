classdef slidingFifoQueue < handle
    % A quick and dirty First-In-First-Out queue done with short vectors
    % so that you can run interp1 on the result to interpolate in time.
    % Intended to be used for modelling delays in real-time systems. The
    % length of the queue should be small for performance reasons.
    % interp1 requires that the "x" array be monotone and have unique
    % values in it. That means this queue has to grow to some length by
    % adding at the right and then throwing data out on the left.
    
    properties(Constant)
    end
    
    properties(Access = private)
        maxElements;
    end
    
    properties(Access = public)
        que;
    end
    
    methods(Static = true)
        
    end
    
    methods(Access = private)
         
    end
            
    methods(Access = public)
        
        function obj = slidingFifoQueue(maxElements)
        %SLIDINGFIFOQUEUE Constructs a sliding first-in-first-out queue.
        %
        %   queue = SLIDINGFIFOQUEUE(num) creates a queue with num elements.
            if  nargin > 0
                obj.maxElements = maxElements;
                obj.que = [];
            end
        end
        
        function add(obj,element)
        %ADD Adds an object to the queue
        %
        %   obj.ADD(element) adds element to the right of the queue and 
        %   slides the remaining elements to the left.
            if(length(obj.que) == obj.maxElements)
                obj.que(1:end-1) = obj.que(2:end);
                obj.que(end) = element;
            else
                obj.que(end+1) = element;
            end
            
        end       
    end
end