classdef rangeImage < handle
    %rangeImage Stores a 1D range image and provides related services.
    
    properties(Constant)
        maxUsefulRange = 2.0;
        minUsefulRange = 0.05;
        maxRangeForTarget = 1.0;
        
    end
    
    properties(Access = public)
        rArray = [];
        tArray = [];
        xArray = [];
        yArray = [];
        numPix;
    end
    
    methods(Access = public)
        function obj = rangeImage(ranges, skip, cleanFlag)
            % Constructs a rangeImage for the supplied data.
            % Converts the data to rectangular coordinates
            if(nargin == 3)
                n=0;
                for i=1:skip:length(ranges)
                    n = n + 1;
                    obj.rArray(n) = ranges(i);
                    obj.tArray(n) = (i-1)*(pi/180);
                    obj.xArray(n) = ranges(i)*cos(obj.tArray(n));
                    obj.yArray(n) = ranges(i)*sin(obj.tArray(n));
                end
                obj.numPix = n;
                if cleanFlag; obj.removeBadPoints(); end;
            end
        end
        
        function removeBadPoints(obj)
            % takes all points above and below two convenient range
            % thresholds out of the arrays.  This is a convenience but the
            % result should not be used by any routine that expects the
            % points to be equally separated in angle. The operation is
            % done inline and removed data is deleted.
            goodR = zeros(1, len(obj.rArray));
            goodT = zeros(1, len(obj.tArray));
            goodX = zeros(1, len(obj.xArray));
            goodY = zeros(1, len(obj.yArray));
            numGood = 0;
            for i=1:length(obj.rArray)
                if obj.rArray(i) <= obj.maxUsefulRange && obj.rArray(i) >= obj.minUsefulRange
                    numGood = numGood + 1;
                    goodR(numGood) = obj.rArray(i);
                    goodT(numGood) = obj.tArray(i);
                    goodX(numGood) = obj.xArray(i);
                    goodY(numGood) = obj.yArray(i);
                end
            end
            if numGood == 0
                obj.rArray = [];
                obj.tArray = [];
                obj.xArray = [];
                obj.yArray = [];
            else
                obj.rArray = goodR(1:numGood);
                obj.tArray = goodT(1:numGood);
                obj.xArray = goodX(1:numGood);
                obj.yArray = goodY(1:numGood);
            end
        end
        
        function plotRvsTh(obj, maxRange)
            % plot the range image after removing all points exceeding
            % maxRange
            inRange = obj.rArray < maxRange;
            goodR = obj.rArray(inRange);
            goodT = obj.tArray(inRange)
            plot(goodR, goodT);
        end
        
        function plotXvsY(obj, maxRange)
            % plot the range image after removing all points exceeding
            % maxRange
            inRange = obj.rArray < maxRange;
            goodX = obj.xArray(inRange);
            goodY = obj.yArray(inRange)
            plot(goodX, goodY);
        end

        function [xPos yPos th err num] = findLineCandidate(obj, middle, maxLen)
            %% ASSUMPTION: middle is a (1,2) vector with (x,y) 
            %% ASSUMPTION: maxLen > 0
            %% ASSUMPTION: middle is a point in the rangeImage
            %% Returns all 0 values if it isn't a good candidate at pixel middle
            %% Retruns xCoord, yCoord, theta of centroid, and error and num points
            % find the longest sequence of pixels centered at pixel
            % "middle" whose endpoints are separated by a length less than
            % the provided maximum.  Return the line fit error, the number
            % of pixels participating, and the angle of the line relative
            % to the sensor
            numPtsThresh = 4;
            eigThresh = 1.3;
            lengthDiffThresh = 0.2;
            
            candidateR = zeros(1, len(obj.rArray));
            candidateT = zeros(1, len(obj.tArray));
            candidateX = zeros(1, len(obj.xArray));
            candidateY = zeros(1, len(obj.yArray));
            candidateIdx = 0;
            middleX = middle(1);
            middleY = middle(2);
            for i=1:len(obj.rArray)
                if sqrt((middleX - obj.xArray(i))^2 + (middleY - obj.yArray(i))^2) < (maxLen / 2)
                    candidateIdx = candidateIdx + 1;
                    candidateR(candidateIdx) = obj.rArray(i);
                    candidateT(candidateIdx) = obj.tArray(i);
                    candidateX(candidateIdx) = obj.xArray(i);
                    candidateY(candidateIdx) = obj.yArray(i);
                end
            end
            r = candidateR(1:candidateIdx);
            t = candidateT(1:candidateIdx);
            x = candidateX(1:candidateIdx);
            y = candidateY(1:candidateIdx);
            
            Ixx = x' * x;
            Iyy = y' * y;
            Ixy = -x' * y;
            inertia = [Ixx Ixy; Ixy Iyy] / numPts; %normalized
            lambda = eig(inertia);
            lambda = sqrt(lambda) * 1000.0;
            
            maxX = max(x);
            minX = min(x);
            maxY = max(y);
            minY = min(y);
            
            boundingLen = sqrt((maxX - minX)^2 + (maxY - minY)^2);
            
            if candidateIdx < numPtsThresh || lambda(1) >= eigThresh || boundingLen > (1 + lengthDiffThresh) * maxLen || boundingLen < (1 - lengthDiffThresh) * maxLen
                xPos = 0;
                yPos = 0;
                err = 0;
                num = 0;
                th = 0;
            else 
                xPos = mean(x);
                yPos = mean(y);
                th = atan2(2 * Ixy, Iyy - Ixx)/2.0;
                num = candidateIdx;

                perpTh = th + (pi/2);
                slope = tan(perpTh);
                
                err = 0;
                for i=1:num
                    expectedY = slope*candidateX(i) - slope*xPos + yPos;
                    residual = candidateY(i) - expectedY;
                    err = err + residual*residual;
                end
            end
        end
        
        function num = numPixels(obj)
            num = obj.numPix;
        end
        
        % Modulo arithmetic on nonnegative integers. MATLABs choice to
        % have matrix indices start at 1 has implications for
        % calculations relating to the position of a number in the
        % matrix. In particular, if you want an operation defined on
        % the sequence of numbers 1 2 3 4 that wraps around, the
        % traditional modulus operations will not work correctly.
        % The trick is to convert the index to 0 1 2 3 4, do the
        % math, and convert back.
        
        function out = inc(obj,in)
            % increment with wraparound over natural numbers
            out = indexAdd(obj,in,1);
        end

        function out = dec(obj,in)
            % decrement with wraparound over natural numbers
            out = indexAdd(obj,in,-1);
        end

        function out = indexAdd(obj,a,b)
            % add with wraparound over natural numbers. First number
            % ?a? is "natural" meaning it >=1. Second number is signed.
            % Convert a to 0:3 and add b (which is already 0:3).
            % Convert the result back by adding 1.
            out = mod((a-1)+b,obj.numPix)+1;
        end
    end
end