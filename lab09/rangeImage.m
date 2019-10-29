classdef rangeImage < handle
    %rangeImage Stores a 1D range image and provides related services.
    
    properties(Constant)
        maxUsefulRange = 2.0;
        minUsefulRange = 0.05;
        maxRangeForTarget = 1.0;
        % [x, y, w, h]
        %boundingRect = [0.0, 0.1, 0.20, -0.2];
        edgeWeights = 10;
        wallThresh = 1.5;
        boundingRect = [0.2, 0.3, 0.6, -0.6]
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
            % should be in robot coordinates
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
            goodR = zeros(1, size(obj.rArray, 2));
            goodT = zeros(1, size(obj.tArray, 2));
            goodX = zeros(1, size(obj.xArray, 2));
            goodY = zeros(1, size(obj.yArray, 2));
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
            goodT = obj.tArray(inRange);
            plot(goodR, goodT);
        end
        
        function plotXvsY(obj, maxRange)
            % plot the range image after removing all points exceeding
            % maxRange
            inRange = obj.rArray < maxRange;
            goodX = obj.xArray(inRange);
            goodY = obj.yArray(inRange);
            scatter(goodX, goodY);
        end
        
        function inRangeMids = roiFilter(obj)
            % Returns a list of potential middle points in the bounding rectangle
            % Assumes that the y coordinates start with the left from the
            % robot front being positive, and the right from the robot's
            % front being negative
            xBound = obj.boundingRect(1);
            yBound = obj.boundingRect(2);
            wBound = obj.boundingRect(3);
            hBound = obj.boundingRect(4);
            combMids = [obj.xArray;
                        obj.yArray];
            combMids = combMids(:, combMids(1, :) > xBound);
            combMids = combMids(:, combMids(1, :) < xBound + wBound);
            combMids = combMids(:, combMids(2, :) < yBound);
            inRangeMids = combMids(:, combMids(2, :) > yBound + hBound);
        end

        function [xPos, yPos, th, err, num] = findLineCandidate(obj, middle, maxLen)
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
            numPtsThresh = 5;
            eigThresh = 2.3;
            lengthDiffThresh = 0.2;
            testBoxX = zeros(1, size(obj.rArray, 2));
            testBoxY = zeros(1, size(obj.rArray, 2));
            candidateR = zeros(1, size(obj.rArray, 2));
            candidateT = zeros(1, size(obj.tArray, 2));
            candidateX = zeros(1, size(obj.xArray, 2));
            candidateY = zeros(1, size(obj.yArray, 2));
            candidateIdx = 0;
            testBoxIdx = 0;
            middleX = middle(1);
            middleY = middle(2);
            for i=1:size(obj.rArray,2)
                % filter out the points not in the bounding box
                if sqrt((middleX - obj.xArray(i))^2 + (middleY - obj.yArray(i))^2) < (maxLen / 2)
                    candidateIdx = candidateIdx + 1;
                    testBoxIdx = testBoxIdx + 1;
                    candidateR(candidateIdx) = obj.rArray(i);
                    candidateT(candidateIdx) = obj.tArray(i);
                    candidateX(candidateIdx) = obj.xArray(i);
                    candidateY(candidateIdx) = obj.yArray(i);
                    testBoxX(testBoxIdx) = obj.xArray(i);
                    testBoxY(testBoxIdx) = obj.yArray(i);
                elseif sqrt((middleX - obj.xArray(i))^2 + (middleY - obj.yArray(i))^2) < (maxLen / 1.5)
                    testBoxIdx = testBoxIdx + 1;
                    testBoxX(testBoxIdx) = obj.edgeWeights * obj.xArray(i);
                    testBoxY(testBoxIdx) = obj.edgeWeights * obj.yArray(i);
                end
            end
            % filter if not enough candidate points or the line is too long
            % (wall)
            if candidateIdx == 0
                xPos = 0;
                yPos = 0;
                th = 0;
                err = 10000;
                num = 0;
            else
                r = candidateR(1:candidateIdx);
                t = candidateT(1:candidateIdx);
                x = candidateX(1:candidateIdx);
                y = candidateY(1:candidateIdx);
                xTest = testBoxX(1:testBoxIdx);
                yTest = testBoxY(1:testBoxIdx);
                xMean = mean(x);
                yMean = mean(y);
                xTestMean = mean(xTest);
                yTestMean = mean(yTest);
                xTestIner = xTest - xTestMean;
                yTestIner = yTest - yTestMean;
                xIner = x - xMean;
                yIner = y - yMean;
                Ixx = xIner * xIner';
                Iyy = yIner * yIner';
                Ixy = -xIner * yIner';
                Testxx = xTestIner * xTestIner';
                Testyy = yTestIner * yTestIner';
                Testxy = -xTestIner * yTestIner';
                
                testInertia = [Testxx Testxy; Testxy Testyy] / testBoxIdx;
                inertia = [Ixx Ixy; Ixy Iyy] / candidateIdx; %normalized
                testLambda = eig(testInertia);
                lambda = eig(inertia);
%                 testBoxIdx
%                 candidateIdx
                   
                lambda = sqrt(lambda) * 1000.0;
            
                maxX = max(x);
                minX = min(x);
                maxY = max(y);
                minY = min(y);

                boundingLen = sqrt((maxX - minX)^2 + (maxY - minY)^2);

                if (candidateIdx < numPtsThresh || lambda(1) >= eigThresh || ...
                        boundingLen > (1 + lengthDiffThresh) * maxLen || ...
                        boundingLen < (1 - lengthDiffThresh) * maxLen || ...
                        abs(testLambda(1) - lambda(1)) < obj.wallThresh * lambda(1)) && ...
                        candidateIdx ~= testBoxIdx
                    disp("FAILING ERRORS BELOW");
                    disp(lambda(1));
                    disp(boundingLen);
                    disp(testLambda(1));
                    disp(candidateIdx);
                    xPos = 0;
                    yPos = 0;
                    err = 100000000;
                    num = 0;
                    th = 0;
                else
                    xPos = mean(x);
                    yPos = mean(y);
                    th = atan2(2 * Ixy, Iyy - Ixx)/2.0;
                    num = candidateIdx;
                    perpTh = th + (pi/2);
                    %slope = tan(perpTh);
                    slope = tan(perpTh);
                    %disp("SLOPE")
                    %disp(slope)
                    err = 0;
                    % Filters out things of more than 13 pixels
                    for i=1:num
                        expectedY = slope*candidateX(i) - slope*xPos + yPos;
                        %disp("CANDIDATE X_I")
                        %disp(candidateX(i))
                        %disp("CANDIDATE Y_I")
                        %disp(candidateY(i))
                        residual = candidateY(i) - expectedY;
                        %disp("RESIDUAL")
                        %disp(residual)
                        err = err + residual*residual;
                        %disp("ERR")
                        %disp(err)
                    end
                    err = err / num / num;
                    scatter(candidateX, candidateY);
                    hold on 
                    scatter(xPos, yPos, 10000);
                    disp(err);
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