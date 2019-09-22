classdef scalarField < handle
    % Implements a discretized field over a finite rectangle 
    % in the plane. Maps a rectangle onto an array and permits accesses to
    % the array based on continuous (x,y) coordinates. Establishes the 
    % relationship between discrete and continuous coordinates at construction
    % time and remembers it thereafter. Therefore, it is suitable for 
    % writing to a file and reading it back again without having to
    % remember the transform externally. The special value "empty" can be
    % used to test if a cell was never written.
    
    % Author: Al Kelly. 
    
    properties(Constant)
        empty = inf;
    end
    
    properties(Access = private)

    end
    
    properties(Access = public)
        xMin;
        xMax;
        numX;
        yMin;
        yMax;
        numY;
        dx;
        dy;
        cellArray = [];
    end
    
    methods(Static = true)
        
    end
    
    methods(Access = private)
         
    end
            
    methods(Access = public)
        
        function obj = scalarField(xMin,xMax,numX,yMin,yMax,numY)
            %SCALARFIELD Constructs a scalar field on the supplied rectangle.
            %
            %   field = SCALARFIELD(xMin, xMax, numX, yMin, yMax, numY)
            %   constructs a scalar field with numX x values from xMin to
            %   xMax and numY y values from yMin to yMax.
            if(nargin == 6)
                obj.xMin = xMin;
                obj.xMax = xMax;
                obj.numX = numX;
                obj.yMin = yMin;
                obj.yMax = yMax;
                obj.numY = numY;
                obj.dx = (xMax-xMin)/numX;
                obj.dy = (yMax-yMin)/numY;
                obj.cellArray = obj.empty*ones(numX,numY);
            end
        end
        
        function setAll(obj,value)
            % Sets all cells to some value.
            for i=1:obj.numX
                for j=1:obj.numY
                    obj.cellArray(i,j) = value;
                end
            end
        end
        function set(obj,x,y,val)
            %SET Sets the value of a cell.
            %
            %   obj.SET(x, y, val) sets the value of (x,y) to val.
            if(~obj.isInBounds(x,y))
                err = MException('scalarField:Out Of Bounds', ...
                    'Exiting...');
                throw(err);
            end
            i = obj.xToi(x);
            j = obj.yToj(y);
            obj.cellArray(i,j) = val;
        end
        
        function val = get(obj,x,y)
            %GET Returns the value of a cell.
            %
            %   obj.GET(x, y) retrieves the value of (x,y).
            if(~obj.isInBounds(x,y))
                err = MException('scalarField:outOfbounds', ...
                    'Out of Bounds Lookup in Scalar Field');
                throw(err);
            end
            i = obj.xToi(x);
            j = obj.yToj(y);
            val = obj.cellArray(i,j);
        end
        
        function val = isInBounds(obj,x,y)
            %ISINBOUNDS Checks to see if a cell is within the field
            % boundaries.
            %
            %   val = obj.ISINBOUNDS(x, y) returns true if (x,y) is within
            %   the boundaries of the scalar field.
            
            if(x<obj.xMin || x>obj.xMax || y<obj.yMin || y>obj.yMax)
                val = false;
            else
                val = true;
            end
        end
        
        function [min, max] = range(obj)
            %RANGE Returns the range of values in the scalar field.
            %
            %   [a, b] = obj.RANGE returns the minimum value a and the
            %   maximum, noninfinite value b from the scalar field.
            min = inf ; max = -inf;
            for i=1:obj.numX
                for j=1:obj.numY
                    if(isinf(obj.cellArray(i,j))); continue; end
                    if(obj.cellArray(i,j) < min) 
                        min = obj.cellArray(i,j); 
                    end
                    if(obj.cellArray(i,j) > max) 
                        max = obj.cellArray(i,j); 
                    end
                end
            end
        end
        
        function points = getNeighbors(obj,pt)
            %GETNEIGHBORS Finds all neighbors of a given point; does not return anything
            % outside the bounds of the cellArray.
            %
            %   points = obj.GETNEIGHBORS(pt) returns an array points of
            %   points neighboring the given point pt.
            points = [];
            num = 1;
            i = pt(1); j = pt(2);
            for ii= -1:1:1
                for jj= -1:1:1
                    if( ii == 0 && jj == 0); continue; end
                    if(    i+ii >= 1 && i+ii <= size(obj.cellArray,1) ...
                       &&  j+jj >= 1 && j+jj <= size(obj.cellArray,2))
                        points(:,num) = [i+ii ; j+jj]; %#ok<AGROW>
                        num = num + 1;
                    end
                end
            end
        end
           
        function pt = xyToIj(obj,x,y)
            %XYTOIJ Converts a cell value to its index.
            %
            %   pt = obj.XYTOIJ(x, y) converts the value (x,y) to the index
            %   (i,j) and returns it in the form pt = [i; j].
            i = xToi(obj,x);
            j = yToj(obj,y);
            pt = [i ; j];
        end
        
        function pt = ijToXy(obj,i,j)
            %IJTOXY Converts a cell index to its value.
            %
            %   pt = obj.IJTOXY(i, j) converts the index (i,j) to the value
            %   (x,y) and returns it in the form pt = [x; y].
            x = iToX(obj,i);
            y = iToy(obj,j);
            pt = [x ; y];
        end
        
        function i = xToi(obj,x)
            %XTOI Converts x coordinate to row index.
            %
            %   i = obj.XTOI(x) returns the row index i for the x
            %   coordinate x.
            i = floor((x-obj.xMin)/obj.dx) + 1;  end  
        function j = yToj(obj,y)
            %YTOJ Converts y coordinate to column index.
            %
            %   j = obj.YTOJ(y) returns the column index j for the y
            %   coordinate y.
            j = floor((y-obj.yMin)/obj.dy) + 1;  end  
        function x = iToX(obj,i)
            %ITOX Converts row index to x coordinate.
            %
            %   x = obj.ITOX(i) returns the x coordinate x for the row
            %   index i.
            x = obj.xMin + obj.dx*(i-0.5);  end % changed Aug 21, 2014, add xmin
        function y = jToY(obj,j)
            %JTOY Converts column index to y coordinate.
            %
            %   y = obj.JTOY(j) returns the y coordinate y for the column
            %   index j.
            y = obj.yMin + obj.dy*(j-0.5);  end % changed Aug 21, 2014 add ymin
        function dx = getDx(obj)
            %GETDX Returns the x resolution of the scalar field.
            dx = obj.dx; end
        function dy = getDy(obj)
            %GETDY Returns the y resolution of the scalar field.
            dy = obj.dy; end
    end
end