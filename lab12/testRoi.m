x = linspace(0, 20, 20);
y = linspace(0, 40, 20);
box = [0, 20, 30, -6];


roiFilter(box, x, y)

function inRangeMids = roiFilter(box, x, y)
    xBound = box(1);
    yBound = box(2);
    wBound = box(3);
    hBound = box(4);
    combMids = [x;
                y];
    combMids = combMids(:, combMids(1, :) > xBound)
    combMids = combMids(:, combMids(1, :) < xBound + wBound)
    combMids = combMids(:, combMids(2, :) < yBound)
    inRangeMids = combMids(:, combMids(2, :) > yBound + hBound)
end