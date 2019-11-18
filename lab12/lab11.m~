% -1, -1, -1 encodes spin to 3pi/2,
% 1, 1, 1 encodes spin to pi/2
% 2, 2, 2 encodes drop off pallet
% 3, 3, 3 encodes use laser to pick up pallet
endpoints = [0,      0,      0;
             1,      1,      1;
             12*0.0254, 30*0.0254, pi()/2.0;
             3,      3,      3;
             -1,     -1,     -1;
             21*0.0254, 9*0.0254, 3*pi()/2.0;
             2,      2,      2; 
             
             1,      1,      1;
             24*0.0254, 30*0.0254, pi()/2.0;
             3,      3,      3;
             -1,     -1,     -1;
             27*0.0254, 9*0.0254, 3*pi()/2.0;
             2,      2,      2;  
             
             1,      1,      1;
             36*0.0254, 30*0.0254, pi()/2.0;
             3,      3,      3;
             -1,     -1,     -1;
             33*0.0254, 9*0.0254, 3*pi()/2.0;
             2,      2,      2;
             ];
mrpl = mrplSystem(endpoints);
mrpl.executeTrajectory();