endpoints = [0,      0,      0;
             0.3048, 0.3048, 0.0;
            -0.3048, -0.3048, - pi()/2.0;
             0,      0,      0];
mrpl = mrplSystem(endpoints);
mrpl.executeTrajectory();