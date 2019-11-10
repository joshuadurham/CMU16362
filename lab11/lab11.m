endpoints = [0,      0,      0;
             0.3048, 0.9144, pi()/2.0;
             0.9144, 0.3048, 0.0;
             0.6096, 0.6096, pi()/2.0];
mrpl = mrplSystem(endpoints);
mrpl.executeTrajectory();