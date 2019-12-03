mrpl = mrplSystem();
global robot;
robot = raspbot('RaspBot-11');
mrpl.runRobot(0.5, false, [0, 0, pi()], false, true, -1);
robot.shutdown()