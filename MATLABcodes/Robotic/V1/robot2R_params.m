function robot = robot2R_params()
% ROBOT2R_PARAMS
% Defines all physical and kinematic parameters of a 2R planar robot

    % ----------------------------
    % Geometry
    % ----------------------------
    robot.L1 = 0.3;   % length of link 1 [m]
    robot.L2 = 0.25;  % length of link 2 [m]

    % ----------------------------
    % Joint limits (rad)
    % ----------------------------
    robot.q1_min = deg2rad(-150);
    robot.q1_max = deg2rad( 150);

    robot.q2_min = deg2rad(-120);
    robot.q2_max = deg2rad( 120);

    % ----------------------------
    % For later (dynamics placeholders)
    % ----------------------------
    robot.m1 = 1.0;
    robot.m2 = 0.8;

end
