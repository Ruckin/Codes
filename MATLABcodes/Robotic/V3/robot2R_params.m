function robot = robot2R_params()

    % ----------------------------
    % Geometry
    % ----------------------------
    robot.L1 = 0.3;
    robot.L2 = 0.25;

    % Center of mass (uniform links)
    robot.Lc1 = robot.L1/2;
    robot.Lc2 = robot.L2/2;

    % ----------------------------
    % Mass properties
    % ----------------------------
    robot.m1 = 1.0;   % kg
    robot.m2 = 0.8;   % kg

    robot.I1 = robot.m1*robot.L1^2/12;
    robot.I2 = robot.m2*robot.L2^2/12;

    % ----------------------------
    % Gravity
    % ----------------------------
    robot.g = 9.81;

    % ----------------------------
    % Joint limits
    % ----------------------------
    robot.q1_min = deg2rad(-150);
    robot.q1_max = deg2rad( 150);

    robot.q2_min = deg2rad(-120);
    robot.q2_max = deg2rad( 120);

end
