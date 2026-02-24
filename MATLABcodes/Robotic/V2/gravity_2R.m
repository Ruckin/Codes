function G = gravity_2R(q, robot)
% GRAVITY_2R
% Gravity torque vector

    q1 = q(1);
    q2 = q(2);

    g  = robot.g;
    m1 = robot.m1;
    m2 = robot.m2;
    L1 = robot.L1;
    Lc1 = robot.Lc1;
    Lc2 = robot.Lc2;

    G1 = g*( m1*Lc1*cos(q1) ...
           + m2*(L1*cos(q1) + Lc2*cos(q1+q2)) );

    G2 = g*( m2*Lc2*cos(q1+q2) );

    G = [G1; G2];
end
