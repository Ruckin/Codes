function M = massMatrix_2R(q, robot)
% MASSMATRIX_2R
% Computes inertia matrix M(q)

    q2 = q(2);

    m1 = robot.m1;
    m2 = robot.m2;
    L1 = robot.L1;
    Lc1 = robot.Lc1;
    Lc2 = robot.Lc2;
    I1 = robot.I1;
    I2 = robot.I2;

    c2 = cos(q2);

    M11 = I1 + I2 ...
        + m1*Lc1^2 ...
        + m2*(L1^2 + Lc2^2 + 2*L1*Lc2*c2);

    M12 = I2 + m2*(Lc2^2 + L1*Lc2*c2);
    M22 = I2 + m2*Lc2^2;

    M = [M11 M12;
         M12 M22];
end
