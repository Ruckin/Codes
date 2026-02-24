function Cqd = coriolis_2R(q, dq, robot)
% CORIOLIS_2R
% Computes C(q,dq)*dq

    q2  = q(2);
    dq1 = dq(1);
    dq2 = dq(2);

    m2 = robot.m2;
    L1 = robot.L1;
    Lc2 = robot.Lc2;

    h = -m2*L1*Lc2*sin(q2);

    Cqd = [ h*(2*dq1*dq2 + dq2^2);
           -h*dq1^2 ];
end
