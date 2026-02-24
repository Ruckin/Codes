function tau = computedTorqueControl_2R( ...
        q, dq, qd, dqd, ddqd, Kp, Kd, robot)

% Tracking errors
e  = q  - qd;
de = dq - dqd;

% Virtual control
v = ddqd - Kd*de - Kp*e;

% Robot dynamics
M   = massMatrix_2R(q, robot);
Cqd = coriolis_2R(q, dq, robot);
G   = gravity_2R(q, robot);

% Control law
tau = M*v + Cqd + G;
end
