function p = fk_2R(q, robot)
% FK_2R
% Forward kinematics of a planar 2R manipulator
%
% Inputs:
%   q     = [q1; q2] joint angles [rad]
%   robot = robot parameter struct
%
% Output:
%   p = [x; y] end-effector position [m]

    q1 = q(1);
    q2 = q(2);

    L1 = robot.L1;
    L2 = robot.L2;

    % End-effector position
    x = L1*cos(q1) + L2*cos(q1 + q2);
    y = L1*sin(q1) + L2*sin(q1 + q2);

    p = [x; y];
end
