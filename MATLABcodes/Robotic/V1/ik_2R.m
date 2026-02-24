function [q_sol, is_valid] = ik_2R(p, robot, elbow)
% IK_2R
% Inverse kinematics for planar 2R arm
%
% Inputs:
%   p      = [x; y] desired position
%   elbow = +1 (elbow-up) or -1 (elbow-down)
%
% Outputs:
%   q_sol   = [q1; q2]
%   is_valid = true if solution respects joint limits

    x = p(1);
    y = p(2);

    L1 = robot.L1;
    L2 = robot.L2;

    r2 = x^2 + y^2;

    % Cosine law
    c2 = (r2 - L1^2 - L2^2)/(2*L1*L2);

    % Outside reachable set
    if abs(c2) > 1
        q_sol = [NaN; NaN];
        is_valid = false;
        return
    end

    s2 = elbow * sqrt(1 - c2^2);
    q2 = atan2(s2, c2);

    q1 = atan2(y, x) - atan2(L2*sin(q2), L1 + L2*cos(q2));

    q_sol = [q1; q2];

    % Joint limit check
    is_valid = ...
        (q1 >= robot.q1_min && q1 <= robot.q1_max) && ...
        (q2 >= robot.q2_min && q2 <= robot.q2_max);
end
