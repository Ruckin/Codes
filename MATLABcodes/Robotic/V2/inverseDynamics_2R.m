function tau = inverseDynamics_2R(q, dq, ddq, robot)
% INVERSEDYNAMICS_2R
% Computes required joint torques

    M   = massMatrix_2R(q, robot);
    Cqd = coriolis_2R(q, dq, robot);
    G   = gravity_2R(q, robot);

    tau = M*ddq + Cqd + G;
end
