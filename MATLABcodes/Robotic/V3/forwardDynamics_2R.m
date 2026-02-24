function ddq = forwardDynamics_2R(q, dq, tau, robot)
% FORWARDDYNAMICS_2R
% Solves: M(q)ddq = tau - C(q,dq)dq - G(q)

    M   = massMatrix_2R(q, robot);
    Cqd = coriolis_2R(q, dq, robot);
    G   = gravity_2R(q, robot);

    ddq = M \ (tau - Cqd - G);
end
