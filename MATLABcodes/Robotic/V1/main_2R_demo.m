clc; clear; close all;

robot = robot2R_params();

% --- Workspace
workspace_2R(robot);

% --- Choose an admissible point
p_ref = [0.35; 0.1];

[q_ref, ok] = ik_2R(p_ref, robot, +1);
if ~ok
    error('Target point is NOT admissible');
end

% --- Fake trajectory (for now)
N = 40;
q_traj = zeros(2,N);
for k = 1:N
    q_traj(:,k) = (k/N)*q_ref;
end

% --- Animate step-by-step
animate_2R_step(q_traj, robot);
