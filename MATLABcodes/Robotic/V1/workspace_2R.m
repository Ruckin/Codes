function workspace_2R(robot)
% WORKSPACE_2R
% Samples joint space and plots reachable Cartesian workspace

    N = 200;  % resolution

    q1 = linspace(robot.q1_min, robot.q1_max, N);
    q2 = linspace(robot.q2_min, robot.q2_max, N);

    X = [];
    Y = [];

    for i = 1:N
        for j = 1:N
            q = [q1(i); q2(j)];
            p = fk_2R(q, robot);
            X(end+1) = p(1); 
            Y(end+1) = p(2);
        end
    end

    figure;
    scatter(X, Y, 3, 'filled');
    axis equal; grid on;
    xlabel('x [m]');
    ylabel('y [m]');
    title('Reachable Workspace (Joint-Limited)');
end
