function animate_2R_step(q_traj, robot)
% ANIMATE_2R_STEP
% Step-by-step animation using a button

    L1 = robot.L1;
    L2 = robot.L2;

    fig = figure;
    axis equal; grid on; hold on;
    xlim([-0.6 0.6]);
    ylim([-0.6 0.6]);
    xlabel('x [m]');
    ylabel('y [m]');
    title('2R Robot Animation (Step Mode)');

    % Plot handles
    h1 = plot([0 0], [0 0], 'r', 'LineWidth', 3);
    h2 = plot([0 0], [0 0], 'b', 'LineWidth', 3);
    hE = plot(0, 0, 'ko', 'MarkerSize', 6, 'MarkerFaceColor','k');

    step = 1;

    uicontrol('Style','pushbutton', ...
              'String','Next', ...
              'Position',[20 20 60 30], ...
              'Callback',@nextStep);

    function nextStep(~,~)
        if step > size(q_traj,2)
            return
        end

        q1 = q_traj(1,step);
        q2 = q_traj(2,step);

        % Joint positions
        p0 = [0; 0];
        p1 = [L1*cos(q1); L1*sin(q1)];
        p2 = p1 + [L2*cos(q1+q2); L2*sin(q1+q2)];

        set(h1,'XData',[p0(1) p1(1)],'YData',[p0(2) p1(2)]);
        set(h2,'XData',[p1(1) p2(1)],'YData',[p1(2) p2(2)]);
        set(hE,'XData',p2(1),'YData',p2(2));

        step = step + 1;
    end
end
