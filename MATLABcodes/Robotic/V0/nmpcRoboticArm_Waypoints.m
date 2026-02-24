%% NONLINEAR MPC CONTROLLER DESIGN EXAMPLE
%  FREDERICO CASARA ANTONIAZZI - 05/02/2026
%
%  MIMO Nonlinear MPC for a 2-DOF robotic arm
%  - Nonlinear dynamics
%  - Torque constraints
%  - State constraints (angles + velocities)
%  - End-effector tracking

clc; clear; close all; format short;

%% ====================== PHYSICAL PARAMETERS ======================

g = 9.81;  % gravity

% Link lengths
a1 = 0.2;
a2 = 0.15;

% Masses (simplified from geometry)
m1 = 2.0;
m2 = 1.5;

% Centers of mass
ac1 = a1/2;
ac2 = a2/2;

% Inertias
I1 = m1*a1^2/12;
I2 = m2*a2^2/12;

%% ====================== MPC PARAMETERS ======================

Ts = 0.02;     % sampling time
N  = 15;       % prediction horizon

% Cost weights
Qy = diag([200 200]);   % output tracking
Ru = diag([0.05 0.05]); % control effort

% Input constraints (torques)
u_min = [-5; -5];
u_max = [ 6;  6];

%% ====================== STATE CONSTRAINTS ======================

% Joint angle limits (rad)
limits.theta1 = [deg2rad(-120), deg2rad(150)];
limits.theta2 = [deg2rad(-120), deg2rad(150)];

% Joint velocity limits (rad/s)
limits.dtheta = [-5, 5];

%% ====================== INITIAL STATE ======================

% x = [theta1; dtheta1; theta2; dtheta2]
x = [0.1; 0; 0.1; 0];

Tf = 5;                      % simulation time
steps = round(Tf/Ts);

% Logs
X_log = zeros(4,steps);
U_log = zeros(2,steps);
Y_log = zeros(2,steps);

%% ====================== REFERENCE ======================

% Waypoints (y,z)
waypoints = [ ...
    0.15  0.22  0.25;
   -0.10  0.05  0.15];

durations = [80 80 120];

Yref_series = cartesian_waypoints(steps, waypoints, durations);

%% ====================== NMPC SIMULATION LOOP ======================

for k = 1:steps

    % Reference
    y_ref = Yref_series(:,k);

    % Initial guess for optimizer
    u0 = zeros(2*N,1);

    % Solver options
    options = optimoptions('fmincon',...
        'Display','none',...
        'Algorithm','sqp',...
        'MaxIterations',60);

    % NMPC optimization
    u_opt = fmincon( ...
        @(u) costNMPC(u,x,y_ref,N,Ts,Qy,Ru,...
            a1,a2,ac1,ac2,m1,m2,I1,I2,g), ...
        u0,[],[],[],[], ...
        repmat(u_min,N,1), ...
        repmat(u_max,N,1), ...
        @(u) stateConstraints(u,x,N,Ts,limits,...
            a1,a2,ac1,ac2,m1,m2,I1,I2,g), ...
        options);

    % Apply first control move
    u = u_opt(1:2);

    % Propagate system
    x = robotStep(x,u,Ts,a1,a2,ac1,ac2,m1,m2,I1,I2,g);

    % Output (end-effector position)
    y = endEffector(x,a1,a2);

    % Log data
    X_log(:,k) = x;
    U_log(:,k) = u;
    Y_log(:,k) = y;
end

%% ====================== VERIFICATION ======================

e = vecnorm(Y_log - y_ref, 2, 1);

tol = 0.01;

if e(end) > tol
    warning('Reference NOT reached. Final error = %.4f m', e(end));
else
    disp('Reference successfully reached.');
end

% Progress check
if any(diff(e) > 0)
    warning('Non-monotonic convergence detected (possible local minima).');
end

%% ====================== PLOTS ======================

t = (0:steps-1)*Ts;

figure;
plot(Y_log(1,:),Y_log(2,:),'LineWidth',2); hold on;
plot(y_ref(1),y_ref(2),'rx','MarkerSize',12,'LineWidth',2);
grid on;
xlabel('y (m)'); ylabel('z (m)');
title('End-Effector Trajectory');
legend('NMPC Path','Reference');

figure;
subplot(2,1,1)
plot(t,U_log(1,:),'LineWidth',1.5); grid on;
ylabel('\tau_1 (Nm)');

subplot(2,1,2)
plot(t,U_log(2,:),'LineWidth',1.5); grid on;
ylabel('\tau_2 (Nm)');
xlabel('Time (s)');

%% ====================== ANIMATIONS ======================

animate_robot_arm(X_log(1,:), X_log(3,:), a1, a2, Ts);

animate_robot_arm_stepwise(X_log(1,:), X_log(3,:), a1, a2, Ts);

%% ====================== HELPER FUNCTIONS ======================

function J = costNMPC(u,x0,yref,N,Ts,Qy,Ru,...
    a1,a2,ac1,ac2,m1,m2,I1,I2,g)

x = x0;
J = 0;

for k = 1:N
    uk = u(2*k-1:2*k);
    x  = robotStep(x,uk,Ts,a1,a2,ac1,ac2,m1,m2,I1,I2,g);
    y  = endEffector(x,a1,a2);

    % Tracking error
    e = y - yref;

    % Accumulate cost
    J = J + e'*Qy*e + uk'*Ru*uk;
end
end

function [c, ceq] = stateConstraints(u,x0,N,Ts,limits,...
    a1,a2,ac1,ac2,m1,m2,I1,I2,g)

x = x0;
c = [];
ceq = [];

for k = 1:N

    uk = u(2*k-1:2*k);
    x  = robotStep(x,uk,Ts,a1,a2,ac1,ac2,m1,m2,I1,I2,g);

    % Joint angle limits
    c = [c;
         x(1) - limits.theta1(2);
         limits.theta1(1) - x(1);
         x(3) - limits.theta2(2);
         limits.theta2(1) - x(3)];

    % Velocity limits
    c = [c;
         x(2) - limits.dtheta(2);
         limits.dtheta(1) - x(2);
         x(4) - limits.dtheta(2);
         limits.dtheta(1) - x(4)];
end
end

function xnext = robotStep(x,u,Ts,a1,a2,ac1,ac2,m1,m2,I1,I2,g)

th1 = x(1); dth1 = x(2);
th2 = x(3); dth2 = x(4);

C2 = cos(th2);
S2 = sin(th2);

% Mass matrix
M = [m1*ac1^2 + m2*(a1^2 + ac2^2 + 2*a1*ac2*C2) + I1 + I2,...
     m2*(ac2^2 + a1*ac2*C2) + I2;
     m2*(ac2^2 + a1*ac2*C2) + I2,...
     m2*ac2^2 + I2];

% Coriolis matrix
C = [-2*m2*a1*ac2*S2*dth2, -m2*a1*ac2*S2*dth2;
      m2*a1*ac2*S2*dth1,  0];

% Gravity vector
G = [m1*g*ac1*cos(th1) + m2*g*(a1*cos(th1) + ac2*cos(th1+th2));
     m2*g*ac2*cos(th1+th2)];

% Joint accelerations
ddth = M\(u - C*[dth1; dth2] - G);

% Euler integration
xnext = x + Ts*[dth1; ddth(1); dth2; ddth(2)];
end

function y = endEffector(x,a1,a2)
th1 = x(1);
th2 = x(3);
y = [a1*cos(th1) + a2*cos(th1+th2);
     a1*sin(th1) + a2*sin(th1+th2)];
end

function animate_robot_arm_stepwise(theta1,theta2,a1,a2,Ts)

fig = figure;
axis equal; grid on; hold on;

L = a1 + a2 + 0.05;
xlim([-L L]); ylim([-L L]);

xlabel('x (m)');
ylabel('y (m)');
title('Step-by-step 2-DOF Robot Animation');

% Base
plot(0,0,'ko','MarkerFaceColor','k');

% Links
h1 = plot([0 0],[0 0],'r','LineWidth',3);
h2 = plot([0 0],[0 0],'b','LineWidth',3);

% Button
btn = uicontrol('Style','pushbutton',...
    'String','NEXT',...
    'FontSize',12,...
    'Position',[20 20 80 40],...
    'Callback',@nextStep);

k = 1;

    function nextStep(~,~)
        if k > length(theta1)
            return;
        end

        th1 = theta1(k);
        th2 = theta2(k);

        x1 = a1*cos(th1);
        y1 = a1*sin(th1);

        x2 = x1 + a2*cos(th1+th2);
        y2 = y1 + a2*sin(th1+th2);

        set(h1,'XData',[0 x1],'YData',[0 y1]);
        set(h2,'XData',[x1 x2],'YData',[y1 y2]);

        drawnow;
        k = k + 1;
    end
end

function animate_robot_arm(theta1, theta2, a1, a2, Ts)
% Animate a 2-DOF planar robotic arm
%
% theta1, theta2 : joint angle histories (rad)
% a1, a2         : link lengths (m)
% Ts             : sampling time (s)

figure;
axis equal;
grid on;
hold on;

axis_lim = a1 + a2 + 0.05;
xlim([-axis_lim axis_lim]);
ylim([-axis_lim axis_lim]);

xlabel('x (m)');
ylabel('y (m)');
title('2-DOF Robotic Arm NMPC Animation');

% Base
plot(0,0,'ko','MarkerSize',8,'MarkerFaceColor','k');

% Graphics objects
h_link1 = plot([0 0],[0 0],'r','LineWidth',3);
h_link2 = plot([0 0],[0 0],'b','LineWidth',3);
h_traj  = plot(0,0,'k--');

xe_hist = [];
ye_hist = [];

for k = 1:length(theta1)

    th1 = theta1(k);
    th2 = theta2(k);

    % Joint positions
    x1 = a1*cos(th1);
    y1 = a1*sin(th1);

    x2 = x1 + a2*cos(th1 + th2);
    y2 = y1 + a2*sin(th1 + th2);

    % Update links
    set(h_link1,'XData',[0 x1],'YData',[0 y1]);
    set(h_link2,'XData',[x1 x2],'YData',[y1 y2]);

    % Trajectory
    xe_hist(end+1) = x2;
    ye_hist(end+1) = y2;
    set(h_traj,'XData',xe_hist,'YData',ye_hist);

    drawnow;
    pause(Ts*0.8);  % slow-down for visualization
end
end

function Yref = cartesian_waypoints(steps, waypoints, durations)
% waypoints: 2 x Np   [y; z]
% durations: 1 x Np   number of steps for each waypoint

Np = size(waypoints,2);

Yref = zeros(2,steps);
pos = 1;

for i = 1:Np
    dur = durations(i);
    if pos + dur - 1 > steps
        dur = steps - pos + 1;
    end

    Yref(:,pos:pos+dur-1) = repmat(waypoints(:,i),1,dur);
    pos = pos + dur;

    if pos > steps
        break;
    end
end
end
