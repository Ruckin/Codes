%% NONLINEAR MPC CONTROLLER DESIGN EXAMPLE
%  FREDERICO CASARA ANTONIAZZI - 05/02/2026
%  MIMO Nonlinear MPC for a 2-DOF robotic arm

clc; clear; close all; format short;

%% ====================== MODEL DATA ======================

g = 9.81;

% ARM LENGTHS
a1 = .2;  r1 = .01;
a2 = .15; r2 = .01;

% JOINT GEOMETRY
lj2 = .06; rj2 = .02;

% END EFFECTOR
le1 = .05; re1 = .012;
le2 = .04; re2 = .002;

% MATERIAL
rho_steel = 7850;

% MASSES
ml1 = rho_steel*pi*r1^2*a1;
ml2 = rho_steel*pi*r2^2*a2;
mj2 = rho_steel*pi*rj2^2*lj2;
me1 = rho_steel*pi*re1^2*le1;
me2 = rho_steel*pi*re2^2*le2;

m1 = ml1 + mj2;
m2 = ml2 + me1 + me2;

% CENTERS OF MASS
ac1 = (ml1*a1/2 + mj2*a1)/m1;
ac2 = (ml2*a2/2 + me1*a2 + me2*a2)/m2;

% INERTIAS
I1 = m1*a1^2/12;
I2 = m2*a2^2/12;

%% ====================== MPC PARAMETERS ======================

Ts = 0.02;          % sampling time
N  = 15;            % prediction horizon

Qy = diag([200 200]);     % output tracking
Ru = diag([0.01 0.01]);   % control effort

u_min = [-3; -3];
u_max = [ 3;  3];

%% ====================== REFERENCE ======================

y_ref = [0.25; 0.15];   % desired end-effector position

%% ====================== INITIAL STATE ======================

x = [0.1; 0; 0.1; 0];

Tf = 5;
steps = round(Tf/Ts);

X_log = zeros(4,steps);
U_log = zeros(2,steps);
Y_log = zeros(2,steps);

%% ====================== SIMULATION LOOP ======================

for k = 1:steps

    % Solve NMPC
    u0 = zeros(2*N,1);

    options = optimoptions('fmincon','Display','none',...
        'Algorithm','sqp','MaxIterations',50);

    u_opt = fmincon(@(u) costNMPC(u,x,y_ref,N,Ts,Qy,Ru,...
        a1,a2,ac1,ac2,m1,m2,I1,I2,g),...
        u0,[],[],[],[],...
        repmat(u_min,N,1),repmat(u_max,N,1),[],options);

    u = u_opt(1:2);

    % System propagation
    x = robotStep(x,u,Ts,a1,a2,ac1,ac2,m1,m2,I1,I2,g);

    % Output
    y = endEffector(x,a1,a2);

    % Logging
    X_log(:,k) = x;
    U_log(:,k) = u;
    Y_log(:,k) = y;
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

theta1 = X_log(1,:);
theta2 = X_log(3,:);

animate_robot_arm(theta1, theta2, a1, a2, Ts);

%% ====================== FUNCTIONS ======================

function J = costNMPC(u,x0,yref,N,Ts,Qy,Ru,...
    a1,a2,ac1,ac2,m1,m2,I1,I2,g)

x = x0;
J = 0;

for i = 1:N
    ui = u(2*i-1:2*i);
    x  = robotStep(x,ui,Ts,a1,a2,ac1,ac2,m1,m2,I1,I2,g);
    y  = endEffector(x,a1,a2);

    e = y - yref;
    J = J + e'*Qy*e + ui'*Ru*ui;
end
end

function xnext = robotStep(x,u,Ts,a1,a2,ac1,ac2,m1,m2,I1,I2,g)

th1 = x(1); dth1 = x(2);
th2 = x(3); dth2 = x(4);

C2 = cos(th2); S2 = sin(th2);

M = [m1*ac1^2 + m2*(a1^2 + ac2^2 + 2*a1*ac2*C2) + I1 + I2,...
     m2*(ac2^2 + a1*ac2*C2) + I2;
     m2*(ac2^2 + a1*ac2*C2) + I2,...
     m2*ac2^2 + I2];

C = [-2*m2*a1*ac2*S2*dth2, -m2*a1*ac2*S2*dth2;
      m2*a1*ac2*S2*dth1,  0];

G = [m1*g*ac1*cos(th1) + m2*g*(a1*cos(th1) + ac2*cos(th1+th2));
     m2*g*ac2*cos(th1+th2)];

ddth = M\(u - C*[dth1; dth2] - G);

xnext = x + Ts*[dth1; ddth(1); dth2; ddth(2)];
end

function y = endEffector(x,a1,a2)
th1 = x(1);
th2 = x(3);
y = [a1*cos(th1) + a2*cos(th1+th2);
     a1*sin(th1) + a2*sin(th1+th2)];
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
