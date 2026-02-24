%% NONLINEAR MPC CONTROLLER DESIGN EXAMPLE
%  FREDERICO CASARA ANTONIAZZI - 05/02/2026
%
%  MIMO Nonlinear MPC for a 2-DOF robotic arm
%  - Nonlinear dynamics
%  - Torque constraints
%  - State constraints (angles + velocities)
%  - End-effector tracking
%  - MAS-based reachability verification
%  - Terminal cost
%  - Continuous + step-by-step animation

clc; clear; 
close all; 
format short;

%% ====================== PHYSICAL PARAMETERS ======================

g = 9.81;

a1 = 0.20;      % link lengths
a2 = 0.15;

m1 = 2.0;       % masses
m2 = 1.5;

ac1 = a1/2;     % COM
ac2 = a2/2;

I1 = m1*a1^2/12;
I2 = m2*a2^2/12;

%% ====================== MPC PARAMETERS ======================

Ts = 0.02;
N  = 15;

Qy = diag([200 200]);      % tracking
Ru = diag([0.05 0.05]);    % effort
Qt = diag([500 500]);      % terminal cost

u_min = [-3; -3];
u_max = [ 3;  3];

%% ====================== STATE CONSTRAINTS ======================

limits.theta1 = [-pi/2, pi/2];
limits.theta2 = [-pi/2, pi/2];
limits.dtheta = [-4, 4];

%% ====================== INITIAL STATE ======================

x = [0.1; 0; 0.1; 0];   % [th1 dth1 th2 dth2]

Tf = 5;
steps = round(Tf/Ts);

X_log = zeros(4,steps);
U_log = zeros(2,steps);
Y_log = zeros(2,steps);

%% ====================== REFERENCE WAYPOINTS ======================

waypoints = [ ...
    0.15  0.22  0.25;
   -0.10  0.05  0.15];

durations = [80 80 120];

%% ====================== MAS REACHABILITY FILTER ======================

valid_wp = [];
for i = 1:size(waypoints,2)

    [okKin, q] = checkKinematicReachability(waypoints(:,i),a1,a2,limits);

    okDyn = false;
    if okKin
        okDyn = checkDynamicReachability(q,x,Ts,limits,...
            a1,a2,ac1,ac2,m1,m2,I1,I2,g,u_min,u_max);
    end

    if okKin && okDyn
        valid_wp = [valid_wp waypoints(:,i)];
    else
        warning('Waypoint %d rejected (outside MAS).',i);
    end
end

waypoints = valid_wp;
Yref_series = cartesian_waypoints(steps,waypoints,durations);

%% ====================== NMPC SIMULATION LOOP ======================

for k = 1:steps

    y_ref = Yref_series(:,k);
    u0 = zeros(2*N,1);

    options = optimoptions('fmincon',...
        'Algorithm','sqp',...
        'Display','none',...
        'MaxIterations',60);

    u_opt = fmincon( ...
        @(u) costNMPC(u,x,y_ref,N,Ts,Qy,Ru,Qt,...
            a1,a2,ac1,ac2,m1,m2,I1,I2,g), ...
        u0,[],[],[],[], ...
        repmat(u_min,N,1), ...
        repmat(u_max,N,1), ...
        @(u) stateConstraints(u,x,N,Ts,limits,...
            a1,a2,ac1,ac2,m1,m2,I1,I2,g), ...
        options);

    u = u_opt(1:2);
    x = robotStep(x,u,Ts,a1,a2,ac1,ac2,m1,m2,I1,I2,g);

    y = endEffector(x,a1,a2);

    X_log(:,k) = x;
    U_log(:,k) = u;
    Y_log(:,k) = y;
end

%% ====================== VERIFICATION ======================

e = vecnorm(Y_log - Yref_series,2,1);
tol = 0.01;

if min(e(end-10:end)) > tol
    warning('Terminal set not reached → outside MAS.');
else
    disp('Reference reached inside admissible set.');
end

%% ====================== PLOTS ======================

t = (0:steps-1)*Ts;

figure;
plot(Y_log(1,:),Y_log(2,:),'LineWidth',2); hold on;
plot(Yref_series(1,end),Yref_series(2,end),'rx','MarkerSize',12,'LineWidth',2);
grid on; axis equal;
xlabel('y (m)'); ylabel('z (m)');
title('End-Effector Trajectory');

figure;
subplot(2,1,1); plot(t,U_log(1,:)); grid on; ylabel('\tau_1');
subplot(2,1,2); plot(t,U_log(2,:)); grid on; ylabel('\tau_2'); xlabel('t (s)');

%% ====================== ANIMATIONS ======================

animate_robot_arm(X_log(1,:),X_log(3,:),a1,a2,Ts);
animate_robot_arm_stepwise(X_log(1,:),X_log(3,:),a1,a2);

%% ====================== HELPER FUNCTIONS ======================

function J = costNMPC(u,x0,yref,N,Ts,Qy,Ru,Qt,...
    a1,a2,ac1,ac2,m1,m2,I1,I2,g)

x = x0; J = 0;

for k = 1:N
    uk = u(2*k-1:2*k);
    x  = robotStep(x,uk,Ts,a1,a2,ac1,ac2,m1,m2,I1,I2,g);
    y  = endEffector(x,a1,a2);
    e  = y - yref;
    J  = J + e'*Qy*e + uk'*Ru*uk;
end

J = J + e'*Qt*e;  % terminal cost
end

function [c,ceq] = stateConstraints(u,x0,N,Ts,limits,...
    a1,a2,ac1,ac2,m1,m2,I1,I2,g)

x = x0; c = []; ceq = [];

for k = 1:N
    uk = u(2*k-1:2*k);
    x  = robotStep(x,uk,Ts,a1,a2,ac1,ac2,m1,m2,I1,I2,g);

    c = [c;
         x(1)-limits.theta1(2);
         limits.theta1(1)-x(1);
         x(3)-limits.theta2(2);
         limits.theta2(1)-x(3);
         x(2)-limits.dtheta(2);
         limits.dtheta(1)-x(2);
         x(4)-limits.dtheta(2);
         limits.dtheta(1)-x(4)];
end
end

function xnext = robotStep(x,u,Ts,a1,a2,ac1,ac2,m1,m2,I1,I2,g)

th1=x(1); d1=x(2); th2=x(3); d2=x(4);
C2=cos(th2); S2=sin(th2);

M=[m1*ac1^2+m2*(a1^2+ac2^2+2*a1*ac2*C2)+I1+I2,...
   m2*(ac2^2+a1*ac2*C2)+I2;
   m2*(ac2^2+a1*ac2*C2)+I2,...
   m2*ac2^2+I2];

C=[-2*m2*a1*ac2*S2*d2, -m2*a1*ac2*S2*d2;
    m2*a1*ac2*S2*d1, 0];

G=[m1*g*ac1*cos(th1)+m2*g*(a1*cos(th1)+ac2*cos(th1+th2));
   m2*g*ac2*cos(th1+th2)];

dd = M\(u - C*[d1;d2] - G);
xnext = x + Ts*[d1;dd(1);d2;dd(2)];
end

function y = endEffector(x,a1,a2)
y=[a1*cos(x(1))+a2*cos(x(1)+x(3));
   a1*sin(x(1))+a2*sin(x(1)+x(3))];
end

function [ok,q] = checkKinematicReachability(y,a1,a2,limits)
r2=y(1)^2+y(2)^2;
if r2>(a1+a2)^2 || r2<(a1-a2)^2, ok=false; q=[]; return; end
c2=(r2-a1^2-a2^2)/(2*a1*a2); s2=sqrt(max(0,1-c2^2));
th2=atan2(s2,c2);
th1=atan2(y(2),y(1))-atan2(a2*sin(th2),a1+a2*cos(th2));
q=[th1;th2];
ok= th1>=limits.theta1(1)&&th1<=limits.theta1(2)&&...
    th2>=limits.theta2(1)&&th2<=limits.theta2(2);
end

function ok = checkDynamicReachability(q,x0,Ts,limits,...
    a1,a2,ac1,ac2,m1,m2,I1,I2,g,u_min,u_max)

Kp=30; Kd=5;
u=Kp*(q-x0([1 3]))-Kd*x0([2 4]);
u=max(min(u,u_max),u_min);
x1=robotStep(x0,u,Ts,a1,a2,ac1,ac2,m1,m2,I1,I2,g);
ok=all(x1([2 4])>=limits.dtheta(1))&&all(x1([2 4])<=limits.dtheta(2));
end

function animate_robot_arm_stepwise(t1,t2,a1,a2)
figure; axis equal; grid on; hold on;
L=a1+a2+0.05; xlim([-L L]); ylim([-L L]);
h1=plot([0 0],[0 0],'r','LineWidth',3);
h2=plot([0 0],[0 0],'b','LineWidth',3);
k=1;
uicontrol('Style','pushbutton','String','NEXT',...
 'Position',[20 20 80 40],'Callback',@(~,~) step);
    function step
        if k>length(t1),return;end
        x1=a1*cos(t1(k)); y1=a1*sin(t1(k));
        x2=x1+a2*cos(t1(k)+t2(k));
        y2=y1+a2*sin(t1(k)+t2(k));
        set(h1,'XData',[0 x1],'YData',[0 y1]);
        set(h2,'XData',[x1 x2],'YData',[y1 y2]);
        k=k+1;
    end
end

function animate_robot_arm(t1,t2,a1,a2,Ts)
figure; axis equal; grid on; hold on;
L=a1+a2+0.05; xlim([-L L]); ylim([-L L]);
h1=plot([0 0],[0 0],'r','LineWidth',3);
h2=plot([0 0],[0 0],'b','LineWidth',3);
for k=1:length(t1)
    x1=a1*cos(t1(k)); y1=a1*sin(t1(k));
    x2=x1+a2*cos(t1(k)+t2(k));
    y2=y1+a2*sin(t1(k)+t2(k));
    set(h1,'XData',[0 x1],'YData',[0 y1]);
    set(h2,'XData',[x1 x2],'YData',[y1 y2]);
    drawnow; pause(Ts*0.8);
end
end

function Yref = cartesian_waypoints(steps,w,d)
Yref=zeros(2,steps); p=1;
for i=1:size(w,2)
    dur=min(d(i),steps-p+1);
    Yref(:,p:p+dur-1)=repmat(w(:,i),1,dur);
    p=p+dur; if p>steps,break;end
end
end
