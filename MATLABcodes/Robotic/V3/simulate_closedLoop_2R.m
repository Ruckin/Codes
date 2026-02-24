clc; clear; close all;

robot = robot2R_params();

% Simulation
Ts = 0.001;
T  = 3;
t  = 0:Ts:T;

% Gains
Kp = diag([100 80]);
Kd = diag([20 15]);

% Desired trajectory
qd  = [ 0.5*sin(2*pi*t);
        0.3*cos(2*pi*t) ];

dqd  = gradient(qd, Ts);
ddqd = gradient(dqd, Ts);

% Initial state
q  = [0; 0];
dq = [0; 0];

% Logs
Q  = zeros(2,length(t));
DQ = zeros(2,length(t));
TAU = zeros(2,length(t));

for k = 1:length(t)

    % Desired values
    qdk  = qd(:,k);
    dqdk = dqd(:,k);
    ddqdk = ddqd(:,k);

    % Controller
    tau = computedTorqueControl_2R( ...
        q, dq, qdk, dqdk, ddqdk, Kp, Kd, robot);

    % Plant
    ddq = forwardDynamics_2R(q, dq, tau, robot);

    % Integrate
    dq = dq + Ts*ddq;
    q  = q  + Ts*dq;

    % Log
    Q(:,k) = q;
    DQ(:,k) = dq;
    TAU(:,k) = tau;
end

% Plot tracking
figure;
subplot(2,1,1)
plot(t, qd(1,:), 'k--', t, Q(1,:), 'LineWidth',1.5);
legend('Trajectory', 'State');
ylabel('q1 [rad]'); grid on;

subplot(2,1,2)
plot(t, qd(2,:), 'k--', t, Q(2,:), 'LineWidth',1.5);
legend('Trajectory', 'State');
ylabel('q2 [rad]'); xlabel('Time [s]'); grid on;

figure;
subplot(2,1,1)
plot(t, TAU(1,:), 'LineWidth',1.5); grid on;
ylabel('\tau_1');

subplot(2,1,2)
plot(t, TAU(2,:), 'LineWidth',1.5); grid on;
ylabel('\tau_2'); xlabel('Time [s]');
