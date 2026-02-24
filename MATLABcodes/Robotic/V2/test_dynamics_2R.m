clc; clear; close all;

robot = robot2R_params();

% Time
Ts = 0.01;
T  = 2;
t  = 0:Ts:T;

% Desired joint trajectory
q1 = 0.4*sin(2*pi*t/T);
q2 = 0.3*cos(2*pi*t/T);

dq1  = gradient(q1, Ts);
dq2  = gradient(q2, Ts);

ddq1 = gradient(dq1, Ts);
ddq2 = gradient(dq2, Ts);

tau = zeros(2,length(t));

for k = 1:length(t)
    q   = [q1(k); q2(k)];
    dq  = [dq1(k); dq2(k)];
    ddq = [ddq1(k); ddq2(k)];

    tau(:,k) = inverseDynamics_2R(q, dq, ddq, robot);
end

% Plot torques
figure;
subplot(2,1,1)
plot(t, tau(1,:), 'LineWidth',1.5); grid on;
ylabel('\tau_1 [Nm]');

subplot(2,1,2)
plot(t, tau(2,:), 'LineWidth',1.5); grid on;
ylabel('\tau_2 [Nm]');
xlabel('Time [s]');
