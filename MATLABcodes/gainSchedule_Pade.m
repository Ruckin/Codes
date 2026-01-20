clear; clc; close all;

%% Symbolic setup
syms x
f = sin(x);

% Expansion points (radians)
x0_deg = [0 60 110 155];
x0 = deg2rad(x0_deg);

N = 3;  % Padé numerator order
M = 3;  % Padé denominator order
K = length(x0);

P = cell(K,1);
Q = cell(K,1);

%% Compute local Padé models (offline, exact)
for k = 1:K
    % Taylor expansion around x0(k)
    T = taylor(f, x, 'ExpansionPoint', x0(k), 'Order', N+M+1);
    [P{k}, Q{k}] = pade(T, x, 'Order', [N M]);
end

%% Numerical evaluation grid
xv = linspace(0, 2*pi, 2000);

f_true = sin(xv);

% Padé at x0 = 0 only
pade_0_fun = matlabFunction(P{1}/Q{1}, 'Vars', x);
f_pade_0 = pade_0_fun(xv);

%% Gain-scheduled Padé approximation
alpha = 4;  % controls locality of weights
f_sched = zeros(size(xv));

for i = 1:length(xv)
    % Compute weights
    w = zeros(K,1);
    for k = 1:K
        w(k) = exp(-alpha*(xv(i)-x0(k))^2);
    end
    w = w/sum(w);  % normalize
    
    % Blend Padé models
    for k = 1:K
        pade_fun_k = matlabFunction(P{k}/Q{k}, 'Vars', x);
        f_sched(i) = f_sched(i) + w(k)*pade_fun_k(xv(i));
    end
end

%% Errors
err_pade0 = abs(f_true - f_pade_0);
err_sched = abs(f_true - f_sched);

%% Plots
figure;
plot(xv, f_true, 'k', 'LineWidth', 2); hold on;
plot(xv, f_pade_0, '--r', 'LineWidth', 2);
plot(xv, f_sched, ':b', 'LineWidth', 2);
grid on;
xlabel('x (rad)');
ylabel('Function value');
legend('sin(x)', 'Single Padé at x_0 = 0', 'Gain-scheduled Padé', ...
       'Location', 'Best');
title('Padé Approximation of sin(x)');

figure;
semilogy(xv, err_pade0, 'r', 'LineWidth', 2); hold on;
semilogy(xv, err_sched, 'b', 'LineWidth', 2);
grid on;
xlabel('x (rad)');
ylabel('Absolute error (log scale)');
legend('Error: Padé at x_0 = 0', 'Error: Gain-scheduled Padé', ...
       'Location', 'Best');
title('Approximation Error Comparison');
