clear; clc; close all;

%% Symbolic setup
syms x
% f = exp(x);
f = sin(x);

% Padé orders
N = 3;
M = 3;

% Expansion points (degrees -> radians)
x0_deg = [0 45 80 135 190 225 280 315 350];
x0 = deg2rad(x0_deg);
K = length(x0);

% Store Padé models
P = cell(K,1);
Q = cell(K,1);
pade_fun = cell(K,1);

%% Build local Padé models
for k = 1:K
    [P{k}, Q{k}] = pade_symbolic(f, x, x0(k), N, M);
    pade_fun{k} = matlabFunction(P{k}/Q{k}, 'Vars', x);
end

%% Evaluation grid
xv = linspace(0, 2*pi, 20000);
f_true = sin(xv);

%% Single Padé at x0 = 0
f_pade0 = pade_fun{1}(xv);

%% Gain-scheduled Padé
alpha = 5;   % locality parameter
f_sched = zeros(size(xv));

for i = 1:length(xv)

    % Scheduling weights
    w = zeros(K,1);
    for k = 1:K
        w(k) = exp(-alpha*(xv(i)-x0(k))^2);
    end
    w = w/sum(w);

    % Blend Padé outputs
    for k = 1:K
        f_sched(i) = f_sched(i) + w(k)*pade_fun{k}(xv(i));
    end
end

%% Errors
err_pade0 = abs(f_true - f_pade0);
err_sched = abs(f_true - f_sched);

%% Plots
figure;
plot(xv, f_true, 'k', 'LineWidth', 2); hold on;
plot(xv, f_pade0, '--r', 'LineWidth', 2);
plot(xv, f_sched, ':b', 'LineWidth', 2);
grid on;
xlabel('x (rad)');
ylabel('Value');
legend('sin(x)', 'Single Padé at x_0 = 0', 'Gain-scheduled Padé', 'Location', 'best');
title('Padé Approximation of sin(x)');

figure;
semilogy(xv, err_pade0, 'r', 'LineWidth', 2); hold on;
semilogy(xv, err_sched, 'b', 'LineWidth', 2);
grid on;
xlabel('x (rad)');
ylabel('Absolute error');
legend('Single Padé error', 'Gain-scheduled Padé error', 'Location', 'best');
title('Approximation Error Comparison');

function [P,Q,A,B] = pade_symbolic(f, x, x0, N, M)

    % Taylor coefficients
    c = sym(zeros(1, N+M+1));
    for k = 0:N+M
        c(k+1) = subs(diff(f, x, k), x, x0)/factorial(k);
    end

    % Build linear system for denominator coefficients
    Cmat = sym(zeros(M,M));
    rhs  = sym(zeros(M,1));

    for i = 1:M
        rhs(i) = -c(N+i+1);
        for j = 1:M
            Cmat(i,j) = c(N+i-j+1);
        end
    end

    % Solve linear system (robust)
    if rank(Cmat) < M
        error('Padé degeneracy at x0 = %f rad: reduce order or change center.', double(x0));
    end

    B = (Cmat \ rhs).';   % row vector

    % Numerator coefficients
    A = sym(zeros(1,N+1));
    for k = 0:N
        s = 0;
        for j = 1:min(k,M)
            s = s + B(j)*c(k-j+1);
        end
        A(k+1) = c(k+1) + s;
    end

    % Local variable
    xi = x - x0;

    % Construct Padé polynomials
    P = sum(A .* xi.^(0:N));
    Q = 1 + sum(B .* xi.^(1:M));

end
