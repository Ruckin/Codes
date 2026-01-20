clc; clear; close all; format short;

syms x
f = sin(x);

N = 3; 
M = 3;

[P,Q,A,B] = pade_symbolic(f, x, N, M);

disp('Numerator P(x):')
pretty(P)

disp('Denominator Q(x):')
pretty(Q)

plot_pade_approximation(f, P, Q, x, 0, 4, 5000);

function [P,Q,A,B] = pade_symbolic(f, x, N, M)

    % Taylor expansion up to N+M
    T = taylor(f, x, 'Order', N+M+1);
    T = expand(T);

    % Extract Taylor coefficients c_k
    c = sym(zeros(1, N+M+1));
    for k = 0:N+M
        c(k+1) = subs(diff(T, x, k)/factorial(k), x, 0);
    end

    % Unknown Padé coefficients
    B = sym('B', [1 M]);
    A = sym('A', [1 N+1]);

    % Build equations for B (higher-order terms = 0)
    eqB = sym(zeros(1, M));
    for i = 1:M
        s = 0;
        for j = 1:M
            if (N+i-j) >= 0
                s = s + B(j)*c(N+i-j+1);
            end
        end
        eqB(i) = c(N+i+1) + s;
    end

    % Solve for B
    solB = solve(eqB == 0, B);

    % Substitute B into A equations
    for k = 0:N
        s = 0;
        for j = 1:min(k,M)
            s = s + solB.(sprintf('B%d',j))*c(k-j+1);
        end
        A(k+1) = c(k+1) + s;
    end

    % Construct polynomials
    P = sum(A .* x.^(0:N));
    Q = 1 + sum(arrayfun(@(k) solB.(sprintf('B%d',k))*x^k, 1:M));

end

function plot_pade_approximation(f, P, Q, x, xmin, xmax, npts)

    % Convert symbolic expressions to numeric functions
    f_fun    = matlabFunction(f,    'Vars', x);
    pade_fun = matlabFunction(P/Q,  'Vars', x);

    % Evaluation grid
    xv = linspace(xmin, xmax, npts);

    % Evaluate functions
    f_val    = f_fun(xv);
    pade_val = pade_fun(xv);

    % Plot original vs Padé
    figure;
    plot(xv, f_val, 'k', 'LineWidth', 2); hold on;
    plot(xv, pade_val, '--r', 'LineWidth', 2);
    grid on;

    xlabel('x');
    ylabel('Function value');
    title('Original Function vs Padé Approximation');
    legend('Original function', 'Padé approximation', 'Location', 'Best');

    % Plot approximation error
    figure;
    plot(xv, abs(f_val - pade_val), 'b', 'LineWidth', 2);
    grid on;

    xlabel('x');
    ylabel('|f(x) - P(x)/Q(x)|');
    title('Padé Approximation Error');

end
