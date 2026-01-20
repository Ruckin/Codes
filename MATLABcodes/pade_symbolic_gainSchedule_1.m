function [P,Q,A,B] = pade_symbolic(f, x, x0, N, M)

    % Local variable
    xi = x - x0;

    % Taylor expansion around x0
    T = taylor(f, x, 'ExpansionPoint', x0, 'Order', N+M+1);
    T = expand(subs(T, x, xi + x0));

    % Extract Taylor coefficients c_k
    c = sym(zeros(1, N+M+1));
    for k = 0:N+M
        c(k+1) = subs(diff(T, xi, k)/factorial(k), xi, 0);
    end

    % Unknown Padé coefficients
    B = sym('B', [1 M]);
    A = sym('A', [1 N+1]);

    % Equations for denominator
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

    % Solve for A
    for k = 0:N
        s = 0;
        for j = 1:min(k,M)
            s = s + solB.(sprintf('B%d',j))*c(k-j+1);
        end
        A(k+1) = c(k+1) + s;
    end

    % Construct Padé polynomials (global x)
    P = sum(A .* xi.^(0:N));
    Q = 1 + sum(arrayfun(@(k) solB.(sprintf('B%d',k))*xi^k, 1:M));

end