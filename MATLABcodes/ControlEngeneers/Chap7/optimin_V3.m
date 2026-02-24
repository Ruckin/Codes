%% MATLBA CODE FOR OPTIMIZATION OF CONTROLLER PARAMETERS:
%  FREDERICO CASARA ANTONIAZZI - 23/02/2026
%  BASED ON MATLAB FOR CONTROL ENGINEERS - OGATA
%  CHAPTER 7

%  VERSION 3: SIMPLE EXAMPLE OF CODING AN CLOSED LOOP SYSTEM MODEL AND
%  FINDING A CONFIGURATION TO GARANTEE OVERSHOOT AND SETTLING TIME 
%  REQUIREMENTS ARE MET, AND PLOTTING THE RESULT FOR DECISION MAKING

clc; clear; close all; format short;

t = 0:.01:8;
k = 0;
precision = 3;
numberOfRequirements = 2;

for K = 3:.2:5
    for a = .1:.1:3

        % SYSTEM
        num = [4*K 8*K*a 4*K*a^2];
        den = [1 6 8+4*K 4+8*K*a 4*K*a^2];

        y = step(num, den, t);

        s = 801;

        while y(s) > .98 && y(s) < 1.02

            s = s - 1;

        end

        ts = (s - 1)*.01; % SETTLING TIME OF THE SYSTEM
        m = max(y);       % OVERSHOOT OF THE SYSTEM

       if m < 1.15 && m > 1.1
           if ts < 3
               k = k + 1;
               solution(k, :) = [round(K, precision) round(a, precision) round(m, precision) round(ts, precision)];
           end
        end
    end
end

disp('##############################');
disp('Feasible Controller Configurations:');
fprintf('   K        a        Mp       Ts\n');
fprintf('--------------------------------------\n');

for i = 1:size(solution,1)
    fprintf('%6.3f   %6.3f   %6.3f   %6.3f\n', ...
        solution(i,1), solution(i,2), solution(i,3), solution(i,4));
end

disp('##############################');

%% PLOTANDO OS RESULTADOS DE FORMA COMPARATIVA:

if exist('solution','var')
    plotSolutions(solution, t);
else
    disp('No feasible solutions found.');
end

function plotSolutions(solution, t)

    N = size(solution,1);
    colors = lines(N);   % Colormap consistente
    
    % Ordenações
    [~, idx_m]  = sort(solution(:,3)); % Ordena por overshoot
    [~, idx_ts] = sort(solution(:,4)); % Ordena por settling time

    figure;

    % -----------------------------
    % SUBPLOT 1 - Ordenado por Overshoot
    % -----------------------------
    subplot(1,2,1); hold on; grid on;
    title('Step Response - Ordered by Overshoot');
    xlabel('Time / [s]');
    ylabel('Amplitude');

    for i = 1:N
        
        K = solution(idx_m(i),1);
        a = solution(idx_m(i),2);
        
        num = [4*K 8*K*a 4*K*a^2];
        den = [1 6 8+4*K 4+8*K*a 4*K*a^2];

        y = step(num, den, t);

        % mantém cor baseada na posição original
        originalIndex = idx_m(i);
        plot(t, y, 'Color', colors(originalIndex,:), 'LineWidth', 1.5);

    end

    legendStrings = compose('K=%.2f, a=%.2f', solution(:,1), solution(:,2));
    legend(legendStrings,'Location','best');


    % -----------------------------
    % SUBPLOT 2 - Ordenado por Settling Time
    % -----------------------------
    subplot(1,2,2); hold on; grid on;
    title('Step Response - Ordered by Settling Time');
    xlabel('Time / [s]');
    ylabel('Amplitude');

    for i = 1:N
        
        K = solution(idx_ts(i),1);
        a = solution(idx_ts(i),2);
        
        num = [4*K 8*K*a 4*K*a^2];
        den = [1 6 8+4*K 4+8*K*a 4*K*a^2];

        y = step(num, den, t);

        originalIndex = idx_ts(i);
        plot(t, y, 'Color', colors(originalIndex,:), 'LineWidth', 1.5);

    end

    legend(legendStrings,'Location','best');

end
