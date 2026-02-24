%% MATLBA CODE FOR OPTIMIZATION OF CONTROLLER PARAMETERS:
%  FREDERICO CASARA ANTONIAZZI - 23/02/2026
%  BASED ON MATLAB FOR CONTROL ENGINEERS - OGATA
%  CHAPTER 7

%  VERSION 4: SIMPLE EXAMPLE OF CODING AN CLOSED LOOP SYSTEM MODEL AND
%  FINDING A CONFIGURATION TO GARANTEE OVERSHOOT AND SETTLING TIME 
%  REQUIREMENTS ARE MET

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
    colors = lines(N);

    [~, idx_m]  = sort(solution(:,3)); % overshoot
    [~, idx_ts] = sort(solution(:,4)); % settling time

    figure;

    % ==========================================
    % SUBPLOT 1 - Overshoot
    % ==========================================
    subplot(1,2,1); hold on; grid on;
    title('Ordered by Overshoot');
    xlabel('Time (s)');
    ylabel('Amplitude');

    for rank = 1:N
        
        solIndex = idx_m(rank);
        K = solution(solIndex,1);
        a = solution(solIndex,2);
        [~, bestMpPosition] = min(solution(:, 3));

        num = [4*K 8*K*a 4*K*a^2];
        den = [1 6 8+4*K 4+8*K*a 4*K*a^2];
        y = step(num, den, t);

        if rank == bestMpPosition
            % Melhor overshoot destacado
            plot(t,y,'k','LineWidth',3);
            
            text(4,1.12,...
                sprintf('BEST Mp\nK=%.2f  a=%.2f',solution(bestMpPosition, 1),solution(bestMpPosition, 2)),...
                'FontWeight','bold',...
                'BackgroundColor','w');
        else
            plot(t,y,'Color',colors(rank,:),'LineWidth',1.2);
        end
    end

    legendStrings = compose('K=%.2f, a=%.2f', solution(:,1), solution(:,2));
    legend(legendStrings,'Location','best')

    % ==========================================
    % SUBPLOT 2 - Settling Time
    % ==========================================
    subplot(1,2,2); hold on; grid on;
    title('Ordered by Settling Time');
    xlabel('Time (s)');
    ylabel('Amplitude');

    for rank = 1:N
        
        solIndex = idx_ts(rank);
        K = solution(solIndex,1);
        a = solution(solIndex,2);
        [~, bestTsPosition] = min(solution(:, 4));

        num = [4*K 8*K*a 4*K*a^2];
        den = [1 6 8+4*K 4+8*K*a 4*K*a^2];
        y = step(num, den, t);

        if rank == bestTsPosition
            % Melhor Ts destacado
            plot(t,y,'k','LineWidth',3);
            text(4,1.12,...
                sprintf('BEST Ts\nK=%.2f  a=%.2f',solution(bestTsPosition, 1),solution(bestTsPosition, 2)),...
                'FontWeight','bold',...
                'BackgroundColor','w');        else
            plot(t,y,'Color',colors(rank,:),'LineWidth',1.2);
        end
    end

    legendStrings = compose('K=%.2f, a=%.2f', solution(:,1), solution(:,2));
    legend(legendStrings,'Location','best')

end
