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
    plotSolutionsFull(solution, t);
else
    disp('No feasible solutions found.');
end

function plotSolutionsFull(solution, t)

    N = size(solution,1);
    colors = lines(N);

    % Ordenações
    [~, idx_m]  = sort(solution(:,3)); % menor overshoot
    [~, idx_ts] = sort(solution(:,4)); % menor settling time

    bestMpIndex = idx_m(1);
    bestTsIndex = idx_ts(1);

    figure('Position',[100 100 1400 600]);

    % =========================================================
    % 1️⃣ STEP RESPONSE – Overshoot
    % =========================================================
    subplot(2,2,1); hold on; grid on;
    title('Step Response - Ordered by Overshoot');
    xlabel('Time (s)');
    ylabel('Amplitude');

    for rank = 1:N

        solIndex = idx_m(rank);
        K = solution(solIndex,1);
        a = solution(solIndex,2);

        num = [4*K 8*K*a 4*K*a^2];
        den = [1 6 8+4*K 4+8*K*a 4*K*a^2];
        y = step(num, den, t);

        if solIndex == bestMpIndex
            plot(t,y,'k','LineWidth',3);
            [ymax, idxPeak] = max(y);
            plot(t(idxPeak),ymax,'ro','MarkerFaceColor','r');
            text(3.5, .55,...
                sprintf('  BEST Mp\n  K=%.2f a=%.2f',K,a),...
                'FontWeight','bold');
        else
            plot(t,y,'Color',colors(rank,:),'LineWidth',1.2);
        end
    end


    % =========================================================
    % 2️⃣ STEP RESPONSE – Settling Time
    % =========================================================
    subplot(2,2,2); hold on; grid on;
    title('Step Response - Ordered by Settling Time');
    xlabel('Time (s)');
    ylabel('Amplitude');

    for rank = 1:N

        solIndex = idx_ts(rank);
        K = solution(solIndex,1);
        a = solution(solIndex,2);

        num = [4*K 8*K*a 4*K*a^2];
        den = [1 6 8+4*K 4+8*K*a 4*K*a^2];
        y = step(num, den, t);

        if solIndex == bestTsIndex
            plot(t,y,'k','LineWidth',3);
            text(3.5, .55,...
                sprintf('BEST Ts\nK=%.2f a=%.2f',K,a),...
                'FontWeight','bold');
        else
            plot(t,y,'Color',colors(rank,:),'LineWidth',1.2);
        end
    end


    % =========================================================
    % 3️⃣ TRADE-OFF Mp × Ts
    % =========================================================
    subplot(2,2,3); hold on; grid on;
    title('Trade-off: Overshoot vs Settling Time');
    xlabel('Overshoot (Mp)');
    ylabel('Settling Time (Ts)');

    scatter(solution(:,3),solution(:,4),80,'filled');

    % Destacar melhores
    scatter(solution(bestMpIndex,3),solution(bestMpIndex,4),...
            150,'r','filled');
    scatter(solution(bestTsIndex,3),solution(bestTsIndex,4),...
            150,'k','filled');

    legend('Solutions','Best Mp','Best Ts','Location','best');


    % =========================================================
    % 4️⃣ PARALLEL COORDINATES (Normalizado)
    % =========================================================
    subplot(2,2,4);
    title('Parallel Coordinates (Normalized)');
    
    % Normalização 0-1
    dataNorm = normalize(solution,'range');

    parallelcoords(dataNorm,...
        'Labels',{'K','a','Mp','Ts'},...
        'LineWidth',1.2);

end
