%% MATLBA CODE FOR OPTIMIZATION OF CONTROLLER PARAMETERS:
%  FREDERICO CASARA ANTONIAZZI - 23/02/2026
%  BASED ON MATLAB FOR CONTROL ENGINEERS - OGATA
%  CHAPTER 7

%  VERSION 1: SIMPLE EXAMPLE OF CODING AN CLOSED LOOP SYSTEM MODEL AND
%  FINDING A CONFIGURATION TO GARANTEE OVERSHOOT REQUIREMENT IS MET

clc; clear; close all; format short;

t = 0:.01:8;

for K = [3:.2:5 NaN]
    for a = [.1:.1:3 NaN]

        num = [4*K 8*K*a 4*K*a^2];
        den = [1 6 8+4*K 4+8*K*a 4*K*a^2];

        y = step(num, den, t);

        m = max(y);

        % if m < 2.60 && m > 2.55
       if m < 1.20 && m > 1.15
            break;
        end
    end
    % if m < 2.60 && m > 2.55
    if m < 1.20 && m > 1.15
       break;
    end
end

solution = [round(K, 3) round(a, 3) round(m, 3)];

disp('##############################');
disp(['The Optimal Solution is: ', num2str(solution)]);
disp('##############################');
