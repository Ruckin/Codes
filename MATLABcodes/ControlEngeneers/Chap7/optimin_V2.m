%% MATLBA CODE FOR OPTIMIZATION OF CONTROLLER PARAMETERS:
%  FREDERICO CASARA ANTONIAZZI - 23/02/2026
%  BASED ON MATLAB FOR CONTROL ENGINEERS - OGATA
%  CHAPTER 7

%  VERSION 2: SIMPLE EXAMPLE OF CODING AN OPEN LOOP SYSTEM MODEL AND A PID
%  CONTROLLER, FINDING A CONFIGURATION TO GARANTEE OVERSHOOT REQUIREMENT 
%  IS MET AND PLOTTING THE SOLUTION

clc; clear; close all; format short;

t = 0:.01:8;

for K = [3:.2:5 NaN]      % Starts the outer loop to vary the K value
    for a = [.1:.1:3 NaN] % Starts the inner loop to vary the a values

        num1 = K*[1 2*a a^2];
        den1 = [1 0];

        tf_c = tf(num1, den1);

        num2 = 4;
        den2 = [1 6 8 4];

        tf_p = tf(num2, den2);

        tf_mf = feedback(tf_c*tf_p, 1);

        y = step(tf_mf, t);

        m = max(y);

       if m < 1.15 && m > 1.1 % OVERSHOOT BETWEEN 10%~15%
           plot(t, y);
           grid on;
           title('Unit-Step Response');
           xlabel('Time / [sec]');
           ylabel('Output / [u.m.]');
           legend('y(t)');
           text(2.20, .73, ['K = ', num2str(K)]);
           text(2.20, .63, ['a = ', num2str(a)]);
           solution = [round(K, 3) round(a, 3) round(m, 3)];
           break;
        end
    end
    if m < 1.15 && m > 1.1
        break;
    end
end

disp('##############################');
disp(['The Optimal Solution is: ', num2str(solution)]);
disp('##############################');
