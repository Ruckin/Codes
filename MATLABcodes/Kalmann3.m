clc;
clear;
close all;

format short;

%% DEFINIÇÕES DA SIMULAÇÃO
tFinal = 5;
tStep = 0.025;
timeSimulation = 0:tStep:tFinal;
numSteps = length(timeSimulation);

%% DEFINIÇÃO DO SISTEMA DINÂMICO CONTÍNUO
Ac = [0 1 0 0;
     -100 -20 100 20;
      0 0 0 1;
     66.667 13.334 -166.667 -33.334];

Bc = [0 1 0 0]';          % 4x1
Cc = [1 0 0 0];           % 1x4
Dc = [0 0 0 0];           % 1x4

%% DEFINIÇÃO DOS RUÍDOS
Vc = eye(4);              % 4x4: ruído de processo
Fc = 1;                   % ruído de medição
M = 0.3 * eye(4);         % variância do ruído de processo
N = 0.3;                  % variância do ruído de medição

%% SISTEMA COM RUÍDO (CONTÍNUO)
sysC = ss(Ac, [Bc Vc], Cc, [Dc Fc]);

%% DISCRETIZAÇÃO
sysD = c2d(sysC, tStep, 'zoh');

%% ENTRADAS E RUÍDOS
u = ones(numSteps, 1);                      % entrada de controle
v = sqrt(0.3) * randn(numSteps, 4);         % ruído de processo com 4 colunas
n = sqrt(0.3) * randn(numSteps, 1);         % ruído de medição
input = [u v];                              % matriz numSteps × 5

%% CONDIÇÕES INICIAIS
x0 = [-1; 0; 0; 0];

%% SIMULAÇÕES DO SISTEMA
[yc, ~, ~] = lsim(sysC, input, timeSimulation, x0);   % contínuo
ync = yc + Fc * n;                                    % com ruído medição

[yd, ~, ~] = lsim(sysD, input, timeSimulation, x0);   % discreto
ynd = yd + Fc * n;                                    % com ruído medição

%% FILTRO DE KALMAN DISCRETO
Ad = get(sysD, 'A');
Bd = get(sysD, 'B');
Cd = get(sysD, 'C');

Vd = eye(4);  % ruído de processo

xHatkm1 = x0;
Pkm1 = 20 * eye(4);
xKalmanVec = zeros(numSteps, 4);
KalmanGains = zeros(4, numSteps);

for k = 1:numSteps
    % Predição
    xHatkm = Ad * xHatkm1 + Bd(:,1) * u(k);
    Pkm = Ad * Pkm1 * Ad' + Vd * M * Vd';

    % Correção
    Kk = Pkm * Cd' / (Cd * Pkm * Cd' + N);
    xHatkM = xHatkm + Kk * (ync(k) - Cd * xHatkm);

    % Atualização
    KalmanGains(:, k) = Kk;
    PkM = (eye(4) - Kk * Cd) * Pkm;
    xHatkm1 = xHatkM;
    Pkm1 = PkM;
    xKalmanVec(k, :) = xHatkM';
end

%% PLOTAGENS

% Resposta do sistema
figure;
plot(timeSimulation, ync, 'k', 'LineWidth', 1.5); hold on;
plot(timeSimulation, ynd, 'r', 'LineWidth', 1.5);
plot(timeSimulation, xKalmanVec(:, 1), 'b', 'LineWidth', 1.5);
legend('y com ruído (cont.)', 'y com ruído (disc.)', 'Estimativa Kalman');
xlabel('Tempo [s]');
ylabel('Saída');
title('Resposta com Filtro de Kalman');
grid on;

% Ganhos de Kalman
figure;
for i = 1:4
    subplot(2, 2, i);
    plot(timeSimulation, KalmanGains(i, :), 'LineWidth', 1.5);
    xlabel('Tempo [s]');
    ylabel(sprintf('K_{%d}', i));
    title(sprintf('Ganho K_{%d}', i));
    grid on;
end
