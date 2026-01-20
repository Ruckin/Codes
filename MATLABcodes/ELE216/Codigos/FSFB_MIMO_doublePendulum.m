%% REALIMENTACAO DE ESTADOS PARA UM PENDULO DUPLO:
%  FREDERICO CASARA ANTONIAZZI - 17/09/2024
%
%  CODIGO DESENVOLVIDO PARA O PROJETO DE UM CONTROLADOR POR REALIMENTACAO
%  DE ESTADOS APLICADO A UM MODELO LINEAR DE PENDULO DUPLO INVERTIDO,
%  UTILIZANDO O METODO CICLICO E O METODO DE MOORE, AMBOS NO CASO
%  MULTIVARIAVEL.
%
%  MATERIA ELE-216 CONTROLE MULTIVARIAVEL.
%  PROFESSOR: JOAO MANOEL

%% DEFINICOES INICIAIS:

clc;
clear;
close all;

format short;

%% VARIAVEIS GLOBAIS:

syms s;           % VARIAVEL PARA DETERMINAR OS POLINOMIOS
n = 1000;         % NUMERO DE PONTOS DE SIMULACAO
t0 = 0;           % TEMPO INICIAL DA SIMULACAO
tfinal = 10;      % TEMPO FINAL DA SIMULACAO
dt = tfinal/n;    % STEP USADO NA SIMULACAO
t = t0:dt:tfinal; % VETOR DE TEMPO

%% MODELO DO PENDULO DUPLO: LINEARIZADO PARA O EQUILIBRIO INSTAVEL

A=[0 1 0 0; 9.8 0 -9.8 0; 0 0 0 1; -9.8 0 2.94 0];
B=[0 0; 1 -2; 0 0; -2 5];
C=[1 0 0 0; 0 0 1 0];

% TESTANDO ESTABILIDADE:
lamba_MA = eig(A);

estavel = real(lamba_MA) < 0;

if sum(estavel) ~= size(A)
    disp(lamba_MA);
    disp("SISTEMA EM MALHA ABERTA INSTAVEL!");

else
    disp(lamba_MA);
    disp("SISTEMA EM MALHA ABERTA ESTAVEL!");

end

% TESTANDO CONTROLABILIDADE:

controlavel = rank(ctrb(A, B));

if controlavel == size(A)
    disp("SISTEMA CONTROLAVEL!");

else
    disp("SISTEMA NAO CONTROLAVEL!");

end

%% POLOS DE MALHA FECHADA:

autovalores_MF = [-1 -2 -3 -4];

%% USANDO O METODO CICLICO:

% FIRST STEP: VERIFICATION OF THE A MATRIX IN OPEN-LOOP HAS ALL DISTINCT
% EIGAINVALUES
EIG_values_MA = eig(A);

% ESCOLHER V:
V_ciclico = [1 -3]';

% CALCULANDO b:
b = B*V_ciclico;

% TESTANDO CONTROLABILIDADE DO SISTEMA MONO:
controlavel_casoMono_ciclico = rank(ctrb(A, b));

if controlavel_casoMono_ciclico == size(A)
    disp("SISTEMA CONTROLAVEL!");
    disp("CONTROLE CICLICO TESTE MONO");

else
    disp("SISTEMA NAO CONTROLAVEL!");
    disp("CONTROLE CICLICO TESTE MONO");

end

% AGORA REALIZAR O PROJETO COMO SISTEMA MONOVARIAVEL: APLICAR TRANSFORMACAO
% DE SIMILARIDADE:
coeficientes_malhaAberta = det(s*eye(size(A)) - A);
coeficientes = [0 -637/50 0 -16807/250];

Q = ctrb(A, b)*[1 coeficientes(1, 1:3); 0 1 coeficientes(1, 1:2); 0 0 1 coeficientes(1, 1); 0 0 0 1];

% CALCULANDO O SISTEMA TRANSFORMADO:
Abar = Q\A*Q;
bbar = Q\b;

% CALCULANDO O GANHO NO DOMINIO TRANSFORMADO:
coeficientes_malhaFechada = poly(autovalores_MF);

% KBAR: FOI FEITO PARA A + BK, APENAS TROCAR O SINAL CASO SE QUEIRA FAZER 
% A - BK.
Kbar_ciclico = coeficientes - coeficientes_malhaFechada(1, 2:end);

% K NO CASO MONO:
K_ciclico = Kbar_ciclico/Q;  % MONO
% K NO CASO MULTI:
K2 = V_ciclico*K_ciclico; % MULTIVARIAVEL

% TESTANDO A SOLUCAO:
eig(Abar + bbar*Kbar_ciclico); % MONO TRANSFORMADO
eig(A + b*K_ciclico);          % MONO NAO TRANSFORMADO
eig(A + B*K2);                 % MULTI NAO TRANSFORMADO

%% USANDO O METODO DE MOORE:

% ESCOLHENDO O W:
W_moore = [[1 2]' [-1 1]' [7 -2]' [0 5]'];

% CALCULANDO V:
v = zeros(size(A));

for i = 1:size(A)

    v(:, i) = (autovalores_MF(i)*eye(4) - A)\B*W_moore(:, i);

end
V_moore = v;

% CALCULANDO O GANHO:
K_moore = W_moore/V_moore;

% TESTANDO:
eig(A + B*K_moore);

%% TESTANDO OS CONTROLADORES DE REALIMENTACAO DE ESTADOS:

u = zeros(2, length(t)); % ENTRADA NULA

% CONDICOES INICIAIS:
x0 = [0 1 0 1]; % POSICOES ZERO, VELOCIDADES DE .01 RAD/S

% SIMULANDO O SISTEMA EM MALHA FECHADA USANDO O CONTROLADOR CALCULADO PELO
% METODO CICLICO:
sistema_ciclico = ss(A + B*K2, B, C, 0);

[~, t, x_ciclico] = lsim(sistema_ciclico, u, t, x0);

% SIMULANDO O SISTEMA EM MALHA FECHADA USANDO O CONTROLADOR CALCULADO PELO
% METODO DE MOORE:
sistema_moore = ss(A + B*K_moore, B, C, 0);

[~, ~, x_moore] = lsim(sistema_moore, u, t, x0);

% PLOTANDO OS RESULTADOS DE CADA ESTADO:
figure;
plot(t, x_ciclico(:, 1));
hold on;
plot(t, x_moore(:, 1), 'r');
grid on;
title("Unforced Response for Double Pendulum");
xlabel("Time [s]");
ylabel("Position First Rod");
legend("x_{1_{c}}", "x_{1_{m}}");

figure;
plot(t, x_ciclico(:, 2));
hold on;
plot(t, x_moore(:, 2), 'r');
grid on;
title("Unforced Response for Double Pendulum");
xlabel("Time [s]");
ylabel("Velocity First Rod");
legend("x_{2_{c}}", "x_{2_{m}}");

figure;
plot(t, x_ciclico(:, 3));
hold on;
plot(t, x_moore(:, 3), 'r');
grid on;
title("Unforced Response for Double Pendulum");
xlabel("Time [s]");
ylabel("Position Second Rod");
legend("x_{3_{c}}", "x_{3_{m}}");

figure;
plot(t, x_ciclico(:, 4));
hold on;
plot(t, x_moore(:, 4), 'r');
grid on;
title("Unforced Response for Double Pendulum");
xlabel("Time [s]");
ylabel("Velocity Second Rod");
legend("x_{4_{c}}", "x_{4_{m}}");
