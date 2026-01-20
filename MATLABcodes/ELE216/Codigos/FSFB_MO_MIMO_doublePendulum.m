%% OBSERVADORES DE ORDEM PLENA PARA PENDULO DUPLO:
%  FREDERICO CASARA ANTONIAZZI - 17/09/2024
%
%  CODIGO DESENVOLVIDO PARA O PROJETO DE UM OBSERVADOR DE ESTADOS DE ORDEM
%  REDUZIDA PARA PERMITIR A REALIMENTACAO DE ESTADOS APLICADO A UM MODELO 
%  LINEAR DE PENDULO DUPLO INVERTIDO, UTILIZANDO DE METODO DE MOORE PARA O 
%  CASO MULTIVARIAVEL.
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
x0 = [0 1 0 1];   % CONDICOES INICIAIS

%% MODELO DO PENDULO DUPLO: LINEARIZADO PARA O EQUILIBRIO INSTAVEL

A=[0 1 0 0; 9.8 0 -9.8 0; 0 0 0 1; -9.8 0 2.94 0];
B=[0 0; 1 -2; 0 0; -2 5];
C=[1 0 0 0; 0 0 1 0];
C_full = eye(max(size(A)));

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
polos_desejados_observador_rapido = 3*(min(autovalores_MF(1, 3:4))) - autovalores_MF(1, 3:4); % OBSERVADOR RAPIDO

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

%% AGORA PROJETANDO O OBSERVADOR:

% MATRIZ COM OS AUTOVALORES DESEJADOS:
MatrizPI = diag(polos_desejados_observador_rapido);

% PROJETANDO PELO METODO 2(MOORE):

T = -[[0 1]; [1 1]];

% CALCULANDO V:
q = zeros(max(size(A)) - min(size(C)), max(size(A)));

for i = 1:min(size(q))


    q(i, :) = T(i, :)*C/(A - polos_desejados_observador_rapido(i)*eye(4));

end

Q_moore = -q;

% TESTANDO O POSTO DA MATRIZ DE ESTIMACAO:
Ma = [C; Q_moore];

if rank(Ma) == max(size(A))
    disp('POSTO COMPLETO');
else
    disp('POSTO NAO COMPLETO');
end

Inversa_MA = Ma^-1;

% REALIZANDO A SIMULACAO COM AUXILIO DO SIMULINK:

sim("FSFB_MO_DoublePendulum.slx");
