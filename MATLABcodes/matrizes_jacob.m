clear all
close all
clc
syms Fi Ti Fv k At rho Cp lambda L T

% Equacoes de estado
f1 = ( Fi - k*sqrt(L) ) / At;
f2 = ( rho*Fi*Cp*(Ti - T) +  Fv*lambda )/ ( rho*At*L*Cp ) ;
f = [f1; f2];

% vetor de estados
x = [L;T];
% vetor de entradas
u = [Fi; Ti; Fv];

% Parametros
rho = 1e3       ;
Cp  = 4.18      ;
At  = pi*0.5^2  ;
k   = 7         ;
lambda = 2.257e4;

% Estado estacionario na entrada
Fi = 10; % vazao volumetrica [=] m3/min
Ti = 40; % temperatura [=] ºC
Fv = 10; % vazao massica [=] kg/min

% Estado estacionario no estados
L = 2.0408; % nivel [=] m
T = 45.4  ; % temperatura [=] ºC

% Matriz jacobiana dos estados
A = double(subs(jacobian(f,x)));
% A = [                                 -k/(2*At*L^(1/2)),          0
%      -(Fv*lambda - Cp*Fi*rho*(T - Ti))/(At*Cp*L^2*rho), -Fi/(At*L) ];
% Matriz jacobiana das entradas
B = double(subs(jacobian(f,u)));
% B = [             1/At,         0,                    0
%      -(T - Ti)/(At*L), Fi/(At*L), lambda/(At*Cp*L*rho) ]; 

C = eye(2);
D = zeros(2,3);

sys=ss(A,B,C,D);
[num,den]=tfdata(sys,'v');
G=tf(num,den);

% step(G(2,1))