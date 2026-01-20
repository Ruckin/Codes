%Initialize constant for the model
clear
clc
close all
%% Define constant 
x0=[5;
    0;
    0;
    0;
    0;
    0;
    0;
    0;
    0];

U=0 ;
TF=10;

%% Run the model
result=sim('FoilSimulation.slx');

%% Plot results

t=result.tout;
u1=result.simU.Data(:,1);

x1=result.simX.Data(:,1);
x2=result.simX.Data(:,2);
x3=result.simX.Data(:,3);
x4=rad2deg(result.simX.Data(:,4));
x5=rad2deg(result.simX.Data(:,5));
x6=rad2deg(result.simX.Data(:,6));
x7=rad2deg(result.simX.Data(:,7));
x8=rad2deg(result.simX.Data(:,8));
x9=rad2deg(result.simX.Data(:,9));

% mod(angles, 360)
% figure
% hold on
% plot (t,u1,'LineWidth',1.2)
% xlabel('t (s)')
% ylabel('Xg in m')
% grid on
% hold off

%Plot the state

figure 
%plot u,v,w
subplot (3,3,1)
plot(t,x1)
ylabel ('u, m/s')
grid on

subplot (3,3,2)
plot(t,x2)
ylabel ('v, m/s')
grid on

subplot (3,3,3)
plot(t,x3)
ylabel ('w, m/s')
grid on

%plot p,q,r
subplot (3,3,4)
plot(t,x4)
ylabel ('p, dg/s')
grid on

subplot (3,3,5)
plot(t,x5)
ylabel ('q, dg/s')
grid on

subplot (3,3,6)
plot(t,x6)
ylabel ('r, dg/s')
grid on

%plot phi, theta, psi
subplot (3,3,7)
plot(t,x7)
ylabel ('$\phi$ in dg/s', 'Interpreter', 'latex')
grid on

subplot (3,3,8)
plot(t,x8)
ylabel ('$\theta$ in dg/s', 'Interpreter', 'latex')
grid on

subplot (3,3,9)
plot(t,x9)
ylabel ('$\psi$ in dg/s', 'Interpreter', 'latex')
grid on