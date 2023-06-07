% Aircraft model example using the H-infinity robust controller
clc
clear
close all

% aircraft model
A = [0 0 1.132 0 -1;
    0 -0.0538 -0.1712 0 0.0705;
    0 0 0 1 0;
    0 0.0485 0 -0.8556 -1.013;
    0 -0.2909 0 1.0532 -0.6859];
B = [0 0 0;
    -0.12 0 0;
    0 0 0;
    4.419 0 -1.665;
    1.575 0 -0.0732];
C = [1 0 0 0 0;0 1 0 0 0;0 0 1 0 0];
D = zeros(3,3);

G = ss(A,B,C,D);

% design of weighting filters
Ms = 10^-4; ws = 2.5*10^-4;
nws = ws/Ms;
dws = [1 ws];
Wss = tf(nws,dws); % to shape sensitivity
% Ws = [Wss 0 0;0 Wss 0;0 0 (10*Wss)];
Ws=eye(3)*(tf(nws,dws));

Mk = 10^2; wk = 10^2; c = 10^3;
nwk = [1 wk];
dwk = [1 c*wk];
wk = (c/Mk)*(tf(nwk,dwk)); % to shape control sensitivity
% Wks = [wk 0 0;0 wk 0;0 0 wk];
Wks = eye(3)*((c/Mk)*(tf(nwk,dwk)));

% Plant design
Wt = [];
P = augw(G,Ws,Wks,Wt);

% Controller design
nmeas = 3;
nu = 3;
gmn = 0; %gamma min val
gmx = 10; %gamma max val
tol = 0.001; %rel error tol
gmr = [gmn, gmx]; % gamma range

% opt = hinfsynOptions('AbsTol',1e-4);
opt = hinfsynOptions('RelTol',0.05);
% [K,CL,gopt] = hinfsyn(P,nmeas,nu,gmn,gmx,tol);
% [K,CL,gopt] = hinfsyn(P,nmeas,nu,gmr,opt);
[K,CL,gopt] = hinfsyn(P,nmeas,nu,opt);

% Plot
% step response
figure
step(CL)

% Sensitivity
S = inv(eye(3)+G*K);
% Complementary Sensitivity
T = G*K*S;
% Control Sensitivity_input to plant
KS = K*S;

figure % input output plot
subplot(2,1,1)
step(T(1,1),T(2,1),T(3,1)); xlim([0,2]);
grid
legend('y1','y2','y3')
xlabel('Time (s)');ylabel('Output y')
subplot(2,1,2)
step(KS(1,1),KS(2,1),KS(3,1))
grid
legend('u1','u2','u3')
xlabel('Time (s)');ylabel('Input u')

figure % sensitivity and constraint plot
sigma(S,'b',1/Ws,'r')
grid
title('Sensitivity and constraint')
legend('s','1/Ws')

figure % control-sensitivity and constraint
sigma(KS,1/Wks)
grid
title('Control Sensitivity and constraint')
legend('cs','1/Wks')

% intoducing the scaled constraint
figure
sigma(S,gopt/Ws)
grid
title('Sensitivity and scaled constraint')
legend('s','gopt/Ws')
figure
sigma(T,gopt/Wks)
grid
title('Control Sensitivity and scaled constraint')
legend('cs','gopt/Wks')