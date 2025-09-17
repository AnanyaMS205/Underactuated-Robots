
%% Inverted Pendulum Control Comparison
clc; clear; close all;

%% System Parameters
M = 0.5;     % Mass of the cart (kg)
m = 0.2;     % Mass of the pendulum (kg)
b = 0.1;     % Friction (N/m/sec)
I = 0.006;   % Pendulum inertia (kg.m^2)
g = 9.8;     % Gravity (m/s^2)
l = 0.3;     % Length to pendulum COM (m)

%% State-Space Model
p = I*(M+m) + M*m*l^2;

A = [0      1            0           0;
     0 -(I+m*l^2)*b/p  (m^2*g*l^2)/p  0;
     0      0            0           1;
     0 -(m*l*b)/p      m*g*l*(M+m)/p  0];

B = [0;
     (I+m*l^2)/p;
     0;
     m*l/p];

C = eye(4); D = zeros(4,1);

initial_state = [0.1; 0; 0.2; 0];
t = 0:0.01:5;

%% 1. LQR Controller
Q = diag([10 1 10 1]); R = 0.01;
K_lqr = lqr(A, B, Q, R);
sys_lqr = ss(A - B*K_lqr, B, C, D);
[y_lqr,~,x_lqr] = initial(sys_lqr, initial_state, t);

%% 2. Pole Placement
K_pp = place(A, B, [-2 -3 -4 -5]);
sys_pp = ss(A - B*K_pp, B, C, D);
[y_pp,~,x_pp] = initial(sys_pp, initial_state, t);

%% 3. MPC (requires MPC Toolbox)
try
    Ts = 0.05;
    mpcobj = mpc(ss(A,B,C,D), Ts);
    mpcobj.PredictionHorizon = 20;
    mpcobj.ControlHorizon = 5;
    sim(mpcobj);  % for use in Simulink
catch
    disp('MPC Toolbox not installed.');
end

%% 4. H-infinity Control (requires Robust Control Toolbox)
try
    nmeas = 2; ncon = 1;
    [~, K_hinf] = hinfsyn(ss(A,B,C,D), nmeas, ncon);
    sys_hinf = ss(A - B*K_hinf, B, C, D);
    [y_hinf,~,x_hinf] = initial(sys_hinf, initial_state, t);
catch
    disp('Robust Control Toolbox not installed.');
end

%% Plot Results
figure;
subplot(2,1,1); hold on; grid on;
plot(t, x_lqr(:,3), 'b', 'LineWidth', 1.5);
plot(t, x_pp(:,3), 'r--', 'LineWidth', 1.5);
title('Pendulum Angle θ (rad)');
legend('LQR','Pole Placement');

subplot(2,1,2); hold on; grid on;
plot(t, x_lqr(:,1), 'b', 'LineWidth', 1.5);
plot(t, x_pp(:,1), 'r--', 'LineWidth', 1.5);
title('Cart Position x (m)');
xlabel('Time (s)');
legend('LQR','Pole Placement');

%% Additional controllers (SMC, EPSAC, NDI+SMC) would be implemented using Simulink or nonlinear simulation.
