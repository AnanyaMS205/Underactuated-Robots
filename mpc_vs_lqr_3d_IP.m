clc;
clear;

%% System Definition: 3D Inverted Pendulum (with Pitch + Yaw) and 2 Inputs
A = [0 1  0       0 0 0;
     0 0 -9.8/3   0 0 0;
     0 0  0       1 0 0;
     0 0  19.6/3  0 0 0;
     0 0  0       0 0 1;
     0 0  0       0 9.8 0];

B = [0     0;
     2/3   0;
     0     0;
     -1/3  0;
     0     0;
     0     1];

C = [1 0 0 0 0 0;    % Cart Position
     0 0 1 0 0 0;    % Pitch Angle
     0 0 0 0 1 0];   % Yaw Angle

D = zeros(3,2);

x0 = [0.1; 0; 0.2; 0; 0.1; 0];  % Initial condition

%% Discretize system for MPC
Ts = 0.05;
sys = ss(A, B, C, D);
sys_d = c2d(sys, Ts);

%% Create MPC Controller
mpc_obj = mpc(sys_d, Ts);

mpc_obj.PredictionHorizon = 20;
mpc_obj.ControlHorizon = 5;

mpc_obj.Weights.OutputVariables = [150 200 100];           % Position, Pitch, Yaw
mpc_obj.Weights.ManipulatedVariables = [0.1 0.1];           % Two inputs
mpc_obj.Weights.ManipulatedVariablesRate = [0.01 0.01];

mpc_obj.MV(1).Min = -10; mpc_obj.MV(1).Max = 10;    % Force
mpc_obj.MV(2).Min = -5;  mpc_obj.MV(2).Max = 5;     % Torque

%% MPC Simulation
ref = zeros(60,3);  % Reference: hold all outputs at zero
sim_opts = mpcsimopt();
sim_opts.PlantInitialState = x0;

[y_mpc, t_mpc, u_mpc, x_mpc] = sim(mpc_obj, 60, ref, sim_opts);

%% LQR Simulation (2-input)
Q = diag([150 50 200 10 100 10]);
R = 0.01 * eye(2);         % 2 inputs → 2x2 R
K_lqr = lqr(A, B, Q, R);   % Full 2-input LQR
A_lqr_cl = A - B * K_lqr;
sys_lqr = ss(A_lqr_cl, B, C, D);
[y_lqr, t_lqr, x_lqr] = initial(sys_lqr, x0);

%% Plot Results
figure;

subplot(3,2,1);
plot(t_lqr, y_lqr(:,1), 'b--', 'LineWidth', 1.5); hold on;
plot(t_mpc*Ts, y_mpc(:,1), 'g', 'LineWidth', 1.5);
title('Cart Position (m)'); xlabel('Time (s)'); ylabel('Position');
legend('LQR', 'MPC'); grid on;

subplot(3,2,2);
plot(t_lqr, y_lqr(:,2), 'b--', 'LineWidth', 1.5); hold on;
plot(t_mpc*Ts, y_mpc(:,2), 'g', 'LineWidth', 1.5);
title('Pitch Angle (rad)'); xlabel('Time (s)'); ylabel('Pitch');
legend('LQR', 'MPC'); grid on;

subplot(3,2,3);
plot(t_lqr, x_lqr(:,2), 'b--', 'LineWidth', 1.5); hold on;
plot(t_mpc*Ts, x_mpc(:,2), 'g', 'LineWidth', 1.5);
title('Cart Velocity (m/s)'); xlabel('Time (s)'); ylabel('Velocity');
legend('LQR', 'MPC'); grid on;

subplot(3,2,4);
plot(t_lqr, x_lqr(:,4), 'b--', 'LineWidth', 1.5); hold on;
plot(t_mpc*Ts, x_mpc(:,4), 'g', 'LineWidth', 1.5);
title('Pitch Rate (rad/s)'); xlabel('Time (s)'); ylabel('Angular Velocity');
legend('LQR', 'MPC'); grid on;

subplot(3,2,5);
plot(t_lqr, y_lqr(:,3), 'b--', 'LineWidth', 1.5); hold on;
plot(t_mpc*Ts, y_mpc(:,3), 'g', 'LineWidth', 1.5);
title('Yaw Angle (rad)'); xlabel('Time (s)'); ylabel('Yaw');
legend('LQR', 'MPC'); grid on;

subplot(3,2,6);
plot(t_lqr, x_lqr(:,6), 'b--', 'LineWidth', 1.5); hold on;
plot(t_mpc*Ts, x_mpc(:,6), 'g', 'LineWidth', 1.5);
title('Yaw Rate (rad/s)'); xlabel('Time (s)'); ylabel('Yaw Velocity');
legend('LQR', 'MPC'); grid on;

sgtitle('3D Inverted Pendulum: LQR vs MPC (2-input System)');
