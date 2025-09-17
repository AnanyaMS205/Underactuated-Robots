clc;
clear;

%% System Definition (Cart + Pitch + Yaw)
A = [0 1   0 0 0 0;
     0 0  18 0 5 0;
     0 0   0 1 0 0;
     0 0 175 0 0 0;
     0 0   0 0 0 1;
     0 0   5 0 0 0];

B = [0;
     1;
     0;
     5;
     0;
     2];   

C = [1 0 0 0 0 0;    % Cart Position
     0 0 1 0 0 0;    % Pitch Angle
     0 0 0 0 1 0];   % Yaw Angle

D = zeros(3,1);

% Initial Condition: 0.1 m cart offset, 0.2 rad pitch, 0.1 rad yaw
x0 = [0.1; 0; 0.2; 0; 0.1; 0];

%% Discretize System for MPC
Ts = 0.05;  % Sampling time (s)
sys = ss(A, B, C, D);
sys_d = c2d(sys, Ts);  % Discrete system

%% Create MPC Controller
mpc_obj = mpc(sys_d, Ts);

% Set Horizons
mpc_obj.PredictionHorizon = 20;
mpc_obj.ControlHorizon = 5;

% Reasonable Weights (Avoid huge gain explosions)
mpc_obj.Weights.OutputVariables = [5 10 5];         % Weights for Position, Pitch, Yaw
mpc_obj.Weights.ManipulatedVariables = 0.1;
mpc_obj.Weights.ManipulatedVariablesRate = 0.01;

% Input constraints (optional)
mpc_obj.MV.Min = -10;
mpc_obj.MV.Max = 10;

%% Simulation Setup
% Reference is zero for stabilization
ref = zeros(60,3);  % 60 time steps, 3 outputs

% Simulate for 60 steps
sim_opts = mpcsimopt();
sim_opts.PlantInitialState = x0;

[y_mpc, t_mpc, u_mpc, x_mpc] = sim(mpc_obj, 60, ref, sim_opts);

%% Plot Results

figure;

% 1. Cart Position
subplot(3,2,1);
plot(t_mpc*Ts, y_mpc(:,1), 'g', 'LineWidth', 1.5);
title('Cart Position (m)'); xlabel('Time (s)'); ylabel('Position');
ylim([-0.2 0.2]); grid on;

% 2. Pitch Angle
subplot(3,2,2);
plot(t_mpc*Ts, y_mpc(:,2), 'g', 'LineWidth', 1.5);
title('Pitch Angle (rad)'); xlabel('Time (s)'); ylabel('Pitch');
ylim([-0.3 0.3]); grid on;

% 3. Cart Velocity
subplot(3,2,3);
plot(t_mpc*Ts, x_mpc(:,2), 'g', 'LineWidth', 1.5);
title('Cart Velocity (m/s)'); xlabel('Time (s)'); ylabel('Velocity');
ylim([-1 1]); grid on;

% 4. Pitch Angular Velocity
subplot(3,2,4);
plot(t_mpc*Ts, x_mpc(:,4), 'g', 'LineWidth', 1.5);
title('Pitch Rate (rad/s)'); xlabel('Time (s)'); ylabel('Angular Velocity');
ylim([-2 2]); grid on;

% 5. Yaw Angle
subplot(3,2,5);
plot(t_mpc*Ts, y_mpc(:,3), 'g', 'LineWidth', 1.5);
title('Yaw Angle (rad)'); xlabel('Time (s)'); ylabel('Yaw');
ylim([-0.3 0.3]); grid on;

% 6. Yaw Rate
subplot(3,2,6);
plot(t_mpc*Ts, x_mpc(:,6), 'g', 'LineWidth', 1.5);
title('Yaw Rate (rad/s)'); xlabel('Time (s)'); ylabel('Yaw Velocity');
ylim([-2 2]); grid on;

sgtitle('3D Inverted Pendulum: MPC Response');



