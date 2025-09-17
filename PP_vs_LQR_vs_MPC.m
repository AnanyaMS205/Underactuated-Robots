clc; clear;

%% Define a fully controllable 6-state system (Cart + Pitch + Yaw)
A = [0 1 0 0 0 0;
     0 0 18 0 5 0;
     0 0 0 1 0 0;
     0 0 175 0 0 0;
     0 0 0 0 0 1;
     0 0 5 0 0 0];

B = [0;
     1;
     0;
     5;
     0;
     2];   % Fully controllable with 1 input

C = [1 0 0 0 0 0;  % Cart position
     0 0 1 0 0 0;  % Pitch angle
     0 0 0 0 1 0]; % Yaw angle

D = zeros(3,1);

% Initial condition: 10 cm cart offset, 0.2 rad pitch, 0.1 rad yaw
x0 = [0.1; 0; 0.2; 0; 0.1; 0];

%% 1. Pole Placement Control
K_pole = place(A, B, [-2 -2.5 -3 -3.5 -4 -4.5]);
A_pole_cl = A - B * K_pole;
sys_pole = ss(A_pole_cl, B, C, D);
[y_pole, t_pole, x_pole] = initial(sys_pole, x0);

%% 2. LQR Control
Q = diag([150 50 200 10 100 10]);  % State penalty
R = 0.01;                          % Control penalty
K_lqr = lqr(A, B, Q, R);
A_lqr_cl = A - B * K_lqr;
sys_lqr = ss(A_lqr_cl, B, C, D);
[y_lqr, t_lqr, x_lqr] = initial(sys_lqr, x0);

%% 3. MPC Control
Ts = 0.05;               % Sampling time
p = 20;                  % Prediction horizon
m = 10;                  % Control horizon

sys_d = c2d(ss(A,B,C,D), Ts);  % Discrete-time model

mpcobj = mpc(sys_d, Ts, p, m);

% Set weights explicitly
mpcobj.Weights.ManipulatedVariables = 0.1;
mpcobj.Weights.ManipulatedVariablesRate = 0.1;
mpcobj.Weights.OutputVariables = [1 1 1];

% Set input constraints (optional)
mpcobj.MV.Min = -10;
mpcobj.MV.Max = 10;

% Reference signal (all outputs to zero)
ref = [0; 0; 0];

% Initial MPC state and simulation parameters
state = mpcstate(mpcobj);
xmpc = x0';
ympc = [];
tmpc = 0:Ts:5;

for k = 1:length(tmpc)
    % Calculate current output from state
    y_current = sys_d.C * xmpc(end,:)' + sys_d.D * 0;  % output y (3x1)

    % Compute control input using MPC
    u = mpcmove(mpcobj, state, y_current, ref);

    % Simulate discrete dynamics for one step
    x_next = (sys_d.A * xmpc(end,:)' + sys_d.B * u)';

    % Store results
    xmpc = [xmpc; x_next]; %#ok<AGROW>
    ympc = [ympc; (sys_d.C * x_next')']; %#ok<AGROW>
end

x_mpc = xmpc(1:end-1, :);
y_mpc = ympc;
t_mpc = tmpc;


x_mpc = xmpc(1:end-1, :);
y_mpc = ympc;
t_mpc = tmpc;

%% 4. Plot Comparison
figure;

% 1. Cart Position
subplot(3,2,1);
plot(t_pole, y_pole(:,1), 'r', 'LineWidth', 1.5); hold on;
plot(t_lqr, y_lqr(:,1), 'b--', 'LineWidth', 1.5);
plot(t_mpc, y_mpc(:,1), 'g-.', 'LineWidth', 1.5);
title('Cart Position (m)'); xlabel('Time (s)'); ylabel('Position');
legend('Pole', 'LQR', 'MPC'); grid on;

% 2. Pitch Angle
subplot(3,2,2);
plot(t_pole, y_pole(:,2), 'r', 'LineWidth', 1.5); hold on;
plot(t_lqr, y_lqr(:,2), 'b--', 'LineWidth', 1.5);
plot(t_mpc, y_mpc(:,2), 'g-.', 'LineWidth', 1.5);
title('Pitch Angle (rad)'); xlabel('Time (s)'); ylabel('Pitch');
legend('Pole', 'LQR', 'MPC'); grid on;

% 3. Cart Velocity
subplot(3,2,3);
plot(t_pole, x_pole(:,2), 'r', 'LineWidth', 1.5); hold on;
plot(t_lqr, x_lqr(:,2), 'b--', 'LineWidth', 1.5);
plot(t_mpc, x_mpc(:,2), 'g-.', 'LineWidth', 1.5);
title('Cart Velocity'); xlabel('Time (s)'); ylabel('Velocity');
legend('Pole', 'LQR', 'MPC'); grid on;

% 4. Pitch Rate
subplot(3,2,4);
plot(t_pole, x_pole(:,4), 'r', 'LineWidth', 1.5); hold on;
plot(t_lqr, x_lqr(:,4), 'b--', 'LineWidth', 1.5);
plot(t_mpc, x_mpc(:,4), 'g-.', 'LineWidth', 1.5);
title('Pitch Angular Velocity'); xlabel('Time (s)'); ylabel('Rate');
legend('Pole', 'LQR', 'MPC'); grid on;

% 5. Yaw Angle
subplot(3,2,5);
plot(t_pole, y_pole(:,3), 'r', 'LineWidth', 1.5); hold on;
plot(t_lqr, y_lqr(:,3), 'b--', 'LineWidth', 1.5);
plot(t_mpc, y_mpc(:,3), 'g-.', 'LineWidth', 1.5);
title('Yaw Angle'); xlabel('Time (s)'); ylabel('Yaw');
legend('Pole', 'LQR', 'MPC'); grid on;

% 6. Yaw Rate
subplot(3,2,6);
plot(t_pole, x_pole(:,6), 'r', 'LineWidth', 1.5); hold on;
plot(t_lqr, x_lqr(:,6), 'b--', 'LineWidth', 1.5);
plot(t_mpc, x_mpc(:,6), 'g-.', 'LineWidth', 1.5);
title('Yaw Angular Velocity'); xlabel('Time (s)'); ylabel('Yaw Rate');
legend('Pole', 'LQR', 'MPC'); grid on;

sgtitle('3D Inverted Pendulum: Pole Placement vs LQR vs MPC');
