clc; clear;

%% Define system (6-state inverted pendulum)
A = [0 1 0 0 0 0;
     0 0 18 0 5 0;
     0 0 0 1 0 0;
     0 0 175 0 0 0;
     0 0 0 0 0 1;
     0 0 5 0 0 0];

B = [0; 1; 0; 5; 0; 2];

C = [1 0 0 0 0 0;  % Cart position
     0 0 1 0 0 0;  % Pitch angle
     0 0 0 0 1 0]; % Yaw angle

D = zeros(3,1);
x0 = [0.1; 0; 0.2; 0; 0.1; 0];

%% ---------- 1. MPC ----------
Ts = 0.05;
p = 20;
m = 10;

sys_d = c2d(ss(A,B,C,D), Ts);

mpcobj = mpc(sys_d, Ts, p, m);

% Set weights properly to avoid warnings
mpcobj.Weights.ManipulatedVariables = 0.1;
mpcobj.Weights.ManipulatedVariablesRate = 0.1;
mpcobj.Weights.OutputVariables = [1 1 1];

% Input constraints
mpcobj.MV.Min = -10;
mpcobj.MV.Max = 10;

% Simulate MPC
ref = [0; 0; 0];
state = mpcstate(mpcobj);
xmpc = x0';
ympc = [];
tmpc = 0:Ts:5;

for k = 1:length(tmpc)
    y_measured = C * xmpc(end,:)';
    u = mpcmove(mpcobj, state, y_measured, ref);

    % FIXED: Pass 2 samples of u and time to lsim
    [y, ~, x_next] = lsim(sys_d, [u; u], [0 Ts], xmpc(end,:)');

    xmpc = [xmpc; x_next(end,:)]; %#ok<AGROW>
    ympc = [ympc; y(end,:)];      %#ok<AGROW>
end

x_mpc = xmpc(1:end-1, :);
y_mpc = ympc;
t_mpc = tmpc;

%% ---------- 2. Sliding Mode Control (SMC) ----------
lambda = diag([10 8 10]);  % sliding surface gain
eta = 5;                  % switching gain

dt = 0.01;
t_smc = 0:dt:5;
x_smc = x0';
y_smc = [];
u_smc = [];

for k = 1:length(t_smc)
    x = x_smc(end,:)';
    y = C * x;
    y_dot = C * (A * x);  % no initial input assumed

    s = lambda * y + y_dot;
    u = -eta * sign(sum(s));
    u = max(-10, min(10, u));  % saturate

    dx = A * x + B * u;
    x_next = x + dt * dx;

    x_smc = [x_smc; x_next']; %#ok<AGROW>
    y_smc = [y_smc; y']; %#ok<AGROW>
    u_smc = [u_smc; u]; %#ok<AGROW>
end

x_smc = x_smc(1:end-1,:);
t_smc = t_smc;

%% ---------- 3. Plotting ----------
figure;

subplot(3,2,1);
plot(t_mpc, y_mpc(:,1), 'g', 'LineWidth', 1.5); hold on;
plot(t_smc, y_smc(:,1), 'm--', 'LineWidth', 1.5);
title('Cart Position'); ylabel('m'); legend('MPC','SMC'); grid on;

subplot(3,2,2);
plot(t_mpc, y_mpc(:,2), 'g', 'LineWidth', 1.5); hold on;
plot(t_smc, y_smc(:,2), 'm--', 'LineWidth', 1.5);
title('Pitch Angle'); ylabel('rad'); grid on;

subplot(3,2,3);
plot(t_mpc, x_mpc(:,2), 'g', 'LineWidth', 1.5); hold on;
plot(t_smc, x_smc(:,2), 'm--', 'LineWidth', 1.5);
title('Cart Velocity'); grid on;

subplot(3,2,4);
plot(t_mpc, x_mpc(:,4), 'g', 'LineWidth', 1.5); hold on;
plot(t_smc, x_smc(:,4), 'm--', 'LineWidth', 1.5);
title('Pitch Rate'); grid on;

subplot(3,2,5);
plot(t_mpc, y_mpc(:,3), 'g', 'LineWidth', 1.5); hold on;
plot(t_smc, y_smc(:,3), 'm--', 'LineWidth', 1.5);
title('Yaw Angle'); ylabel('rad'); xlabel('Time (s)'); grid on;

subplot(3,2,6);
plot(t_mpc, x_mpc(:,6), 'g', 'LineWidth', 1.5); hold on;
plot(t_smc, x_smc(:,6), 'm--', 'LineWidth', 1.5);
title('Yaw Rate'); xlabel('Time (s)'); grid on;

sgtitle('3D Inverted Pendulum: MPC vs SMC');
