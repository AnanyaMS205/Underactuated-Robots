clc; clear;

%% Define a fully controllable 6-state system (Cart + Pitch + Yaw)
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
     2];   % Fully controllable with 1 input

% Outputs: cart position, pitch angle, yaw angle
C = [1 0 0 0 0 0;  % Position
     0 0 1 0 0 0;  % Pitch angle
     0 0 0 0 1 0]; % Yaw angle

D = zeros(3,1);

% Initial condition: 10 cm cart offset, 0.2 rad pitch, 0.1 rad yaw
x0 = [0.1; 0; 0.2; 0; 0.1; 0];

%% 1. Pole Placement Control (no error now)
K_pole = place(A, B, [-2 -2.5 -3 -3.5 -4 -4.5]);
A_pole_cl = A - B * K_pole;
sys_pole = ss(A_pole_cl, B, C, D);
[y_pole, t_pole, x_pole] = initial(sys_pole, x0);

%% 2. LQR Control
Q = diag([150 50 200 10 100 10]);  % Penalize cart, pitch, yaw
R = 0.01;
K_lqr = lqr(A, B, Q, R);
A_lqr_cl = A - B * K_lqr;
sys_lqr = ss(A_lqr_cl, B, C, D);
[y_lqr, t_lqr, x_lqr] = initial(sys_lqr, x0);

%% 3. Plot Comparison (6 states)
figure;

% 1. Cart Position
subplot(3,2,1);
plot(t_pole, y_pole(:,1), 'r', 'LineWidth', 1.5); hold on;
plot(t_lqr, y_lqr(:,1), 'b--', 'LineWidth', 1.5);
title('Cart Position (m)'); xlabel('Time (s)'); ylabel('Position');
legend('Pole Placement', 'LQR'); grid on;

% 2. Pendulum Pitch Angle
subplot(3,2,2);
plot(t_pole, y_pole(:,2), 'r', 'LineWidth', 1.5); hold on;
plot(t_lqr, y_lqr(:,2), 'b--', 'LineWidth', 1.5);
title('Pitch Angle (rad)'); xlabel('Time (s)'); ylabel('Pitch');
legend('Pole Placement', 'LQR'); grid on;

% 3. Cart Velocity
subplot(3,2,3);
plot(t_pole, x_pole(:,2), 'r', 'LineWidth', 1.5); hold on;
plot(t_lqr, x_lqr(:,2), 'b--', 'LineWidth', 1.5);
title('Cart Velocity (m/s)'); xlabel('Time (s)'); ylabel('Velocity');
legend('Pole Placement', 'LQR'); grid on;

% 4. Pitch Angular Velocity
subplot(3,2,4);
plot(t_pole, x_pole(:,4), 'r', 'LineWidth', 1.5); hold on;
plot(t_lqr, x_lqr(:,4), 'b--', 'LineWidth', 1.5);
title('Pitch Rate (rad/s)'); xlabel('Time (s)'); ylabel('Angular Velocity');
legend('Pole Placement', 'LQR'); grid on;

% 5. Yaw Angle
subplot(3,2,5);
plot(t_pole, y_pole(:,3), 'r', 'LineWidth', 1.5); hold on;
plot(t_lqr, y_lqr(:,3), 'b--', 'LineWidth', 1.5);
title('Yaw Angle (rad)'); xlabel('Time (s)'); ylabel('Yaw');
legend('Pole Placement', 'LQR'); grid on;

% 6. Yaw Rate
subplot(3,2,6);
plot(t_pole, x_pole(:,6), 'r', 'LineWidth', 1.5); hold on;
plot(t_lqr, x_lqr(:,6), 'b--', 'LineWidth', 1.5);
title('Yaw Rate (rad/s)'); xlabel('Time (s)'); ylabel('Yaw Velocity');
legend('Pole Placement', 'LQR'); grid on;

sgtitle('3D Inverted Pendulum: Pole-Placement vs LQR Control');
