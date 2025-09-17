
clc; clear;

% System Matrices from your image
A = [0     1       0           0;
     0  -0.0645   18.0377      0;
     0     0       0           1;
     0  -0.4032  174.0484      0];

B = [0;
     0.6452;
     0;
     4.0323];

C = [1 0 0 0;      % Output: Position and angle
     0 0 1 0];

D = [0; 0];

% Initial condition: small displacement and angle
x0 = [0.1; 0; 0.2; 0];  % 10 cm cart offset, 0.2 rad (11°) tilt

%% 1. Pole Placement Control
K_pole = [-1.7650, -2.0159, 55.1393, 3.2635];

A_pole_cl = A - B * K_pole;

sys_pole_cl = ss(A_pole_cl, B, C, D);

[y_pole, t_pole, x_pole] = initial(sys_pole_cl, x0);

%% 2. LQR Control
Q = diag([150, 50, 200, 10]);
R = 0.01;

K_lqr = lqr(A, B, Q, R);

A_lqr_cl = A - B * K_lqr;

sys_lqr_cl = ss(A_lqr_cl, B, C, D);

[y_lqr, t_lqr, x_lqr] = initial(sys_lqr_cl, x0);

%% 3. Plotting Results

figure;

subplot(2,2,1);
plot(t_pole, y_pole(:,1), 'r', 'LineWidth', 1.5);
hold on;
plot(t_lqr, y_lqr(:,1), 'b--', 'LineWidth', 1.5);
title('Cart Position (m)');
legend('Pole Placement', 'LQR');
xlabel('Time (s)');
ylabel('Position');
grid on;

subplot(2,2,2);
plot(t_pole, y_pole(:,2), 'r', 'LineWidth', 1.5);
hold on;
plot(t_lqr, y_lqr(:,2), 'b--', 'LineWidth', 1.5);
title('Pendulum Angle (rad)');
legend('Pole Placement', 'LQR');
xlabel('Time (s)');
ylabel('Angle');
grid on;

subplot(2,2,3);
plot(t_pole, x_pole(:,2), 'r', 'LineWidth', 1.5);
hold on;
plot(t_lqr, x_lqr(:,2), 'b--', 'LineWidth', 1.5);
title('Cart Velocity (m/s)');
legend('Pole Placement', 'LQR');
xlabel('Time (s)');
ylabel('Velocity');
grid on;

subplot(2,2,4);
plot(t_pole, x_pole(:,4), 'r', 'LineWidth', 1.5);
hold on;
plot(t_lqr, x_lqr(:,4), 'b--', 'LineWidth', 1.5);
title('Pendulum Angular Velocity (rad/s)');
legend('Pole Placement', 'LQR');
xlabel('Time (s)');
ylabel('Angular Velocity');
grid on;

sgtitle('Comparison of Pole-Placement vs LQR Control');
