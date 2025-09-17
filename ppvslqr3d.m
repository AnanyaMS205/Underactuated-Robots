clc; clear; close all;

%% System Matrices (Extended 3D State-Space)
A = [ 0     1      0       0      0      0;
      0  -0.0645  18.0377  0      0      0;
      0     0      0       1      0      0;
      0  -0.4032 174.0484  0      0      0;
      0     0      0       0      0      1;
      0     0      0       0   -0.2     0 ];  % Added simple yaw dynamics

B = [ 0;
      0.6452;
      0;
      4.0323;
      0;
      1 ];  % Assume torque input affects yaw

C = [1 0 0 0 0 0;    % Cart position
     0 0 1 0 0 0;    % Pendulum angle
     0 0 0 0 1 0];   % Yaw angle

D = [0; 0; 0];

x0 = [0.1; 0; 0.2; 0; 0.1; 0];  % Initial disturbance in position, tilt and yaw

%% Pole Placement Control
K_pole = place(A, B, [-1.5+1j, -1.5-1j, -4, -5, -2+1j, -2-1j]);
A_pp = A - B*K_pole;
sys_pp = ss(A_pp, B, C, D);

[y_pp, t_pp, x_pp] = initial(sys_pp, x0);

%% LQR Control
Q = diag([150, 50, 200, 10, 100, 10]);  % Add yaw penalties
R = 0.01;

K_lqr = lqr(A, B, Q, R);
A_lqr = A - B*K_lqr;
sys_lqr = ss(A_lqr, B, C, D);

[y_lqr, t_lqr, x_lqr] = initial(sys_lqr, x0);

%% Plotting
figure;

subplot(3,1,1);
plot(t_pp, y_pp(:,1), 'r', 'LineWidth', 1.5); hold on;
plot(t_lqr, y_lqr(:,1), 'b--', 'LineWidth', 1.5);
title('Cart Position (m)'); ylabel('x (m)');
legend('Pole Placement', 'LQR'); grid on;

subplot(3,1,2);
plot(t_pp, y_pp(:,2), 'r', 'LineWidth', 1.5); hold on;
plot(t_lqr, y_lqr(:,2), 'b--', 'LineWidth', 1.5);
title('Pendulum Angle (rad)'); ylabel('\theta (rad)');
legend('Pole Placement', 'LQR'); grid on;

subplot(3,1,3);
plot(t_pp, y_pp(:,3), 'r', 'LineWidth', 1.5); hold on;
plot(t_lqr, y_lqr(:,3), 'b--', 'LineWidth', 1.5);
title('Yaw Angle (rad)'); ylabel('\psi (rad)');
xlabel('Time (s)');
legend('Pole Placement', 'LQR'); grid on;

sgtitle('Comparison of Pole Placement vs LQR (3D Inverted Pendulum)');
