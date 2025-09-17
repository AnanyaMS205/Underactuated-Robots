clc;
clear;
close all;

% Define a transfer function G(s) = num(s)/den(s)
num = [1];                    % Numerator coefficients
den = [1, 5, 6];              % Denominator coefficients => s^2 + 5s + 6
G = tf(num, den);            % Create transfer function

disp('Transfer Function:');
G

% 1. Step Response
figure;
step(G);
title('Step Response');
grid on;

% 2. Impulse Response
figure;
impulse(G);
title('Impulse Response');
grid on;

% 3. Pole-Zero Map
figure;
pzmap(G);
title('Pole-Zero Map');
grid on;

% 4. Bode Plot
figure;
bode(G);
title('Bode Plot');
grid on;

% 5. Nyquist Plot
figure;
nyquist(G);
title('Nyquist Plot');
grid on;

% 6. Nichols Plot
figure;
nichols(G);
title('Nichols Plot');
grid on;

% 7. Root Locus
figure;
rlocus(G);
title('Root Locus');
grid on;

% 8. Frequency Response (Manual magnitude & phase plot)
w = logspace(-1, 2, 1000);      % Frequency range from 0.1 to 100 rad/s
[mag, phase, wout] = bode(G, w);
mag = squeeze(mag);
phase = squeeze(phase);

% Magnitude vs Frequency
figure;
semilogx(wout, 20*log10(mag));
xlabel('Frequency (rad/s)');
ylabel('Magnitude (dB)');
title('Frequency Response - Magnitude');
grid on;

% Phase vs Frequency
figure;
semilogx(wout, phase);
xlabel('Frequency (rad/s)');
ylabel('Phase (deg)');
title('Frequency Response - Phase');
grid on;

% 9. Time Response for Custom Input (e.g., ramp input)
t = 0:0.01:10;                 % Time vector
u = t;                         % Ramp input: u(t) = t
[y, t_out] = lsim(G, u, t);    % Simulate time response
figure;
plot(t_out, y, 'b', t, u, 'r--');
legend('System Output', 'Ramp Input');
xlabel('Time (s)');
ylabel('Response');
title('Time Response to Ramp Input');
grid on;
