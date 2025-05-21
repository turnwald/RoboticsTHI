clc; clear;

% Simulationsparameter
T = 1;           % Abtastzeit [s]
N = 50;          % Zeitschritte
v_true = 1;      % konstante Geschwindigkeit
x_true = zeros(1,N);  % echte Position

v_meas = v_true + rand(1,N);   % verrauschte Geschw.-messung
z = zeros(1,N);  % verrauschte Positionsmessung

% Rauschgrößen
R = 1.0;  % Messrauschen (Position)
Q = 1.0;  % Prozessrauschen (Modellfehler)

% Initialisierung
x_true(1) = 0;
z(1) = x_true(1) + sqrt(R)*randn;

% Beobachter mit fester Verstärkung
L = 0.2;
x_obs = zeros(1,N);

x_obs2 = zeros(1,N);


% Kalman-Filter
x_kalman = zeros(1,N);
P = 1;  % Startvarianz

% Odometrie (Integration der Geschwindigkeit)
x_odom = zeros(1,N);
x_odom(1) = 0;

% Simulation
for k = 2:N
    % Wahre Position
    x_true(k) = x_true(k-1) + T*v_true;

    % Messung der Position
    z(k) = x_true(k) + sqrt(R)*randn;

    % Odometrie
    x_odom(k) = x_odom(k-1) + T*v_meas(k-1);

    % Beobachter (fester L)
    x_obs(k) = x_obs(k-1) + T*v_meas(k-1) + L*(z(k-1) - x_obs(k-1));

    % Beobachter 2-Stufig
    % % Beobachter (Zweistufig)
    x_2stp_pred = x_obs2(k-1) + T*v_meas(k-1);
    x_obs2(k) = x_2stp_pred + L*(z(k) - x_2stp_pred);

    % ------ Kalman-Filter
    % Prediction
    x_pred = x_kalman(k-1) + T*v_meas(k-1);
    P_pred = P + Q;

    % Update
    K = P_pred / (P_pred + R);
    x_kalman(k) = x_pred + K*(z(k) - x_pred);
    P = (1 - K)*P_pred;
end

% Plotten
figure(1);
clf
plot(x_true, 'k-', 'LineWidth', 2); hold on;
plot(x_odom, '--', 'LineWidth', 1.5);
plot(x_obs, '-.', 'LineWidth', 1.5);
plot(x_obs2, '-.', 'LineWidth', 1.5);
plot(x_kalman, '-', 'LineWidth', 2);
legend('Wahr', 'Odometrie', 'Beobachter', 'Beobachter 2Stf.', 'Kalman');
xlabel('Zeit [k]');
ylabel('Position');
title('Vergleich: Odometrie, Beobachter, Kalman-Filter');
grid on;
