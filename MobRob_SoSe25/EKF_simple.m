% EKF für mobilen Roboter mit x, y, psi (Orientierung) und v, omega als Eingänge

clear; clc; close all;

% Simulationseinstellungen
dt = 0.1;                   % Zeitschritt (s)
T = 20;                     % Gesamtdauer (s)
N = T / dt;                 % Anzahl Zeitschritte

% Wahre Anfangswerte
x_true = [0; 0; 0]; % [x; y; psi]

% Anfangswerte der Schätzungen
x_est = [0; 0; 0];
x_odo = [0; 0; 0];

% Kovarianzmatrizen
P = diag([0.1, 0.1, 0.5]); % Anfangsunsicherheit
Q_model = diag([0.1, 0.1, 0.5]);  % Modell (Odometrie und EKF) glauben 
R = 0.05^2; % Messrauschen (IMU für psi)

% Speicher für Verlauf
X_true = zeros(N, 3);
X_est = zeros(N, 3);
X_odo = zeros(N, 3);

% Angenommene Eingänge (Fahr- und Drehgeschwindigkeit)
b = 0.5;
v_R = 0.3;               % m/s
v_L = 0.4;               % m/s
v = (v_R + v_L) / 2;
omega = (v_R - v_L) / b;

for k = 2:N
    % === Wahre Bewegung (reales Modell mit Schlupf) ===
    v_R_meas = v_R  + 0.2*rand();
    v_L_meas = v_L  + 0.1*rand();
    v_meas = (v_R_meas + v_L_meas) / 2;  % Verstärkter Messfehler
    omega_meas = (v_R_meas - v_L_meas) / b;

    % Wahre Bewegung
    x_true(1) = x_true(1) + v * cos(x_true(3)) * dt;
    x_true(2) = x_true(2) + v * sin(x_true(3)) * dt;
    x_true(3) = x_true(3) + omega * dt;

    % Simulation der IMU-Messung (psi)
    z = x_true(3) + sqrt(R) * randn();

    % === Odometrie (stark gestört) ===
    x_odo(1) = x_odo(1) + v_meas * cos(x_odo(3)) * dt;
    x_odo(2) = x_odo(2) + v_meas * sin(x_odo(3)) * dt;
    x_odo(3) = x_odo(3) + omega_meas * dt;

    % === EKF: Prädiktion ===
    x_pred = x_est;
    x_pred(1) = x_pred(1) + v_meas * cos(x_pred(3)) * dt;
    x_pred(2) = x_pred(2) + v_meas * sin(x_pred(3)) * dt;
    x_pred(3) = x_pred(3) + omega_meas * dt;

    % Jacobimatrix für den EKF
    F = [1 0 -v_meas * sin(x_est(3)) * dt;
         0 1  v_meas * cos(x_est(3)) * dt;
         0 0 1];

    % Prädizierte Kovarianz
    P = F * P * F' + Q_model;

    % === EKF: Korrektur ===
    H = [0 0 1]; % Messung nur für psi
    K = P * H' / (H * P * H' + R);

    % Zustandskorrektur
    x_est = x_pred + K * (z - x_pred(3));

    % Kovarianz-Update
    P = (eye(3) - K * H) * P;

    % === Speicherung ===
    X_true(k, :) = x_true';
    X_est(k, :) = x_est';
    X_odo(k, :) = x_odo';
end

% === Ergebnisse plotten ===
figure;
plot(X_true(:,1), X_true(:,2), '.-g', 'DisplayName', 'Wahre Trajektorie (mit Schlupf)');
hold on;
plot(X_odo(:,1), X_odo(:,2), '--r', 'DisplayName', 'Odometrie (mit Fehler)');
plot(X_est(:,1), X_est(:,2), '.-b', 'DisplayName', 'EKF (bessere Korrektur)');
legend();
xlabel('X-Position [m]');
ylabel('Y-Position [m]');
title('Vergleich: Wahre Trajektorie, Odometrie und EKF (Schlupf unbekannt)');
grid on;
