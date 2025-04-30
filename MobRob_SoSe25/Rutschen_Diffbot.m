clc; clear;

% Parameter
r = 0.1;      % Radradius [m]
b = 0.5;      % Achsabstand [m]
T = 1;        % Zeitschritt [s]
N = 5;       % Anzahl Schritte

% Nominale Werte (Kreisfahrt: L schneller als R)
omega_L_nom = 10 * ones(1,N);  % rad/s
omega_R_nom = 8 * ones(1,N);

% Tatsächliche Bewegung: linkes Rad rutscht, aber Encoder zeigt trotzdem 10
omega_L_actual = 9 * ones(1,N);       % effektiv weniger
% omega_L_measured = 10 * ones(1,N);    % wird fälschlich so angenommen
omega_R_actual = 8 * ones(1,N);       % korrekt

% Positionen: [x; y; theta]
pose_nom = zeros(3, N+1);      % ohne Schlupf
pose_rutsch = zeros(3, N+1);   % mit Rutschfehler

% Schleife
for k = 1:N
    % --- Nominell ---
    v_nom = r/2 * (omega_R_nom(k) + omega_L_nom(k));
    w_nom = r/b * (omega_R_nom(k) - omega_L_nom(k));
    theta = pose_nom(3,k);
    pose_nom(1,k+1) = pose_nom(1,k) + T * v_nom * cos(theta);
    pose_nom(2,k+1) = pose_nom(2,k) + T * v_nom * sin(theta);
    pose_nom(3,k+1) = theta + T * w_nom;

    % --- Mit Schlupf (linkes Rad rutscht) ---
    v_schlupf = r/2 * (omega_R_actual(k) + omega_L_actual(k));
    w_schlupf = r/b * (omega_R_actual(k) - omega_L_actual(k));
    theta = pose_rutsch(3,k);
    pose_rutsch(1,k+1) = pose_rutsch(1,k) + T * v_schlupf * cos(theta);
    pose_rutsch(2,k+1) = pose_rutsch(2,k) + T * v_schlupf * sin(theta);
    pose_rutsch(3,k+1) = theta + T * w_schlupf;
end

% Plot
figure;
plot(pose_nom(1,:), pose_nom(2,:), 'b-o', 'LineWidth', 2); hold on;
plot(pose_rutsch(1,:), pose_rutsch(2,:), 'r-s', 'LineWidth', 2);
legend('Nominell (10/8 rad/s)', 'Mit Rutschfehler (L effektiv 9)');
xlabel('x [m]'); ylabel('y [m]');
title('Einfluss von Rutschfehler beim Differentialantrieb');
axis equal; grid on;
