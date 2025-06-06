% Monte-Carlo-Lokalisierung (MCL) in 2D mit GPS und Odometrie

clear; clc; 

figure(1)
clf

rng(42); % Für Vergleichbarkeit

% Parameter
N = 500;                % Anzahl Partikel
T = 20;                 % Zeitschritte
gps_noise = 1.0;        % GPS-Rauschen (Standardabweichung)
odo_noise_v = 0.2;      % Odometrie-Rauschen (Bewegung)
odo_noise_psi = deg2rad(5); % Rotationsrauschen
odo_slip_factor = 0.9;  % Simulierter Schlupf (z.B. 90% der Bewegung)

% Wahre Roboterposition
v = 1.0;
omega = 0.1;
true_pos = zeros(T,3); % [x, y, psi]
for t = 2:T
    true_pos(t,1) = true_pos(t-1,1) + v*cos(true_pos(t-1,3));
    true_pos(t,2) = true_pos(t-1,2) + v*sin(true_pos(t-1,3));
    true_pos(t,3) = true_pos(t-1,3) + omega;
end

% GPS-Messungen mit Rauschen
gps = true_pos(:,1:2) + gps_noise * randn(T,2);

% Odometrie-Trajektorie (mit Schlupf) und Start bei erster GPS-Messung
odo_pos = zeros(T, 3);
odo_pos(1,1:2) = gps(1,:); % Start bei GPS
for t = 2:T
    noisy_omega = omega + odo_noise_psi * randn();
    noisy_v = v * odo_slip_factor + odo_noise_v * randn();
    odo_pos(t,1) = odo_pos(t-1,1) + noisy_v * cos(odo_pos(t-1,3));
    odo_pos(t,2) = odo_pos(t-1,2) + noisy_v * sin(odo_pos(t-1,3));
    odo_pos(t,3) = odo_pos(t-1,3) + noisy_omega; % psi
end

% Partikel initialisieren
particles = [10*rand(N,2)-5, 2*pi*rand(N,1)-pi];
weights = ones(N,1) / N;

% Schätzungsspeicher
estimates = zeros(T,3);


for t = 1:T
    % --- Bewegungsschritt ---
    noisy_omega_vek = omega + odo_noise_psi*randn(N,1);
    noisy_v_vek = v * odo_slip_factor + odo_noise_v*randn(N,1);
    particles(:,1) = particles(:,1) + noisy_v_vek .* cos(particles(:,3));
    particles(:,2) = particles(:,2) + noisy_v_vek .* sin(particles(:,3));
    particles(:,3) = particles(:,3) + noisy_omega_vek;

    % --- Messungsschritt ---
    dx = particles(:,1) - gps(t,1);
    dy = particles(:,2) - gps(t,2);
    weights = exp(-(dx.^2 + dy.^2) / (2*gps_noise^2));
    weights = weights + 1e-300; % Null vermeiden
    weights = weights / sum(weights); % Normieren

    % --- Resampling ---
    idx = randsample(1:N, N, true, weights); % Resample anhand von weights
    particles = particles(idx,:);

    % --- Schätzung ---
    estimates(t,:) = mean(particles);

    % --- Plot ---
    clf; hold on; grid on;
    scatter(particles(:,1), particles(:,2), 10, 'b', 'filled', 'MarkerFaceAlpha', 0.2);
    plot(true_pos(1:t,1), true_pos(1:t,2), 'k-', 'LineWidth', 2);
    plot(gps(1:t,1), gps(1:t,2), 'gx', 'MarkerSize', 8);
    plot(estimates(1:t,1), estimates(1:t,2), 'r--', 'LineWidth', 1.5);
    plot(odo_pos(1:t,1), odo_pos(1:t,2), 'c-.', 'LineWidth', 1.5);
    legend('Partikel', 'Wahre Position', 'GPS', 'MCL-Schätzung', 'Odometrie');
    title(sprintf('Monte-Carlo-Lokalisierung - Schritt %d', t));
    xlim([-5 20]); ylim([-5 15]);
    drawnow;
    pause(0.2);
end
