clc; clear; close all;

% Welt: 10 Zellen, Türen bei Positionen 3, 5, 9
world = [0 0 1 0 1 0 0 0 1 0];
num_cells = length(world);

% Partikel
num_particles = 1000;

% Feinheit: 10 Subpositionen pro Zelle
subdiv = 10;
max_pos = num_cells;  % z. B. 10.0
dx = 1 / subdiv;

% Sensor-Modell
pHit = 0.7;
pMiss = 0.1;

% Bewegungsmodell (kontinuierlich mit Gauß-Rauschen)
motion_step = 1.0;
motion_noise_std = 0.1;

% Bewegungen & Messungen
motions = [1 1 1 1 1];
true_pos = 0.0;
measurements = zeros(1, length(motions));

% Wahre Position simulieren
for i = 1:length(motions)
    true_pos = mod(true_pos + motions(i), max_pos);
    cell_index = floor(true_pos) + 1;
    measurements(i) = world(cell_index);
end

% Partikel initialisieren (kontinuierlich gleichverteilt)
particles = rand(1, num_particles) * max_pos;

% Wahre Position zurücksetzen
true_pos = 0.0;

figure('Name', 'Monte-Carlo-Filter mit feinem Gitter', 'NumberTitle', 'off');

for k = 1:length(motions)
    % --- Bewegung ---
    noise = randn(1, num_particles) * motion_noise_std;
    particles = mod(particles + motions(k) + noise, max_pos);

    % Wahre Position bewegen
    true_pos = mod(true_pos + motions(k), max_pos);

    % --- Sensorupdate ---
    weights = zeros(1, num_particles);
    for i = 1:num_particles
        cell_idx = floor(particles(i)) + 1;
        if cell_idx < 1 || cell_idx > num_cells
            continue;
        end
        hit = (world(cell_idx) == measurements(k));
        weights(i) = hit * pHit + (1 - hit) * pMiss;
    end
    weights = weights / sum(weights);

    % --- Resampling ---
    indices = randsample(1:num_particles, num_particles, true, weights);
    particles = particles(indices);

    % --- Plot ---
        % --- Plot ---
    subplot(length(motions), 1, k);
    histogram(particles - 0.5, 0:dx:max_pos, ...
        'Normalization', 'probability', ...
        'FaceColor', [0.2 0.6 0.8], 'EdgeAlpha', 0.4);
    hold on;

    % Türen markieren (zentriert)
    for i = 1:num_cells
        if world(i) == 1
            rectangle('Position', [i - 1.5, 0, 1, 0.1], ...  % i-1.5 statt i-1
                      'FaceColor', [1 0.4 0.4], 'EdgeColor', 'r', ...
                      'FaceAlpha', 0.3);
        end
    end

    % Wahre Position (zentriert)
    rectangle('Position', [true_pos , 0, 0.1, 0.08], ...
              'FaceColor', 'k', 'EdgeColor', 'k');

    title(sprintf('Schritt %d: Messung = %d, Bewegung = %.1f', ...
        k, measurements(k), motions(k)));
    ylabel('P(x)');
    if k == length(motions)
        xlabel('Kontinuierliche Position');
    end
    ylim([0 0.3]);
    grid on;
    hold off;
end
