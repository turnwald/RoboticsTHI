clc; clear; close all

% Weltdefinition: Türen an Positionen 3, 5, 9 (Index-basiert ab 1)
world = [0 0 1 0 1 0 0 0 1 0];
N = length(world);

% Startverteilung: Roboter startet sicher bei Position 1 (0 in 0-basiert)
bel = ones(1, N) / N;


% Sensormodell
pHit = 0.7;
pMiss = 0.1;

% Wenn der Roboter etwas misst (z. B. „ich sehe eine Tür“), vergleicht das System diese Messung mit dem tatsächlichen Zustand an jeder Position.
% 
% Dann:
% 
% Wenn Messung = Weltzustand, erhöhe die Wahrscheinlichkeit mit pHit.
% 
% Wenn Messung ≠ Weltzustand, dämpfe die Wahrscheinlichkeit mit pMiss.
% 
% Das bedeutet:
% 
% Der Sensor ist nicht perfekt – er hat eine gewisse Treffsicherheit von 60 % (pHit)
% 
% Und irrt sich manchmal – aber immerhin mit nur 20 % (pMiss)

% Bewegungsmodell
pExact = 0.8;
pUndershoot = 0.1;
pOvershoot = 0.1;



% Bewegungen: 5 Schritte nach rechts
motions = [1 1 1 1 1];

% Wahre Startposition (Index 1 == Position 0 in 0-basierter Sicht)
true_pos = 1;

% Simuliere Messdaten (aus Sicht des echten Roboters)
measurements = zeros(1, length(motions));
for i = 1:length(motions)
    true_pos = mod(true_pos - 1 + motions(i), N) + 1;
    measurements(i) = world(true_pos);  % perfekte Messung (kann verrauscht werden)
end

% Plot-Vorbereitung
figure('Name', 'Bayes-Filter Lokalisierung', 'NumberTitle', 'off');

% Wahre Position zurücksetzen
true_pos = 1;

for k = 1:length(motions)
    % --- Bewegung ---
    bel = move(bel, motions(k), pExact, pOvershoot, pUndershoot);
    
    % Wahre Position aktualisieren
    true_pos = mod(true_pos - 1 + motions(k), N) + 1;
    
    % --- Sensor ---
    bel = sense(bel, measurements(k), world, pHit, pMiss);

    % --- Plot ---
    subplot(length(motions), 1, k);
    hold on;

    % Wahrscheinlichkeitsverteilung
    %bar(bel, 'FaceColor', [0.2 0.6 0.8]);
    % Statt: bar(bel)
    sigma = 0.3;
    x_vals = linspace(1, N, 1000);
    curve = zeros(size(x_vals));
    for i = 1:N
        curve_neu = bel(i) * normpdf(x_vals, i, sigma);
        plot(x_vals, curve_neu/2, 'LineWidth', 0.5); % /2 nur für die Visualisierung
        curve = curve+curve_neu;
    end
    plot(x_vals, curve, 'b-', 'LineWidth', 2);


    % Türen markieren
    % Türen als Rechtecke einzeichnen (z. B. Höhe = 0.15)
    door_height = 0.8;
    for i = 1:N
        if world(i) == 1
            rectangle('Position', [i - 0.4, 0, 0.8, door_height], ...
                      'FaceColor', [1 0.4 0.4], 'EdgeColor', 'r','FaceAlpha', 0.4);
        end
    end


   
    % Roboter als schwarzer Kreis am Boden
    % plot(true_pos, 0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
    % Quadratgröße
    sq_size = 0.4;
    
    % Quadrat zeichnen, zentriert auf true_pos bei y=0
    rectangle('Position', [true_pos - sq_size/2, -sq_size/2, sq_size, sq_size], ...
            'FaceColor', 'k', 'EdgeColor', 'k','FaceAlpha', 0.7);



    title(sprintf('Schritt %d: Messung = %d, Bewegung = %d', ...
        k, measurements(k), motions(k)));
    ylim([-0.05 0.8]);
    ylabel('P(x)');
    if k == length(motions)
        xlabel('Position');
    end
    grid on;
    hold off;
end

% --- Funktionen ---
function q = sense(p, Z, world, pHit, pMiss)
    q = zeros(size(p));
    for i = 1:length(p)
        hit = (Z == world(i));
        q(i) = p(i) * (hit * pHit + (1 - hit) * pMiss);
    end
    q = q / sum(q);
end

function q = move(p, U, pExact, pOvershoot, pUndershoot)
    N = length(p);
    q = zeros(1, N);
    for i = 1:N
        q(i) = ...
            pExact      * p(mod(i - U - 1, N) + 1) + ...
            pOvershoot  * p(mod(i - U - 2, N) + 1) + ...
            pUndershoot * p(mod(i - U, N) + 1);
    end
end
