clc; clear;

% Zufalls-Seed für Reproduzierbarkeit
rng(42);  % beliebige feste Zahl

% Zielfunktion
f = @(x, y) sin(x) .* cos(y) .* exp(-0.1 * (x.^2 + y.^2));

% Parameter
N = 200;             % Anzahl Partikel
steps = 6;          % Optimierungsschritte
elite_ratio = 0.54;   % Anteil "guter" Partikel
domain = [-5, 5];    % Suchraum

% Initialisierung
particles = rand(N, 2) * diff(domain) + domain(1);

% Gitter zur Darstellung der Funktion
[xg, yg] = meshgrid(linspace(domain(1), domain(2), 100));
zg = f(xg, yg);

% Plot vorbereiten
figure(1);
colormap parula

% Visualisierung
clf;
contourf(xg, yg, zg, 20, 'LineColor', 'none');
pause(0.5)

for t = 1:steps
    % Bewertung
    x = particles(:,1);
    y = particles(:,2);
    z = f(x, y);

    % Sortieren nach Güte
    [~, idx_sorted] = sort(z, 'descend');
    elite_count = round(N * elite_ratio);
    elite_particles = particles(idx_sorted(1:elite_count), :);
    
    % Neue Partikel um gute herum erzeugen (leicht gestreut)
    new_particles = [];
    sigma = 0.3; % Streuung für neue Kandidaten

    while size(new_particles, 1) < N
        parent = elite_particles(randi(elite_count), :);
        offspring = parent + randn(1, 2) * sigma;
        if all(offspring >= domain(1)) && all(offspring <= domain(2))
            new_particles = [new_particles; offspring];
        end
    end
    particles = new_particles;

    % Visualisierung
    clf;
    contourf(xg, yg, zg, 20, 'LineColor', 'none');
    hold on;
    scatter(particles(:,1), particles(:,2), 20, 'k', 'filled');
    scatter(elite_particles(:,1), elite_particles(:,2), 40, 'r', 'filled');
    title(sprintf('Schritt %d – Partikel sammeln sich beim Maximum', t));
    xlabel('x'); ylabel('y'); 
    % axis equal tight;
    drawnow;pause(0.4)
end
