function DrawSpaceStationCylinder()
L = 4000;
R = 1000;

xMin = -L/2;
xMax =  L/2;

theta = linspace(0, 2*pi, 80);
x = linspace(xMin, xMax, 30);

[Theta, X] = meshgrid(theta, x);
zOffset = 1000;
Y = R * cos(Theta);
Z = R * sin(Theta)+zOffset;

surf(X, Y, Z, ...
    'FaceColor', [0.65 0.75 0.95], ...
    'FaceAlpha', 0.25, ...
    'EdgeColor', [0.25 0.35 0.65], ...
    'EdgeAlpha', 0.35);
hold on;

plot3(xMin * ones(size(theta)), R*cos(theta), R*sin(theta) + zOffset, ...
    'Color', [0.10 0.20 0.45], 'LineWidth', 2.5);

plot3(xMax * ones(size(theta)), R*cos(theta), R*sin(theta) + zOffset, ...
    'Color', [0.10 0.20 0.45], 'LineWidth', 2.5);

plot3([xMin xMax], [0 0], [zOffset zOffset], 'k--', 'LineWidth', 1.2);

end
