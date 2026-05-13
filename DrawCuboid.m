function h = DrawCuboid(pos, az, len, width, height, col)
% Draw a cuboid starting at pos with its long axis direction az.
% The local z-axis of the cuboid is aligned with az.
%
% Usage:
%   h = DrawCuboid(pos, az, len, width, height, col)
%
% Inputs:
%   pos    : 3x1 or 1x3 start position on the cuboid center axis
%   az     : 3x1 or 1x3 direction of the long axis
%   len    : length along az
%   width  : local x direction size
%   height : local y direction size
%   col    : color, such as 0.8 or [0.8 0.8 0.8]
% Build a rotation matrix that aligns the cylinder local z-axis with az.

pos = pos(:);
pos = pos(1:3);
az = az(:);
az = az(1:3);
if norm(az) < eps
    az = [0; 0; 1];
else
    az = az / norm(az);
end

az0 = [0;0;1];
ax  = cross(az0,az);
ax_n = norm(ax);
if ax_n < eps 
    if dot(az0, az) >= 0
        rot = eye(3);
    else
        rot = [1 0 0; 0 -1 0; 0 0 -1];
    end
else
    ax = ax/ax_n;
    ay = cross(az,ax);
    ay = ay/norm(ay);
    rot = [ax ay az];
end

% Generate a standard cuboid mesh in the local frame.

lx = width / 2;
ly = height / 2;
lz = len / 2;

verticesLocal = [
    -lx, -ly, -lz;
     lx, -ly, -lz;
     lx,  ly, -lz;
    -lx,  ly, -lz;
    -lx, -ly,  lz;
     lx, -ly,  lz;
     lx,  ly,  lz;
    -lx,  ly,  lz
];

faces = [
    1, 2, 3, 4;
    5, 6, 7, 8;
    1, 2, 6, 5;
    2, 3, 7, 6;
    3, 4, 8, 7;
    4, 1, 5, 8
];

center = pos + az * len / 2;
vertices = zeros(size(verticesLocal));

for n = 1:size(verticesLocal, 1)
    xyz = verticesLocal(n, :).';
    xyz2 = rot * xyz;
    vertices(n, :) = (xyz2 + center).';
end

if numel(col) == 3
    h = patch('Vertices', vertices, 'Faces', faces, ...
        'FaceColor', col(:).', 'EdgeColor', 'k');
else
    h = patch('Vertices', vertices, 'Faces', faces, ...
        'FaceVertexCData', col * ones(size(vertices, 1), 1), ...
        'FaceColor', 'flat', 'EdgeColor', 'k');
end

