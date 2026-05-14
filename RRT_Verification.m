clear;
clc;
close all;

figure;
hold on;
grid on;
axis equal;
view(3);
rotate3d on;
xlabel('x');
ylabel('y');
zlabel('z');
title('RRT末端轨迹与球障碍物可视化');

q_start = [0; 90; 75; 100; -60; 110; -66];
q_goal  = [70; 60; 75; 500; -97; 20; 80];

% ========= 球障碍物 =========
DrawSpaceStationCylinder();
obstacles = GenerateBallsInCylinder();


% 先把球画出来
for k = 1:length(obstacles)
    DrawBall(obstacles(k).center, obstacles(k).radius, [0.3 0.6 1.0]);
end


% ========= RRT规划 =========
[path, success, tree] = RRT_7DoF_Aboka(q_start, q_goal, obstacles);

% 验证减支代码
if success
    path_short = RRT_ShortCut(path, obstacles);
end
% ========= 画整棵树（蓝色） =========
for i = 2:size(tree.nodes, 1)
    idx_parent = tree.parent(i);

    q_child = tree.nodes(i, :)';
    q_parent = tree.nodes(idx_parent, :)';

    p_child = FK_EndPoint(q_child);
    p_parent = FK_EndPoint(q_parent);

    plot3([p_parent(1), p_child(1)], ...
          [p_parent(2), p_child(2)], ...
          [p_parent(3), p_child(3)], 'b-');

    plot3(p_child(1), p_child(2), p_child(3), 'b.');
end

% 起点画大一点,蓝色
p_start = FK_EndPoint(q_start);
plot3(p_start(1), p_start(2), p_start(3), 'bo', ...
    'MarkerSize', 8, 'MarkerFaceColor', 'b');

% ========= 画最终路径（红色） =========
if success
    for i = 2:size(path, 1)
        q1 = path(i-1, :)';
        q2 = path(i, :)';

        p1 = FK_EndPoint(q1);
        p2 = FK_EndPoint(q2);

        plot3([p1(1), p2(1)], ...
              [p1(2), p2(2)], ...
              [p1(3), p2(3)], 'r-', 'LineWidth', 2);

        plot3(p2(1), p2(2), p2(3), 'ro', ...
            'MarkerSize', 3, 'MarkerFaceColor', 'r');
    end
end

% 终点画大一点，绿色
p_goal = FK_EndPoint(q_goal);
plot3(p_goal(1), p_goal(2), p_goal(3), 'go', ...
    'MarkerSize', 8, 'MarkerFaceColor', 'g');

% ========= 画后处理后的路径（绿色） =========
if success
    for i = 2:size(path_short, 1)
        q1 = path_short(i-1, :)';
        q2 = path_short(i, :)';

        p1 = FK_EndPoint(q1);
        p2 = FK_EndPoint(q2);

        plot3([p1(1), p2(1)], ...
              [p1(2), p2(2)], ...
              [p1(3), p2(3)], 'g', 'LineWidth', 2);

        plot3(p2(1), p2(2), p2(3), 'ro', ...
            'MarkerSize', 3, 'MarkerFaceColor', 'g');
    end
end


disp(['success = ', num2str(success)]);
if success
    disp('找到路径。');
else
    disp('未找到路径。');
end


function p_end = FK_EndPoint(q)
    global Link
    ToRad = pi/180;

    Build_7DOFRobot_Aboka;

    Link(2).th = q(1)*ToRad;
    Link(3).th = q(2)*ToRad;
    Link(4).th = q(3)*ToRad;
    Link(5).dz = q(4);
    Link(6).th = q(5)*ToRad;
    Link(7).th = q(6)*ToRad;
    Link(8).th = q(7)*ToRad;

    for i = 1:8
        Matrix_DH_Ln(i);
    end

    Link(1).p = Link(1).p(1:3);

    for i = 2:8
        Link(i).A = Link(i-1).A * Link(i).A;
        Link(i).p = Link(i).A(1:3,4);
    end

    p_end = Link(8).p(1:3);
end


function DrawBall(center, radius, colorValue)
    [X, Y, Z] = sphere(24);
    X = radius * X + center(1);
    Y = radius * Y + center(2);
    Z = radius * Z + center(3);

    surf(X, Y, Z, ...
        'FaceColor', colorValue, ...
        'EdgeColor', 'none', ...
        'FaceAlpha', 0.5);

    %camlight;
    %lighting gouraud;
end

function obstacles = GenerateBallsInCylinder()
% 在圆柱空间内随机生成10个球
% 球半径 50~100 mm
% 球与球之间净间距 > 200 mm

    numBalls = 10;
    zOffset = 1000;
    % 圆柱参数
    R_cyl = 1000;      % 圆柱半径
    x_min = -2000;         % 圆柱底面 x
    x_max = 2000;      % 圆柱顶面 x

    r_min = 50;
    r_max = 100;

    maxTry = 10000;
    obstacles = struct('center', {}, 'radius', {});

    n = 0;
    tryCount = 0;

    while n < numBalls && tryCount < maxTry
        tryCount = tryCount + 1;

        % 1. 随机半径
        r_ball = r_min + (r_max - r_min) * rand;

        % 2. 在圆柱横截面内均匀采样球心
        theta = 2*pi*rand; %在圆柱横截面的圆里随机选方向
        rho = sqrt(rand) * (R_cyl - r_ball);   % 随机半径，保证整个球不出圆柱侧壁

        y = rho * cos(theta);
        z = rho * sin(theta)+zOffset;

        % 3. x方向采样，保证整个球不穿出上下端面
        x = x_min + r_ball + (x_max - x_min - 2*r_ball) * rand;

        c_new = [x; y; z];%c_new为新生成的球的球心坐标

        % 4. 检查与已有球的距离约束
        isValid = true;
        for k = 1:n
            c_old = obstacles(k).center;
            r_old = obstacles(k).radius;

            % 净间距 > 200 mm
            if norm(c_new - c_old) <= (r_ball + r_old + 200)
                isValid = false;
                break;
            end
        end

        % 5. 通过检查则加入
        if isValid
            n = n + 1;
            obstacles(n).center = c_new;
            obstacles(n).radius = r_ball;
        end
    end

    if n < numBalls
        error('在最大尝试次数内未能生成足够数量的小球，请放宽约束或增大圆柱空间。');
    end
end
