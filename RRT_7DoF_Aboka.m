function [path, success, tree] = RRT_7DoF_Aboka(q_start, q_goal, obstacles)
% RRT_7DoF_Aboka 7自由度关节空间RRT（带球障碍物碰撞检测）
%
% 输入:
%   q_start   : 7x1 或 1x7 起点关节变量
%   q_goal    : 7x1 或 1x7 终点关节变量
%   obstacles : 球障碍物结构体数组
%               obstacles(k).center = [x; y; z];
%               obstacles(k).radius = r;
%
% 输出:
%   path      : Mx7 路径节点序列，每一行是一个关节节点
%   success   : 是否成功找到路径
%   tree      : RRT树结构，保存节点和父节点

    % 保证输入是行向量
    q_start = q_start(:)';
    q_goal  = q_goal(:)';

    % ========= 参数设置 =========
    maxIter = 5000;      % 最大迭代次数
    stepSize = 0.1;      % 归一化空间单次扩展步长
    rho = 0.2;           % 目标偏置概率
    tol = 0.1;           % 判定到达目标的阈值（归一化空间距离）

    linkRadius = 45;     % 连杆胶囊体半径，单位 mm
    edgeStep = 0.05;     % 边碰撞检测时的归一化插值步长

    % ========= 关节范围 =========
    q_min = [-180, -120, -120,    0, -180, -120, -180];
    q_max = [ 180,  120,  120, 1500,  180,  120,  180];

    % 归一化尺度
    span = q_max - q_min;

    % ========= 初始化树 =========
    tree.nodes = q_start;   % 每一行一个节点
    tree.parent = 0;        % 根节点父节点记为0

    success = false;
    path = [];

    % 起点或终点本身碰撞时，直接失败
    if IsConfigCollision(q_start, obstacles, linkRadius)
        warning('起点发生碰撞，RRT无法开始规划。');
        return;
    end

    if IsConfigCollision(q_goal, obstacles, linkRadius)
        warning('终点发生碰撞，RRT无法规划到目标。');
        return;
    end

    % ========= 主循环 =========
    for iter = 1:maxIter

        % 1. 随机采样
        if rand < rho
            q_sample = q_goal;
        else
            q_sample = q_min + rand(1,7) .* (q_max - q_min);
        end

        % 2. 找最近节点
        n = size(tree.nodes, 1);
        dist = zeros(n, 1);

        for i = 1:n
            dq = (tree.nodes(i,:) - q_sample) ./ span;
            dist(i) = norm(dq);
        end

        [~, idx_near] = min(dist);
        q_near = tree.nodes(idx_near, :);

        % 3. 朝采样点扩展一步
        d = (q_sample - q_near) ./ span;
        d_norm = norm(d);

        if d_norm < 1e-6
            continue;
        end

        d_unit = d / d_norm;
        q_new = q_near + d_unit * stepSize .* span;

        % 4. 限位
        q_new = min(max(q_new, q_min), q_max);

        % 5. 若新节点与最近节点太近，则跳过
        dq_same = (q_new - q_near) ./ span;
        if norm(dq_same) < 1e-6
            continue;
        end

        % 6. 碰撞检测
        if IsConfigCollision(q_new, obstacles, linkRadius)
            continue;
        end

        if ~IsEdgeCollisionFree(q_near, q_new, obstacles, linkRadius, span, edgeStep)
            continue;
        end

        % 7. 加入树
        tree.nodes = [tree.nodes; q_new];
        tree.parent = [tree.parent; idx_near];

        new_idx = size(tree.nodes, 1);

        % 8. 判断是否接近目标
        dq_goal = (q_new - q_goal) ./ span;
        goal_dist = norm(dq_goal);

        if goal_dist < tol
            % 最后再检查 q_new 到 q_goal 这一段是否无碰撞
            if IsEdgeCollisionFree(q_new, q_goal, obstacles, linkRadius, span, edgeStep)
                success = true;

                tree.nodes = [tree.nodes; q_goal];
                tree.parent = [tree.parent; new_idx];

                goal_idx = size(tree.nodes, 1);

                % 回溯得到路径
                idx = goal_idx;
                while idx ~= 0
                    path = [tree.nodes(idx, :); path];
                    idx = tree.parent(idx);
                end
                return;
            end
        end
    end

    % 若迭代结束仍未找到路径
    success = false;
    path = [];
end


function isCollide = IsConfigCollision(q, obstacles, linkRadius)
% 判断单个关节姿态 q 是否与任意球障碍物碰撞

    points = FK_LinkPoints(q);

    isCollide = false;

    % 连杆按线段处理：points(:,i) -> points(:,i+1)
    for i = 1:size(points, 2)-1
        a = points(:, i);
        b = points(:, i+1);

        for k = 1:length(obstacles)
            c = obstacles(k).center(:);
            r_ball = obstacles(k).radius;

            d = PointToSegmentDistance(c, a, b);

            if d <= (linkRadius + r_ball)
                isCollide = true;
                return;
            end
        end
    end
end


function isFree = IsEdgeCollisionFree(q1, q2, obstacles, linkRadius, span, edgeStep)
% 判断从 q1 到 q2 这条边是否全程无碰撞

    dq_norm = norm((q2 - q1) ./ span);
    nStep = max(2, ceil(dq_norm / edgeStep));

    isFree = true;

    for i = 0:nStep
        alpha = i / nStep;
        q_mid = (1 - alpha) * q1 + alpha * q2;

        if IsConfigCollision(q_mid, obstacles, linkRadius)
            isFree = false;
            return;
        end
    end
end


function points = FK_LinkPoints(q)
% 对当前关节变量做正运动学，返回各关节点位置
% 输出 points 为 3x8，每一列是一个关节点位置

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

    % 基坐标原点
    Link(1).p = Link(1).p(1:3);

    for i = 2:8
        Link(i).A = Link(i-1).A * Link(i).A;
        Link(i).p = Link(i).A(1:3,4);
    end

    points = zeros(3, 8);
    for i = 1:8
        points(:, i) = Link(i).p(1:3);
    end
end


function d = PointToSegmentDistance(c, a, b)
% 计算点 c 到线段 ab 的最短距离

    ab = b - a;
    ac = c - a;

    if norm(ab) < 1e-12
        d = norm(c - a);
        return;
    end

    t = dot(ac, ab) / dot(ab, ab);
    t = max(0, min(1, t));

    p_closest = a + t * ab;
    d = norm(c - p_closest);
end

