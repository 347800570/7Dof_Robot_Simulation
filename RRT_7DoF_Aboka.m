function [path, success, tree] = RRT_7DoF_Aboka(q_start, q_goal)
% RRT_7DoF 最基础的7自由度关节空间RRT
% 输入:
%   q_start : 7x1 起点关节变量
%   q_goal  : 7x1 终点关节变量
% 输出:
%   path    : Mx7 路径节点序列，每一行是一个关节节点
%   success : 是否成功找到路径
%   tree    : RRT树结构，暂时只保存节点和父节点

% 保证输入是行向量，便于后续拼接
q_start = q_start(:)';
q_goal  = q_goal(:)';

% ========= 参数设置 =========
maxiter = 5000;      % 最大迭代次数
stepSize = 0.1;       % 单次扩展步长（在归一化空间里的步长）
rho = 0.2;      % 目标偏置概率 ρ 
tol = 0.1;        % 判定到达目标的阈值（归一化空间距离）

% ========= 关节范围 =========
q_min = [-180, -120, -120,    0, -180, -120, -180];
q_max = [ 180,  120,  120, 1500,  180,  120,  180];

% 用于归一化距离
span = q_max - q_min;

% ========= 初始化树 =========
tree.nodes = q_start;     % 每一行一个节点
tree.parent = 0;          % 根节点父节点记为0

success = false;
path = [];

% ========= 主循环 =========
for iter = 1:maxiter

    % 1. 随机采样
    if rand < rho
        q_sample = q_goal;
    else
        q_sample = q_min + rand(1,7) .* (q_max - q_min);
    end

    % 2. 找最近节点
    n = size(tree.nodes, 1); % 看tree.nodes的行数，有n个节点
    dist = zeros(n, 1); % 准备一个nx1的矩阵放距离

    for i = 1:n
        d_goal = (tree.nodes(i,:) - q_sample) ./ span;
        dist(i) = norm(d_goal);
    end

    [~, idx_near] = min(dist);% 得到最小距离以及所在行数

    q_near = tree.nodes(idx_near, :);% 将那一行的节点取出来

    % 3. 朝采样点扩展一步
    d_sample = (q_sample - q_near) ./ span;
    d_sample_norm = norm(d_sample);

    if d_sample_norm < 1e-6
        continue;   % 如果太靠近就不走了
    else
        d_unit = d_sample / d_sample_norm;
        q_new = q_near + d_unit * stepSize .* span;
    end

    % 4. 限位
    q_new = min(max(q_new, q_min), q_max);
    if d_sample_norm < 1e-6
        continue;   % 如果太靠近就不走了
    end
    % 5. 符合条件的新节点加入树
    tree.nodes = [tree.nodes; q_new];
    tree.parent = [tree.parent; idx_near];

    new_idx = size(tree.nodes, 1);

    % 7. 判断是否到达目标
    d_goal = (q_new - q_goal) ./ span;%计算当前节点到目标节点的距离
    d_goal_norm = norm(d_goal);

    if d_goal_norm < tol
        success = true;

        % 把目标点也接到树上
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

% 如果迭代结束还没找到路径
success = false;
path = [];
end











