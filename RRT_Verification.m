figure;
hold on;
grid on;
axis equal;
view(3);
xlabel('x');
ylabel('y');
zlabel('z');
title('RRT末端轨迹可视化');

q_start = [0; 90; 75; 100; -60; 110; -66];
q_goal  = [-35; 55; -40; 850; 70; -50; 80];

%将起点和终点的机械臂画出来
DHfk_7DoFRobot_Aboka(0,90,75,100,-60,110,-66,0);
DHfk_7DoFRobot_Aboka(-35,55,-40,850,70,-50,80,0);

[path, success, tree] = RRT_7DoF_Aboka(q_start, q_goal);

% ========= 先画整棵树（蓝色） =========
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

% ========= 再画最终路径（红色） =========
if success
    for i = 2:size(path, 1)
        q1 = path(i-1, :)';
        q2 = path(i, :)';

        p1 = FK_EndPoint(q1);
        p2 = FK_EndPoint(q2);

        plot3([p1(1), p2(1)], ...
              [p1(2), p2(2)], ...
              [p1(3), p2(3)], 'r-', 'LineWidth', 2);

        plot3(p2(1), p2(2), p2(3), 'r.', 'MarkerSize', 15);
    end
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

    for i = 2:8
        Link(i).A = Link(i-1).A * Link(i).A;
        Link(i).p = Link(i).A(1:3,4);
    end

    p_end = Link(8).p(1:3);
end
