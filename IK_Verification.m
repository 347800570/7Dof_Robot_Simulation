% IK_Verification
% 思路：用正运动学画一个姿态，然后从另一个初始量解出逆运动学，比较两个位姿之间的差距。

clear;
clc;
close all;

global Link

% 设定目标位姿和起始位姿
q_target = [90; 90; 0; 500; 90; 90; 0];
q0       = [60; 60; -30; 300; 60; 110; 66];

% 正运动学计算目标位姿.
DHfk_7DoFRobot_Aboka(q0(1),q0(2),q0(3),q0(4),q0(5),q0(6),q0(7),0);
DHfk_7DoFRobot_Aboka(q_target(1),q_target(2),q_target(3),q_target(4),q_target(5),q_target(6),q_target(7),0);

T_target = Link(8).A;
R_target = T_target(1:3, 1:3);
p_target = T_target(1:3, 4);

% 逆运动学.
[q_sol, info] = IK_NS_7DoF(q0, T_target);
DHfk_7DoFRobot_Aboka(q_sol(1),q_sol(2),q_sol(3),q_sol(4),q_sol(5),q_sol(6),q_sol(7),0);
