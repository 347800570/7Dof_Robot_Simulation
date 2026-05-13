function [q, info] = IK_NS_7DoF(q0, T_target)
% IK_NS_7DoF Numerical inverse kinematics for the 7-DOF Aboka robot.
%
% Inputs:
%   q0       : 7x1 initial joint vector [th1; th2; th3; d4; th5; th6; th7]
%              revolute joints are in degrees, d4 is in mm.
%   T_target : 4x4 target homogeneous transformation matrix.
%
% Outputs:
%   q        : 7x1 solved joint vector, same unit convention as q0.
%   info     : solution information.

global Link

learning_rate = 0.5;
lambda = 0.05;
iter = 0;
maxiter = 50000;
tol = 1e-5;

ToDeg = 180/pi;
ToRad = pi/180;

% Task-space scaling. Position is in mm, orientation is in rad.
posScale = 100;
rotScale = 1;

% Switch to full pose tracking after the position is close enough.
posSwitchTol = 30; % mm

% Joint limits. Revolute joints are in degrees, the prismatic joint is in mm.
q_min = [-180; -120; -120; 0;    -180; -120; -180];
q_max = [ 180;  120;  120; 1500;  180;  120;  180];

% span must match dq units: revolute updates are in rad, d4 update is in mm.
span = [
    (q_max(1)-q_min(1))*ToRad;
    (q_max(2)-q_min(2))*ToRad;
    (q_max(3)-q_min(3))*ToRad;
    q_max(4)-q_min(4);
    (q_max(5)-q_min(5))*ToRad;
    (q_max(6)-q_min(6))*ToRad;
    (q_max(7)-q_min(7))*ToRad
];
maxNormalizedStep = 0.05;

th1 = q0(1);
th2 = q0(2);
th3 = q0(3);
d4  = q0(4);
th5 = q0(5);
th6 = q0(6);
th7 = q0(7);

p_ref = T_target(1:3, 4);
R_ref = T_target(1:3, 1:3);

while true
    % Current pose.
    Link(2).th = th1*ToRad;
    Link(3).th = th2*ToRad;
    Link(4).th = th3*ToRad;
    Link(5).dz = d4;
    Link(6).th = th5*ToRad;
    Link(7).th = th6*ToRad;
    Link(8).th = th7*ToRad;

    for i = 1:8
        Matrix_DH_Ln(i);
    end

    for i = 2:8
        Link(i).A = Link(i-1).A * Link(i).A;
        Link(i).p = Link(i).A(1:3,4);
        Link(i).n = Link(i).A(:,1);
        Link(i).o = Link(i).A(:,2);
        Link(i).a = Link(i).A(:,3);
        Link(i).R = [Link(i).n(1:3), Link(i).o(1:3), Link(i).a(1:3)];
    end

    p_now = Link(8).p(1:3);
    R_now = Link(8).R;

    % Pose error.
    p_err = p_ref - p_now;
    R_err = R_ref * R_now';
    w_err = LocalRotationVector(R_err);

    positionOnly = norm(p_err) > posSwitchTol;
    if positionOnly
        err = p_err / posScale;
    else
        err = [
            p_err / posScale;
            w_err / rotScale
        ];
    end
    Loss = norm(err);

    % Only declare success after switching to full pose tracking.
    if ~positionOnly && Loss < tol
        info.success = true;
        info.loss = Loss;
        info.iter = iter;
        info.phase = 'full_pose';
        info.positionError = norm(p_err);
        info.rotationError = norm(w_err);
        q = [th1; th2; th3; d4; th5; th6; th7];
        break;
    end

    if iter >= maxiter
        info.success = false;
        info.loss = Loss;
        if positionOnly
            info.phase = 'position_only';
        else
            info.phase = 'full_pose';
        end
        info.iter = iter;
        info.positionError = norm(p_err);
        info.rotationError = norm(w_err);
        q = [th1; th2; th3; d4; th5; th6; th7];
        break;
    end

    % Joint update.
    J = Jacobian7DoF_Ln(th1, th2, th3, d4, th5, th6, th7);
    if positionOnly
        J_s = J(1:3,:) / posScale;
        taskEye = eye(3);
    else
        J_s = [
            J(1:3,:) / posScale;
            J(4:6,:) / rotScale
        ];
        taskEye = eye(6);
    end

    J_dls = J_s' / (J_s * J_s' + lambda^2 * taskEye);
    dq = learning_rate * J_dls * err;

    % Limit the update by each joint's own motion range.
    normalizedStep = norm(dq ./ span);
    if normalizedStep > maxNormalizedStep
        dq = dq * (maxNormalizedStep / normalizedStep);
    end

    th1 = th1 + dq(1)*ToDeg;
    th2 = th2 + dq(2)*ToDeg;
    th3 = th3 + dq(3)*ToDeg;
    d4  = d4  + dq(4);
    th5 = th5 + dq(5)*ToDeg;
    th6 = th6 + dq(6)*ToDeg;
    th7 = th7 + dq(7)*ToDeg;

    q_now = [th1; th2; th3; d4; th5; th6; th7];
    q_now = min(max(q_now, q_min), q_max);

    th1 = q_now(1);
    th2 = q_now(2);
    th3 = q_now(3);
    d4  = q_now(4);
    th5 = q_now(5);
    th6 = q_now(6);
    th7 = q_now(7);

    iter = iter + 1;
end

end


function w_err = LocalRotationVector(R_err)
% Convert a rotation error matrix into a 3x1 rotation-vector error.
cos_theta = (trace(R_err) - 1) / 2;
cos_theta = max(min(cos_theta, 1), -1);
theta = acos(cos_theta);

if abs(theta) < 1e-6
    w_err = [0; 0; 0];
else
    w_err = theta / (2*sin(theta)) * [
        R_err(3,2) - R_err(2,3);
        R_err(1,3) - R_err(3,1);
        R_err(2,1) - R_err(1,2)
    ];
end
end
