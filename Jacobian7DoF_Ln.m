function J=Jacobian7DoF_Ln(th1,th2,th3,d4,th5,th6,th7)
% Geometric Jacobian of the 7-DOF robot.
% Joint inputs: revolute joints in degrees, prismatic joint d4 in length unit.

global Link

jsize=7;
J=zeros(6,jsize);

Link(2).th=th1*pi/180;
Link(3).th=th2*pi/180;
Link(4).th=th3*pi/180;
Link(5).dz=d4;
Link(6).th=th5*pi/180;
Link(7).th=th6*pi/180;
Link(8).th=th7*pi/180;

% Compute the local transforms of all joints.
for i=1:8
    Matrix_DH_Ln(i);
end

Link(1).p=Link(1).p(1:3);
% Accumulate them to the base frame.
for i=2:8
    Link(i).A=Link(i-1).A*Link(i).A;
    Link(i).p= Link(i).A(1:3,4);
    Link(i).n= Link(i).A(:,1);
    Link(i).o= Link(i).A(:,2);
    Link(i).a= Link(i).A(:,3);
    Link(i).R=[Link(i).n(1:3),Link(i).o(1:3),Link(i).a(1:3)];
end

% Revolute-joint Jacobian column: [z_i x (p_e - p_i); z_i].
for n=1:jsize
    a=Link(n).R*Link(n).az;
    if n == 4
        J(:,n) = [a; [0; 0; 0]];
    else
        J(:,n)=[cross(a,Link(8).p-Link(n).p); a];
    end
end
