function DHfk_7DoFRobot_Aboka(th1,th2,th3,d4,th5,th6,th7,fcla)  %%输入单位 度
global Link

Build_7DOFRobot_Aboka;
radius    = 30;
len       = 90;
joint_col = 0;

len_trans = d4-50;
if len_trans <= 0
    len_trans = 0;
end

width = 60;
height = 60;

ToDeg=pi/180;

plot3(0,0,0,'ro');

Link(2).th = th1*ToDeg;
Link(3).th = th2*ToDeg;
Link(4).th = th3*ToDeg;
Link(5).dz = d4;
Link(6).th = th5*ToDeg;
Link(7).th = th6*ToDeg;
Link(8).th = th7*ToDeg;
Link(9).th = 0*ToDeg;
Link(10).th = 0*ToDeg;

for i=1:10
    Matrix_DH_Ln(i); %% 计算各个关节之间的齐次变换矩阵
end

%% 计算世界坐标系到末端的变换矩阵
for i=2:10
    
    Link(i).A=Link(i-1).A*Link(i).A;
    Link(i).p= Link(i).A(:,4);
    Link(i).n= Link(i).A(:,1);
    Link(i).o= Link(i).A(:,2);
    Link(i).a= Link(i).A(:,3);
    Link(i).R=[Link(i).n(1:3),Link(i).o(1:3),Link(i).a(1:3)];
    Connect3D(Link(i-1).p,Link(i).p,'b',2); hold on;
    plot3(Link(i).p(1),Link(i).p(2),Link(i).p(3),'rx');hold on;
    if i<=8
        if i==5
            DrawCuboid(Link(i-1).p(1:3), Link(i-1).R * Link(i).az, len_trans, width, height, joint_col); hold on;
        else
            DrawCylinder(Link(i-1).p, Link(i-1).R * Link(i).az, radius,len, joint_col); hold on;
        end
    end
end
%DrawSpaceStationCylinder();hold on;
axis([-2500 2500 -1500 1500 -500 2500]);
xlabel('x');
ylabel('y');
zlabel('z');
%%view(107,19);
grid on;
drawnow;
if(fcla)
    cla;
end




