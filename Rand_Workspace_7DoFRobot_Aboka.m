close all;
% A(1:90)=struct('cdata',[],'colormap',[]);
clear;
ToRad=pi/180;
global Link
Build_7DOFRobot_Aboka;

th=[0,0,0,0,0,0,0];
grid on;

for j=1:30000
    th1=-180+360*rand;
    th2=-120+240*rand;
    th3=-120+240*rand;
    d4=0+1500*rand;
    th5=-180+360*rand;
    th6=-120+240*rand;
    th7=-180+360*rand;
    Link(2).th=th1*ToRad;
    Link(3).th=th2*ToRad;
    Link(4).th=th3*ToRad;
    Link(5).dz=d4;
    Link(6).th=th5*ToRad;
    Link(7).th=th6*ToRad;
    Link(8).th=th7*ToRad;
    for i=1:8
        Matrix_DH_Ln(i);
    end
    for i=2:8
        Link(i).A=Link(i-1).A*Link(i).A;
        Link(i).p= Link(i).A(:,4);
    end
    x(j)=Link(8).p(1);
    y(j)=Link(8).p(2);
    z(j)=Link(8).p(3);
end

DrawSpaceStationCylinder();hold on;
plot3(x,y,z,'r.');
axis equal;
axis([-2200 2200 -1200 1200 -300 2300]);
grid on;
xlabel('x');
ylabel('y');
zlabel('z');
    % 
    % for th1=1:1:4
    %     for th2= -110:4:110
    %         for th3=-90:4:70
    %             %             for th4=-160:50:160
    %             %                for th5=-120:60:120
    %             %                    for th6=-226:50:226
    %             Link(2).th=0*pi+th1*pi/180;
    %             Link(3).th=-0.5*pi+th2*pi/180;
    %             Link(4).th=0*pi+th3*pi/180; fprintf('%d %d %d  \n',[th1,th2,th3]');
    %             Link(5).th=0*pi/180;
    %             Link(6).th=0*pi/180;
    %             Link(7).th=0*pi/180;  fprintf('%d %d %d %d %d %d  \n',[th1,th2,th3,0,0,0]');
    %             %                         Link(4).th=th4*pi/180;
    %             %                         Link(5).th=th5*pi/180; fprintf('%d %d %d %d %d  \n',[th1,th2,th3,th4,th5]');
    %             %                         Link(6).th=th6*pi/180; fprintf('%d %d %d %d %d %d
    %             %                         \n',[th1,th2,th3,th4,th5,th6]');
    %             for i=1:7
    %                 Matrix_DH_Ln(i);
    %             end
    %             for i=2:7
    %                 Link(i).A=Link(i-1).A*Link(i).A;
    %                 Link(i).p= Link(i).A(:,4);
    %             end
    %             grid on;
    %             plot3(Link(6).p(1),Link(6).p(2),Link(6).p(3),'r*');pause(0.0001);hold on;
    %             %                     end
    %             %                 end
    %             %             end
    %         end
    %     end
    % end







    % for d1=0:10:200
    %     for th=-120:10:120
    %         for d2=0:10:100
    %             thx=th*pi/180;
    %             A1=[ 1 0 0 0;
    %                  0 1 0 0;
    %                  0 0 1 d1;
    %                  0 0 0 1];
    %             A2=[cos(thx) -sin(thx) 0 0;
    %                 sin(thx) cos(thx) 0 0;
    %                 0 0 1 0;
    %                 0 0 0 1];
    %             A3=[ 1 0 0 d2;
    %                  0 1 0 0;
    %                  0 0 1 0;
    %                  0 0 0 1];
    %
    %             p=A1*A2*A3;
    %             plot3(p(1,4),p(2,4),p(3,4)); hold on;
    %         end
    %     end
    % end







