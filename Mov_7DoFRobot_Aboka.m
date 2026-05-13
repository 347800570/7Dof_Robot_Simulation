close all;
clear;
global Link


ToDeg = 180/pi;
ToRad = pi/180;

th1=90;
th2=90;
th3=0;
d4=100;
th5=90;
th6=90;
th7=0;

DHfk_7DoFRobot_Aboka(th1,th2,th3,d4,th5,th6,th7,0);
view(107,19);
pause;
stp=30;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Joint1
for i=0:stp:360
   DHfk_7DoFRobot_Aboka(th1+i,th2,th3,d4,th5,th6,th7,1);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Joint2
for i=0:stp:360
   DHfk_7DoFRobot_Aboka(th1,th2+i,th3,d4,th5,th6,th7,1);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Joint3
for i=0:stp:360
   DHfk_7DoFRobot_Aboka(th1,th2,th3+i,d4,th5,th6,th7,1);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Joint4
for i=0:stp:360
   DHfk_7DoFRobot_Aboka(th1,th2,th3,d4+i,th5,th6,th7,1);
end
for i=360:-stp:0
   DHfk_7DoFRobot_Aboka(th1,th2,th3,d4+i,th5,th6,th7,1);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Joint5
for i=0:stp:360
   DHfk_7DoFRobot_Aboka(th1,th2,th3,d4,th5+i,th6,th7,1);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Joint6
for i=0:stp:360
   DHfk_7DoFRobot_Aboka(th1,th2,th3,d4,th5,th6+i,th7,1);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Joint7
for i=0:stp:360
   DHfk_7DoFRobot_Aboka(th1,th2,th3,d4,th5,th6,th7+i,1);
end

