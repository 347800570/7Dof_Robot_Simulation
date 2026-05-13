function Matrix_DH_Ln(i)  
% Compute the local DH transform for Link(i) and store it back into Link.
global Link

ToDeg = 180/pi;
ToRad = pi/180;


C=cos(Link(i).th);
S=sin(Link(i).th);
Ca=cos(Link(i).alf);
Sa=sin(Link(i).alf);
a=Link(i).dx;    % DH a parameter: translation along x
d=Link(i).dz;    % DH d parameter: translation along z

% Build the frame axes and origin in homogeneous form.
Link(i).n=[C,S,0,0]';
Link(i).o=[-1*S*Ca,C*Ca,Sa,0]';
Link(i).a=[S*Sa, -1*C*Sa,Ca,0]';
Link(i).p=[a*C,a*S,d,1]';

% R is the rotation block, A is the full homogeneous transform.
Link(i).R=[Link(i).n(1:3),Link(i).o(1:3),Link(i).a(1:3)];
Link(i).A=[Link(i).n,Link(i).o,Link(i).a,Link(i).p];

