% Define the DH parameters for the 7-DOF mechanism.
% The Link struct is shared by FK drawing and Jacobian calculations.


ToDeg = 180/pi;
ToRad = pi/180;
UX = [1 0 0]';
UY = [0 1 0]';
UZ = [0 0 1]';

Link= struct('name','Body' , 'th',  0, 'dz', 0, 'dx', 0, 'alf',90*ToRad,'az',UZ);                   % template
Link(1)= struct('name','Base' , 'th',  0*ToRad, 'dz', 0, 'dx', 0, 'alf',0*ToRad,'az',UZ);           % base frame
Link(2) = struct('name','J1' , 'th',   90*ToRad, 'dz', 700, 'dx', 0, 'alf',90*ToRad,'az',UZ);       % joint 1 -> joint 2
Link(3) = struct('name','J2' , 'th',  90*ToRad, 'dz', 0, 'dx', 700, 'alf',90*ToRad,'az',UZ);        % joint 2 -> joint 3
Link(4) = struct('name','J3' , 'th',  0*ToRad, 'dz', 80, 'dx', 0, 'alf',0*ToRad,'az',UZ);          % joint 3 -> joint 4
Link(5) = struct('name','J4' , 'th',  90*ToRad, 'dz', 20, 'dx', 0, 'alf',90*ToRad,'az',UZ);        % joint 4 -> joint 5
Link(6) = struct('name','J5' , 'th',  90*ToRad, 'dz', 0, 'dx', 400, 'alf',90*ToRad,'az',UZ);        % joint 5 -> joint 6
Link(7) = struct('name','J6' , 'th',  90*ToRad, 'dz', 0, 'dx', 400, 'alf',90*ToRad,'az',UZ);        % joint 6 -> joint 7
Link(8) = struct('name','J7' , 'th',  0*ToRad, 'dz', 100, 'dx', 0, 'alf',0*ToRad,'az',UZ);         % joint 7 -> END
Link(9) = struct('name','F1' , 'th',  0*ToRad, 'dz', 0, 'dx', 60, 'alf',0*ToRad,'az',UZ);          % F1 -> F2
Link(10) = struct('name','F2' , 'th',  0*ToRad, 'dz', 0, 'dx', -120, 'alf',0*ToRad,'az',UZ);          % F1 -> F2



                           
