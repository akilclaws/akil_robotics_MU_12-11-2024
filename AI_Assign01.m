clc;
clear all;
xx = 1;

L1 = 10;
L2 = 10;

Kp = 2;
i_error = 0;
K_i = 0.5;
q = [0;0];

for t = 0.1:0.1:5 

    tf = 5;

    T_f =  [1 0 0 0;
            1 tf tf^2 tf^3;
            0 1 0 0;
            0 1 2*tf 3*tf^2];  % Time period of trajectory movement

    D_x = [15;0.1;0;0];  % Initial & final x position of manipulator
    D_y = [15;18;0;0];  % Initial & final y position of manipulator

    A_x = inv(T_f) * D_x;
    A_y = inv(T_f) * D_y;

    X = A_x(1) + A_x(2)*t + A_x(3)*(t^2) + A_x(4)*(t^3);  % X = a0 + a1*t + a2(t^2) + a3*(t^3) + a4*(t^4);
    Y = A_y(1) + A_y(2)*t + A_y(3)*(t^2) + A_y(4)*(t^3);  % Y = a0 + a1*t + a2(t^2) + a3*(t^3) + a4*(t^4);
    
    Traject(1,xx) = X;
    Traject1(1,xx) = Y;

    plot (Traject,Traject1,'.','MarkerSize',10,'Color','r')
    
    X_d = A_x(2) + 2*A_x(3)*t + 3*A_x(4)*(t^2);
    Y_d = A_y(2) + 2*A_y(3)*t + 3*A_y(4)*(t^2);
    
    Linear_velocity = [X_d;Y_d];

    q2 = acosd((X^2+Y^2-L1^2-L2^2)/(2*L1*L2));
    q1 = atand(Y/X) - atand (L2 *sind(q2)/(L1+(L2*cosd(q2))));

    qd = [real(q1);real(q2)];
    axis([-20 20 -20 20]);

    hold on;

% To find Jacobian matrix 
J = [-L1*sind(q1)-L2*sind(q1+q2) -L2*sind(q1+q2);
      L1*cosd(q1)+L1*cosd(q1+q2) L2*cosd(q1+q2)];
q_d = inv(J)*Linear_velocity;
% location to reach

q_error = real(qd - q);

i_error = i_error + (q_error*0.1);

Feed_forward = (q_d) + Kp*q_error;              % Try using Kp & ki alone to simualite the robot to achieve desired position
% Feed_forward_P = Kp*q_error;
Feed_forward_I = K_i* i_error*0.1;
Feed_forward_PI = Kp*q_error + K_i* i_error;

q = real((Feed_forward * 0.1)+q);
I = Feed_forward_I;
P_I = (Feed_forward_PI*0.1)+q;
Traject2(1,xx) = Feed_forward(1);
Traject2(2,xx) = Feed_forward(2);
Traject2(3,xx) = q(1);
Traject2(4,xx) = q(2);
Traject2(5,xx) = P_I(1);
Traject2(6,xx) = P_I(2);

KK = plot([0, L1 * cosd(q(1)), L1*cosd(q(1))+L2*cosd(q(1)+q(2))], [0, L1 * sind(q(1)), L1*sind(q(1))+L2*sind(q(1)+q(2))], '-o', 'LineWidth', 2);
%LL = plot([0, L1 * cosd(P_I(1)), L1*cosd(P_I(1))+L2*cosd(P_I(1)+P_I(2))], [0, L1 * sind(P_I(1)), L1*sind(P_I(1))+L2*sind(P_I(1)+P_I(2))], '-o', 'LineWidth', 4);
% MM = plot([0, L1 * cosd(q(1)), L1*cosd(q(1))+L2*cosd(q(1)+q(2))], [0, L1 * sind(q(1)), L1*sind(q(1))+L2*sind(q(1)+q(2))], '-o', 'LineWidth', 2);
% NN = plot([0, L1 * cosd(q(1)), L1*cosd(q(1))+L2*cosd(q(1)+q(2))], [0, L1 * sind(q(1)), L1*sind(q(1))+L2*sind(q(1)+q(2))], '-o', 'LineWidth', 2);
pause(0.1);                 % make feed forward in terms of position rather than direct value of angle
delete (KK);
%delete(LL);
% delete (ZZ)
xx = xx+1;
end