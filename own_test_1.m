clc;
clear;

xx = 1;

L1 = 10;
L2 = 10;

Kp = 0.85;
i_error = 0;
K_i = 0;
q = [0;0];

% Initialize Arduino connection
a = arduino('COM9', 'Mega2560', 'Libraries', 'rotaryEncoder');
encoder1 = rotaryEncoder(a, 'D2', 'D3');
encoder2 = rotaryEncoder(a, 'D18', 'D19');

for t = 1:0.5:5

    tf = 5;

    T_f =  [1 0 0 0;1 tf tf^2 tf^3;0 1 0 0;0 1 2*tf 3*tf^2];  % Time period of trajectory movement

    D_x = [15;0.1;0;0];  % Initial & final x position of manipulator
    D_y = [15;18;0;0];  % Initial & final y position of manipulator

    A_x = inv(T_f) * D_x;
    A_y = inv(T_f) * D_y;

    X = A_x(1) + A_x(2)*t + A_x(3)*(t^2) + A_x(4)*(t^3);  % X = a0 + a1*t + a2(t^2) + a3*(t^3) + a4*(t^4);
    Y = A_y(1) + A_y(2)*t + A_y(3)*(t^2) + A_y(4)*(t^3);  % Y = a0 + a1*t + a2(t^2) + a3*(t^3) + a4*(t^4);
    
    Traject(1,xx) = X;
    Traject1(1,xx) = Y;
    
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
    q_d = inv(J)*Linear_velocity; % Read encoder values
    fprintf('The value of q_d is %d\n', q_d)
    [count1, ~] = readCount(encoder1);
    [count2, ~] = readCount(encoder2);

    theta1_actual = ((360 / 1848) * (count1));
    theta1_actual =  theta1_actual/4;
    theta2_actual = ((360 / 1848) * (count2));
    theta2_actual =  theta2_actual/4;

    theta_actual = [theta1_actual;theta2_actual];
    fprintf('The value of theta_1 is %d\n', theta1_actual)
    fprintf('The value of theta_2 is %d\n', theta2_actual)
    q_error = real(qd - theta_actual);
    fprintf('The value of q_error(1) is %d\n', q_error(1))
    fprintf('The value of q_error(2) is %d\n', q_error(2))
    
    i_error = i_error + (q_error*0.1);

    Feed_forward = real((q_d) + Kp*q_error+K_i* i_error*0.1)              % Try using Kp & ki alone to simualite the robot to achieve desired position
    % Feed_forward_P = Kp*q_error;
    % Feed_forward_I = K_i* i_error*0.1;
    % Feed_forward_PI = Kp*q_error + K_i* i_error;
    % KK = plot([0, L1 * cosd(Feed_forward(1)), L1*cosd(Feed_forward(1))+L2*cosd(Feed_forward(1)+Feed_forward(2))], [0, L1 * sind(Feed_forward(1)), L1*sind(Feed_forward(1))+L2*sind(Feed_forward(1)+Feed_forward(2))], '-o', 'LineWidth', 2);
    % LL = plot([0, L1 * cosd(P_I(1)), L1*cosd(P_I(1))+L2*cosd(P_I(1)+P_I(2))], [0, L1 * sind(P_I(1)), L1*sind(P_I(1))+L2*sind(P_I(1)+P_I(2))], '-o', 'LineWidth', 4);
    % MM = plot([0, L1 * cosd(q(1)), L1*cosd(q(1))+L2*cosd(q(1)+q(2))], [0, L1 * sind(q(1)), L1*sind(q(1))+L2*sind(q(1)+q(2))], '-o', 'LineWidth', 2);
    % NN = plot([0, L1 * cosd(q(1)), L1*cosd(q(1))+L2*cosd(q(1)+q(2))], [0, L1 * sind(q(1)), L1*sind(q(1))+L2*sind(q(1)+q(2))], '-o', 'LineWidth', 2);
    
        writeDigitalPin(a, 'D4', Feed_forward(1) >= 0);
        writePWMDutyCycle(a, 'D5', min((Feed_forward(1)/360), 1));
    
        writeDigitalPin(a, 'D6', Feed_forward(2) <= 0);
        writePWMDutyCycle(a, 'D7', min((Feed_forward(2)/360), 1));

        fprintf('The value of Feed_forward(1) is %d\n', Feed_forward(1)/360)
        fprintf('The value of Feed_forward(2) is %d\n', Feed_forward(2)/360)
    
    pause(0.1);                 % make feed forward in terms of position rather than direct value of angle
    % delete (KK);
    % delete(LL);
    % delete (ZZ)
    xx = xx+1;
 end
    
    % Clear encoders
    clear encoder1;
    clear encoder2;
    
    % Clear Arduino object
    clear a;