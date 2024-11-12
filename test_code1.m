clc;
clear;
xx = 1;

L1 = 10;
L2 = 10;

q = [0;0];

Proportional_error1 = 0;
Proportional_error2 = 0;
prev_error1 = 0;
prev_error2 = 0;

Kp1 = 0.5;

% Initialize Arduino connection
a = arduino('COM9', 'Mega2560', 'Libraries', 'rotaryEncoder');
encoder1 = rotaryEncoder(a, 'D2', 'D3');
encoder2 = rotaryEncoder(a, 'D18', 'D19');

for t = 0.1:0.1:5 

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

    plot (Traject,Traject1,'.','MarkerSize',10,'Color','r')
    
    X_d = A_x(2) + 2*A_x(3)*t + 3*A_x(4)*(t^2);
    Y_d = A_y(2) + 2*A_y(3)*t + 3*A_y(4)*(t^2);
    
    Linear_velocity = [X_d;Y_d];
    
    q2 = acos((X^2+Y^2-L1^2-L2^2)/(2*L1*L2));
    q1 = atan(Y/X) - atan (L2 *sin(q2)/(L1+(L2*cos(q2))));

    qd = [q1;q2];
    qd = real(qd);
    
    fprintf('The value of qd(1) is %d\n', qd(1))
    fprintf('The value of qd(2) is %d\n', qd(2))
    axis([-20 20 -20 20]);

    hold on;

% To find Jacobian matrix 
J = [-L1*sin(q1)-L2*sin(q1+q2) -L2*sin(q1+q2);
      L1*cos(q1)+L1*cos(q1+q2) L2*cos(q1+q2)];
q_d = inv(J)*Linear_velocity;
% location to reach

    % Read encoder values
    [count1, ~] = readCount(encoder1);
    [count2, ~] = readCount(encoder2);

    theta1_actual = deg2rad((360 / 7392) * count1);
    theta2_actual = deg2rad((360 / 7392) * count2);
    theta_actual = [theta1_actual;theta2_actual];

    fprintf('The value of theta1 is %d\n', theta1_actual)
    fprintf('The value of theta2 is %d\n', theta2_actual)
    % Calculate errors for joint 1
    q_error = qd(1) - theta1_actual;
    Feed_forward_1 = (q_d) + Kp1*q_error;
    
    q_error = qd(1) - theta1_actual;
    Feed_forward_2 = (q_d) + Kp1*q_error;
    fprintf('The value of control1 is %d\n', Feed_forward_1)
    fprintf('The value of control2 is %d\n', Feed_forward_2)

    % Apply control signal as PWM to the motors
    writeDigitalPin(a, 'D4', Feed_forward_1 >= 0);
    writePWMDutyCycle(a, 'D5', min(abs(Feed_forward_1), 1));

    writeDigitalPin(a, 'D6', Feed_forward_2 >= 0);
    writePWMDutyCycle(a, 'D7', min(abs(Feed_forward_2), 1));

    % Update previous errors
    prev_error1 = q_error;
    prev_error2 = error2_actual;

pause(0.1)
xx = xx+1;
end

% Plot desired and actual joint angles
figure;
subplot(2,1,1);
plot(0.1:0.1:5, qd(1), 'r', 'LineWidth', 2);
hold on;
plot(0.1:0.1:5, theta1_actual, 'b', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Joint 1 Angle (rad)');
legend('qd1', 'theta1\_actual');
title('Desired vs Actual Joint 1 Angle');

subplot(2,1,2);
plot(0.1:0.1:5, qd(2), 'r', 'LineWidth', 2);
hold on;
plot(0.1:0.1:5, theta2_actual_array, 'b', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Joint 2 Angle (rad)');
legend('qd2', 'theta2\_actual');
title('Desired vs Actual Joint 2 Angle');
