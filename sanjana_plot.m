clc;
clear;

xx = 1;

L1 = 10;
L2 = 10;

Kp = 3;
i_error = 0;
K_i = 0;
q = [0;0];

% Initialize Arduino connection
a = arduino('COM9', 'Mega2560', 'Libraries', 'rotaryEncoder');
encoder1 = rotaryEncoder(a, 'D2', 'D3');
encoder2 = rotaryEncoder(a, 'D18', 'D19');

% Arrays to store theta_actual, q1, and q_error values
theta_actual_values = [];
q1_values = [];
q_error_values = [];
count1_values = [];
count2_values = [];

tf = 5;

T_f =  [1 0 0 0;1 tf tf^2 tf^3;0 1 0 0;0 1 2*tf 3*tf^2];  % Time period of trajectory movement

D_x = [15;0.1;0;0];  % Initial & final x position of manipulator
D_y = [15;18;0;0];  % Initial & final y position of manipulator

A_x = inv(T_f) * D_x;
A_y = inv(T_f) * D_y;

% Precompute trajectory points
t_values = 0.1:0.5:5;
num_points = length(t_values);

X_values = zeros(1, num_points);
Y_values = zeros(1, num_points);
X_d_values = zeros(1, num_points);
Y_d_values = zeros(1, num_points);

for i = 1:num_points
    t = t_values(i);
    
    X_values(i) = A_x(1) + A_x(2)*t + A_x(3)*(t^2) + A_x(4)*(t^3);  % X = a0 + a1*t + a2(t^2) + a3*(t^3) + a4*(t^4);
    Y_values(i) = A_y(1) + A_y(2)*t + A_y(3)*(t^2) + A_y(4)*(t^3);  % Y = a0 + a1*t + a2(t^2) + a3*(t^3) + a4*(t^4);

    X_d_values(i) = A_x(2) + 2*A_x(3)*t + 3*A_x(4)*(t^2);
    Y_d_values(i) = A_y(2) + 2*A_y(3)*t + 3*A_y(4)*(t^2);
end

for i = 1:num_points
    t = t_values(i);
    X = X_values(i);
    Y = Y_values(i);
    X_d = X_d_values(i);
    Y_d = Y_d_values(i);

    Traject(1,xx) = X;
    Traject1(1,xx) = Y;
    
    Linear_velocity = [X_d; Y_d];

    q2 = acosd((X^2 + Y^2 - L1^2 - L2^2) / (2 * L1 * L2));
    q1 = atand(Y / X) - atand(L2 * sind(q2) / (L1 + (L2 * cosd(q2))));

    qd = [real(q1); real(q2)];
    axis([-20 20 -20 20]);

    hold on;

    % To find Jacobian matrix 
    J = [-L1 * sind(q1) - L2 * sind(q1 + q2), -L2 * sind(q1 + q2);
          L1 * cosd(q1) + L1 * cosd(q1 + q2), L2 * cosd(q1 + q2)];
    q_d = inv(J) * Linear_velocity; % Read encoder values
    fprintf('The value of q_d is %d\n', q_d)
    [count1, ~] = readCount(encoder1);
    [count2, ~] = readCount(encoder2);

    theta1_actual = ((360 / 1848) * (count1/4));
    theta2_actual = ((360 / 1848) * (count2/4));
    theta_actual = [theta1_actual; theta2_actual];
    fprintf('The value of theta_1 is %d\n', theta1_actual)
    fprintf('The value of theta_2 is %d\n', theta2_actual)
    q_error = real(qd - theta_actual);
    
    i_error = i_error + (q_error * 0.1);
    
    Feed_forward = real((q_d) + Kp * q_error + K_i * i_error * 0.1);              % Try using Kp & ki alone to simulate the robot to achieve desired position
    
    writeDigitalPin(a, 'D4', Feed_forward(1) >= 0);
    writePWMDutyCycle(a, 'D5', min(abs(Feed_forward(1) / 360), 1));
    
    writeDigitalPin(a, 'D6', Feed_forward(2) >= 0);
    writePWMDutyCycle(a, 'D7', min(abs(Feed_forward(2) / 360), 1));
    fprintf('The value of Feed_forward(1) is %d\n', Feed_forward(1) / 360)
    fprintf('The value of Feed_forward(2) is %d\n', Feed_forward(2) / 360)
    
    % Store values for plotting
    theta_actual_values = [theta_actual_values, theta_actual];
    q1_values = [q1_values, real(q1)];
    q_error_values = [q_error_values, q_error];
    
    pause(0.01);                 % make feed forward in terms of position rather than direct value of angle
    xx = xx + 1;
end

% Clear encoders
clear encoder1;
clear encoder2;

% Clear Arduino object
clear a;

% Plot theta_actual vs q1 and q_error
figure;
subplot(2,1,1);
plot(theta_actual_values(1, :), 'b', 'DisplayName', 'Theta 1 Actual');
hold on;
plot(q1_values, 'r', 'DisplayName', 'q1 Real');
legend;
title('Theta 1 Actual vs q1 Real');
xlabel('Time Step');
ylabel('Angle (degrees)');

subplot(2,1,2);
plot(q_error_values(1, :), 'g', 'DisplayName', 'q1 Error');
hold on;
plot(q_error_values(2, :), 'm', 'DisplayName', 'q2 Error');
legend;
title('q1 Error vs q2 Error');
xlabel('Time Step');
ylabel('Error (degrees)');

hold off;
