% Define parameters
L1 = 1; % Length of link 1
L2 = 1; % Length of link 2
center = [0, 0]; % Center of the arc
radius = 1; % Radius of the arc
theta_start = 0; % Starting angle of the arc
theta_end = pi/2; % Ending angle of the arc
num_points = 100; % Number of points on the arc

% Generate arc trajectory
theta = linspace(theta_start, theta_end, num_points);
x = center(1) + radius * cos(theta);
y = center(2) + radius * sin(theta);

% Communicate with Arduino
arduino = serialport("COM3", 9600); % Change COM3 to the port your Arduino is connected to

% Send trajectory points to Arduino
for i = 1:num_points
    % Inverse Kinematics to get joint angles
    q = inverse_kinematics(x(i), y(i), L1, L2);
    
    % Send joint angles to Arduino
    write(arduino, sprintf('%d %d\n', round(rad2deg(q(1))), round(rad2deg(q(2)))));
    
    % Pause for Arduino to process
    pause(0.1); % Adjust as needed
end

% Close serial port
delete(arduino);
clear arduino;

% Inverse Kinematics function
function q = inverse_kinematics(x, y, L1, L2)
    q2 = acos((x^2 + y^2 - L1^2 - L2^2) / (2 * L1 * L2));
    q1 = atan2(y, x) - atan2(L2 * sin(q2), L1 + L2 * cos(q2));
    q = [q1, q2];
end
