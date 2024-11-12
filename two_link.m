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

% Plot the arc trajectory
figure;
plot(x, y, 'b');
axis equal;
xlabel('X');
ylabel('Y');
title('Arc Trajectory');
hold on;

% Animation
for i = 1:num_points
    % Inverse Kinematics to get joint angles
    q = inverse_kinematics(x(i), y(i), L1, L2);
    
    % Forward Kinematics to get end effector position
    x_end = L1 * cos(q(1)) + L2 * cos(q(1) + q(2));
    y_end = L1 * sin(q(1)) + L2 * sin(q(1) + q(2));
    
    % Plot manipulator
    plot([0, L1*cos(q(1)), x_end], [0, L1*sin(q(1)), y_end], 'r-o');
    xlim([-2, 2]);
    ylim([-2, 2]);
    drawnow;
    pause(0.1); % Adjust as needed
    if i < num_points
        clf; % Clear figure for the next plot
        plot(x, y, 'b');
        axis equal;
        xlabel('X');
        ylabel('Y');
        title('Arc Trajectory');
        hold on;
    end
end

% Inverse Kinematics function
function q = inverse_kinematics(x, y, L1, L2)
    q2 = acos((x^2 + y^2 - L1^2 - L2^2) / (2 * L1 * L2));
    q1 = atan2(y, x) - atan2(L2 * sin(q2), L1 + L2 * cos(q2));
    q = [q1, q2];
end
