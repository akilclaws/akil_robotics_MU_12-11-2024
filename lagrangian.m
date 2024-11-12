clc;

% Define the Lagrangian function
L = (1/2)*m1*l1^2*theta1_d^2 + (1/2)*m2*((l1^2*theta1_d^2) + l2^2*(theta1_d+theta2_d)^2 + 2*l1*l2*cosd(theta_2)*(theta1_d^2+theta2_d*theta1_d)) - ...
    (-m*g*l1*cosd(theta_1)-m2*g*(l1*cosd(theta_1)+l2*cosd(theta_1+theta_2)));

% Calculate partial derivatives
dL_dtheta1_d = diff(L, theta1_d) % Partial derivative with respect to theta1_d
dL_dtheta2_d = diff(L, theta2_d) % Partial derivative with respect to theta2_d
dL_dtheta1 = diff(L, theta_1) % Partial derivative with respect to theta_1
dL_dtheta2 = diff(L, theta_2) % Partial derivative with respect to theta_2

% Apply the time derivative (d/dt)
dL_dtheta1_dt = diff(dL_dtheta1_d, theta_1) * theta1_d % Time derivative with respect to theta_1
dL_dtheta2_dt = diff(dL_dtheta2_d, theta_2) * theta2_d % Time derivative with respect to theta_2
% Compute the difference
difference = simplify(dL_dtheta1_dt - dL_dtheta1)