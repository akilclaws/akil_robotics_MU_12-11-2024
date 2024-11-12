clc;
clear all;
xx = 1;

L1 = 10;
L2 = 10;

m1 = 0.1;
m2 = 0.1;
I_1 = m1*L1^2;
I_2 = m2*L2^2;
r1 = 0.1;
r2 = 0.1;

a = I_1 + I_2 + m1*r1^2 + m2*(L1^2 +r1^2);
b = m2 * L1 * r2;
c = L2 + m2*r2^2;
q = [0;0];
lambda_2 = 1;
q_vel = [0;0];
lambda_1 = 1;

    % % Initialize Arduino connection
    % a = arduino('COM9', 'Mega2560', 'Libraries', 'rotaryEncoder');
    % encoder1 = rotaryEncoder(a, 'D2', 'D3');
    % encoder2 = rotaryEncoder(a, 'D18', 'D19');

for t = 0.1:0.1:5 
    
    tf = 5;
% Trajectory & velocity

    T_f =  [1 0 0 0 0 0;1 tf tf^2 tf^3 tf^4 tf^5;0 1 0 0 0 0;0 1 2*tf 3*tf^2 4*tf^3 5*tf^4;
            0 0 2 0 0 0; 0 0 2 6*tf 12*tf^2 20*tf^3];
    
    D_x = [15;0.1;0;0;0;0];  % Initial & final x position of manipulator
    D_y = [15;18;0;0;0;0];  % Initial & final y position of manipulator

    A_x = inv(T_f) * D_x;
    A_y = inv(T_f) * D_y;

    X = A_x(1) + A_x(2)*t + A_x(3)*(t^2) + A_x(4)*(t^3) + A_x(5)*(t^4) + A_x(6)*(t^5);  % X = a0 + a1*t + a2(t^2) + a3*(t^3) + a4*(t^4);
    Y = A_y(1) + A_y(2)*t + A_y(3)*(t^2) + A_y(4)*(t^3) + A_y(5)*(t^4) + A_y(6)*(t^5);  % Y = a0 + a1*t + a2(t^2) + a3*(t^3) + a4*(t^4);
    
    Traject_x(1,xx) = X;
    Traject_y(1,xx) = Y;

    X_d = A_x(2) + 2*A_x(3)*t + 3*A_x(4)*(t^2) + 4*A_x(5)*(t^3) + 5*A_x(6)*(t^4);
    Y_d = A_y(2) + 2*A_y(3)*t + 3*A_y(4)*(t^2) + 4*A_y(5)*(t^3) + 5*A_y(6)*(t^4);

    X_dd = 2*A_x(3) + 6*A_x(4)*t + 12*A_x(5)*(t^2) + 20*A_x(6)*(t^3);
    Y_dd = 2*A_y(3) + 6*A_y(4)*t + 12*A_y(5)*(t^2) + 20*A_y(6)*(t^3);

    Linear_velocity = [X_d;Y_d];
    Linear_Acceleration = [X_dd;Y_dd];
    xx = xx+1;

    plot (Traject_x,Traject_y,'.','MarkerSize',10,'Color','r');

    q2 = acosd((X^2+Y^2-L1^2-L2^2)/(2*L1*L2));
    q1 = atand(Y/X) - atand (L2 *sind(q2)/(L1+(L2*cosd(q2))));

    qd = [q1;q2];
    axis([-20 20 -20 20]);

   % Jacobian 
    J = [-L1*sind(q1)-L2*sind(q1+q2) -L2*sind(q1+q2);
      L1*cosd(q1)+L1*cosd(q1+q2) L2*cosd(q1+q2)];
    q_d = inv(J)*Linear_velocity;
    q1_d = q_d(1);
    q2_d = q_d(2);

    J_d = [-L1*cosd(q1)*q1_d-L2*cosd(q1+q2)*(q1_d+q2_d) -L2*cosd(q1+q2)*(q1_d+q2_d);
        -L1*sind(q1)*q1_d-L1*sind(q1+q2)*(q1_d+q2_d) -L2*sind(q1+q2)*(q1_d+q2_d)];
    
    q_dd = inv(J)*(X_dd - J_d*(q_d));

    % Torque control
    d1 = a + 2 * b*cosd(q2); d2 = c + b*cosd(q2); d4 = c;
    d3 = d2;

    M = [d1 d2;d3 d4];                                  % M(q)

    c1 = -m2*L2*r2*sind(q2)*q2_d;
    c2 = -m2*L2*r2*sind(q2)*(q1_d+q2_d);
    c3 = -m2*L2*r2*sind(q2)*(q1_d);
    c4 = 0;

    C = [c1 c2;c3 c4];
    
    g = 9.81;
    g1 = m2*g*(L1*cosd(q1)+r2*cosd(q1+q2)+m1*g*r1*cosd(q1));
    g2 = m2*g*r2*cosd(q1+q2);

    g_0 = [g1;g2];
   Torque_act = M*q_dd + C + g_0;

   q_err = qd - q;
   q_err_d = q_d - q_vel;
   % Corrective torque (Computed torque control)
   Torque = M*(q_dd + lambda_1*q_err_d + lambda_2*q_err) + C + g_0       % find error terms

   % Tov = g_0+J'*(F_des+k_p*(Fe));
   % Tov_1 = g_0+J'*(F_des+k_p*(Fe)-Fe*V); 
   % 

        % writeDigitalPin(a, 'D4', Feed_forward(1) >= 0);
        % writePWMDutyCycle(a, 'D5', min(abs(Feed_forward(1)/360), 1));
        % writeDigitalPin(a, 'D6', Feed_forward(2) >= 0);
        % writePWMDutyCycle(a, 'D7', min(abs(Feed_forward(2)/360), 1));
 
        % fprintf('The value of Feed_forward(1) is %d\n', Feed_forward(1))
        % fprintf('The value of Feed_forward(2) is %d\n', Feed_forward(2))
   % q_d = (q_dd*0.1) + q_d;       update error terms this wrong
   % q = (q_err_d *0.1) + q;
end