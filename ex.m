clc;
clear all;
xx = 1;

L1 = 10;
L2 = 10;

Kp = 2.5;
i_error = 0;
K_i = 0.5;
K_d = 10;
q = [1;0];

for t = 0.1:0.1:5  
    tf = 5;
    T_f =  [1 0 0 0;1 tf tf^2 tf^3;0 1 0 0;0 1 2*tf 3*tf^2];  % Time period of trajectory movement

    D_x = [15;0.1;0;0];  % Initial & final x position of manipulator
    D_y = [15;18;0;0];  % Initial & final y position of manipulator

    A_x = inv(T_f) * D_x;
    A_y = inv(T_f) * D_y;

    X = A_x(1) + A_x(2)*t + A_x(3)*(t^2) + A_x(4)*(t^3);  % X = a0 + a1*t + a2(t^2) + a3*(t^3) + a4*(t^4);
    Y = A_y(1) + A_y(2)*t + A_y(3)*(t^2) + A_y(4)*(t^3);  % Y = a0 + a1*t + a2(t^2) + a3*(t^3) + a4*(t^4);
    
    Trajectx(1,xx) = X;
    Trajecty(1,xx) = Y;
    
    X_d = A_x(2) + 2*A_x(3)*t + 3*A_x(4)*(t^2);
    Y_d = A_y(2) + 2*A_y(3)*t + 3*A_y(4)*(t^2);
    
    Linear_velocity = [X_d;Y_d];

    q2 = acosd((X^2+Y^2-L1^2-L2^2)/(2*L1*L2));
    q1 = atand(Y/X) - atand (L2 *sind(q2)/(L1+(L2*cosd(q2))));

    qd_1 = [q1;q2];

    % To find Jacobian matrix 
    J = [-L1*sind(q1)-L2*sind(q1+q2) -L2*sind(q1+q2);
          L1*cosd(q1)+L1*cosd(q1+q2) L2*cosd(q1+q2)];
    q_d = inv(J)*Linear_velocity;

    % Calculate errors
    q_error = qd_1 - q;
    i_error = i_error + (q_error*0.1);
    d_error = -q_d;

    Feed_forward_P = (q_d) + Kp*q_error;        
    Feed_forward_I = K_i * i_error;
    Feed_forward_D = K_d*d_error;
    Feed_forward_PI = Kp*q_error + K_i* i_error;

    q = (Feed_forward_P * 0.1)+q;
    I = (Feed_forward_I*0.1)+q;
    D = (Feed_forward_D*0.1)+q;
    P_I = (Feed_forward_PI*0.1)+q;

    Final_theta_q = q;
    Final_theta_I = I;
    Final_theta_D = D;
    Final_theta_PI = P_I;

    P_linkx = L1 * cosd(q(1));
    P_linky = L1 * sind(q(1));
    P_End_Effectorx = L1*cosd(q(1))+L2*cosd(q(1)+q(2));
    P_End_Effectory = L1*sind(q(1))+L2*sind(q(1)+q(2));

    I_linkx = L1 * cosd(I(1));
    I_linky = L1 * sind(I(1));
    I_End_Effectorx = L1*cosd(I(1))+L2*cosd(I(1)+I(2));
    I_End_Effectory = L1*sind(I(1))+L2*sind(I(1)+I(2));

    D_linkx = L1 * cosd(D(1));
    D_linky = L1 * sind(D(1));
    D_End_Effectorx = L1*cosd(D(1))+L2*cosd(D(1)+D(2));
    D_End_Effectory = L1*sind(D(1))+L2*sind(D(1)+D(2));

    P_I_linkx = L1 * cosd(P_I(1));
    P_I_linky = L1 * sind(P_I(1));
    P_I_End_Effectorx = L1*cosd(P_I(1))+L2*cosd(P_I(1)+P_I(2));
    P_I_End_Effectory = L1*sind(P_I(1))+L2*sind(P_I(1)+P_I(2));

    subplot(2,2,1)
    axis([-20 20 -20 20]);
    plot (Trajectx,Trajecty,'.','MarkerSize',10,'Color','r')
    hold on;
    KK = plot([0, P_linkx, P_End_Effectorx], [0, P_linky, P_End_Effectory], '-o', 'LineWidth', 2);
    title('Proportional Control');
    text(-18, 18, sprintf('Kp = %.2f', Kp), 'FontSize', 12, 'Color', 'b');

    subplot(2,2,2)
    axis([-20 20 -20 20]);
    plot (Trajectx,Trajecty,'.','MarkerSize',10,'Color','r')
    hold on;
    MM = plot([0, I_linkx, I_End_Effectorx], [0, I_linky, I_End_Effectory], '-o', 'LineWidth', 2);
    title('Integral Control');
    text(-18, 16, sprintf('Ki = %.2f', K_i), 'FontSize', 12, 'Color', 'b');

    subplot(2,2,3)
    axis([-20 20 -20 20]);
    plot (Trajectx,Trajecty,'.','MarkerSize',10,'Color','r')
    hold on;
    NN = plot([0, D_linkx, D_End_Effectorx], [0, D_linky, D_End_Effectory], '-o', 'LineWidth', 2);
    title('Differential Control');
    text(-18, 16, sprintf('Kd = %.2f', K_d), 'FontSize', 12, 'Color', 'b');

    subplot(2,2,4)
    axis([-20 20 -20 20]);
    plot (Trajectx,Trajecty,'.','MarkerSize',10,'Color','r')
    hold on;
    LL = plot([0, P_I_linkx, P_I_End_Effectorx], [0, P_I_linky, P_I_End_Effectory], '-o', 'LineWidth', 2);
    title('P & I Control');
    text(-18, 16, sprintf('Ki = %.2f', K_i), 'FontSize', 12, 'Color', 'b');
    text(-18, 18, sprintf('Kp = %.2f', Kp), 'FontSize', 12, 'Color', 'b');
    
    pause(0.01);
    delete(KK);
    delete(LL);
    delete(MM);
    delete(NN);
    
    % Store errors
    Traject3(1,xx) = real(P_End_Effectorx);
    Traject3(2,xx) = real(P_End_Effectory);
    Traject3(3,xx) = real(I_End_Effectorx);
    Traject3(4,xx) = real(I_End_Effectory);
    Traject3(5,xx) = real(D_End_Effectorx);
    Traject3(6,xx) = real(D_End_Effectory);
    Traject3(7,xx) = real(P_I_End_Effectorx);
    Traject3(8,xx) = real(P_I_End_Effectory);    
 
    xx = xx+1;
end

% Plot errors
figure;
subplot(2,5,1)
axis([-20 20 -20 20]);
plot(Trajectx(1,:),'g','LineWidth',2);
title('Actual Position X');
subplot(2,5,2);
axis([-20 20 -20 20]);
plot(Traject3(1,:),'r','LineWidth',2);
title('P End position X');

subplot(2,5,6)
axis([-20 20 -20 20]);
plot(Trajecty(1,:),'g','LineWidth',2);
title('Actual Position Y');
subplot(2,5,7);
axis([-20 20 -20 20]);
plot(Traject3(2,:),'r','LineWidth',2);
title('P End position Y');

subplot(2,5,4)
axis([-20 20 -20 20]);
plot(Traject3(5,:),'r','LineWidth',2);
title('D End position X');
subplot(2,5,9);
axis([-20 20 -20 20]);
plot(Traject3(6,:),'r','LineWidth',2);
title('D End position Y');

subplot(2,5,3)
axis([-20 20 -20 20]);
plot(Traject3(3,:),'r','LineWidth',2);
title('I End position X');
subplot(2,5,8);
axis([-20 20 -20 20]);
plot(Traject3(4,:),'r','LineWidth',2);
title('I End position Y');

subplot(2,5,5)
axis([-20 20 -20 20]);
plot(Traject3(7,:),'r','LineWidth',2);
title('P & I End position X');
subplot(2,5,10);
axis([-20 20 -20 20]);
plot(Traject3(8,:),'r','LineWidth',2);
title('P & I End position Y');

    E_Px = Trajectx - real(P_End_Effectorx);
    E_Py = Trajecty - real(P_End_Effectory);
    E_Ix = Trajectx - real(I_End_Effectorx);
    E_Iy = Trajecty - real(I_End_Effectory);
    E_Dx = Trajectx - real(D_End_Effectorx);
    E_Dy = Trajecty - real(D_End_Effectory);
    E_PIx = Trajectx - real(P_I_End_Effectorx);
    E_PIy = Trajecty - real(P_I_End_Effectory);    
 
    % Plot errors
figure;
subplot(2,5,1)
axis([-20 20 -20 20]);
plot(E_Px(1,:),'g','LineWidth',2);
title('Error in P along x');
subplot(2,5,5);
axis([-20 20 -20 20]);
plot(E_Py(1,:),'r','LineWidth',2);
title('Error in P along y');

subplot(2,5,2)
axis([-20 20 -20 20]);
plot(E_Ix(1,:),'g','LineWidth',2);
title('Error in I along x');
subplot(2,5,6);
axis([-20 20 -20 20]);
plot(E_Iy(1,:),'r','LineWidth',2);
title('Error in I along y');

subplot(2,5,3)
axis([-20 20 -20 20]);
plot(E_Dx(1,:),'g','LineWidth',2);
title('Error in D along x');
subplot(2,5,7);
axis([-20 20 -20 20]);
plot(E_Dy(1,:),'r','LineWidth',2);
title('Error in D along y');

subplot(2,5,4)
axis([-20 20 -20 20]);
plot(E_PIx(1,:),'g','LineWidth',2);
title('Error in PI along x');
subplot(2,5,8);
axis([-20 20 -20 20]);
plot(E_PIy(1,:),'r','LineWidth',2);
title('Error in PI along y');

    E_P = Final_theta_q - qd_1;
    P_Errorx = E_P(1);
    P_Errory = E_P(2);
    E_I = Final_theta_I - qd_1;
    P_Errorx = E_I(1);
    P_Errory = E_I(2);
    E_D = Final_theta_D - qd_1; 
    P_Errorx = E_D(1);
    P_Errory = E_D(2);
    E_PI = Final_theta_PI - qd_1; 
    P_Errorx = E_PI(1);
    P_Errory = E_PI(2);
    % Plot errors
figure;
subplot(2,5,1)
axis([-20 20 -20 20]);
plot(E_P,'g','LineWidth',2);
title('Error in P along x');
subplot(2,5,5);
axis([-20 20 -20 20]);
plot(E_P,'r','LineWidth',2);
title('Error in P along y');

subplot(2,5,2)
axis([-20 20 -20 20]);
plot(E_I,'g','LineWidth',2);
title('Error in I along x');
subplot(2,5,6);
axis([-20 20 -20 20]);
plot(E_I,'r','LineWidth',2);
title('Error in I along y');

subplot(2,5,3)
axis([-20 20 -20 20]);
plot(E_D,'g','LineWidth',2);
title('Error in D along x');
subplot(2,5,7);
axis([-20 20 -20 20]);
plot(E_D,'r','LineWidth',2);
title('Error in D along y');

subplot(2,5,4)
axis([-20 20 -20 20]);
plot(E_PI,'g','LineWidth',2);
title('Error in PI along x');
subplot(2,5,8);
axis([-20 20 -20 20]);
plot(E_PI,'r','LineWidth',2);
title('Error in PI along y');
