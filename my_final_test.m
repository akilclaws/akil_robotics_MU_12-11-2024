clc;
clear;

xi=15;
xf=-15;
xdi=1;
xdf=2;
tf=5;
totalx=[xi;xf;xdi;xdf];
tfmatrix=[1 0 0 0;1 tf tf^2 tf^3;0 1 0 0;0 1 2*tf 3*tf^2];
ax=inv(tfmatrix)*totalx;

yi=17;
yf=17;
ydi=0;
ydf=0;
tf=5;
totaly=[yi;yf;ydi;ydf];
% tfmatrix=[1 0 0 0;1 tf tf^2 tf^3;0 1 0 0;0 1 2*tf 3*tf^2];
t=0:0.1:5;b=inv(tfmatrix)*totaly;

l1=15;
l2=15;
Kp = 2;
i_error = 0;
K_i = 0;
% Arrays to store theta_actual, q1, and q_error values
theta_actual_values = [];
q1_values = [];
q_error_values = [];

for i=1:1:length(t)
   
X(i) = ax(1,1) + ax(2,1)*t(i) + ax(3,1)*t(i)^2 + ax(4,1)*t(i)^3;
Y(i) = b(1,1) + b(2,1)*t(i) + b(3,1)*t(i)^2 + b(4,1)*t(i)^3;

end
a = arduino('COM9','Mega2560','Libraries','rotaryEncoder');
encoder1 = rotaryEncoder(a,'D2','D3');
encoder2 = rotaryEncoder(a,'D18','D19');

for i=1:1:length(t)
    %% getting positions X and Y 
     positionx(i)=  ax(1,1) + ax(2,1)*t(i) + ax(3,1)*t(i)^2 + ax(4,1)*t(i)^3;
     devpositionx(i)=ax(2,1)+ax(3,1)*t(i)*2+3*ax(4,1)*t(i)^2;
     positiony(i)= b(1,1) + b(2,1)*t(i) + b(3,1)*t(i)^2 +b(4,1)*t(i)^3;
     devpositiony(i)=b(2,1)+b(3,1)*t(i)*2+3*b(4,1)*t(i)^2;
     derivativexy=[devpositionx(i);devpositiony(i)];
        %% getting ik .. theta1 and theta2
    
     c = (X(i)^2 + Y(i)^2 - l1^2 - l2^2)/(2*l1*l2);
     s = sqrt(1-c^2);
     q1(i) = atan2(Y(i),X(i)) - atan2((l1*s),(l1 + (l2*c)));
     q2(i) = atan2(s,c);
        
     qd = [q1(i);q2(i)];
     %% getting encoder value from arduino
    [count1, ~] = readCount(encoder1);
    [count2, ~] = readCount(encoder2);

    theta1_actual = ((360 / 7392) * count1);
    theta2_actual = ((360 / 7392) * count2);
    theta_actual = [theta1_actual; theta2_actual];
    fprintf('The value of theta_1 is %d\n', theta1_actual)
    fprintf('The value of theta_2 is %d\n', theta2_actual)

%% jacobian

    [q_d,det(i)]=jacobian(l1,l2,q1(i),q2(i),derivativexy);
    q_error = real(qd - theta_actual);
    
    i_error = i_error + (q_error * 0.1);
    
    Feed_forward = real((q_d) + Kp * q_error + K_i * i_error * 0.1);              % Try using Kp & ki alone to simulate the robot to achieve desired position
    
    writeDigitalPin(a, 'D4', Feed_forward(1) >= 0);
    writePWMDutyCycle(a, 'D5', min(abs(Feed_forward(1) / 360), 1));
    
    writeDigitalPin(a, 'D6', Feed_forward(2) >= 0);
    writePWMDutyCycle(a, 'D7', min(abs(Feed_forward(2) / 360), 1));
    fprintf('The value of Feed_forward(1) is %d\n', Feed_forward(1) / 360)
    fprintf('The value of Feed_forward(2) is %d\n', Feed_forward(2) / 360)
end 

clear a;
