% a = arduino();
% writeDigitalPin(a, 'D12', 1);
% pause(2);
%    writeDigitalPin(a, 'D12', 0);


   %give the board name and usb port name, and interrupt pins
   clc;
   clear;






   a = arduino('COM7','Mega2560','Libraries','rotaryEncoder');
   true=1;
   encoder1 = rotaryEncoder(a,'D2','D3');
   encoder2 = rotaryEncoder(a,'D18','D19');
   while(true)
       
           writeDigitalPin(a,'D4',0);
           writePWMDutyCycle(a,'D5',0.1);
           [count,time] = readCount(encoder1)
           
          % writeDigitalPin(a,'D4',1);
           % writePWMDutyCycle(a,'D5',0.1);
           % pause(2);
           encodervalue=count;
           degree=(360/7392)*encodervalue

           

           writeDigitalPin(a,'D6',1);
           writePWMDutyCycle(a,'D7',0.1);
           [count2,time2] = readCount(encoder2)
           
          % writeDigitalPin(a,'D4',1);
           % writePWMDutyCycle(a,'D5',0.1);
           % pause(2);
           encodervalue2=count2;
           degree2=(360/7392)*encodervalue2

           (degree2>=90 && degree>=90)
               true=0;
               writePWMDutyCycle(a,'D5',0);
               writePWMDutyCycle(a,'D7',0);

   end