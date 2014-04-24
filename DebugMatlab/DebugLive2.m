%run('clean');
clear all;
close all;
 
s = serial('COM4');                 %assigns the object s to serial port
 
set(s, 'InputBufferSize', 256);     %number of bytes in inout buffer
set(s, 'FlowControl', 'hardware');
set(s, 'BaudRate', 115200);
set(s, 'Parity', 'none');
set(s, 'DataBits', 8);
set(s, 'StopBit', 1);
set(s, 'Timeout',10);
%clc;
 
disp(get(s,'Name'));
prop(1)=(get(s,'BaudRate'));
prop(2)=(get(s,'DataBits'));
prop(3)=(get(s, 'StopBit'));
prop(4)=(get(s, 'InputBufferSize'));
 
disp(['Port Setup Done!!',num2str(prop)]);
 
fopen(s);                       %opens the serial port
fscanf(s);                      %FlushSerial
t=0;
window = 100;
max=1000;
disp('Running...');
x=zeros(1,window);                 %Initialize x array
y=zeros(1,window);                 %Initialize y array
p=plot(1:window,y);                %Initial ploting
while(t < max)
 
   a = fscanf(s, '%8f %8f');  %Read from serial
 
   if t>=window                    %Initialize array
       x = [x a(1)]; 
       y = [y a(2)];
       disp_y = y(t-window+(1:window));
   else                         %Add in array
       x(t+1) = a(1);
       y(t+1) = a(2); 
       disp_y = y;
   end
  
   set(p, 'ydata', disp_y);   	%Update data
   
   axis auto;
   grid on;
   hold on;
   %disp([num2str(t+1),'th iteration max= ',num2str(max)]);
   
   t=t+1;
   a=0;                         %Clear the buffer
   drawnow;
end

%Trace entire figure at the end
figure;
plot(x,y)
axis auto;
grid on;

disp('Done');
 
fclose(s); %close the serial port