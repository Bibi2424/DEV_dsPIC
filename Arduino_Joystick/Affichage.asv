%Affichage du joystick

%Ouverture du port COM

mySerial = serial('COM4');
fclose(mySerial);
set(mySerial, 'InputBufferSize', 256);     %number of bytes in inout buffer
set(mySerial, 'BaudRate', 115200);
set(mySerial, 'Parity', 'none');
set(mySerial, 'DataBits', 8);
set(mySerial, 'StopBit', 1);
set(mySerial, 'Timeout',10);

error = 0;
try
    fopen(mySerial);
catch err
    error = 1;
end

if error == 1
    disp('Probleme d ouverture du port COM');
else
    flushinput(mySerial);
    plot([0 1023 1023 0], [0 0 1023 1023]);
    while(1)
        in = fscanf(mySerial, '%8f %8f %8f');
        plot([0 1023 1023 0], [0 0 1023 1023], in(1), in(2), 'x');
        drawnow;
    end

end