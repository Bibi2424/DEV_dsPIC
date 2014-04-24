delete(instrfind);

s=serial('COM6');

try 
    fopen(s);
catch
    disp('COM6 not possible');
end
    

