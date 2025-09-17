port = 'COM7'
clearSerial(port)
[Serial] = setupSerial(port)
t = 0;
while(0)
    
    t = mod(t+1,1000);
    fwrite(Serial,t,'uint8')
    pause(0.1)
end