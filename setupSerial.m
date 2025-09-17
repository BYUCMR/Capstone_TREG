function[Serial] = setupSerial(comPort)
% It accept as the entry value, the index of the serial port
% Arduino is connected to, and as output values it returns the serial 
% element obj 
% Initialize Serial object
Serial = serial(comPort);
set(Serial,'DataBits',8);
set(Serial,'StopBits',1);
set(Serial,'BaudRate',115200);
set(Serial,'Parity','none');
Serial.BytesAvailableFcnCount = 1;
Serial.BytesAvailableFcnMode = 'byte';
Serial.BytesAvailableFcn = @serialInterrupt;
fopen(Serial);
if Serial.Status == 'open'
    mbox = msgbox('Serial Communication setup'); uiwait(mbox);
else
    mbox = msgbox('Communication not established'); uiwait(mbox);    
end
end
