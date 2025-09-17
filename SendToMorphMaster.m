function SendToMorphMaster()

global Serial comPort StartByte EndByte SpecialByte 
global ReceiveBuffer BytesReceived MessageReceiveInProgress

StartByte = uint8(254);
EndByte = uint8(255);
SpecialByte = uint8(253);
BytesReceived = 0;
MessageReceiveInProgress = false;
ReceiveBuffer = uint8(zeros(1,100));
    
    
port = 'COM25';
clearSerial(port);
setupSerial(port)
n = 1;
t = 0;
% while(1)
%     Message = [];
%     for i = 1:n
%         Message = [Message typecast(cast(randi(1000*[-1 1]),'int32'),'uint8')];
%     end
%     SendToMaster(Message);
%     pause(1);
%     
%     t = t+1
% end

i = 1;
while(i == 1)
   ticks = 10000;
    Message = [typecast(cast(ticks,'int32'),'uint8')];
    SendToMaster(Message);
    pause(1);
%     i = 0
end

clearSerial(port);

function clearSerial(comPort)
    out = instrfind;
    for i = 1:length(out)
        if (out(i).Port == comPort)
            fclose(out(i));
            delete(out(i));
        end
    end

function clearAllSerial()
    out = instrfind;
    for i = 1:length(out)
            fclose(out(i));
            delete(out(i));
    end

function    [] = setupSerial(comPort)
    % Initialize Serial object
    global Serial
    Serial = serial(comPort);
    set(Serial,'DataBits',8);
    set(Serial,'StopBits',1);
    set(Serial,'BaudRate',115200);
    set(Serial,'Parity','none');
    Serial.BytesAvailableFcnCount = 1;
    Serial.BytesAvailableFcnMode = 'byte';
    Serial.BytesAvailableFcn = @SerialInterrupt;
    fopen(Serial);
    if strcmp(Serial.Status,'open')
        mbox = msgbox('Serial Communication setup'); uiwait(mbox);
    else
        mbox = msgbox('Communication not established'); uiwait(mbox);    
    end

function [] = SerialInterrupt(Serial,event)
    global StartByte EndByte MessageReceiveInProgress BytesReceived
    global ReceiveBuffer Commands Mode Ghandles ManualButtons
    NewByte = fread(Serial,1,'uint8')
    if (NewByte == StartByte)
      BytesReceived = 0; 
      MessageReceiveInProgress = true;
    elseif (NewByte == EndByte) 
      MessageReceiveInProgress = false;
      %save the number of bytes that were sent
      DecodeMessage(ReceiveBuffer);
    elseif(MessageReceiveInProgress) 
      BytesReceived = 1 + BytesReceived;
      ReceiveBuffer(BytesReceived) = NewByte;
    end
    

   
function [] = SendToMaster(Message)
    global StartByte EndByte Serial
    MessageLength = uint8(length(Message));
    EncodedMessage = EncodeMessage(Message);
    EncodedMessage = [StartByte MessageLength EncodedMessage EndByte];
    fwrite(Serial,EncodedMessage,'uint8');

function [DecodedMessage] = DecodeMessage(Message)
    global  BytesReceived
    MessageLength = Message(1);
    if MessageLength == 0  %debug
        display(native2unicode(Message));
    else
        DecodedMessage = uint8(zeros(1,MessageLength));
        i = 2;
        j = 0;
        while (i <= BytesReceived)
         if Message(i) == 253
            i = i + 1;
            NewByte = uint8(253 + Message(i));  
         else
            NewByte = Message(i);  
         end
         j = j+1;
         DecodedMessage(j) = NewByte;
         i = i + 1;
        end
    end
    
    
function [EncdodedMessage] = EncodeMessage(Message)
    global SpecialByte
    EncdodedMessage = [];
    for i = 1:length(Message)
    NextByte = Message(i);

    if NextByte >= SpecialByte
       EncdodedMessage = [EncdodedMessage uint8(SpecialByte)];
       EncdodedMessage = [EncdodedMessage uint8(NextByte - SpecialByte)];
    else
       EncdodedMessage = [EncdodedMessage uint8(NextByte)];
    end
    end