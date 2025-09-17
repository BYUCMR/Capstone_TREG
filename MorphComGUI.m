function varargout = MorphComGUI(varargin)
% MORPHCOMGUI MATLAB code for MorphComGUI.fig
%      MORPHCOMGUI, by itself, creates a new MORPHCOMGUI or raises the existing
%      singleton*.
%
%      H = MORPHCOMGUI returns the handle to a new MORPHCOMGUI or the handle to
%      the existing singleton*.
%
%      MORPHCOMGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MORPHCOMGUI.M with the given input arguments.
%
%      MORPHCOMGUI('Property','Value',...) creates a new MORPHCOMGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before MorphComGUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to MorphComGUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help MorphComGUI

% Last Modified by GUIDE v2.5 25-Mar-2020 12:02:46

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @MorphComGUI_OpeningFcn, ...
                   'gui_OutputFcn',  @MorphComGUI_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT
end

% --- Executes just before MorphComGUI is made visible.
function MorphComGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to MorphComGUI (see VARARGIN)

% Choose default command line output for MorphComGUI
handles.output = hObject;

e=[1 2 3 4 5 6];
setappdata(0,'evalue',e);

setappdata(0,'Pos',zeros(4,2));

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes MorphComGUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


global Serial comPort StartByte EndByte SpecialByte Commands
global ReceiveBuffer BytesReceived MessageReceiveInProgress
global NodeNumber RollerNumber NodePositions
global TicksPerInch
global toggle
global toggle_control

toggle = -1;
toggle_control=-1;

Commands.Zero = 100;
Commands.SetDesiredPositionAll = Commands.Zero + 1;
Commands.ResetRadio = Commands.Zero + 2;
Commands.SetCurrentPosition = Commands.Zero + 3;
Commands.ReadCurrentPosition = Commands.Zero + 4;
Commands.SetDesiredPosition = Commands.Zero + 5;


gear_ratio_motor = 139.138;
gear_ratio_gear_train = 0.5/1.5;
roller_diameter = 0.29;
counts_per_revolution = 48;
edge_count_multiplier = 2;

% TicksPerInch = counts_per_revolution*gear_ratio_motor*gear_ratio_gear_train*edge_count_multiplier*(pi*roller_diameter);
TicksPerInch=890*2.54;  %Experimentally determined ticks/inch for Linguica
NodeNumber = 4;
RollerNumber = 2;
NodePositions = zeros(NodeNumber,RollerNumber);

ColumnNames = {'A Positions [in]' 'B Positions [in]'}
%RowNames = 0:10;
RowNames = {'Kielbasa 0' 'Polish 1' 'Chorizo 2' 'Linguica 3'};
set(handles.NodePositions, 'Data', cell(NodeNumber,RollerNumber));
set(handles.NodePositions, 'ColumnName', ColumnNames(1:RollerNumber));
set(handles.NodePositions, 'RowName', RowNames(1:NodeNumber));
for i = 1:NodeNumber
    for j = 1:RollerNumber
        handles.NodePositions.Data{i,j} = num2str(NodePositions(i,j));
    end
end

StartByte = uint8(254);
EndByte = uint8(255);
SpecialByte = uint8(253);
BytesReceived = 0;
MessageReceiveInProgress = false;
ReceiveBuffer = uint8(zeros(1,100));
    
comPort = 'COM3';  %'COM27';
set(handles.com_port_text_box,'String',comPort);



clearAllSerial();
end

% --- Outputs from this function are returned to the command line.
function varargout = MorphComGUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;
end


function clearSerial(comPort)
    out = instrfind;
    for i = 1:length(out)
        if (out(i).Port == comPort)
            fclose(out(i));
            delete(out(i));
        end
    end
end

function clearAllSerial()
    out = instrfind;
    for i = 1:length(out)
            fclose(out(i));
            delete(out(i));
    end
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
end

function [] = SerialInterrupt(Serial,event)
    global StartByte EndByte MessageReceiveInProgress BytesReceived
    global ReceiveBuffer 
    while (Serial.BytesAvailable)
        NewByte = fread(Serial,1,'uint8');
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
    end
end
   
function [] = SendToMaster(Message)
    global StartByte EndByte Serial
    MessageLength = uint8(length(Message));
    EncodedMessage = EncodeMessage(Message);
    EncodedMessage = [StartByte MessageLength EncodedMessage EndByte];
    fwrite(Serial,EncodedMessage,'uint8');
end

function [DecodedMessage] = DecodeMessage(Message)
    global  BytesReceived TicksPerInch
    MessageLength = Message(1);
    %Right now only using this case.  Never passing something besides 0 to
    %start the message
%     disp(MessageLength) 
    %disp('Message Received')
    DISPLAY_MESSAGE=0;
    
    switch MessageLength
        case  0
            display(native2unicode(Message(1:BytesReceived)));
        case  1  %Translate the rest of the message. Ticks 
            if DISPLAY_MESSAGE
             display(native2unicode(Message(2:BytesReceived)));
            end
            %Need to store these as some sort of value here. 
            Mess_Char=native2unicode(Message(2:BytesReceived));
            Display_Ticks=str2num(Mess_Char)/TicksPerInch; %Convert this back into inches
            %How to index through all of these things? 
            setappdata(0,'Pos',Display_Ticks);
%             disp(Display_Ticks)
        case  2  %Orientation i
            if DISPLAY_MESSAGE
                display(native2unicode(Message(2:BytesReceived)));
            end
            %Need to store these as some sort of value here. 
            Mess_Char=native2unicode(Message(2:BytesReceived));
            Display_Ticks=str2num(Mess_Char);
            %How to index through all of these things? 
            setappdata(0,'Orient_i',Display_Ticks);
%             disp(Display_Ticks)
        case  3  %Orientation j 
            if DISPLAY_MESSAGE
                display(native2unicode(Message(2:BytesReceived)));
            end
            %Need to store these as some sort of value here.
            Mess_Char=native2unicode(Message(2:BytesReceived));
            Meas_j=str2num(Mess_Char);
            %How to index through all of these things? 
            setappdata(0,'Orient_j',Meas_j);
%             disp(Meas_j)
        case 4  %Orientation k
            if DISPLAY_MESSAGE
                display(native2unicode(Message(2:BytesReceived)));
            end
            %Need to store these as some sort of value here.
            Mess_Char=native2unicode(Message(2:BytesReceived));
            Meas_k=str2num(Mess_Char);
            %How to index through all of these things? 
            setappdata(0,'Orient_k',Meas_k);
%             disp(Meas_k)
        case 5  %Last Value?
            if DISPLAY_MESSAGE
                display(native2unicode(Message(2:BytesReceived)));
            end
            %Need to store these as some sort of value here.
            Mess_Char=native2unicode(Message(2:BytesReceived));
            Meas_k=str2num(Mess_Char);
            %How to index through all of these things? 
            setappdata(0,'Orient_k',Meas_k);
%             disp(Meas_k)
        otherwise  
            display(native2unicode(Message(1:BytesReceived)));
    end
%     if MessageLength == 0  %debug
% %         Mess_Char=native2unicode(Message(1:BytesReceived));
% %         display(Mess_Char)
% %         disp('Num')
% %         Ticks=str2num(Mess_Char)
%         display(native2unicode(Message(1:BytesReceived)));
%         
%     else
%         DecodedMessage = uint8(zeros(1,MessageLength));
%         i = 2;
%         j = 0;
%         while (i <= BytesReceived)
%          if Message(i) == 253
%             i = i + 1;
%             NewByte = uint8(253 + Message(i));  
%          else
%             NewByte = Message(i);  
%          end
%          j = j+1;
%          DecodedMessage(j) = NewByte;
%          i = i + 1;
%         end
%     end
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
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%       Button Functions       %%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function com_port_text_box_Callback(hObject, eventdata, handles)
% hObject    handle to com_port_text_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of com_port_text_box as text
%        str2double(get(hObject,'String')) returns contents of com_port_text_box as a double
global comPort
comPort = hObject.String;
end

% --- Executes during object creation, after setting all properties.
function com_port_text_box_CreateFcn(hObject, eventdata, handles)
% hObject    handle to com_port_text_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end


% --- Executes on button press in connect_button.
function connect_button_Callback(hObject, eventdata, handles)
% hObject    handle to connect_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global comPort
    if strcmp(hObject.String,'Connect')
        clearSerial(comPort);
        setupSerial(comPort);
        hObject.String = 'Disconnect';
    else 
        clearSerial(comPort);
        hObject.String = 'Connect';
    end
end 

% --- Executes on button press in send_button.
function send_button_Callback(hObject, eventdata, handles)
% hObject    handle to send_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%This is the function that I need to execute at a certain frequency. Timing
%of this all may be tricky. 

global NodeNumber RollerNumber TicksPerInch NodePositions Commands
Message = [Commands.SetDesiredPositionAll];
NodePositions(2,2)=rand(1)*100; %Radmon command to make it a new message
Command_from_controller=getappdata(0,'Command_from_controller')

%Commented out to allow the GUI to function properly?
NodePositions(2,1)=Command_from_controller(1);
NodePositions(4,1)=Command_from_controller(2);

%Typecasting as an int, not correct

for i = 1:NodeNumber
    for j = 1:RollerNumber
        %Position Control
%         Ticks = cast(TicksPerInch*NodePositions(i,j),'int32');
        %Velocity Control
        Ticks = cast(NodePositions(i,j),'int32');
        Message = [Message typecast(Ticks,'uint8')];
    end
end

% NodePositions

% PlotRobot();


SendToMaster(Message);
end


% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global comPort
if ~isempty(comPort)
    clearSerial(comPort);
end
% Hint: delete(hObject) closes the figure
delete(hObject);
end


% --- Executes when entered data in editable cell(s) in NodePositions.
function NodePositions_CellEditCallback(hObject, eventdata, handles)
% hObject    handle to NodePositions (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.CONTROL.TABLE)
%	Indices: row and column indices of the cell(s) edited
%	PreviousData: previous data for the cell(s) edited
%	EditData: string(s) entered by the user
%	NewData: EditData or its converted form set on the Data property. Empty if Data was not changed
%	Error: error string when failed to convert EditData to appropriate value for Data
% handles    structure with handles and user data (see GUIDATA)
global NodePositions
for i = 1:size(eventdata.Indices,1)
    row = eventdata.Indices(i,1);
    col = eventdata.Indices(i,2);
    temp = str2num(hObject.Data{row,col});
    if isempty(temp)
        hObject.Data{row,col} = num2str(NodePositions(row,col));
    else
        NodePositions(row,col) = temp;
    end
end
end


% --- Executes on button press in CycleButton.
function CycleButton_Callback(hObject, eventdata, handles)
% hObject    handle to CycleButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global NodeNumber RollerNumber TicksPerInch  toggle Commands
    tic
while(handles.ContinueCycle.Value)
    display(toc)
    toggle = toggle * -1;
    Message = [Commands.SetDesiredPositionAll];
    for i = 1:NodeNumber
        for j = 1:RollerNumber
            Ticks = cast(TicksPerInch*5*toggle,'int32');
            Message = [Message typecast(Ticks,'uint8')];
        end
    end
    SendToMaster(Message);
    pause(10);
end
end

% --- Executes on button press in ContinueCycle.
function ContinueCycle_Callback(hObject, eventdata, handles)
% hObject    handle to ContinueCycle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
end
% Hint: get(hObject,'Value') returns toggle state of ContinueCycle


% --- Executes on selection change in NodeSelect.
function NodeSelect_Callback(hObject, eventdata, handles)
% hObject    handle to NodeSelect (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
end
% Hints: contents = cellstr(get(hObject,'String')) returns NodeSelect contents as cell array
%        contents{get(hObject,'Value')} returns selected item from NodeSelect


% --- Executes during object creation, after setting all properties.
function NodeSelect_CreateFcn(hObject, eventdata, handles)
% hObject    handle to NodeSelect (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

end

% --- Executes on button press in ZeroButton.
function ZeroButton_Callback(hObject, eventdata, handles)
% hObject    handle to ZeroButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global NodeNumber RollerNumber TicksPerInch NodePositions Commands
Message = [];
if (1 == handles.NodeSelect.Value)  % 1 is all
     for i = 1:NodeNumber
            Message = [Message Commands.Zero (i-1) (handles.RollerSelect.Value-1)]; % subtract one because matlab is 1 index
     end   
else
    Message = [Commands.Zero (handles.NodeSelect.Value-2) (handles.RollerSelect.Value-1)]; %subtract one for pop up menu all first and one for matlab 1 index
end
SendToMaster(Message);
end


% --- Executes on button press in ResetRadioButton.
function ResetRadioButton_Callback(hObject, eventdata, handles)
% hObject    handle to ResetRadioButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global NodeNumber RollerNumber TicksPerInch NodePositions Commands
Message = [];
if (1 == handles.NodeSelect.Value)  % 1 is all
     for i = 1:NodeNumber
            Message = [Message Commands.ResetRadio (i-1) (handles.RollerSelect.Value-1)]; % subtract one because matlab is 1 index
     end   
else
    Message = [Commands.ResetRadio (handles.NodeSelect.Value-2) (handles.RollerSelect.Value-1)]; %subtract one for pop up menu all first and one for matlab 1 index
end
SendToMaster(Message);
end

% --- Executes on button press in SetCurrentPositionButton.
function SetCurrentPositionButton_Callback(hObject, eventdata, handles)
% hObject    handle to SetCurrentPositionButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
end

% --- Executes on selection change in RollerSelect.
function RollerSelect_Callback(hObject, eventdata, handles)
% hObject    handle to RollerSelect (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
end
% Hints: contents = cellstr(get(hObject,'String')) returns RollerSelect contents as cell array
%        contents{get(hObject,'Value')} returns selected item from RollerSelect


% --- Executes during object creation, after setting all properties.
function RollerSelect_CreateFcn(hObject, eventdata, handles)
% hObject    handle to RollerSelect (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end

% --- Executes on selection change in ShapeSelect.
function ShapeSelect_Callback(hObject, eventdata, handles)
% hObject    handle to ShapeSelect (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
end
% Hints: contents = cellstr(get(hObject,'String')) returns ShapeSelect contents as cell array
%        contents{get(hObject,'Value')} returns selected item from ShapeSelect


% --- Executes during object creation, after setting all properties.
function ShapeSelect_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ShapeSelect (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end


% --- Executes on button press in SendShapeButton.
function SendShapeButton_Callback(hObject, eventdata, handles)
% hObject    handle to SendShapeButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global Commands 
global NodePositions
global NodeNumber RollerNumber TicksPerInch 
%global Shapes
%Shapes.Equilateral = 1;
i = handles.ShapeSelect.Value;

Shapes = {
    [0 0 ; 0 0 ; 0 0 ; 0 0]             % 1 Equilateral Triangle
    [-7 -7 ; 14 0 ; -14 0 ; 7 7]        % 2 Tall Triangle
    [5 5; -10 0 ; 10 0 ; -5 -5]         % 3 Short Triangle
    [-7 14.5 ; 15 0 ; -3.5 0 ; 8 6 ]    % 4 Square
    [-10 5 ; 5 0 ; -5 0 ; 10 -5]        % 5 Hexagon 
    [-10 -10 ; -10 0 ; -5 0 ; -5 -5]    % 6 Pincer
    [0 0 ; 0 0 ; 0 0 ; 0 0]             % 7 Single Tube Square
    [0 0 ; 0 0 ; -8 0 ; 0 0]            % 8 Single Tube Move Active Node
    [-8 -8 ; 8 0 ; -8 0 ; 8 8]          % 9 Single Tube Move Passive Node
    [-3 -15 ; 17 0 ; 5 0 ; 15 -15]      % 10 Single Tube Diamond
    [0 0 ; 0 0 ; 0 0 ; 0 0]             % 11 Locomotion 1.0  
    [0 5 ; 5 10 ; 0 5 ; 0 5]            % 12 Locomotion 1.1
    [0 0 ; 0 0 ; 0 0 ; 0 0]             % 13 Locomotion 2.0  
    [8 -8 ; 0 -4 ; 0 -8 ; 0 0 ]     % 14 Locomotion 2.1
    [0 0 ; 0 0 ; 0 0 ; 0 0]             % 15 Locomotion 3.0  
    [-5 -5 ; 0 0 ; 5 5  ; 0 0]          % 16 Locomotion 3.1
    [0 0 ; 0 0 ; 0 0 ; 0 0]             % 17 Locomotion 4.0  
    [8 0 ; 0 0 ; 4 0; 8 -8 ]        % 18 Locomotion 4.1
    [0 0 ; 0 0 ; 0 0 ; 0 0]             % 19 Locomotion 5.0  
    [-5 0 ;-5 0 ; -5 0; -10 -5 ]        % 20 Locomotion 5.1
    [0 0 ; 0 0 ; 0 0 ; 0 0]             % 21 Locomotion 6.0  
    [0 8 ; -4 -8 ; 0 -8 ; 4 8]  % 22 Locomotion 6.1
    [0 -7 ; -7 12 ; 0 6 ; 7 -14]        % 23 Tall 
    [0 4 ; 4 -9 ; 0 -5 ; -4 9]          % 24 Short  
    [0 -6 ; 4 -4 ; 0 5 ; 1 4]           % 25 Howl To The Moon 
    [0 7 ; -7 0 ; -7 0 ; 0 7]           % 26 Twist 
    -1*[0 7 ; -7 0 ; -7 0 ; 0 7]        % 27 Twist  back
    [0 0 ; -3 -5; 3 5 ; 0 0]            % 28 interaction video powered roll
    [0 0 ; 0 0; 0 0 ; 0 0]              % 29 interaction equilateral 1
    [0 7 ; -7 0 ; -7 0 ; 0 7]           % 30 interaction Twist
    [0 0 ; 0 0; 0 0 ; 0 0]              % 31 interaction equilateral 2
    [0 0 ; -5 5; 5 -5 ; 0 0]            % 32 interaction video open
    [0 0 ; 0 0; 0 0 ; 0 0]              % 33 interaction equilateral 3
    [0 0 ; 0 0; 0 0 ; 0 0]              
    };

NextShape = [2:length(Shapes) 1];
NextShape(22) = 11;
NextShape(10) = 7;
NextShape(6) = 1;

NewNodePositions = Shapes{i}
handles.ShapeSelect.Value = NextShape(i);


Message = [Commands.SetDesiredPositionAll];
for i = 1:NodeNumber
    for j = 1:RollerNumber
        Ticks = cast(TicksPerInch*NewNodePositions(i,j),'int32');
        Message = [Message typecast(Ticks,'uint8')];
        handles.NodePositions.Data{i,j} = num2str(NewNodePositions(i,j));
        NodePositions(i,j) = NewNodePositions(i,j);
   
    end
end
SendToMaster(Message);
end

% --- Executes during object creation, after setting all properties.
function popupmenu4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end

% --- Executes on selection change in popupmenu5.
function popupmenu4_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu5 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu5

end

% --- Executes on button press in radiobutton4.
function radiobutton4_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    global toggle_control
    toggle_control=toggle_control*-1;
end



% Hint: get(hObject,'Value') returns toggle state of radiobutton4


% --- Executes on button press in checkbox1.
function checkbox1_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox1
end


% --- Executes on selection change in popupmenu5.
function popupmenu5_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu5 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu5

end



% --- Executes during object creation, after setting all properties.
function popupmenu5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

end


% --- Executes on selection change in popupmenu6.
function popupmenu6_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
end
% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu6 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu6


% --- Executes during object creation, after setting all properties.
function popupmenu6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end

% --- Executes on button press in broadcast_command.
function broadcast_command_Callback(hObject, eventdata, handles)
% hObject    handle to broadcast_command (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of broadcast_command
end



function Node_Control_Edit_Callback(hObject, eventdata, handles)
% hObject    handle to Node_Control_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Node_Control_Edit as text
%        str2double(get(hObject,'String')) returns contents of Node_Control_Edit as a double
end

% --- Executes during object creation, after setting all properties.
function Node_Control_Edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Node_Control_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

end
