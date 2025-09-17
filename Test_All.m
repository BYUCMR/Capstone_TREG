clc
clear

obj=MorphComGUI

data=guidata(obj)


%% Extract the Values from the GUI
result=data.popupmenu4
Node_Control=result.Value

%Now attempt to plot to the Figure in the GUI?

%% Plot something to the GUI Figure

% figure(data.figure1)
% plot(1:10,1:10)


%%

Find_Result=findobj(obj, 'type', 'axes')
% set('CurrentAxes',Find_Result)
% axes(Find_Result);
plot(Find_Result,1:10,1:10)
% figure(Find_Result)
% plot(1:10,-1*1:10)

