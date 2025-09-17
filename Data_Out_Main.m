obj=MorphComGUI
data=guidata(obj)
e = getappdata(0,'evalue'); %THis allows me to pass data.  Now, I just need to decode it from the message

%%


%% Visualize the IMU
clf
count=1;
while 1
    clf
    %Trigger an event? 
    MorphComGUI('@send_button_Callback',obj,[],guidata(obj))

    Orient_i=getappdata(0,'Orient_i')
    Orient_j=getappdata(0,'Orient_j')
    Orient_k=getappdata(0,'Orient_k')
    Node=2; %Now 1 Indexed
    Roll=1;
    
    Quat_4=sqrt(1-(Orient_i(Node,Roll)^2+Orient_j(Node,Roll)^2+Orient_k(Node,Roll)^2));
    
    %Only proceed if the value is real
    if isreal(Quat_4) 
        quat=[Quat_4, Orient_i(Node,Roll),Orient_j(Node,Roll),Orient_k(Node,Roll)];
        R_mat=quat2rotm(quat)
        Hist(:,:,count)=R_mat;
        count=count+1;
        d_length=1;
        d_width=.5
        d_down=.1;
        d_up=.1;
        %Generate a Prism
            vertices = ([-d_length, -d_width, -d_down;
            d_length, -d_width, -d_down;
            d_length, d_width, -d_down;
            -d_length, d_width, -d_down;
            -d_length, -d_width, d_up;
            d_length, -d_width, d_up;
            d_length, d_width, d_up;
            -d_length, d_width, d_up]);
        % clf
%         R_switch=[0 0 1; 0 -1 0; 1 0 0];
        R_switch=eye(3);
        Vert_Rot=(R_mat*R_switch*vertices')';
%         Vert_Rot=(vertices*R_mat)
            %Rotate the Vertices
        %     Vec_x=cross(Vec_Out,Vec_Up);
            faces = [1 2 6 5; 2 3 7 6; 3 4 8 7; 1 4 8 5; 1 2 3 4; 5 6 7 8];
            color=[1 0 0];
            patch('faces',faces,'vertices',Vert_Rot,'facecolor',color,'facealpha',.1, 'edgecolor','k','linewidth', 1);
            hold on
            axis equal
            xlabel('x')
            ylabel('y')
            zlabel('z')
            view([18 27])
        axis ([-1.5 1.5 -1.5 1.5 -1.5 1.5])
    end
%     clf
    pause(.1)
end

%% Can I trigger events from the main file? 



