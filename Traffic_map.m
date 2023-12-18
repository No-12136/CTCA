function Ctrl_para=Traffic_map(Pre_para,Traffic_para,Ctrl_para)
% Dynamic diagram of the evolution of the traffic flow
Road_L=Pre_para.Road_L;
LaneN=Pre_para.LaneN;
WayN=Pre_para.WayN;
Wheelbase=Traffic_para.Wheelbase;
Num=Ctrl_para.Num;
V_info_Cur=Ctrl_para.V_info_Cur;

% The y coordinate offset of the lanes used for plotting
y_offset=zeros(1,LaneN*WayN);
for k=1:LaneN*WayN
    if k>LaneN
        y_offset(k)=-(k-LaneN);
    else
        y_offset(k)=k;
    end
end
Plot_xy=[];
for k=1:LaneN*WayN
    if Num(k)==0
        continue;
    end
    X0=V_info_Cur(k).X;
    X_k=[];
    for Type=1:length(Wheelbase)
        J_Type=(V_info_Cur(k).Type==Type);
        if sum(J_Type)==0
            continue;
        end
        % axle positions
        X=X0(1,J_Type)-Wheelbase{Type}';
        X=reshape(X,1,[]);
        if k>LaneN
            X=Road_L-X;
        end
        X_k=[X_k,X];
    end
    Plot_xy=[Plot_xy,[X_k;ones(1,length(X_k))*y_offset(k)]];
end

h=figure('Visible','off');
scatter(Plot_xy(1,:),Plot_xy(2,:),50,[153,51,250]/255,'Marker','|');
xlim([0,Road_L]);ylim([min(y_offset)-0.5,max(y_offset)+0.5]);
set(gcf,'Position',[480,310,1000,200],'Color','w');
set(gca,'TickLength',[0,0],'Box','on','YTick',[]);

[A,map] = rgb2ind(frame2im(getframe(h,[112.5,0,810,200])),256);
if Ctrl_para.T==Pre_para.T_stable
    imwrite(A,map,[Pre_para.Fileaddress,'\Traffic evolution map.gif'],'gif','Loopcount',0,'DelayTime',1);
else
    imwrite(A,map,[Pre_para.Fileaddress,'\Traffic evolution map.gif'],'gif','WriteMode','append','DelayTime',1);
end
end