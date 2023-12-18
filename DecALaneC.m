function [Ve_Change,V_cur]=DecALaneC(k,J_unacc,D,V_cur,Ctrl_para,Pre_para)
% Judging whether a non-accelerating vehicle is slowing down in its current
% lane or changing lanes without slowing down Executes vehicle
% deceleration, but does not execute lane change, only returns lane change
% information
delta_T=Pre_para.delta_T;
Road_L=Pre_para.Road_L;
V_info_Prev=Ctrl_para.V_info_Prev;
Num=Ctrl_para.Num;
if Pre_para.Boundary==0
    searchRL=@SearchRL.PBC;
else
    searchRL=@SearchRL.OBC;
end

% Array of non-accelerated vehicle numbers
VeNum=find(J_unacc);
if sum(J_unacc)==0
    Ve_Change=[];
    return;
end
V_Prev=V_info_Prev(k).V;
% The first line of Ve_Change stores the target lane of the lane-changing
% vehicle in the current lane, -1: left side, 1: right side The second line
% stores the vehicle number of the lane-changing vehicle in the current lane
Ve_Change=[];

LaneType=Basic_fn.GLtype(k,Pre_para.LaneN);
d=D(VeNum);
% Asymmetric lane changing rules for a typical three lanes (overtaking,
% travelling and slow lanes) When the number of lanes in a single
% directiondoes not meet 3, there will be an error in the probability of
% changing lanes in the rule, please adjust it yourself
if LaneType==1  %overtaking lane
    % Search for vehicles ahead and behind in the right-hand lane
    [Rfr,Rbe,dRfr,dRbe,vRbe]=searchRL(k,k+1,VeNum,V_info_Prev,Num,Road_L);
    p=rand(size(VeNum));
    % Vehicles changing lanes to the right
    J_ChgR=(Rbe-Rfr==1)&(V_Prev(VeNum)*delta_T<=dRfr)&(vRbe*delta_T<=dRbe)&p<0.5;
    Ve_Change=[Ve_Change,[ones(1,sum(J_ChgR));VeNum(J_ChgR)]];
    V_cur(VeNum(J_ChgR))=V_Prev(VeNum(J_ChgR));
    % Decelerating vehicles
    J_Dec=~J_ChgR;
    V_cur(VeNum(J_Dec))=max(d(J_Dec)/delta_T,0);
elseif LaneType==2  %travelling lane
    % Search for vehicles ahead and behind in the right-hand lane
    [Rfr,Rbe,dRfr,dRbe,vRbe]=searchRL(k,k+1,VeNum,V_info_Prev,Num,Road_L);
    % Search for vehicles ahead and behind in the left-hand lane
    [Lfr,Lbe,dLfr,dLbe,vLbe]=searchRL(k,k-1,VeNum,V_info_Prev,Num,Road_L);
    J1=(Rbe-Rfr==1);
    J2=(Lbe-Lfr==1);
    J3=(V_Prev(VeNum)*delta_T<=dRfr);
    J4=(V_Prev(VeNum)*delta_T<=dLfr);
    J5=(vRbe*delta_T<=dRbe);
    J6=(vLbe*delta_T<=dLbe);
    p=rand(size(VeNum));

    % Vehicles changing lanes to the right
    J_ChgR=(J1&J2&J3&J4&J5&J6&p<0.14)|(J1&J3&J5&(~J2|~J4|~J6)&p<0.7);
    Ve_Change=[Ve_Change,[ones(1,sum(J_ChgR));VeNum(J_ChgR)]];
    V_cur(VeNum(J_ChgR))=V_Prev(VeNum(J_ChgR));
    % Vehicles changing lanes to the left
    J_ChgL=(J1&J2&J3&J4&J5&J6&p>=0.14&p<0.7)|(J2&J4&J6&(~J1|~J3|~J5)&p<0.7);
    Ve_Change=[Ve_Change,[-1*ones(1,sum(J_ChgL));VeNum(J_ChgL)]];
    V_cur(VeNum(J_ChgL))=V_Prev(VeNum(J_ChgL));
    % Decelerating vehicles
    J_Dec=~J_ChgR&~J_ChgL;
    V_cur(VeNum(J_Dec))=max(d(J_Dec)/delta_T,0);
else    %slow lane
    [Lfr,Lbe,dLfr,dLbe,vLbe]=searchRL(k,k-1,VeNum,V_info_Prev,Num,Road_L);%确定左侧车道的前后方车辆
    p=rand(size(VeNum));
    % Vehicles changing lanes to the left
    J_ChgL=(Lbe-Lfr==1)&(V_Prev(VeNum)*delta_T<=dLfr)&(vLbe*delta_T<=dLbe)&p<0.7;
    Ve_Change=[Ve_Change,[-1*ones(1,sum(J_ChgL));VeNum(J_ChgL)]];
    V_cur(VeNum(J_ChgL))=V_Prev(VeNum(J_ChgL));
    % Decelerating vehicles
    J_Dec=~J_ChgL;
    V_cur(VeNum(J_Dec))=max(d(J_Dec)/delta_T,0);
end
end