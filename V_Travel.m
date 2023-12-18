function Ctrl_para=V_Travel(Pre_para,Traffic_para,Ctrl_para)
% Vehicle travelling module
% Update vehicle speed and position for timestep T+1

%Storage of vehicle information at time step T
Ctrl_para.V_info_Prev=Ctrl_para.V_info_Cur;
for k=1:Pre_para.LaneN*Pre_para.WayN
    Ctrl_para.V_info_Cur(k).X=zeros(2,0);
    Ctrl_para.V_info_Cur(k).V=[];
end
% Speed Update Module
[Ve_Change,Ctrl_para]=UPD_v(Pre_para,Ctrl_para,Traffic_para);
% Location Update Module
Ctrl_para=UPD_x(Ve_Change,Pre_para,Ctrl_para);
end


function [Ve_Change,Ctrl_para]=UPD_v(Pre_para,Ctrl_para,Traffic_para)
% Update the vehicle speed for the T+1 time step
% (strictly speaking, the speed during the T~T+1 time step)
LaneN=Pre_para.LaneN;
WayN=Pre_para.WayN;
pslow=Pre_para.pslow;
v_pre=Pre_para.v_pre;
Boundary=Pre_para.Boundary;
delta_T=Pre_para.delta_T;
Road_L=Pre_para.Road_L;
Num=Ctrl_para.Num;
V_info_Cur=Ctrl_para.V_info_Cur;
V_info_Prev=Ctrl_para.V_info_Prev;
vmax=Traffic_para.vmax;
amax=Traffic_para.amax;

% Velocity change during delta_T for randomly slowed deceleration
V_unit=2.5*delta_T;
% Record lane change information, see DecALaneC function for details
Ve_Change=cell(LaneN*WayN,1);
for k=1:LaneN*WayN
    if Num(k)==0
        Ve_Change{k}=[];
        continue;
    end
    d=zeros(1,Num(k));
    X=V_info_Prev(k).X;
    V=V_info_Prev(k).V;
    Type=V_info_Prev(k).Type;
    % Variable acceleration mode acceleration
    ac=min((vmax(Type)-V)/2,amax(Type));
    % Get all vehicle distances in the current lane
    if Boundary==0
        J1=X(1,:)<X(2,:);
        X(1,J1)=X(1,J1)+Road_L;
        d(1)=Road_L-X(1,1)+X(2,Num(k));
        d(2:Num(k))=X(2,1:Num(k)-1)-X(1,2:Num(k));  % Vehicle distance
    else
        d(1)=Road_L;
        d(2:Num(k))=X(2,1:Num(k)-1)-X(1,2:Num(k));
    end

    % The VE model with anticipation
    % Virtual speed
    v_virt=min([vmax(Type(1:end-1))-V_unit;V(1:end-1);max(0,d(1:end-1)/delta_T-V_unit)]);
    d(2:end)=d(2:end)+(1-v_pre)*v_virt*delta_T;
    if Boundary==0
        v_virt=min([vmax(Type(end))-V_unit;V(end);max(0,d(end)/delta_T-V_unit)]);
        d(1)=d(1)+(1-v_pre)*v_virt*delta_T;
    end
    d=max(d-1,0); % Minimum safe distance 1m

    V_info_Cur(k).V=V*nan;  % Array placeholder
    % Acceleration
    J_acc=(V*delta_T<d);
    V_info_Cur(k).V(J_acc)=min(min(V(J_acc)+ac(J_acc)*delta_T,vmax(Type(J_acc))),d(J_acc)/delta_T);
    if LaneN==1
        % Deceleration
        V_info_Cur(k).V(J_unacc)=d(J_unacc)/delta_T;
    else
        % Deceleration and Lane Change
        % Only lane change information is recorded, no lane change at this time
        V_info_Cur(k).V(~J_acc&V==0)=V_info_Prev(k).V(~J_acc&V==0);
        [Ve_Change{k},V_info_Cur(k).V]=DecALaneC(k,~J_acc&V~=0,d,V_info_Cur(k).V,Ctrl_para,Pre_para);
    end
    % Randomization
    r=rand(1,Num(k));
    J_slow=(r<=pslow);
    if ~isempty(Ve_Change{k})
        % Changing lanes vehicles do not participate in slowing down
        J_slow(Ve_Change{k}(2,:))=false;
    end
    V_info_Cur(k).V(J_slow)=max(V_info_Cur(k).V(J_slow)-V_unit,0);
end
% Upload speed information for T+1 time step
Ctrl_para.V_info_Cur=V_info_Cur;
end

function Ctrl_para=UPD_x(Ve_Change,Pre_para,Ctrl_para)
% Update vehicle location for T+1 time step
delta_T=Pre_para.delta_T;
LaneN=Pre_para.LaneN;
WayN=Pre_para.WayN;
V_info_Cur=Ctrl_para.V_info_Cur;
V_info_Prev=Ctrl_para.V_info_Prev;

% Location Update
for k=1:LaneN*WayN
    if Ctrl_para.Num(k)==0
        continue;
    end
    Ctrl_para.V_info_Cur(k).X=V_info_Prev(k).X+V_info_Cur(k).V*delta_T;
end
% Execute lane change
if LaneN==1
    return;
else
    Ctrl_para=ChangeLine(Ve_Change,Ctrl_para,Pre_para);
end
end

function Ctrl_para=ChangeLine(Ve_Change,Ctrl_para,Pre_para)
% Execute lane change
V_info_Cur=Ctrl_para.V_info_Cur;
Num=Ctrl_para.Num;
LaneN=Pre_para.LaneN;
WayN=Pre_para.WayN;
Boundary=Pre_para.Boundary;
Road_L_ex=Pre_para.Road_L;
for k=1:LaneN*WayN
    % The first line of Vc_k stores the target lane of the lane-changing
    % vehicle in the current lane, -1: left side, 1: right side The second
    % line stores the vehicle number of the lane-changing vehicle in the
    % current lane.
    Vc_k=Ve_Change{k};
    if isempty(Vc_k)
        continue;
    end
    J_ChgL=(Vc_k(1,:)==-1);
    J_ChgR=~J_ChgL;
    LaneType=Basic_fn.GLtype(k,LaneN);
    if LaneType==1      %overtaking lane
        % change lanes to the right
        V_info_Cur(k+1).Type=[V_info_Cur(k+1).Type(1:Num(k+1)),V_info_Cur(k).Type(Vc_k(2,J_ChgR))];
        V_info_Cur(k+1).V=[V_info_Cur(k+1).V(1:Num(k+1)),V_info_Cur(k).V(Vc_k(2,J_ChgR))];
        V_info_Cur(k+1).Wt=[V_info_Cur(k+1).Wt(1:Num(k+1)),V_info_Cur(k).Wt(Vc_k(2,J_ChgR))];
        V_info_Cur(k+1).X=[V_info_Cur(k+1).X(:,1:Num(k+1)),V_info_Cur(k).X(:,Vc_k(2,J_ChgR))];
        Num(k+1)=Num(k+1)+sum(J_ChgR);
    elseif LaneType==2  %travelling lane
        % change lanes to the left
        V_info_Cur(k-1).Type=[V_info_Cur(k-1).Type(1:Num(k-1)),V_info_Cur(k).Type(Vc_k(2,J_ChgL))];
        V_info_Cur(k-1).V=[V_info_Cur(k-1).V(1:Num(k-1)),V_info_Cur(k).V(Vc_k(2,J_ChgL))];
        V_info_Cur(k-1).Wt=[V_info_Cur(k-1).Wt(1:Num(k-1)),V_info_Cur(k).Wt(Vc_k(2,J_ChgL))];
        V_info_Cur(k-1).X=[V_info_Cur(k-1).X(:,1:Num(k-1)),V_info_Cur(k).X(:,Vc_k(2,J_ChgL))];
        Num(k-1)=Num(k-1)+sum(J_ChgL);
        % change lanes to the right
        V_info_Cur(k+1).Type=[V_info_Cur(k+1).Type(1:Num(k+1)),V_info_Cur(k).Type(Vc_k(2,J_ChgR))];
        V_info_Cur(k+1).V=[V_info_Cur(k+1).V(1:Num(k+1)),V_info_Cur(k).V(Vc_k(2,J_ChgR))];
        V_info_Cur(k+1).Wt=[V_info_Cur(k+1).Wt(1:Num(k+1)),V_info_Cur(k).Wt(Vc_k(2,J_ChgR))];
        V_info_Cur(k+1).X=[V_info_Cur(k+1).X(:,1:Num(k+1)),V_info_Cur(k).X(:,Vc_k(2,J_ChgR))];
        Num(k+1)=Num(k+1)+sum(J_ChgR);
    else    %slow lane
        % change lanes to the left
        V_info_Cur(k-1).Type=[V_info_Cur(k-1).Type(1:Num(k-1)),V_info_Cur(k).Type(Vc_k(2,J_ChgL))];
        V_info_Cur(k-1).V=[V_info_Cur(k-1).V(1:Num(k-1)),V_info_Cur(k).V(Vc_k(2,J_ChgL))];
        V_info_Cur(k-1).Wt=[V_info_Cur(k-1).Wt(1:Num(k-1)),V_info_Cur(k).Wt(Vc_k(2,J_ChgL))];
        V_info_Cur(k-1).X=[V_info_Cur(k-1).X(:,1:Num(k-1)),V_info_Cur(k).X(:,Vc_k(2,J_ChgL))];
        Num(k-1)=Num(k-1)+sum(J_ChgL);
    end
    % Clearing the information of the lane-changing vehicle in the original lane
    V_info_Cur(k).Type(Vc_k(2,:))=[];
    V_info_Cur(k).V(Vc_k(2,:))=[];
    V_info_Cur(k).Wt(Vc_k(2,:))=[];
    V_info_Cur(k).X(:,Vc_k(2,:))=[];
    Num(k)=Num(k)-length(Vc_k(2,:));
end
% Reordering the array of vehicles in each lane by spatial position
for k=1:LaneN*WayN
    if Boundary==0
        X=V_info_Cur(k).X;
        J1=X(1,:)<X(2,:);
        X(1,J1)=X(1,J1)+Road_L_ex;
    else
        X=V_info_Cur(k).X;
    end
    if ~issorted(X(1,:),'descend')
        [~,Index]=sort(X(1,:),'descend');
        V_info_Cur(k).Type=V_info_Cur(k).Type(Index);
        V_info_Cur(k).V=V_info_Cur(k).V(Index);
        V_info_Cur(k).Wt=V_info_Cur(k).Wt(Index);
        V_info_Cur(k).X=V_info_Cur(k).X(:,Index);
    end
end
Ctrl_para.V_info_Cur=V_info_Cur;
Ctrl_para.Num=Num;
end