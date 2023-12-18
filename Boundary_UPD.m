classdef Boundary_UPD < handle
    % Updating the exit and entrance boundaries of road
    properties
    end
    methods
    end 
    methods(Static)
        function Ctrl_para=PBC(Pre_para,Ctrl_para)
            % Periodic boundary condition
            Road_L=Pre_para.Road_L;
            LaneN=Pre_para.LaneN;
            WayN=Pre_para.WayN;
            V_info_Cur=Ctrl_para.V_info_Cur;
            
            % Periodic Boundary Processing/Cycling
            for k=1:LaneN*WayN
                J1=V_info_Cur(k).X(:,:)>Road_L;
                J2=V_info_Cur(k).X(2,:)>Road_L;
                V_info_Cur(k).X(J1)=V_info_Cur(k).X(J1)-Road_L;
                n=sum(J2);
                if  n>0
                    temp_Type=V_info_Cur(k).Type(J2);
                    temp_V=V_info_Cur(k).V(J2);
                    temp_X=V_info_Cur(k).X(:,J2);
                    temp_Wt=V_info_Cur(k).Wt(J2);
                    V_info_Cur(k).Type(J2)=[];
                    V_info_Cur(k).V(J2)=[];
                    V_info_Cur(k).Wt(J2)=[];
                    V_info_Cur(k).X(:,J2)=[];
                    V_info_Cur(k).Type=[V_info_Cur(k).Type,temp_Type];
                    V_info_Cur(k).V=[V_info_Cur(k).V,temp_V];
                    V_info_Cur(k).Wt=[V_info_Cur(k).Wt,temp_Wt];
                    V_info_Cur(k).X=[V_info_Cur(k).X,temp_X];
                end
            end      
            Ctrl_para.V_info_Cur=V_info_Cur;
        end
        
        function Ctrl_para=OBC(Pre_para,Traffic_para,Ctrl_para)
            % Open boundary condition
            Road_L=Pre_para.Road_L;
            delta_T=Pre_para.delta_T;
            LaneN=Pre_para.LaneN;
            WayN=Pre_para.WayN;
            V_info_Cur=Ctrl_para.V_info_Cur;
            Num=Ctrl_para.Num;
            T=Ctrl_para.T;
            Scale=Traffic_para.Scale;
            L0=Traffic_para.L0;
            vmax=Traffic_para.vmax;

            persistent N_Pass
            if T==0
                % N_Pass is used to record the number of vehicles that
                % failed to reach the bridge according to the Poisson
                % distribution and will be made up at subsequent time step
                N_Pass=zeros(1,LaneN*WayN);
            end

            % Exit Boundary
            for k=1:LaneN*WayN
                J1=V_info_Cur(k).X(2,:)>Road_L;
                n=sum(J1);
                V_info_Cur(k).Type(J1)=[];
                V_info_Cur(k).V(J1)=[];
                V_info_Cur(k).Wt(J1)=[];
                V_info_Cur(k).X(:,J1)=[];
                Num(k)=Num(k)-n;
            end
            
            % Entrance boundary
            for k=1:LaneN*WayN
                LaneType=Basic_fn.GLtype(k,LaneN);     
                % Poisson distribution based departure model
                n=random('poisson',Traffic_para.Lam(LaneType)*delta_T);
                if n>1
                    % It is generally assumed that random events with a
                    % Poisson distribution occur at most once in a
                    % relatively short period of time
                    N_Pass(k)=N_Pass(k)+n-1;
                    n=1;
                end
                if n==0&&N_Pass(k)>0
                    % Compensation at the time step when the vehicle is not
                    % generated
                    n=1;
                    N_Pass(k)=N_Pass(k)-1;
                end
                if n>0
                    if Num(k)~=0&&V_info_Cur(k).X(2,end)<=0
                        % lack of space
                        continue;
                    end
                    Type=Basic_fn.Rand_Type(Scale(LaneType,:));
                    V_info_Cur(k).Type(Num(k)+1)=Type;
                    if Num(k)==0
                        V_info_Cur(k).V(Num(k)+1)=vmax(Type);     
                    else
                        V_info_Cur(k).V(Num(k)+1)=min(vmax(Type),V_info_Cur(k).X(2,Num(k))/delta_T);
                    end
                    V_info_Cur(k).Wt(Num(k)+1)=Basic_fn.Rand_Wt(Type,Traffic_para);
                    V_info_Cur(k).X(:,Num(k)+1)=[0;-L0(Type)];
                    Num(k)=Num(k)+1;
                end
            end
            Ctrl_para.V_info_Cur=V_info_Cur;
            Ctrl_para.Num=Num;
        end
    end
end