classdef Road_initial < handle
    % Initialising the vehicle situation on the road
    properties
    end
    methods
    end
    methods(Static)
        function Ctrl_para=Jam(Pre_para,Traffic_para,Ctrl_para)
            % Compact superjam
            LaneN=Pre_para.LaneN;
            WayN=Pre_para.WayN;
            Road_L=Pre_para.Road_L;
            N_initial=Pre_para.N_initial;
            L0=Traffic_para.L0;
            Scale=Traffic_para.Scale;
            V_info_Cur=Ctrl_para.V_info_Cur;
            Num=Ctrl_para.Num;

            for k=1:LaneN*WayN
                LaneType=Basic_fn.GLtype(k,LaneN);
                while Num(k)<N_initial(k)
                    Type=Basic_fn.Rand_Type(Scale(LaneType,:));
                    if Num(k)==0
                        X=Road_L;
                    else
                        X=V_info_Cur(k).X(2,Num(k))-1;
                    end
                    if X-L0(Type)<=0
                        % No room for additional vehicles
                        break;
                    end
                    V_info_Cur(k).X(:,Num(k)+1)=[X;X-L0(Type)];
                    V_info_Cur(k).Type(Num(k)+1)=Type;
                    V_info_Cur(k).V(Num(k)+1)=0;
                    V_info_Cur(k).Wt(Num(k)+1)=Basic_fn.Rand_Wt(Type,Traffic_para);
                    Num(k)=Num(k)+1;
                end
            end
            Ctrl_para.Num=Num;
            Ctrl_para.V_info_Cur=V_info_Cur;
        end

        function Ctrl_para=Hom(Pre_para,Traffic_para,Ctrl_para)
            % Homogeneous
            LaneN=Pre_para.LaneN;
            WayN=Pre_para.WayN;
            Road_L_ex=Pre_para.Road_L_ex;
            N_initial=Pre_para.N_initial;
            L0=Traffic_para.L0;
            Scale=Traffic_para.Scale;
            V_info_Cur=Ctrl_para.V_info_Cur;
            Num=Ctrl_para.Num;

            for k=1:LaneN*WayN
                LaneType=Basic_fn.GLtype(k,LaneN);
                Type_array=[];
                while length(Type_array)<N_initial(k)
                    Type_array=[Type_array,Basic_fn.Rand_Type(Scale(LaneType,:))];
                    if (sum(L0(Type_array))+length(Type_array)-1)>Road_L_ex
                        % No room for additional vehicles
                        Type_array(end)=[];
                        break;
                    end
                end
                % Even vehicle spacing
                d_L=(Road_L_ex-sum(L0(Type_array)))/length(Type_array);
                for Type=Type_array
                    if Num(k)==0
                        X=Road_L_ex;
                    else
                        X=V_info_Cur(k).X(2,end)-d_L;
                    end
                    V_info_Cur(k).X(:,Num(k)+1)=[X;X-L0(Type)];
                    V_info_Cur(k).Type(Num(k)+1)=Type;
                    V_info_Cur(k).V(Num(k)+1)=0;
                    V_info_Cur(k).Wt(Num(k)+1)=Basic_fn.Rand_Wt(Type,Traffic_para);
                    Num(k)=Num(k)+1;
                end
            end
            Ctrl_para.Num=Num;
            Ctrl_para.V_info_Cur=V_info_Cur;
        end
    end
end




