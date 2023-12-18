classdef SearchRL < handle
    % Search for vehicles ahead and behind in the adjacent lane
    properties
    end
    methods
    end
    methods(Static)
        function [fr,be,dfr,dbr,Vbr]=PBC(k,k_new,VeNum,V_info_Prev,Num,Road_L)
            % Periodic boundary condition
            % k: current lane, k_new: target lane,
            % VeNum: Number array of vehicles on lane k
            X_k=V_info_Prev(k).X;
            J=X_k(1,:)<X_k(2,:);
            X_k(1,J)=X_k(1,J)+Road_L;
            X_knew=V_info_Prev(k_new).X;
            J=X_knew(1,:)<X_knew(2,:);
            X_knew(1,J)=X_knew(1,J)+Road_L;
            fr=zeros(size(VeNum));  %Number array of the vehicle in front
            be=zeros(size(VeNum));  %Number array of the vehicle behind
            dfr=zeros(size(VeNum)); %Distance from the vehicle in front
            dbr=zeros(size(VeNum)); %Distance from the vehicle behind
            Vbr=zeros(size(VeNum)); %Speed of the vehicle behind

            for i=1:length(VeNum)
                fr0=find(X_knew(2,:)>X_k(1,VeNum(i)),1,'last');
                be0=find(X_knew(1,:)<X_k(2,VeNum(i)),1);
                if isempty(be0)
                    % No vehicles behind in adjacent lanes
                    be0=Num(k_new)+1;
                    if Num(k_new)==0
                        % No vehicles in adjacent lanes
                        Xbr=0;
                        Vbr(i)=0;
                    else
                        Xbr=X_knew(1,1)-Road_L;
                        Vbr(i)=V_info_Prev(k_new).V(1);
                    end
                else
                    Xbr=X_knew(1,be0);
                    Vbr(i)=V_info_Prev(k_new).V(be0);
                end

                if isempty(fr0)
                    % No vehicles ahead in adjacent lanes
                    fr0=0;
                    if Num(k_new)==0
                        % No vehicles in adjacent lanes
                        Xfr=2*Road_L;
                    else
                        Xfr=X_knew(2,Num(k_new))+Road_L;
                    end
                else
                    Xfr=X_knew(2,fr0);
                end
                % Distance between the vehicle and the vehicle in front in the adjacent lane
                dfr(i)=Xfr-X_k(1,VeNum(i));
                % Distance between the vehicle and the vehicle behind in the adjacent lane
                dbr(i)=X_k(2,VeNum(i))-Xbr;
                fr(i)=fr0;
                be(i)=be0;
            end
        end


        function [fr,be,dfr,dbr,Vbr]=OBC(k,k_new,VeNum,V_info_Prev,Num,Road_L)
            % Open boundary condition
            % k: current lane, k_new: target lane,
            % VeNum: Number array of vehicles on lane k
            fr=zeros(size(VeNum));
            be=zeros(size(VeNum));
            dfr=zeros(size(VeNum));
            dbr=zeros(size(VeNum));
            Vbr=zeros(size(VeNum));
            for i=1:length(VeNum)
                fr0=find(V_info_Prev(k_new).X(2,:)>V_info_Prev(k).X(1,VeNum(i)),1,'last');
                be0=find(V_info_Prev(k_new).X(1,:)<V_info_Prev(k).X(2,VeNum(i)),1);
                if isempty(be0)
                    % No vehicles behind in adjacent lanes
                    be0=Num(k_new)+1;
                    Xbr=0;
                    Vbr(i)=0;
                else
                    Xbr=V_info_Prev(k_new).X(1,be0);
                    Vbr(i)=V_info_Prev(k_new).V(be0);
                end
                if isempty(fr0)
                    % No vehicles ahead in adjacent lanes
                    fr0=0;
                    Xfr=Road_L;
                else
                    Xfr=V_info_Prev(k_new).X(2,fr0);
                end
                % Distance between the vehicle and the vehicle in front in the adjacent lane
                dfr(i)=Xfr-V_info_Prev(k).X(1,VeNum(i));
                % Distance between the vehicle and the vehicle behind in the adjacent lane
                dbr(i)=V_info_Prev(k).X(2,VeNum(i))-Xbr;
                fr(i)=fr0;
                be(i)=be0;
            end
        end
    end
end

