classdef Basic_fn < handle
    properties
    end
    methods
    end

    methods(Static)
        function Type=Rand_Type(Scale)
            % Generate random number of vehicle type
            Type=randsample((1:length(Scale)),1,true,Scale);
        end

        function Wt=Rand_Wt(Type,Traffic_para)
            % Generate random numbers of vehicle weight, unit:kN
            % lognormal distribution
            M_m1=9.8/1000*Traffic_para.M_m;
            M_v1=9.8^2/1000^2*Traffic_para.M_v;
            mu = log((M_m1(Type).^2)./sqrt(M_v1(Type)+M_m1(Type).^2));
            sigma = sqrt(log(M_v1(Type)./(M_m1(Type).^2)+1));
            Wt=lognrnd(mu,sigma,size(Type));
        end

        function LType=GLtype(k,LaneN)
            % Get the lane type of the kth lane
            % 1: Overtaking lane, 2: Travelling lane, 3: slow lane
            if LaneN==2
                % two-lane road
                if k>LaneN
                    k=k-LaneN;
                end
                LType=2*k-1;
            elseif LaneN==3
                % three-lane road
                if k>LaneN
                    k=k-LaneN;
                end
                LType=k;
            else
                % Multi-lane (number of lanes > 3)
                if k>LaneN
                    k=k-LaneN;
                end
                if k==1
                    LType=1;
                elseif k==LaneN
                    LType=3;
                else
                    LType=2;
                end
            end
        end
    end
end