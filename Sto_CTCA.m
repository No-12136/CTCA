function []=Sto_CTCA(L,Way,Lane,T_sim,rou_initial,boundary)
% Version 1.0
% Author：Minyi Di
% Referring to：
% Please Cite as：
% Copyright Minyi Di and Danhui Dan. All Rights Reserved.

% Simulation Algorithm for Stochastic CTCA Model
% with adjustable boundary conditions, multi-lane, multi-vehicle type,
% and including axle information
%
% Main function
%
% input:    L -- road length, unit:m
%           Way -- Number of lanes in a single direction, non-negative integer
%           Lane -- One-way (1) or two-way lanes (2)
%           T_sim -- Time length of the simulation algorithm execution, unit:s
%           rou_initial -- initial traffic density for each lane
%                          (vector form, number of components == LaneN*WayN,
%                           1 and LaneN+1 are passing lanes,
%                           LaneN and WayN*LaneN are slow lanes,
%                           and the rest are travel lanes)
%           boundary -- boundary condition of CA model, optional, default value 0,
%                       taking 0 for periodic boundary condition and 1 for open boundary condition

%% Preset parameters
% Adjustable parameters
Pre_para=[];                    % Parameter set
Pre_para.pslow=0.3;             % The randomisation probability of slowing down, 0 to 1
Pre_para.v_pre=0.5;               % Anticipated driving parameters, 0 to 1
Pre_para.delta_T=1;             % Time step size of the CA model, default value 1s
% Number of time steps required for traffic flow to evolve to stability
Pre_para.T_stable=10000;
% Storage address for traffic evolution map
Pre_para.Fileaddress='C:\Users\oy135\Desktop';

% Non-tunable parameters or the main function's input parameters
% LaneN: Number of lanes in a single direction, non-negative integer
% when LaneN=1, the lane change rule is automatically blocked
Pre_para.LaneN=Lane;
Pre_para.WayN=Way;                % One-way (1) or two-way lanes (2)
% Maximum execution time steps
Pre_para.Tsum=Pre_para.T_stable+floor(T_sim/Pre_para.delta_T);
Pre_para.Road_L=L;              % Actual length of bridge
% Initial number of vehicles in each lane
Pre_para.N_initial=ceil(rou_initial*Pre_para.Road_L);
% Boundary condition
% 0: periodic boundary condition, 1: open boundary condition, default 0
if nargin<5
    Pre_para.Boundary=0;
else
    Pre_para.Boundary=boundary;
end

%% Statistical parameters of traffic flow
% Controlling the simulated traffic flow, adjustable
Traffic_para=[];
% Cross-sectional departure rates for all types of lanes
% overtaking lane/travelling lane/slow lane
Traffic_para.Lam=[0.365,0.289,0.119];
% Upper speed limit for each vehicle type, unit: m/s
Traffic_para.vmax=[25,25,25,20,15];
% Average value of vehicle mass for each type,unit: kg
Traffic_para.M_m=[1696.04,4315.15,12001.14,21309.25,48581.78];
% Variance of vehicle mass for each type,unit: kg^2
Traffic_para.M_v=[268.77,1580.83,2428.18,4249.13,9366.10];
% Proportion of vehicles in each type of lane,
% each row representing a type of lane
Traffic_para.Scale=[0.952,0.012,0.026,0.007,0.003;
    0.723,0.094,0.119,0.045,0.019;
    0.370,0.169,0.290,0.118,0.053];
% Vehicle length for each type, unit: m
Traffic_para.L0=[5,6,9,10,15];
% Upper limit of acceleration for each type, unit: m/s^2,
% refer to the acceleration time of 100km/h
Traffic_para.amax=[6,4,3.5,3,2];
% Wheelbase information for each type of vehicle
% (distance from each axle to the front of the vehicle)
% Contains axle information, using the parameters below
Traffic_para.Wheelbase={[1,4.2],[1.2,5],[1.9,7.2],[2.1,6.6,8.1],[1.95,5.35,6.7,13.3,14.65]};
% Not containing axle information, i.e. the vehicle mass is concentrated in
% the mass centre, use the following parameters
% Traffic_para.wheelbase={[2.5],[3],[4.5],[5],[7.5]};
%% Control and statistical parameters of the programme, initialisation
% Control parameter
Ctrl_para=[];
Ctrl_para.T=0;                    % Current time step
% Number of vehicles on each road
Ctrl_para.Num=zeros(Pre_para.LaneN*Pre_para.WayN,1);
% Struct array whose size is the number of lanes, holding information about
% vehicles (cells) in each lane of the bridge at the current time step
% (vehicle type, velocity, weight, and position of the front and rear of
% the vehicle)
Ctrl_para.V_info_Cur=struct('Type',cell(Pre_para.LaneN*Pre_para.WayN,1),...
    'V',cell(Pre_para.LaneN*Pre_para.WayN,1),'Wt',cell(Pre_para.LaneN*Pre_para.WayN,1),...
    'X',cell(Pre_para.LaneN*Pre_para.WayN,1));
% Struct array of vehicle cells at the previous time step
Ctrl_para.V_info_Prev=Ctrl_para.V_info_Cur;
%%
h=waitbar(0,'Simulation in progress ...');  % Simulation progress bar
pause(1);
% Road initialisation module, T=0
% Jam function for Compact superjam, Hom function for Homogeneous
% choose one of the two
Ctrl_para=Road_initial.Jam(Pre_para,Traffic_para,Ctrl_para);
%Ctrl_para=Road_initial.Hom(Pre_para,Traffic_para,Ctrl_para);
while Ctrl_para.T<Pre_para.Tsum
    % Simulation progress bar
    p=fix(Ctrl_para.T/Pre_para.Tsum*10000)/100;
    str={['Output is in progress and the estimated progress is ',num2str(p),' %'],['Completed ',num2str(Ctrl_para.T),'/',num2str(Pre_para.Tsum)]};
    waitbar(Ctrl_para.T/Pre_para.Tsum,h,str);

    % Vehicle travelling module
    Ctrl_para=V_Travel(Pre_para,Traffic_para,Ctrl_para);
    % Boundary updating module
    if Pre_para.Boundary==0
        % periodic boundary condition
        Ctrl_para=Boundary_UPD.PBC(Pre_para,Ctrl_para);
    else
        % open boundary condition
        Ctrl_para=Boundary_UPD.OBC(Pre_para,Traffic_para,Ctrl_para);
    end
    % Time step+1
    Ctrl_para.T=Ctrl_para.T+1;
    % Traffic evolution mapping module
    if Ctrl_para.T>=Pre_para.T_stable&&Ctrl_para.T-Pre_para.T_stable<60
        % Evolution is plotted for only 60 time steps and can be adjusted
        % by yourself
        Traffic_map(Pre_para,Traffic_para,Ctrl_para);
    end
end
close(h);
msgbox('Simulation completed');
end