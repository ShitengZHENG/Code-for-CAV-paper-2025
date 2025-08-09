clc
clear
close all

% Code for simulation of two-lane highway with four on-/off-ramps
% Code for Figure 4c in main text and  Section 4.4 in SI.
%% Basic simulation setting
max_vehnum=5000;
dt=0.05;
step=8000;
a_comf=5;
v_max=120/3.6;
L_veh=4.835;
dur_lc=2;
a_Signal=inf;

simu_setting=[dt,v_max,L_veh,a_comf];
%% Parameter
%----------------------------------------------------AV-Upper controller
paraUpper=[0.6265    0.3703    0.4437    0.2889    0.6676    0.0736    1];
kv1_=paraUpper(1);kv2_=paraUpper(2);kg_=paraUpper(3);ka1_=paraUpper(4);ka2_=paraUpper(5);Tg_=paraUpper(6);
Gmin0=paraUpper(7);
Gmin=L_veh+Gmin0;

dx_=Tg_*v_max+Gmin;
Q_max=1/dx_*v_max*3600;%veh/h
%----------------------------------------------------AV-Lower controller
para_AV=[78.8201836419897,2.07647787797603,9.17891305242795,4;
    58.9344898328179,1.90325205249387,7.86998881634774,4;
    95.9937146710127,2.32906700594111,10.0111919741363,4;
    43.0500120244655,1.62000868090560,6.87020881889564,4];
para_AV=repmat(para_AV,max_vehnum/4,1);
%--------------------------------------------------------------------------Stochasticity
NoiseStrength=[0.8556    0.0123];
kappa=NoiseStrength(1);sigma=NoiseStrength(2);
rng(100);
rnd_ksi=randn(step,max_vehnum);
noise=zeros(step,max_vehnum);
for t=1:step-1
    noise(t+1,:)=(1-kappa*dt)*noise(t,:)+sigma*rnd_ksi(t,:)*(dt^0.5);
end
%% Road network
lane_width=3.75;
num_lane=2;%lane number
%---------
L_subroad=1500;
L_subByroad=250;
L_acclane=100;
x_cooperate=150;
numIntersections=4;
L_road=L_subroad*numIntersections;%m
Loc_onRamp_1=L_subroad/3;
Loc_offRamp_1=L_subroad*2/3;
Loc_onRamp_2=L_subroad+L_subroad/3;
Loc_offRamp_2=L_subroad+L_subroad*2/3;
Loc_onRamp_3=L_subroad*2+L_subroad/3;
Loc_offRamp_3=L_subroad*2+L_subroad*2/3;
Loc_onRamp_4=L_subroad*3+L_subroad/3;
Loc_offRamp_4=L_subroad*3+L_subroad*2/3;
% detector position
pos_detector_main=[0,Loc_offRamp_1,Loc_offRamp_2,Loc_offRamp_3]+50;
pos_detector_onramp=[Loc_onRamp_1,Loc_onRamp_2,Loc_onRamp_3,Loc_onRamp_4]-200;

% lane number
Mainlane_ID=[1,2];
num_mainlane=length(Mainlane_ID);
Bylane_ID=[3,5,7,9];
num_bylane=length(Bylane_ID);

% position of entry and exit
pos_Enter=[0,0,...%1-2
           Loc_onRamp_1-L_subByroad,nan,...%3-4
           Loc_onRamp_2-L_subByroad,nan,...%5-6
           Loc_onRamp_3-L_subByroad,nan,...%7-8
           Loc_onRamp_4-L_subByroad,nan];%9-10
pos_Exit=[L_road,L_road,...%1-2
          Loc_onRamp_1+L_acclane,Loc_offRamp_1+L_subByroad,...%3-4
          Loc_onRamp_2+L_acclane,Loc_offRamp_2+L_subByroad,...%5-6
          Loc_onRamp_3+L_acclane,Loc_offRamp_3+L_subByroad,...%7-8
          Loc_onRamp_4+L_acclane,Loc_offRamp_4+L_subByroad];%9-10
pos_obstacle=[inf,inf,...%1-2
          Loc_onRamp_1+L_acclane,Loc_offRamp_1,...%3-4
          Loc_onRamp_2+L_acclane,Loc_offRamp_2,...%5-6
          Loc_onRamp_3+L_acclane,Loc_offRamp_3,...%7-8
          Loc_onRamp_4+L_acclane,Loc_offRamp_4];%9-10;
%-------------------------------------------------------Adjacency matrix
adjmatrix_p0=zeros(10,10);
adjmatrix_p11=adjmatrix_p0;
adjmatrix_p11(1,2)=1;adjmatrix_p11(2,1)=1;
adjmatrix_p12=adjmatrix_p0;
adjmatrix_p12_2=adjmatrix_p0;
adjmatrix_p12_2(3,2)=1;
adjmatrix_p13=adjmatrix_p11;
adjmatrix_p14=adjmatrix_p11;
adjmatrix_p14(2,4)=1;

adjmatrix_p21=adjmatrix_p11;
adjmatrix_p22=adjmatrix_p0;
adjmatrix_p22_2=adjmatrix_p0;
adjmatrix_p22_2(5,2)=1;
adjmatrix_p23=adjmatrix_p11;
adjmatrix_p24=adjmatrix_p11;
adjmatrix_p24(2,6)=1;

adjmatrix_p31=adjmatrix_p11;
adjmatrix_p32=adjmatrix_p0;
adjmatrix_p32_2=adjmatrix_p0;
adjmatrix_p32_2(7,2)=1;
adjmatrix_p33=adjmatrix_p11;
adjmatrix_p34=adjmatrix_p11;
adjmatrix_p34(2,8)=1;

adjmatrix_p41=adjmatrix_p11;
adjmatrix_p42=adjmatrix_p0;
adjmatrix_p42_2=adjmatrix_p0;
adjmatrix_p42_2(9,2)=1;
adjmatrix_p43=adjmatrix_p11;
adjmatrix_p44=adjmatrix_p11;
adjmatrix_p44(2,10)=1;
adjmatrix_p45=adjmatrix_p11;
%----------------------------
coop_adjmatrix_p0=zeros(10,10);
coop_adjmatrix_p11=coop_adjmatrix_p0;
coop_adjmatrix_p11(2,3)=1;coop_adjmatrix_p11(3,2)=1;
coop_adjmatrix_p21=coop_adjmatrix_p0;
coop_adjmatrix_p21(2,5)=1;coop_adjmatrix_p21(5,2)=1;
coop_adjmatrix_p31=coop_adjmatrix_p0;
coop_adjmatrix_p31(2,7)=1;coop_adjmatrix_p31(7,2)=1;
coop_adjmatrix_p41=coop_adjmatrix_p0;
coop_adjmatrix_p41(2,9)=1;coop_adjmatrix_p41(9,2)=1;
%% Flow load
mainLaneRatio = 0.85;
offRampRatio = 1-mainLaneRatio;
prob_Mainlane_Exit=[mainLaneRatio^numIntersections/2, mainLaneRatio^numIntersections/2,...
           0, offRampRatio,...
           0, mainLaneRatio*offRampRatio,...
           0, mainLaneRatio^2*offRampRatio,...
           0, mainLaneRatio^3*offRampRatio];%
prob_Mainlane_Exit=cumsum(prob_Mainlane_Exit);
prob_Mainlane_Exit=[[0,prob_Mainlane_Exit(1:end-1)]',prob_Mainlane_Exit'];
prob_Mainlane_Exit=[{prob_Mainlane_Exit},{prob_Mainlane_Exit}];

prob_Bylane3_Exit=[mainLaneRatio^4/2, mainLaneRatio^4/2,...
                   0, offRampRatio,...
                   0, mainLaneRatio*offRampRatio,...
                   0, mainLaneRatio^2*offRampRatio,...
                   0, mainLaneRatio^3*offRampRatio];%
prob_Bylane5_Exit=[mainLaneRatio^3/2, mainLaneRatio^3/2,...
                   0, 0,...
                   0, offRampRatio,...
                   0, mainLaneRatio*offRampRatio,...
                   0, mainLaneRatio^2*offRampRatio];%
prob_Bylane7_Exit=[mainLaneRatio^2/2, mainLaneRatio^2/2,...
                   0, 0,...
                   0, 0,...
                   0, offRampRatio,...
                   0, mainLaneRatio*offRampRatio];%
prob_Bylane9_Exit=[mainLaneRatio/2, mainLaneRatio/2,...
                   0, 0,...
                   0, 0,...
                   0, 0,...
                   0, offRampRatio];%
kl=0;prob_Bylane_Exit={};
for lane_=[3,5,7,9]
    kl=kl+1;
    eval(['prob_Bylane_=prob_Bylane',num2str(lane_),'_Exit;'])
    prob_Exit_=cumsum(prob_Bylane_);
    prob_Bylane_Exit(kl)={[[0,prob_Exit_(1:end-1)]',prob_Exit_']};
end

rng(101)
rnd_TurnProb=rand(1e4,1);
%% Condition of time gap
syms tg_main tg_ramp
q_main=1/(tg_main*v_max+Gmin)*v_max*3600;%veh/h
q_ramp=1/(tg_ramp*v_max+Gmin)*v_max*3600;%veh/h
eq1 = 2*q_main + q_ramp == 2*Q_max;
eq2 = 2*q_main / q_ramp == mainLaneRatio / offRampRatio;
[tg_main, tg_ramp] = solve([eq1, eq2], [tg_main, tg_ramp]);
tg_main=double(tg_main);
tg_ramp=double(tg_ramp);

tg_main=(tg_main*v_max+Gmin)/v_max;
tg_ramp=(tg_ramp*v_max+Gmin)/v_max;
%--------------------------------------------------------------------------
nk_set=ones(2+4,1);
rng(102);
rnd_set=rand(step,2+4);
%--------------------------------------------------------------------------main road
lb_tg=0.25;ub_tg=0.5;p_formPlat_main=0;
rng(103);
tgap_main=randi(round([lb_tg,ub_tg]/dt),1,2);
mainplatTimegap_set=zeros(max_vehnum,2);
for j=1:num_mainlane
    rng(j);
    mainplatTimegap_set(:,j)=randi(round([lb_tg,ub_tg]/dt),max_vehnum,1);
end
%--------------------------------------------------------------------------ramp
lb_tg=0.8;ub_tg=1.1;p_formPlat_ramp=p_formPlat_main;
rng(102);
tgap_ramp=randi(round([lb_tg,ub_tg]/dt),1,4);
rampplatTimegap_set=zeros(max_vehnum,4);
kk=0;
for j=[3,5,7,9]
    kk=kk+1;
    rng(j);
    rampplatTimegap_set(:,kk)=randi(round([lb_tg,ub_tg]/dt),max_vehnum,1);
end

%--------------------------------------------------------------------------
flow_test=[];
for j=1:6
    if j<=num_mainlane
        platTimegap_=mainplatTimegap_set(:,j);
    else
        platTimegap_=rampplatTimegap_set(:,j-num_mainlane);
    end
    flow_test(j)=3600/(mean(platTimegap_)*0.05);%veh/h
end
diff_q=2*Q_max-sum(flow_test(1:3));
diff_q2=Q_max-sum(flow_test(2:3));
rate_ML=sum(flow_test(1:2))/(2*Q_max)*100;
rate_onRamp=flow_test(3)/(2*Q_max)*100;

%--------------------------------------------------------------------------
rng(103);
rnd_prelc=rand(max_vehnum,4);
Force_lc=zeros(max_vehnum,4);
%% Initialize
vehicles = struct();
rng(105);
rnd_initialpos=rand(1,4);
for j = 1:num_mainlane
    idx_lane=Mainlane_ID(j);
    vehicles(j).No = j; 
    vehicles(j).time = 1; 
    vehicles(j).pos = pos_Enter(idx_lane)+30*rnd_initialpos(j);   
    vehicles(j).spe = v_max;         
    vehicles(j).acc = 0;
    vehicles(j).acmd = 0;
    vehicles(j).IVS = inf;
    vehicles(j).lane = idx_lane;  
    vehicles(j).path = idx_lane; 
    vehicles(j).tenter = 1;           
    vehicles(j).OutOfSystem = 0;         
    vehicles(j).nextLaneID=idx_lane;
    vehicles(j).LCtLast=0;
    vehicles(j).initial_lane=idx_lane;
    vehicles(j).brakeWarning=0;
end
for j = num_mainlane+1:num_mainlane+num_bylane
    idx_lane=Bylane_ID(j-num_mainlane);
    vehicles(j).No = j;
    vehicles(j).time = 1;
    vehicles(j).pos = pos_Enter(idx_lane);
    vehicles(j).spe = v_max;       
    vehicles(j).acc = 0;
    vehicles(j).acmd = 0;
    vehicles(j).IVS = inf;
    vehicles(j).lane = idx_lane;  
    vehicles(j).path = idx_lane+1; 
    vehicles(j).tenter = 1;  
    vehicles(j).OutOfSystem = 0;
    vehicles(j).nextLaneID=idx_lane;
    vehicles(j).LCtLast=0;
    vehicles(j).initial_lane=idx_lane;
    vehicles(j).brakeWarning=0;
end
%-----------------------------------------------
data_vehicles=struct();
data_vehicles(1).time=vehicles;
Vehicle_Entry=[[vehicles.tenter]',[vehicles.lane]'];
%% main code
vehNo=max([vehicles.No]);
vehset_OutOfSystem=[];
Rate_lc_last=zeros(4,1);
cishu=0;
for t=1:step-1
    nveh=numel(vehicles);
    nveh_last=nveh;
    %----------------------------------------------------------------------main road
    [vehNo,nveh,vehicles,nk_set_,tgap_set]=fun_VehicleEnter(t,vehNo,nveh,vehicles,pos_Enter(1:2),Mainlane_ID,prob_Mainlane_Exit,rnd_TurnProb,mainplatTimegap_set,rnd_set(:,1:2),nk_set(1:2),tgap_main,p_formPlat_main,paraUpper,simu_setting);
    nk_set(1:2)=nk_set_;tgap_main=tgap_set;
    %----------------------------------------------------------------------ramp
    [vehNo,nveh,vehicles,nk_set_,tgap_set]=fun_VehicleEnter(t,vehNo,nveh,vehicles,pos_Enter([3,5,7,9]),Bylane_ID,prob_Bylane_Exit,rnd_TurnProb,rampplatTimegap_set,rnd_set(:,3:end),nk_set(3:end),tgap_ramp,p_formPlat_ramp,paraUpper,simu_setting);
    nk_set(3:end)=nk_set_;tgap_ramp=tgap_set;
    
    num_enter=nveh-nveh_last;
    Vehicle_Entry=[Vehicle_Entry;[[vehicles(nveh-num_enter+1:nveh).tenter]',[vehicles(nveh-num_enter+1:nveh).lane]']];
    
    data_vehicles(t).time=vehicles;
    vehicles(vehset_OutOfSystem)=[];
    nveh=numel(vehicles);
    
    %======================================================================
    if t<=2 
        dd=t-1;
    else
        dd=1;
    end
    if t>1
        vehicles_d1=data_vehicles(t-1).time;
        vehicles_d1_No=[vehicles_d1.No];
    end
    if t>dd 
        vehicles_dd=data_vehicles(t-dd).time;
        vehicles_dd_No=[vehicles_dd.No];
    end
    if t>para_AV(1,4) 
        vehicles_td=data_vehicles(t-td).time;
        vehicles_td_No=[vehicles_td.No];
    end
    
    %======================================================================
    if mod(t,100)==0
        [Rate_lc0,~]=fun_TrafficMonitor(t,Q_max,data_vehicles,pos_detector_main,pos_detector_onramp);
        Rate_lc=Rate_lc0(:,1);
        Rate_lc_last=Rate_lc;
    else
        Rate_lc=Rate_lc_last;
    end
    
    %======================================================================
    veh_status=vehicles;
    vehicles_laneNo=[veh_status.lane];
    vehicles_nextlaneNo=[veh_status.nextLaneID];
    vehicles_pos=[veh_status.pos];
    vehset_OutOfSystem=[];
    for kj=1:nveh
        currentVehicle = veh_status(kj);
        no_kj=currentVehicle.No;
        xj=currentVehicle.pos;
        vj=currentVehicle.spe;
        aj=currentVehicle.acc;
        acmdj=currentVehicle.acmd;
        t_enter=currentVehicle.tenter;
        j_lane=currentVehicle.lane;
        TargetLane_ID=currentVehicle.path;
        t_lc_last=currentVehicle.LCtLast;
        pos_exit_=pos_Exit(TargetLane_ID);

        %==================================================================
        %--------------------------------------------------
        if xj>100 && xj<=Loc_onRamp_1-x_cooperate
            adjmatrix_=adjmatrix_p11;
        elseif xj<=Loc_onRamp_1
            adjmatrix_=adjmatrix_p12;
        elseif xj<=Loc_onRamp_1+L_acclane
            adjmatrix_=adjmatrix_p12_2;
        elseif xj<=Loc_offRamp_1-L_acclane
            adjmatrix_=adjmatrix_p13;
        elseif xj<=Loc_offRamp_1
            adjmatrix_=adjmatrix_p14;
        elseif xj<=Loc_onRamp_2-x_cooperate
            adjmatrix_=adjmatrix_p21;
        elseif xj<=Loc_onRamp_2
            adjmatrix_=adjmatrix_p22;
        elseif xj<=Loc_onRamp_2+L_acclane
            adjmatrix_=adjmatrix_p22_2;
        elseif xj<=Loc_offRamp_2-L_acclane
            adjmatrix_=adjmatrix_p23;
        elseif xj<=Loc_offRamp_2
            adjmatrix_=adjmatrix_p24; 
        elseif xj<=Loc_onRamp_3-x_cooperate
            adjmatrix_=adjmatrix_p31;
        elseif xj<=Loc_onRamp_3
            adjmatrix_=adjmatrix_p32;
        elseif xj<=Loc_onRamp_3+L_acclane
            adjmatrix_=adjmatrix_p32_2;
        elseif xj<=Loc_offRamp_3-L_acclane
            adjmatrix_=adjmatrix_p33;
        elseif xj<=Loc_offRamp_3
            adjmatrix_=adjmatrix_p34;
        elseif xj<=Loc_onRamp_4-x_cooperate
            adjmatrix_=adjmatrix_p41;
        elseif xj<=Loc_onRamp_4
            adjmatrix_=adjmatrix_p42;
        elseif xj<=Loc_onRamp_4+L_acclane
            adjmatrix_=adjmatrix_p42_2;
        elseif xj<=Loc_offRamp_4-L_acclane
            adjmatrix_=adjmatrix_p43;
        elseif xj<=Loc_offRamp_4
            adjmatrix_=adjmatrix_p44;
        elseif xj>Loc_offRamp_4
            adjmatrix_=adjmatrix_p45;
        else
            adjmatrix_=adjmatrix_p0;
        end       
        
        %--------------------------------------------------
        if xj<=Loc_offRamp_1
            Loc_onRamp=Loc_onRamp_1;
            Loc_offRamp=Loc_offRamp_1;
            k_ramp=1;
        elseif xj<=Loc_offRamp_2
            Loc_onRamp=Loc_onRamp_2;
            Loc_offRamp=Loc_offRamp_2;
            k_ramp=2;
        elseif xj<=Loc_offRamp_3
            Loc_onRamp=Loc_onRamp_3;
            Loc_offRamp=Loc_offRamp_3;
            k_ramp=3;
        elseif xj>Loc_offRamp_3
            Loc_onRamp=Loc_onRamp_4;
            Loc_offRamp=Loc_offRamp_4;
            k_ramp=4;
        end
        %-------------------------------------------------- 
        if j_lane==1
            if sum([4,6,8,10]==TargetLane_ID)>0 && ...
                    xj>Loc_onRamp+L_acclane && pos_exit_==Loc_offRamp+L_subByroad 
                nextLaneID=2;
            else
                nextLaneID=0;
            end
        elseif j_lane==2
            if sum([4,6,8,10]==TargetLane_ID)>0 && ...
                    xj>Loc_onRamp+L_acclane && pos_exit_==Loc_offRamp+L_subByroad 
                nextLaneID=TargetLane_ID;
            else
                nextLaneID=0;
            end
        elseif sum([3,5,7,9]==j_lane)>0
            nextLaneID=2;
        elseif sum([4,6,8,10]==j_lane)>0
            nextLaneID=j_lane;
        end
        if j_lane==1 && xj<=Loc_onRamp+L_acclane
            nextLaneID=1;
        elseif j_lane==2
            if xj<Loc_onRamp-x_cooperate && rnd_prelc(no_kj,k_ramp)<Rate_lc(k_ramp) && sum([4,6,8,10]==TargetLane_ID)==0
                nextLaneID=1;
                cishu=cishu+1;
            end
        end
        vehicles(kj).nextLaneID=nextLaneID;
        %--------------------------------------------------
        if nextLaneID~=0
            if nextLaneID==1
                lc_FreeOrNot=-1;
            else
                lc_FreeOrNot=0;
            end
            nextLaneID_=nextLaneID;
        elseif nextLaneID==0
            nextLaneID_=3-j_lane;
            lc_FreeOrNot=1;
        end
        %-------------------------------------------------- 
        a_Exit_currentLane=inf;
        Flag_obstacle=0;
        if sum([3,5,7,9]==j_lane)>0 && nextLaneID==2
            Flag_obstacle=1;
        elseif sum([1,2]==j_lane)>0 && sum([4,6,8,10]==TargetLane_ID)>0
            Flag_obstacle=2;
        end
        if Flag_obstacle>0
            if Flag_obstacle==1
                pos_obstacle_=pos_obstacle(j_lane);
            elseif Flag_obstacle==2
                pos_obstacle_=pos_obstacle(TargetLane_ID);
            end
            if xj>=pos_obstacle_
                a_Exit_currentLane=-inf;
            end
        end
        
        %==================================================================
        [vehIdx_pre,frontVehicle,~,~]=fun_find_AdjacentVeh(veh_status,vehicles_laneNo,vehicles_pos,kj,currentVehicle,j_lane,0);
        if ~isnan(vehIdx_pre)
            dx = frontVehicle.pos - xj;
            nextLaneID_preveh=frontVehicle.nextLaneID;
            if veh_status(vehIdx_pre).tenter==t
                frontVehicle_dd = veh_status(vehIdx_pre);
            else
                no_pre=frontVehicle.No;
                idx_pre=vehicles_dd_No==no_pre;
                frontVehicle_dd = vehicles_dd(idx_pre);
            end
            dv1 = frontVehicle_dd.spe - vj;
            acmd_dd1 = frontVehicle_dd.acmd;
            npre=1;
            [vehIdx_pre2,front2Vehicle,~,~]=fun_find_AdjacentVeh(veh_status,vehicles_laneNo,vehicles_pos,vehIdx_pre,frontVehicle,j_lane,0);
            if ~isnan(vehIdx_pre2)
                nextLaneID_pre2veh=front2Vehicle.nextLaneID;
                if veh_status(vehIdx_pre2).tenter==t
                    front2Vehicle_dd = veh_status(vehIdx_pre2);
                else
                    no_pre2=veh_status(vehIdx_pre2).No;
                    idx_pre=vehicles_dd_No==no_pre2;
                    front2Vehicle_dd = vehicles_dd(idx_pre);
                end
                dv2 = front2Vehicle_dd.spe - vj;
                acmd_dd2 = front2Vehicle_dd.acmd;
                npre=2;
            else
                dv2=dv1;acmd_dd2=acmd_dd1;
                nextLaneID_pre2veh=nan;
            end
        else
            npre=0;
            dx=inf;dv1=0;acmd_dd1=0;
            nextLaneID_preveh=nan;
            vehIdx_pre2=nan;
            dv2=dv1;acmd_dd2=acmd_dd1;
            nextLaneID_pre2veh=nan;
        end
        vehicles(kj).IVS = dx;
        NeighborVeh_=[vehIdx_pre,vehIdx_pre2];
        %-------------------------------------------
        acmd_=fun_Update_VehicleAcmd(npre,dx,dv1,dv2,acmd_dd1,acmd_dd2,paraUpper,vj, a_Exit_currentLane,a_Signal,simu_setting);
        acmd_safe=acmd_;
        
        %==================================================================
        if ~isnan(vehIdx_pre) && veh_status(vehIdx_pre).brakeWarning==1
            dv2=dv1;acmd_dd2=acmd_dd1;
            acmd_brake=fun_Update_VehicleAcmd(npre,dx,dv1,dv2,acmd_dd1,acmd_dd2,paraUpper,vj, a_Exit_currentLane,a_Signal,simu_setting);
            acmd_=min(acmd_,acmd_brake);
            acmd_safe=acmd_;
        end
        
        %==================================================================
        %-------------------------------------------
        if xj>Loc_onRamp_1-x_cooperate && xj<=Loc_onRamp_1+L_acclane
            coop_adjmatrix_=coop_adjmatrix_p11;
        elseif xj>Loc_onRamp_2-x_cooperate && xj<=Loc_onRamp_2+L_acclane
            coop_adjmatrix_=coop_adjmatrix_p21;
        elseif xj>Loc_onRamp_3-x_cooperate && xj<=Loc_onRamp_3+L_acclane
            coop_adjmatrix_=coop_adjmatrix_p31;
        elseif xj>Loc_onRamp_4-x_cooperate && xj<=Loc_onRamp_4+L_acclane
            coop_adjmatrix_=coop_adjmatrix_p41;
        else
            coop_adjmatrix_=coop_adjmatrix_p0;
        end
        row_vec = coop_adjmatrix_(j_lane,:);
        coopAdjLane_set=find(row_vec==1);
        %-------------------------------------------
        Flag_mergecp=0;
        if (j_lane==2 && sum(coop_adjmatrix_(j_lane,[3,5,7,9]))>0) || sum([3,5,7,9]==j_lane)>0 && coop_adjmatrix_(j_lane,2)==1 
            cpLane_1=j_lane;
            cpLane_2=find(coop_adjmatrix_(j_lane,:)==1);
            [vehIdx_pre,frontVehicle]=fun_find_AdjacentVeh_cooperate(veh_status,vehicles_laneNo,vehicles_pos,kj,currentVehicle,cpLane_1,cpLane_2);
            %------------------------------------------
            if ~isnan(vehIdx_pre)
                dx_pre = frontVehicle.pos - xj;
                if veh_status(vehIdx_pre).tenter==t
                    frontVehicle_dd = veh_status(vehIdx_pre);
                else
                    no_pre=frontVehicle.No;
                    idx_pre=vehicles_dd_No==no_pre;
                    frontVehicle_dd = vehicles_dd(idx_pre);
                end
                dv_pre = frontVehicle_dd.spe - vj;
                acmd_dd_pre = frontVehicle_dd.acmd;
                npre=1;
                [vehIdx_pre2,front2Vehicle]=fun_find_AdjacentVeh_cooperate(veh_status,vehicles_laneNo,vehicles_pos,vehIdx_pre,frontVehicle,cpLane_1,cpLane_2);
                if ~isnan(vehIdx_pre2)
                    if veh_status(vehIdx_pre2).tenter==t
                        front2Vehicle_dd = veh_status(vehIdx_pre2);
                    else
                        no_pre2=veh_status(vehIdx_pre2).No;
                        idx_pre=vehicles_dd_No==no_pre2;
                        front2Vehicle_dd = vehicles_dd(idx_pre);
                    end
                    dv_pre2 = front2Vehicle_dd.spe - vj;
                    acmd_dd_pre2 = front2Vehicle_dd.acmd;
                    npre=2;
                else
                    dv_pre2=dv_pre;acmd_dd_pre2=acmd_dd_pre;
                end
            else
                npre=0;
                dx_pre=inf;dv_pre=0;acmd_dd_pre=0;
                vehIdx_pre2=nan;
                dv_pre2=dv_pre;acmd_dd_pre2=acmd_dd_pre;
            end
            a_Exit_adjLane=inf;
            acmd_cp=fun_Update_VehicleAcmd(npre,dx_pre,dv_pre,dv_pre2,acmd_dd_pre,acmd_dd_pre2,paraUpper,vj,a_Exit_adjLane,a_Signal,simu_setting);
            acmd_=min(acmd_,acmd_cp);
            Flag_mergecp=1;
        end
        
        %==================================================================
        if j_lane<=2 || sum([4,6,8,10]==j_lane)>0
            if j_lane<=2
                adjlaneID_=3-j_lane;
            elseif sum([4,6,8,10]==j_lane)>0
                adjlaneID_=2;
            end
            if adjmatrix_(j_lane,adjlaneID_)==1
                %--------------------------------------------------------------
                [vehIdx_pre,frontVehicle,~,~]=fun_find_AdjacentVeh_Giveway(veh_status,vehicles_laneNo,vehicles_nextlaneNo,vehicles_pos,kj,currentVehicle,j_lane,adjlaneID_,0);
                n_Neighborpre=0;
                if ~isnan(vehIdx_pre)
                    dx_pre = frontVehicle.pos - xj;
                    nextLaneID_NeighborPre = frontVehicle.nextLaneID;
                    if veh_status(vehIdx_pre).tenter==t
                        frontVehicle_dd = veh_status(vehIdx_pre);
                    else
                        no_pre=frontVehicle.No;
                        idx_pre=vehicles_dd_No==no_pre;
                        frontVehicle_dd = vehicles_dd(idx_pre);
                    end
                    dv_pre = frontVehicle_dd.spe - vj;
                    acmd_dd_pre = frontVehicle_dd.acmd;
                    n_Neighborpre=1;
                    vehIdx_NeighborPre=vehIdx_pre;
                end
                a_Exit_adjLane=inf;
                if n_Neighborpre==1 && length(nextLaneID_NeighborPre)==1 && nextLaneID_NeighborPre==j_lane 
                    vehIdx_pre=NeighborVeh_(1);
                    if isnan(vehIdx_pre) 
                        dv_pre2=dv_pre;
                        acmd_dd_pre2=acmd_dd_pre;
                        acmd_cp=fun_Update_VehicleAcmd(2,dx_pre,dv_pre,dv_pre2,acmd_dd_pre,acmd_dd_pre2,paraUpper,vj,a_Exit_adjLane,a_Signal,simu_setting);
                        acmd_=min(acmd_,acmd_cp);
                    else
                        if veh_status(vehIdx_NeighborPre).pos<veh_status(vehIdx_pre).pos
                            dv_pre2=dv1;acmd_dd_pre2=acmd_dd1;
                            acmd_cp=fun_Update_VehicleAcmd(2,dx_pre,dv_pre,dv_pre2,acmd_dd_pre,acmd_dd_pre2,paraUpper,vj,a_Exit_adjLane,a_Signal,simu_setting);
                            acmd_=min(acmd_,acmd_cp);
                        end
                    end
                end
                
                %--------------------------------------------------------------
                vehIdx_pre=NeighborVeh_(1);
                if ~isnan(vehIdx_pre)
                    frontvehicle=veh_status(vehIdx_pre);
                    [vehIdx_pre2,front2Vehicle,~,~]=fun_find_AdjacentVeh_Giveway(veh_status,vehicles_laneNo,vehicles_nextlaneNo,vehicles_pos,vehIdx_pre,frontvehicle,j_lane,adjlaneID_,0);
                    n_Neighborpre2=0;
                    if ~isnan(vehIdx_pre2)
                        nextLaneID_NeighborPre2 = front2Vehicle.nextLaneID;
                        if veh_status(vehIdx_pre2).tenter==t
                            front2Vehicle_dd = veh_status(vehIdx_pre2);
                        else
                            no_pre2=front2Vehicle.No;
                            idx_pre=vehicles_dd_No==no_pre2;
                            front2Vehicle_dd = vehicles_dd(idx_pre);
                        end
                        dv_pre2 = front2Vehicle_dd.spe - vj;
                        acmd_dd_pre2 = front2Vehicle_dd.acmd;
                        n_Neighborpre2=1;
                        vehIdx_NeighborPre2=vehIdx_pre2;
                    end
                    vehIdx_pre2=NeighborVeh_(2);
                    if n_Neighborpre2==1 && length(nextLaneID_NeighborPre2)==1 && nextLaneID_NeighborPre2==j_lane && ...
                        (isnan(vehIdx_pre2) || veh_status(vehIdx_NeighborPre2).pos<veh_status(vehIdx_pre2).pos)
                        a_Exit_adjLane=inf;
                        acmd_cp=fun_Update_VehicleAcmd(2,dx,dv1,dv_pre2,acmd_dd1,acmd_dd_pre2,paraUpper,vj,a_Exit_adjLane,a_Signal,simu_setting);
                        acmd_=min(acmd_,acmd_cp);
                    end
                end
            end
        end

        %==================================================================
        if j_lane~=nextLaneID_ && adjmatrix_(j_lane,nextLaneID_)==1
            %-------------------------------------------
            [vehIdx_pre,frontVehicle,vehIdx_fol,folVehicle]=fun_find_AdjacentVeh(veh_status,vehicles_laneNo,vehicles_pos,kj,currentVehicle,nextLaneID_,1);
            %-------------------------------------------
            if ~isnan(vehIdx_pre)
                dx_pre = frontVehicle.pos - xj;
                if veh_status(vehIdx_pre).tenter==t
                    frontVehicle_dd = veh_status(vehIdx_pre);
                else
                    no_pre=frontVehicle.No;
                    idx_pre=vehicles_dd_No==no_pre;
                    frontVehicle_dd = vehicles_dd(idx_pre);
                end
                dv_pre = frontVehicle_dd.spe - vj;
                acmd_dd_pre = frontVehicle_dd.acmd;
                npre=1;
                [vehIdx_pre2,front2Vehicle,~,~]=fun_find_AdjacentVeh(veh_status,vehicles_laneNo,vehicles_pos,vehIdx_pre,frontVehicle,nextLaneID_,0);
                if ~isnan(vehIdx_pre2)
                    if veh_status(vehIdx_pre2).tenter==t
                        front2Vehicle_dd = veh_status(vehIdx_pre2);
                    else
                        no_pre2=veh_status(vehIdx_pre2).No;
                        idx_pre=vehicles_dd_No==no_pre2;
                        front2Vehicle_dd = vehicles_dd(idx_pre);
                    end
                    dv_pre2 = front2Vehicle_dd.spe - vj;
                    acmd_dd_pre2 = front2Vehicle_dd.acmd;
                    npre=2;
                else
                    dv_pre2=dv_pre;acmd_dd_pre2=acmd_dd_pre;
                end
            else
                npre=0;
                dx_pre=inf;dv_pre=0;acmd_dd_pre=0;
                vehIdx_pre2=nan;
                dv_pre2=dv_pre;acmd_dd_pre2=acmd_dd_pre;
            end
            %-------------------------------------------
            a_Exit_adjLane=inf;
            acmd_lc=fun_Update_VehicleAcmd(npre,dx_pre,dv_pre,dv_pre2,acmd_dd_pre,acmd_dd_pre2,paraUpper,vj,a_Exit_adjLane,a_Signal,simu_setting);
            %-------------------------------------------
            if t-t_lc_last>dur_lc 
                %-------------------------------------------
                if ~isnan(vehIdx_fol)
                    dx_fol = xj - folVehicle.pos;
                    if veh_status(vehIdx_fol).tenter==t
                        folVehicle_dd = veh_status(vehIdx_fol);
                    else
                        no_fol=folVehicle.No;
                        idx_pre=vehicles_dd_No==no_fol;
                        folVehicle_dd = vehicles_dd(idx_pre);
                    end
                    dv_fol = vj - folVehicle_dd.spe;
                    dx_desired_fol=Tg_*folVehicle_dd.spe+Gmin;
                    acmd_fol_self = folVehicle_dd.acmd;
                    if ~isnan(vehIdx_pre)
                        dv_fol2 = frontVehicle_dd.spe - folVehicle_dd.spe;
                        acmd_fol=ka2_*acmd_dd_pre+ka1_*acmd_lc+kv2_*dv_fol2+kv1_*dv_fol+kg_*(dx_fol-dx_desired_fol);
                    else
                        acmd_fol=(ka2_+ka1_)*acmd_lc+(kv2_+kv1_)*dv_fol+kg_*(dx_fol-dx_desired_fol);
                    end
                    nfol=1;
                else
                    nfol=0;
                    acmd_fol_self=0;
                    acmd_fol=0;
                    dx_desired_fol=0;
                    dx_fol=inf;dv_fol=0;
                end
                %-------------------------------------------
                dx_desired_ego=Tg_*vj+Gmin;
                TTC_ego=(dx_pre-L_veh)/(-dv_pre);
                TTC_fol=(dx_fol-L_veh)/(-dv_fol);
                %-------------------------------------------
                Flag_lc=0;
                if lc_FreeOrNot==0 && dx_pre>=Gmin && dx_fol>=Gmin && ...
                        (TTC_ego<0 || TTC_ego>1.0) && (TTC_fol<0 || TTC_fol>1.0)
                    Flag_lc=1;
                elseif lc_FreeOrNot==-1 && dx_pre>=Gmin && dx_fol>=Gmin && ...
                        (TTC_ego<0 || TTC_ego>1.0) && (TTC_fol<0 || TTC_fol>1.0) && ...
                        vj>100/3.6 
                    Flag_lc=1;
                elseif lc_FreeOrNot==1 && dx_pre>=Gmin && dx_fol>=Gmin && ...
                        (TTC_ego<0 || TTC_ego>1.0) && (TTC_fol<0 || TTC_fol>1.0) && ...
                        acmd_lc>acmd_+0.5 && acmd_fol>=acmd_fol_self 
                    Flag_lc=1;
                end
                vehIdx_pre=NeighborVeh_(1);
                %-------------------------------------------
                if Flag_lc==1 && ...
                        ((sum([3,5,7,9]==j_lane)>0 && ~isnan(vehIdx_pre)) || ... 
                        (length(nextLaneID_preveh)==1 && nextLaneID_preveh==nextLaneID_) || ...
                        (length(nextLaneID_pre2veh)==1 && nextLaneID_pre2veh==nextLaneID_))
                    Flag_lc=0;
                end
                %-------------------------------------------
                if Flag_lc==1
                    acmd_=acmd_lc;
                    vehicles(kj).lane=nextLaneID_;
                    veh_status(kj).lane=nextLaneID_;
                    vehicles_laneNo(kj)=nextLaneID_;
                    vehicles(kj).LCtLast=t;
                    vehicles(kj).IVS = dx_pre;
                    vehicles(kj).brakeWarning=0;
                    veh_status(kj).brakeWarning=0;
                elseif lc_FreeOrNot==0 && Flag_lc==0
                    if Flag_mergecp==1
                        if dx_pre<Gmin || (TTC_ego>=0 && TTC_ego<=1)
                            acmd_=min([0,acmd_safe,acmd_cp]);
                        elseif dx_fol<Gmin || (TTC_fol>=0 && TTC_fol<=1)
                            acmd_=min(acmd_safe,max([0,acmd_cp]));
                        end
                    else 
                        if dx_pre<Gmin || (TTC_ego>=0 && TTC_ego<=1)
                            acmd_=min([0,acmd_,acmd_lc]);
                            vehicles(kj).brakeWarning=1;
                            veh_status(kj).brakeWarning=1;
                        elseif dx_fol<Gmin || (TTC_fol>=0 && TTC_fol<=1)
                            acmd_=min(acmd_safe,max([0,acmd_lc]));
                        end
                    end
                end
            end
        end
        
        %==================================================================lower
        X=para_AV(no_kj,:);
        K=X(1);theta=X(2);omega=X(3);td=X(4);
        if t-t_enter+1<=td
            adata_=acmd_;
        else
            idx_kj=vehicles_d1_No==no_kj;
            currentVehicle_t_1 = vehicles_d1(idx_kj);
            idx_kj=vehicles_td_No==no_kj;
            currentVehicle_t_td = vehicles_td(idx_kj);
            a_t_1=currentVehicle_t_1.acc;
            acmd_t_td=currentVehicle_t_td.acmd;
            eq_left=K*acmd_t_td;
            adata_=(eq_left-omega^2*aj+theta*omega*a_t_1/(2*dt)-(-2*aj+a_t_1)/(dt^2))*(dt^2)/(1+theta*omega/2*dt);
        end
        %==================================================================Update
        adata_=adata_+noise(t+1,no_kj);%Stochasticity
        vvv=max(0,vj+adata_*dt);
        vdata_new=min(v_max,vvv);
        xdata_new=xj+vdata_new*dt;
        adata_new=(vdata_new-vj)/dt;  
        
        vehicles(kj).time=t+1;
        vehicles(kj).spe=vdata_new;
        vehicles(kj).pos=xdata_new;
        vehicles(kj).acc=adata_new;
        vehicles(kj).acmd=acmd_;
        %==================================================================Boundary
        if (vehicles(kj).lane*TargetLane_ID==2 || vehicles(kj).lane==TargetLane_ID) && vehicles(kj).pos>pos_exit_
            vehicles(kj).OutOfSystem = t;
            vehset_OutOfSystem=[vehset_OutOfSystem;kj];
            continue;
        end
    end
    
    data_vehicles(t+1).time=vehicles;
end
%% plot
fun_DataArrange;
fun_TrafficFlowMeasure;
fun_Plot_BasicResult;