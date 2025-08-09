clc
clear
close all

%% Basic simulation setting
max_vehnum=5000;
dt=0.05;
step=1.2e4;
a_comf=5;
v_max=60/3.6;
L_veh=4.835;
dur_lc=20;

simu_setting=[dt,v_max,L_veh,a_comf];
%% Parameter
%----------------------------------------------------AV-Upper controller
paraUpper=[0.6265    0.3703    0.4437    0.2889    0.6676    0.0736    1];
kv1_=paraUpper(1);kv2_=paraUpper(2);kg_=paraUpper(3);ka1_=paraUpper(4);ka2_=paraUpper(5);Tg_=paraUpper(6);
Gmin0=paraUpper(7);
Gmin=L_veh+Gmin0;

paraUpper1=[0.6265    0.3703    0.4437    0.2889    0.6676    0.0736    2];
dx_desire=Tg_*v_max+Gmin;
Q_max=1/dx_desire*v_max*3600;%veh/h
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
s_ksi = rng;
rnd_ksi=randn(step,max_vehnum);
noise=zeros(step,max_vehnum);
for t=1:step-1
    noise(t+1,:)=(1-kappa*dt)*noise(t,:)+sigma*rnd_ksi(t,:)*(dt^0.5);
end
%% Road network
L_f=30;%m
lane_width=3.75;
L_is=lane_width*2;
dis_leftTurn=L_is-lane_width/2;
dis_rightTurn=lane_width/2;
L_subroad=1000;
L_subByroad=500;
numIntersections=3;
L_road=L_subroad*(numIntersections+1)+L_is*numIntersections;
Loc_jiaochakou_1=L_subroad;
Loc_jiaochakou_2=L_subroad*2+L_is;
Loc_jiaochakou_3=L_subroad*3+L_is*2;

Mainlane_ID=[1,2,3,4];
num_mainlane=length(Mainlane_ID);
Bylane_ID=[5,6,7,8,9,10];
num_bylane=length(Bylane_ID);

pos_Enter=[-220,-220,-220,-220,...%1-4
    Loc_jiaochakou_1-L_subByroad,Loc_jiaochakou_1-L_subByroad+lane_width,...%5-6
    Loc_jiaochakou_2-L_subByroad,Loc_jiaochakou_2-L_subByroad+lane_width,...%7-8
    Loc_jiaochakou_3-L_subByroad,Loc_jiaochakou_3-L_subByroad+lane_width];%9-10
pos_Exit=[Loc_jiaochakou_1+L_is+L_subByroad,L_road,L_road,Loc_jiaochakou_1+lane_width+L_subByroad,...%1-4
    Loc_jiaochakou_2+L_is+L_subByroad,Loc_jiaochakou_2+lane_width+L_subByroad,...%5-6
    Loc_jiaochakou_3+L_is+L_subByroad,Loc_jiaochakou_3+lane_width+L_subByroad,...%7-8
    L_road,L_road];
pos_obstacle=pos_Exit;
%-------------------------------------------------------Adjacency matrix
adjmatrix_p0=zeros(10,10);
adjmatrix_p1=adjmatrix_p0;
adjmatrix_p1(1,2)=1;
adjmatrix_p1(2,1)=1;adjmatrix_p1(2,3)=1;
adjmatrix_p1(3,2)=1;adjmatrix_p1(3,4)=1;
adjmatrix_p1(4,3)=1;
adjmatrix_p2=adjmatrix_p0;
adjmatrix_p2(5,2)=1;
adjmatrix_p2(2,5)=1;adjmatrix_p2(2,3)=1;
adjmatrix_p2(3,2)=1;adjmatrix_p2(3,6)=1;
adjmatrix_p2(6,3)=1;
adjmatrix_p3=adjmatrix_p0;
adjmatrix_p3(7,2)=1;
adjmatrix_p3(2,7)=1;adjmatrix_p3(2,3)=1;
adjmatrix_p3(3,2)=1;adjmatrix_p3(3,8)=1;
adjmatrix_p3(8,3)=1;
adjmatrix_p4=adjmatrix_p0;
adjmatrix_p4(9,2)=1;
adjmatrix_p4(2,9)=1;adjmatrix_p4(2,3)=1;
adjmatrix_p4(3,2)=1;adjmatrix_p4(3,10)=1;
adjmatrix_p4(10,3)=1;
%% Flow load
leftTurnRatio = 0.1;        
rightTurnRatio = leftTurnRatio;        
straightRatio = 1-(leftTurnRatio+rightTurnRatio);         
prob_Mainlane_Exit0=[leftTurnRatio,straightRatio^4/2,straightRatio^4/2,rightTurnRatio,...%1-4
    straightRatio*leftTurnRatio,straightRatio*rightTurnRatio,...%5,6
    straightRatio^2*leftTurnRatio,straightRatio^2*rightTurnRatio,...%7,8
    straightRatio^3*leftTurnRatio,straightRatio^3*rightTurnRatio];%9,10
prob_Mainlane1_Exit=[leftTurnRatio*2,straightRatio^4/2,straightRatio^4/2,0,...%1-4
                    straightRatio*leftTurnRatio*2,0,...%5,6
                    straightRatio^2*leftTurnRatio*2,0,...%7,8
                    straightRatio^3*leftTurnRatio*2,0];%9,10
prob_Mainlane2_Exit=prob_Mainlane_Exit0;
prob_Mainlane3_Exit=prob_Mainlane_Exit0;
prob_Mainlane4_Exit=[0,straightRatio^4/2,straightRatio^4/2,rightTurnRatio*2,...%1-4
                    0,straightRatio*rightTurnRatio*2,...%5,6
                    0,straightRatio^2*rightTurnRatio*2,...%7,8
                    0,straightRatio^3*rightTurnRatio*2];%9,10;
kl=0;prob_Mainlane_Exit={};
for lane_=1:4
    kl=kl+1;
    eval(['prob_Mainlane_=prob_Mainlane',num2str(lane_),'_Exit;'])
    prob_Exit=cumsum(prob_Mainlane_);
    prob_Exit=[[0,prob_Exit(1:end-1)]',prob_Exit'];
    prob_Mainlane_Exit(kl)={prob_Exit};
end

prob_Bylane5_Exit=[0,straightRatio^3/2,straightRatio^3/2,0,...%1-4
    leftTurnRatio,rightTurnRatio,...%5,6
    straightRatio*leftTurnRatio,straightRatio*rightTurnRatio,...%7,8
    straightRatio^2*leftTurnRatio,straightRatio^2*rightTurnRatio];%9,10
prob_Bylane6_Exit=prob_Bylane5_Exit;
prob_Bylane7_Exit=[0,straightRatio^2/2,straightRatio^2/2,0,...%1-4
    0,0,...%5,6
    leftTurnRatio,rightTurnRatio,...%7,8
    straightRatio*leftTurnRatio,straightRatio*rightTurnRatio];%9,10
prob_Bylane8_Exit=prob_Bylane7_Exit;
prob_Bylane9_Exit=[0,straightRatio/2,straightRatio/2,0,...%1-4
    0,0,...%5,6
    0,0,...%7,8
    leftTurnRatio,rightTurnRatio];%9,10
prob_Bylane10_Exit=prob_Bylane9_Exit;
kl=0;prob_Bylane_Exit={};
for lane_=5:10
    kl=kl+1;
    eval(['prob_Bylane_=prob_Bylane',num2str(lane_),'_Exit;'])
    prob_Exit=cumsum(prob_Bylane_);
    prob_Exit=[[0,prob_Exit(1:end-1)]',prob_Exit'];
    prob_Bylane_Exit(kl)={prob_Exit};
end
%--------------------------
rng(101)
rnd_TurnProb=rand(max_vehnum,1);
%% Traffic light
greenTimeMain = 25/dt;          
redTimeMain = 15/dt;
greenTimeRamp = redTimeMain;          
redTimeRamp = greenTimeMain;
signalCycle=greenTimeMain+redTimeMain;
t_BeforeRed=3;
%% Solving the entry conditions
syms tg_main tg_ramp
q_main=1/(tg_main*v_max+Gmin)*v_max*3600;%veh/h
q_ramp=1/(tg_ramp*v_max+Gmin)*v_max*3600;%veh/h
Q_straight=(q_main*2+q_ramp*2)*straightRatio/2;
diff_q0=3.3/7*Q_max-Q_straight; 
diff_q2=(leftTurnRatio+rightTurnRatio)*(q_main*2+q_ramp*2)/2-q_ramp; 
eq1 = diff_q0 == 0;
eq2 = diff_q2 == 0;
[tg_main, tg_ramp] = solve([eq1, eq2], [tg_main, tg_ramp]);
tg_main=double(tg_main);
tg_ramp=double(tg_ramp);
tg_main=(tg_main*v_max+Gmin)/v_max;
tg_ramp=(tg_ramp*v_max+Gmin)/v_max;
%% 
rng(102);
rnd_set=rand(step,10);
nk_set=ones(10,1);
%--------------------------------------------------------------------------main road
mainplatTimegap_set=zeros(max_vehnum,4);
for j=1:num_mainlane
    rng(j);
    if sum([1,4]==j)>0
        lb_tg=tg_ramp-0.15;ub_tg=tg_ramp+0.15;p_formPlat_main=0;
    else
        lb_tg=tg_main-0.15;ub_tg=tg_main+0.15;p_formPlat_main=0;
    end
    mainplatTimegap_set(:,j)=randi(round([lb_tg,ub_tg]/dt),max_vehnum,1);
end
tgap_main=mainplatTimegap_set(end,:);
%--------------------------------------------------------------------------byroad
rampplatTimegap_set=zeros(max_vehnum,6);
kk=0;
for j=5:10
    kk=kk+1;
    rng(j);
    lb_tg=tg_ramp-0.15;ub_tg=tg_ramp+0.15;p_formPlat_ramp=0;
    rampplatTimegap_set(:,kk)=randi(round([lb_tg,ub_tg]/dt),max_vehnum,1);
end
tgap_ramp=rampplatTimegap_set(end,:);
%--------------------------------------------------------------------------Flowrate estimate
flowrate_theo=[];
for j=1:10
    if j<=num_mainlane
        platTimegap_=mainplatTimegap_set(:,j);
    else
        platTimegap_=rampplatTimegap_set(:,j-num_mainlane);
    end
    flowrate_theo(j)=3600/(mean(platTimegap_)*dt);%veh/h
end
Q_main=sum(flowrate_theo(1:4))/4;
Q_ramp=sum(flowrate_theo([5:10]))/6;
Q_straight=(sum(flowrate_theo(2:3))+sum(flowrate_theo([1,4])))*straightRatio/2;
diff_q0=3.3/7*Q_max-Q_straight 
diff_q2=(leftTurnRatio+rightTurnRatio)*Q_main*4/2-Q_ramp 
%--------------------------
all_probExit=[prob_Mainlane_Exit0*4+...
             [prob_Bylane5_Exit+prob_Bylane6_Exit+prob_Bylane7_Exit+...
                  prob_Bylane8_Exit+prob_Bylane9_Exit+prob_Bylane10_Exit]]'/10;
all_qout=[Q_main*prob_Mainlane_Exit0*4+...
          Q_ramp*[prob_Bylane5_Exit+prob_Bylane6_Exit+prob_Bylane7_Exit+...
                  prob_Bylane8_Exit+prob_Bylane9_Exit+prob_Bylane10_Exit]]';
% q_in
qin_main_1=Q_main*4;
qin_main_2=Q_ramp*2+qin_main_1-sum(all_qout([1,4]));
qin_main_3=Q_ramp*2+qin_main_2-sum(all_qout([5,6]));
qin_main_4=Q_ramp*2+qin_main_3-sum(all_qout([7,8]));
q_in=[qin_main_1,qin_main_2,qin_main_3,qin_main_4]';
% q_out
qout_main_1=sum(prob_Mainlane_Exit0)*Q_main*4;
qout_main_2=sum(prob_Mainlane_Exit0([2:3,5:10]))*Q_main*4+sum(prob_Bylane5_Exit([2:3,5:10]))*Q_ramp*2;
qout_main_3=sum(prob_Mainlane_Exit0([2:3,7:10]))*Q_main*4+sum(prob_Bylane5_Exit([2:3,7:10]))*Q_ramp*2+sum(prob_Bylane7_Exit([2:3,7:10]))*Q_ramp*2;
qout_main_4=sum(prob_Mainlane_Exit0([2:3,9:10]))*Q_main*4+sum(prob_Bylane5_Exit([2:3,9:10]))*Q_ramp*2+sum(prob_Bylane7_Exit([2:3,9:10]))*Q_ramp*2+sum(prob_Bylane9_Exit([2:3,9:10]))*Q_ramp*2;
q_out=[qout_main_1,qout_main_2,qout_main_3,qout_main_4]';
q_test=[q_in,q_out,q_in-q_out]
%% Initialize
vehicles = struct();
rng(103);
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
    vehicles(j).stopwarning=[0,1];
    vehicles(j).GiveAwayWarning=0;
    vehicles(j).discontWarning=[0,1];
    vehicles(j).LetGo=0;
    vehicles(j).nolightCycle=-1;
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
    vehicles(j).path = idx_lane;
    vehicles(j).tenter = 1;
    vehicles(j).OutOfSystem = 0;
    vehicles(j).nextLaneID=idx_lane;
    vehicles(j).LCtLast=0;
    vehicles(j).initial_lane=idx_lane;
    vehicles(j).brakeWarning=0;
    vehicles(j).stopwarning=[0,1];
    vehicles(j).GiveAwayWarning=0;
    vehicles(j).discontWarning=[0,1];
    vehicles(j).LetGo=0;
    vehicles(j).nolightCycle=-1;
end
%-----------------------------------------------
data_vehicles=struct();
data_vehicles(1).time=vehicles;
Vehicle_Entry=[[vehicles.tenter]',[vehicles.lane]'];
%% main code
vehNo=max([vehicles.No]);
vehset_OutOfSystem=[];
for t=1:step-1
    nveh=numel(vehicles);
    nveh_last=nveh;
    %----------------------------------------------------------------------main road
    [vehNo,nveh,vehicles,nk_set_,tgap_set]=fun_VehicleEnter(t,vehNo,nveh,vehicles,pos_Enter(1:4),Mainlane_ID,prob_Mainlane_Exit,rnd_TurnProb,mainplatTimegap_set,rnd_set(:,1:4),nk_set(1:4),tgap_main,p_formPlat_main,paraUpper,simu_setting);
    nk_set(1:4)=nk_set_;tgap_main=tgap_set;
    %----------------------------------------------------------------------byroad
    [vehNo,nveh,vehicles,nk_set_,tgap_set]=fun_VehicleEnter(t,vehNo,nveh,vehicles,pos_Enter(5:10),Bylane_ID,prob_Bylane_Exit,rnd_TurnProb,rampplatTimegap_set,rnd_set(:,5:10),nk_set(5:10),tgap_ramp,p_formPlat_ramp,paraUpper,simu_setting);
    nk_set(5:10)=nk_set_;tgap_ramp=tgap_set;
    
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
        L_f_front=-20;
        if xj>L_f_front && xj<Loc_jiaochakou_1-L_f
            adjmatrix_=adjmatrix_p1;
        elseif (xj>=Loc_jiaochakou_1+L_is && xj<Loc_jiaochakou_1+L_is+20) || (xj>Loc_jiaochakou_2-700 && xj<Loc_jiaochakou_2-L_f)
            adjmatrix_=adjmatrix_p2;
        elseif (xj>=Loc_jiaochakou_2+L_is && xj<Loc_jiaochakou_2+L_is+20) || (xj>Loc_jiaochakou_3-700 && xj<Loc_jiaochakou_3-L_f)
            adjmatrix_=adjmatrix_p3;
        elseif (xj>=Loc_jiaochakou_3+L_is && xj<Loc_jiaochakou_3+L_is+20) || xj>L_road-700
            adjmatrix_=adjmatrix_p4;
        else
            adjmatrix_=adjmatrix_p0;
        end
        %-------------------------------------------------- 
        x_light=inf;
        if sum([1:4,5,7,9]==j_lane)>0
            if xj<Loc_jiaochakou_1
                x_light=Loc_jiaochakou_1;
                nlight=1;
            elseif xj<Loc_jiaochakou_2
                x_light=Loc_jiaochakou_2;
                nlight=2;
            elseif xj<Loc_jiaochakou_3
                x_light=Loc_jiaochakou_3;
                nlight=3;
            else
                nlight=4;
            end
        elseif sum([6,8,10]==j_lane)>0
            if xj<Loc_jiaochakou_1+lane_width
                x_light=Loc_jiaochakou_1+lane_width;
                nlight=1;
            elseif xj<Loc_jiaochakou_2+lane_width
                x_light=Loc_jiaochakou_2+lane_width;
                nlight=2;
            elseif xj<Loc_jiaochakou_3+lane_width
                x_light=Loc_jiaochakou_3+lane_width;
                nlight=3;
            else
                nlight=4;
            end
        end
        %---------------------------------
        Flag_mainlight=0;Flag_ramplight=0;
        if sum([1:4]==j_lane)>0
            Flag_mainlight=1;
        elseif j_lane==5
            if xj<Loc_jiaochakou_1
                Flag_ramplight=1;
            elseif xj<Loc_jiaochakou_2
                Flag_mainlight=1;
            end
        elseif j_lane==7
            if xj<Loc_jiaochakou_2
                Flag_ramplight=1;
            elseif xj<Loc_jiaochakou_3
                Flag_mainlight=1;
            end
        elseif j_lane==9
            if xj<Loc_jiaochakou_3
                Flag_ramplight=1;
            end
        elseif j_lane==6
            if xj<Loc_jiaochakou_1+lane_width
                Flag_ramplight=1;
            elseif xj<Loc_jiaochakou_2+lane_width
                Flag_mainlight=1;
            end
        elseif j_lane==8
            if xj<Loc_jiaochakou_2+lane_width
                Flag_ramplight=1;
            elseif xj<Loc_jiaochakou_3+lane_width
                Flag_mainlight=1;
            end
        elseif j_lane==10
            if xj<Loc_jiaochakou_3+lane_width
                Flag_ramplight=1;
            end
        end
        %--------------------------------------------------
        if sum([1,5,7,9]==j_lane)>0 && j_lane~=TargetLane_ID && adjmatrix_(j_lane,2)==1
            nextLaneID=2;
        elseif sum([4,6,8,10]==j_lane)>0 && j_lane~=TargetLane_ID && adjmatrix_(j_lane,3)==1
            nextLaneID=3;
        elseif j_lane==2 && sum([4,6,8,10]==TargetLane_ID)>0 && adjmatrix_(j_lane,3)==1
            nextLaneID=3;
        elseif j_lane==2 && TargetLane_ID==3 && adjmatrix_(j_lane,3)==1
            nextLaneID=0;
        elseif j_lane==3 && sum([1,5,7,9]==TargetLane_ID)>0 && adjmatrix_(j_lane,2)==1
            nextLaneID=2;
        elseif j_lane==3 && TargetLane_ID==2 && adjmatrix_(j_lane,2)==1
            nextLaneID=0;
        else
            if ((nlight==1 && sum([1,4]==TargetLane_ID)>0) || ...
                (nlight==2 && sum([5,6]==TargetLane_ID)>0) || ...
                (nlight==3 && sum([7,8]==TargetLane_ID)>0) || ...
                (nlight==4 && sum([9,10]==TargetLane_ID)>0)) && adjmatrix_(j_lane,TargetLane_ID)==1
                nextLaneID=TargetLane_ID;
            else
                nextLaneID=j_lane;
            end
        end
        vehicles(kj).nextLaneID=nextLaneID;
        %--------------------------------------------------
        if nextLaneID~=0
            lc_FreeOrNot=0;
            nextLaneID_=nextLaneID;
        elseif nextLaneID==0
            nextLaneID_=6/j_lane;
            lc_FreeOrNot=1;
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
        %==================================================================Traffic light
        d_BeforeLight=L_veh/2;
        dv=-vj;
        dx_light=x_light-xj;
        dx_desired=Tg_*vj+Gmin-L_veh/2;
        a_light=(kv2_+kv1_)*dv+kg_*(dx_light-d_BeforeLight-dx_desired);
        if Flag_mainlight==1 && Flag_ramplight==0 
            time_left=greenTimeMain-mod(t, signalCycle);
            no_cycle=fix(t/signalCycle);
        elseif Flag_mainlight==0 && Flag_ramplight==1 
            time_left=greenTimeRamp-(mod(t-redTimeRamp, signalCycle));
            no_cycle=fix((t+greenTimeRamp)/signalCycle);
        end
        signalState = time_left>0;  
        Flag_redlight=0;Flag_greenlight=0;
        if signalState 
            if veh_status(kj).nolightCycle==no_cycle || (vj>1 && (x_light-xj)/vj>time_left-t_BeforeRed/dt) 
                Flag_redlight=1;
                veh_status(kj).nolightCycle=no_cycle;
                vehicles(kj).nolightCycle=no_cycle;
            else
                Flag_greenlight=1;
            end
        else 
            Flag_redlight=1;
            veh_status(kj).nolightCycle=no_cycle;
            vehicles(kj).nolightCycle=no_cycle;
        end
        if Flag_redlight==1
            a_Signal=a_light;
            if a_light<0 
                veh_status(kj).stopwarning=[1,nlight];
                vehicles(kj).stopwarning=[1,nlight];
            end
        else
            a_Signal=inf;
        end
        if vj<10/3.6
            veh_status(kj).stopwarning=[1,nlight];
            vehicles(kj).stopwarning=[1,nlight];
        end
        %================================================================== 
        a_Exit_currentLane=inf;
        Flag_obstacle=1;
        if ((j_lane==2 || j_lane==3) && pos_Exit(TargetLane_ID)-L_subroad-L_is>x_light) || ...
                (xj>Loc_jiaochakou_3 && pos_Exit(TargetLane_ID)==L_road) 
            Flag_obstacle=0;
        end
        if Flag_obstacle==1 && j_lane~=nextLaneID
            x_obstacle=x_light-L_f;
            if xj>=x_obstacle
                a_Exit_currentLane=-inf;
            end
        end
        %==================================================================
        acmd_=fun_Update_VehicleAcmd(npre,dx,dv1,dv2,acmd_dd1,acmd_dd2,paraUpper,vj, a_Exit_currentLane,a_Signal,simu_setting);
        acmd_safe=acmd_;
        %==================================================================
        %------------------------------------------------------
        if Flag_redlight==1
            if isnan(NeighborVeh_(1)) || (~isnan(NeighborVeh_(1)) && frontVehicle.pos>x_light) 
                dx=dx_light;dv1_1=dv;dv2_1=dv;acmd_dd1_1=0;acmd_dd2_1=0;
                veh_status(kj).discontWarning=[1,nlight];
                vehicles(kj).discontWarning=[1,nlight];
            elseif isnan(NeighborVeh_(2)) || (~isnan(NeighborVeh_(2)) && front2Vehicle.pos>x_light) 
                dv1_1=dv1;dv2_1=dv1;acmd_dd1_1=acmd_dd1;acmd_dd2_1=acmd_dd1;
                veh_status(kj).discontWarning=[1,nlight];
                vehicles(kj).discontWarning=[1,nlight];
            end
            if veh_status(kj).discontWarning(1)==1 
                acmd_brake=fun_Update_VehicleAcmd(npre,dx,dv1_1,dv2_1,acmd_dd1_1,acmd_dd2_1,paraUpper,vj, a_Exit_currentLane,a_Signal,simu_setting);
                acmd_=min(acmd_,acmd_brake);
                acmd_safe=acmd_;
            end
        end        
        %------------------------------------------------------
        if ~isnan(vehIdx_pre2) && (front2Vehicle_dd.pos-xj>5*dx)
            veh_status(kj).discontWarning=[-1,nlight];
            vehicles(kj).discontWarning=[-1,nlight];
        end
        if veh_status(kj).discontWarning(1)~=0 && nlight>veh_status(kj).discontWarning(2) && ...
                (~isnan(vehIdx_pre2) && front2Vehicle.pos-xj<=dx_desire*2)
            veh_status(kj).discontWarning=[0,nlight];
            vehicles(kj).discontWarning=[0,nlight];
        end
        
        if veh_status(kj).discontWarning(1)==-1 
            dv1_2=dv1;dv2_2=dv1;acmd_dd1_2=acmd_dd1;acmd_dd2_2=acmd_dd1;
            acmd_=fun_Update_VehicleAcmd(npre,dx,dv1_2,dv2_2,acmd_dd1_2,acmd_dd2_2,paraUpper,vj, a_Exit_currentLane,a_Signal,simu_setting);
            acmd_safe=acmd_;
        end
        %==================================================================
        %------------------------------------------------------
        if (Flag_greenlight==1 && vj>10/3.6) || nlight>veh_status(kj).stopwarning(2)
            veh_status(kj).stopwarning=[0,nlight];
            vehicles(kj).stopwarning=[0,nlight];
        end
        if (~isnan(vehIdx_pre) && veh_status(vehIdx_pre).stopwarning(1)==1) 
            acmd_brake=fun_Update_VehicleAcmd(npre,dx,dv1_3,dv2_3,acmd_dd1_3,acmd_dd2_3,paraUpper1,vj, a_Exit_currentLane,a_Signal,simu_setting);
            acmd_=min(acmd_,acmd_brake);
            acmd_safe=acmd_;
        end
        %------------------------------------------------------ 
        if ~isnan(vehIdx_pre) && vj<frontVehicle_dd.spe && acmdj<=0
            veh_status(kj).GiveAwayWarning=1;
            vehicles(kj).GiveAwayWarning=1;
        end
        if ~isnan(vehIdx_pre) && vj>=frontVehicle_dd.spe && acmdj>=0 && dx<=vj*Tg_+Gmin
            veh_status(kj).GiveAwayWarning=0;
            vehicles(kj).GiveAwayWarning=0;
        end
        %------------------------------------------------------
        if (~isnan(vehIdx_pre) && veh_status(vehIdx_pre).brakeWarning==1) || ... 
           (~isnan(vehIdx_pre2) && veh_status(vehIdx_pre2).brakeWarning==1) || ...
           (~isnan(vehIdx_pre) && veh_status(vehIdx_pre).GiveAwayWarning==1) 
            dv1_4=dv1;dv2_4=dv1;acmd_dd1_4=acmd_dd1;acmd_dd2_4=acmd_dd1;
            acmd_brake=fun_Update_VehicleAcmd(npre,dx,dv1_4,dv2_4,acmd_dd1_4,acmd_dd2_4,paraUpper,vj, a_Exit_currentLane,a_Signal,simu_setting);
            acmd_=min(acmd_,acmd_brake);
            acmd_safe=acmd_;
        end
        
        %==================================================================
        if lc_FreeOrNot==0
            cpLane_1=j_lane;
            cpLane_2=nextLaneID_;
            [vehIdx_pre,frontVehicle]=fun_find_AdjacentVeh_cooperate(veh_status,vehicles_laneNo,vehicles_pos,kj,currentVehicle,cpLane_1,cpLane_2);
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
        end
        
        %==================================================================
        row_vec = adjmatrix_(j_lane,:);
        adj_lane_set=find(row_vec==1);
        for jj=1:length(adj_lane_set)
            adjlaneID_=adj_lane_set(jj);
            if lc_FreeOrNot==1 && veh_status(kj).LetGo~=0 
               break 
            end
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
                    veh_status(vehIdx_NeighborPre).LetGo=1;
                    vehicles(vehIdx_NeighborPre).LetGo=1;
                else
                    if veh_status(vehIdx_NeighborPre).pos<veh_status(vehIdx_pre).pos
                        dv_pre2=dv1;acmd_dd_pre2=acmd_dd1;
                        acmd_cp=fun_Update_VehicleAcmd(2,dx_pre,dv_pre,dv_pre2,acmd_dd_pre,acmd_dd_pre2,paraUpper,vj,a_Exit_adjLane,a_Signal,simu_setting);
                        acmd_=min(acmd_,acmd_cp);
                        veh_status(vehIdx_NeighborPre).LetGo=1;
                        vehicles(vehIdx_NeighborPre).LetGo=1;
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
                    veh_status(vehIdx_NeighborPre2).LetGo=1;
                    vehicles(vehIdx_NeighborPre2).LetGo=1;
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
                Flag_lc=0;TTC_threshold=1;
                if lc_FreeOrNot==0 && dx_pre>=dx_desired_ego && dx_fol>=dx_desired_fol && ...
                        (TTC_ego<0 || TTC_ego>TTC_threshold) && (TTC_fol<0 || TTC_fol>TTC_threshold)
                    Flag_lc=1;
                elseif lc_FreeOrNot==1 && dx_pre>=dx_desired_ego && dx_fol>=dx_desired_fol && ...
                        (TTC_ego<0 || TTC_ego>TTC_threshold) && (TTC_fol<0 || TTC_fol>TTC_threshold) && ...
                        acmd_lc>acmd_+0.5 && acmd_fol>=acmd_fol_self
                    Flag_lc=1;
                end
                vehIdx_pre=NeighborVeh_(1);
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
                    veh_status(kj).LetGo=0;
                    vehicles(kj).LetGo=0;
                elseif lc_FreeOrNot==0 && Flag_lc==0 
                    if dx_pre<dx_desired_ego || (TTC_ego>=0 && TTC_ego<=TTC_threshold)
                        acmd_=min([0,acmd_,acmd_lc]);
                        vehicles(kj).brakeWarning=1;
                        veh_status(kj).brakeWarning=1;
                    elseif dx_fol<dx_desired_ego || (TTC_fol>=0 && TTC_fol<=TTC_threshold)
                        acmd_=min(acmd_safe,max([0,acmd_lc]));
                    end
                end
            end
        end
        %==================================================================Lower
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
        if vdata_new==0 && adata_new==0 && acmd_<0
            acmd_=0;
        end
        vehicles(kj).time=t+1;
        vehicles(kj).spe=vdata_new;
        vehicles(kj).pos=xdata_new;
        vehicles(kj).acc=adata_new;
        vehicles(kj).acmd=acmd_;
        %==================================================================Boundary
        if ((vehicles(kj).lane==2 && TargetLane_ID==3) || (vehicles(kj).lane==3 && TargetLane_ID==2) || vehicles(kj).lane==TargetLane_ID) && ...
                vehicles(kj).pos>pos_exit_
            vehicles(kj).OutOfSystem = t;
            vehset_OutOfSystem=[vehset_OutOfSystem;kj];
            continue;
        end
    end
    
    data_vehicles(t+1).time=vehicles;
end
%% 
fun_DataArrange;
fun_TrafficFlowMeasure;
fun_Plot_BasicResult;