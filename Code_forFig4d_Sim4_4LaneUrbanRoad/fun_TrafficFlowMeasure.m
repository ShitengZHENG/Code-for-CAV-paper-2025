
% Mainroad
idx_t=mod(t0+[1:step]', signalCycle)==0;
idx_t=repmat(idx_t,1,total_nveh);
idx_r=data_movdir==0;
pos_1=Loc_jiaochakou_1-data_pos;pos_1(pos_1<0)=nan;
pos_2=Loc_jiaochakou_2-data_pos;pos_2(pos_2<0)=nan;
pos_3=Loc_jiaochakou_3-data_pos;pos_3(pos_3<0)=nan;
idx=idx_t & idx_r;
mindis_fromSL1=min([pos_1(idx);pos_2(idx);pos_3(idx)]);

% byroad in the left
idx_t=mod(t0+[1:step]'-redTimeRamp, signalCycle)==0;
idx_t=repmat(idx_t,1,total_nveh);
idx_r=data_movdir==pi/2;
pos_1=Loc_jiaochakou_1-data_pos;pos_1(pos_1<0)=nan;
pos_2=Loc_jiaochakou_2-data_pos;pos_2(pos_2<0)=nan;
pos_3=Loc_jiaochakou_3-data_pos;pos_3(pos_3<0)=nan;
idx=idx_t & idx_r;
mindis_fromSL2=min([pos_1(idx);pos_2(idx);pos_3(idx)]);

% byroad in the right
idx_t=mod(t0+[1:step]'-redTimeRamp, signalCycle)==0;
idx_t=repmat(idx_t,1,total_nveh);
idx_r=data_movdir==-pi/2;
pos_1=Loc_jiaochakou_1+lane_width-data_pos;pos_1(pos_1<0)=nan;
pos_2=Loc_jiaochakou_2+lane_width-data_pos;pos_2(pos_2<0)=nan;
pos_3=Loc_jiaochakou_3+lane_width-data_pos;pos_3(pos_3<0)=nan;
idx=idx_t & idx_r;
mindis_fromSL3=min([pos_1(idx);pos_2(idx);pos_3(idx)]);
dis_=[mindis_fromSL1,mindis_fromSL2,mindis_fromSL3]-L_veh/2;

%% Flow rate estimate
tenter_=Vehicle_Entry(:,1);
laneID_=Vehicle_Entry(:,2);
flowrate_Input=nan(10,1);
tenter_main_set=[];tenter_ramp_set=[];
for k_lane=1:10
    idx=laneID_==k_lane;
    t_enter=tenter_(idx);
    t_enter_=(t_enter(2:end)-t_enter(1:end-1))*0.05;
    flowrate_Input(k_lane,1)=3600/mean(t_enter_);%veh/h
    if k_lane>=2 && k_lane<=3
        tenter_main_set=[tenter_main_set;t_enter_];
    else
        tenter_ramp_set=[tenter_ramp_set;t_enter_];
    end
end
diff=flowrate_Input-flowrate_theo;
%% 
Result_total={};
%--------------------------------------------------------------------------Average travel delay
td_set=[];
for j=1:total_nveh
    I=find(~isnan(data_pos(:,j)));
    et=max(I);st=min(I);
    t_travel=(et-st)*dt;
    t_theo=(data_pos(et,j)-data_pos(st,j))/v_max;
    td_=t_travel-t_theo;
    td_set=[td_set;td_];
end
ave_delay=mean(td_set);
Result_total(1)={ave_delay};
%--------------------------------------------------------------------------Average duration
tlc_set=[];
for j=1:total_nveh
    id_1=data_initLaneID(j);
    id_2=data_targetLaneID(j);
    if isnan(id_1)|| isnan(id_2) || ...
            id_1==id_2 || (id_1==2 && id_2==3) || (id_1==3 && id_2==2)
        continue
    elseif id_2==1 || id_2==4
        s_loc=0;
    elseif id_2==5 || id_2==6
        s_loc=Loc_jiaochakou_1+L_is;
    elseif id_2==7 || id_2==8
        s_loc=Loc_jiaochakou_2+L_is;
    elseif id_2==9 || id_2==10
        s_loc=Loc_jiaochakou_3+L_is;
    end
    laneID_=data_laneNo(:,j);
    laneID_(data_movdir(:,j)~=0 | data_pos(:,j)<=s_loc)=nan;
    st=min(find(laneID_==id_1));
    et=min(find(laneID_==id_2));
    t_lc=(et-st)*dt;
    tlc_set=[tlc_set;t_lc];
end
ave_tlc=mean(tlc_set);
Result_total(2)={ave_tlc};
%% Flow and speed
%--------------------------------------------------------------------------detector
dir=data_movdir;
idx=dir~=0;
xx=data_pos;xx(idx)=nan;
vv=vdata;vv(idx)=nan;
pos_detector=10:10:L_road;
Result_FlowANDSpe=[];
for jj=1:length(pos_detector)
    xloc=pos_detector(jj);
    t_set=[];v_set=[];
    vehnum_=0;
    for j=1:total_nveh
        tI=find(xx(1:step-1,j)<xloc & xx(2:step,j)>=xloc);
        if isempty(tI)
            v_set(j)=nan;
            t_set(j)=nan;
            continue
        end
        vehnum_=vehnum_+1;
        v_set(j)=vv(tI,j);
        t_set(j)=tI;
    end
    flow=3600*vehnum_/((max(t_set)-min(t_set))*dt);%veh/h
    speed=v_set*3.6;
    ave_Speed=1/(nanmean(1./speed));
    Result_FlowANDSpe(jj,1)=flow;
    Result_FlowANDSpe(jj,2)=ave_Speed;
    Result_FlowANDSpe(jj,3)=vehnum_;
end
Result_total(3)={Result_FlowANDSpe};

%--------------------------------------------------------------------------detector
pos_detector2=[Loc_jiaochakou_1-600,Loc_jiaochakou_1-300,Loc_jiaochakou_1+300, ...
              Loc_jiaochakou_2-600,Loc_jiaochakou_2-300,Loc_jiaochakou_2+300, ...
              Loc_jiaochakou_3-600, Loc_jiaochakou_3-300,Loc_jiaochakou_3+300, ...
              L_road-600];
laneID_set=[1,2,3,4;
            5,1,6,4;
            5,1,6,4;
            5,2,3,6;
            7,5,8,6;
            7,5,8,6;
            7,2,3,8;
            9,7,10,8;
            9,7,10,8;
            9,2,3,10];
num_segment=length(pos_detector2);
Result_flow=[];Result_spe=[];
for jj=1:num_segment
    xloc=pos_detector2(jj);
    %---------------------------------main road
    flow_main_loop=[];spe_main_loop=[];
    for k_lane=1:10
        xx=data_pos;
        vv=vdata;
        idx_1=data_laneNo==k_lane;
        xx(~idx_1)=nan;
        vv(~idx_1)=nan;
        t_set=[];v_set=[];
        vehnum_=0;
        for j=1:total_nveh
            tI=find(xx(1:step-1,j)<xloc & xx(2:step,j)>=xloc);
            if isempty(tI)
                v_set(j)=nan;
                t_set(j)=nan;
                continue
            end
            vehnum_=vehnum_+1;
            v_set(j)=vv(tI,j);
            t_set(j)=tI;
        end
        flow=3600*vehnum_/((max(t_set)-min(t_set))*dt);%veh/h
        speed=v_set*3.6;
        ave_Speed=1/(nanmean(1./speed));
        flow_main_loop(k_lane)=flow;
        spe_main_loop(k_lane)=ave_Speed;
    end
    laneID_=laneID_set(jj,:);
    flow_=flow_main_loop(laneID_);
    spe_=spe_main_loop(laneID_);
    Result_flow=[Result_flow;flow_];
    Result_spe=[Result_spe;spe_];
end
Result_flow(2,:)=[nan,Result_flow(2,1),Result_flow(3,2),nan];
Result_flow(3,:)=[nan,Result_flow(2,3),Result_flow(3,4),nan];
Result_flow(5,:)=[nan,Result_flow(5,1),Result_flow(6,2),nan];
Result_flow(6,:)=[nan,Result_flow(5,3),Result_flow(6,4),nan];
Result_flow(8,:)=[nan,Result_flow(8,1),Result_flow(9,2),nan];
Result_flow(9,:)=[nan,Result_flow(8,3),Result_flow(9,4),nan];
Result_spe(2,:)=[nan,Result_spe(2,1),Result_spe(3,2),nan];
Result_spe(3,:)=[nan,Result_spe(2,3),Result_spe(3,4),nan];
Result_spe(5,:)=[nan,Result_spe(5,1),Result_spe(6,2),nan];
Result_spe(6,:)=[nan,Result_spe(5,3),Result_spe(6,4),nan];
Result_spe(8,:)=[nan,Result_spe(8,1),Result_spe(9,2),nan];
Result_spe(9,:)=[nan,Result_spe(8,3),Result_spe(9,4),nan];
Result_total(4)={Result_flow};
Result_total(5)={Result_spe};
