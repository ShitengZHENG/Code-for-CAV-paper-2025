
%% 
Result_total={};
%--------------------------------------------------------------------------Average travel delay
td_set=[];
for j=1:total_nveh
    I=find(~isnan(data_pos(:,j)));
    et=max(I);st=min(I);
    t_travel=(et-st)*0.05;
    t_theo=(data_pos(et,j)-data_pos(st,j))/v_max;
    td_=t_travel-t_theo;
    td_set=[td_set;td_];
end
ave_delay=mean(td_set);
Result_total(1)={ave_delay};
%--------------------------------------------------------------------------Average duration
Result_=[];
for jj=1:4
    if jj==1
        Loc_onRamp=Loc_onRamp_1;
        Loc_offRamp=Loc_offRamp_1;
    elseif jj==2
        Loc_onRamp=Loc_onRamp_2;
        Loc_offRamp=Loc_offRamp_2;
    elseif jj==3
        Loc_onRamp=Loc_onRamp_3;
        Loc_offRamp=Loc_offRamp_3;
    elseif jj==4
        Loc_onRamp=Loc_onRamp_4;
        Loc_offRamp=Loc_offRamp_4;
    end

    dur_set_onramp=[];dur_set_offramp=[];
    for j=1:total_nveh
        idx=find(~isnan(data_laneNo(:,j)));
        if isempty(idx)
            continue
        end
        min_idx=min(idx);
        max_idx=max(idx);
        initialLane_ID=data_laneNo(min_idx,j);
        targetLane_ID=data_laneNo(max_idx,j);
        %----------------on-ramp
        if sum([3,5,7,9]==initialLane_ID)>0 && (initialLane_ID-1)/2==jj
            st=find(data_pos(1:end-1,j)<=Loc_onRamp & data_pos(2:end,j)>Loc_onRamp);
            et=find(data_laneNo(1:end-1,j)>2 & data_laneNo(2:end,j)<=2);
            t_dur=(et-st)*0.05;
            dur_set_onramp=[dur_set_onramp;t_dur];
        end
        %----------------off-ramp
        if sum([4,6,8,10]==targetLane_ID)>0 && (targetLane_ID-2)/2==jj
            st=find(data_pos(1:end-1,j)<=Loc_offRamp-L_acclane & data_pos(2:end,j)>Loc_offRamp-L_acclane);
            et=find(data_laneNo(1:end-1,j)==2 & data_laneNo(2:end,j)>2);
            t_dur=(et-st)*0.05;
            dur_set_offramp=[dur_set_offramp;t_dur];
        end
    end
    ave_dur_onramp=mean(dur_set_onramp);
    ave_dur_offramp=mean(dur_set_offramp);
    
    Result_=[Result_;ave_dur_onramp,ave_dur_offramp];
end
Result_total(2)={Result_};
%--------------------------------------------------------------------------Average speed
label_noncoop=[1,3,5,7,9,11,13,15,17];
label_onramp=[2,6,10,14];
label_offramp=[4,8,12,16];
area=[0,Loc_onRamp_1-x_cooperate;%1
      Loc_onRamp_1-x_cooperate,Loc_onRamp_1+L_acclane;%2
      Loc_onRamp_1+L_acclane,Loc_offRamp_1-L_acclane;%3
      Loc_offRamp_1-L_acclane,Loc_offRamp_1;%4
      Loc_offRamp_1,Loc_onRamp_2-x_cooperate;%5
      Loc_onRamp_2-x_cooperate,Loc_onRamp_2+L_acclane;%6
      Loc_onRamp_2+L_acclane,Loc_offRamp_2-L_acclane;%7
      Loc_offRamp_2-L_acclane,Loc_offRamp_2;%8
      Loc_offRamp_2,Loc_onRamp_3-x_cooperate;%9
      Loc_onRamp_3-x_cooperate,Loc_onRamp_3+L_acclane;%10
      Loc_onRamp_3+L_acclane,Loc_offRamp_3-L_acclane;%11
      Loc_offRamp_3-L_acclane,Loc_offRamp_3;%12
      Loc_offRamp_3,Loc_onRamp_4-x_cooperate;%13
      Loc_onRamp_4-x_cooperate,Loc_onRamp_4+L_acclane;%14
      Loc_onRamp_4+L_acclane,Loc_offRamp_4-L_acclane;%15
      Loc_offRamp_4-L_acclane,Loc_offRamp_4;%16
      Loc_offRamp_4,L_road];%17
num_segment=size(area,1);
Result_=[];
for jj=1:num_segment
    idx_1=data_laneNo<=2;
    idx_2=data_pos>area(jj,1) & data_pos<area(jj,2);
    idx=idx_1 & idx_2;
    aveV_=mean(vdata(idx))*3.6;
    %----------------------------Save
    Result_=[Result_;aveV_];
end
Result_total(3)={Result_};
%% Flow rate
%--------------------------------------------------------------------------detector
pos_detector=mean(area,2);
pos_detector(label_offramp)=[Loc_offRamp_1,Loc_offRamp_2,Loc_offRamp_3,Loc_offRamp_4]';
Result_=[];k_onramp=0;k_offramp=0;
for jj=1:num_segment
    xloc=pos_detector(jj);
    %---------------------------------main road
    flow_main_loop=[];
    for k_lane=1:2
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
        flow_main_loop(k_lane)=flow;
    end
    %---------------------------------on-ramp
    if sum(jj==label_onramp)>0
        xx=data_pos;
        vv=vdata;
        idx_1=data_laneNo==3 | data_laneNo==5 | data_laneNo==7 | data_laneNo==9;
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
        flow_onramp_loop=flow;
    else
        flow_onramp_loop=nan;
    end
    
    %---------------------------------off-ramp
    if sum(jj==label_offramp)>0
        xx=data_pos;
        vv=vdata;
        idx_1=data_laneNo==4 | data_laneNo==6 | data_laneNo==8 | data_laneNo==10;
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
        flow_offramp_loop=flow;
    else
        flow_offramp_loop=nan;
    end
    Result_=[Result_;flow_main_loop,flow_onramp_loop,flow_offramp_loop];
end
Result_total(4)={Result_};
%% Flow and speed
%--------------------------------------------------------------------------detector
xx=data_pos;
vv=vdata;
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
end
%------------------------------------------------
area=[0,Loc_onRamp_1-L_subByroad;%1
    Loc_onRamp_1+L_acclane,Loc_offRamp_1-L_acclane;
    Loc_offRamp_1+L_subByroad,Loc_onRamp_2-L_subByroad;%6
    Loc_onRamp_2+L_acclane,Loc_offRamp_2-L_acclane;
    Loc_offRamp_2+L_subByroad,Loc_onRamp_3-L_subByroad;%6
    Loc_onRamp_3+L_acclane,Loc_offRamp_3-L_acclane;
    Loc_offRamp_3+L_subByroad,Loc_onRamp_4-L_subByroad;%6
    Loc_onRamp_4+L_acclane,Loc_offRamp_4-L_acclane;
    Loc_offRamp_4+L_subByroad,L_road];%17
num_seg=size(area,1);
label_=zeros(length(pos_detector),1);
for jj=1:length(pos_detector)
    idx=pos_detector(jj)>area(:,1) & pos_detector(jj)<=area(:,2);
    if sum(idx)>0
        label_(jj)=1;
    end
end
Result_FlowANDSpe=[Result_FlowANDSpe,label_];