clc
clear
close all

% Code for Figure 3a-right in main text
%% Load
path='.';
load([path,'\Dataset_v3.mat']) 

label_UseOrNot=label_abnormal==0 & para_UpperLevel(:,7)==5;
idx=label_UseOrNot==0;
Dataset_v3(idx,:)=[];
Dataset_EqualTp_v2(idx,:)=[];
exp_scenario(idx,:)=[];
num_Run=length(Dataset_v3);
for nf=1:num_Run
    dataset_=Dataset_v3(nf,:);
    fun_dealNaN;
    Dataset_v3(nf,:)=dataset_;
end
%% Track
origin_xg=4.7090e+05;
origin_yg=4.3981e+06;
track_coef=[1084.75128505543;-12.2365842156786];
A=track_coef(2);B=-1;C=track_coef(1);
syms x0 y0
f_xnew=@(x0,y0)( (B*B*x0-A*B*y0-A*C)/(A*A+B*B) );
f_ynew=@(x0,y0)( (A*A*y0-A*B*x0-B*C)/(A*A+B*B) );
%%
nf2=0;
num_Run=length(Dataset_v3);
Stat_v={};Stat_dx={};
for nf=1:num_Run
    nf2=nf2+1;
    dataset_=Dataset_v3(nf,:);
    tp=dataset_{2}(:,1);
    data_xg=dataset_{3}-origin_xg;
    data_yg=dataset_{4}-origin_yg;
    data_dx=dataset_{5};
    data_v=dataset_{6};
    v_leader=data_v(:,2);
    data_Acmd=dataset_{8};  
    data_Aactual=dataset_{9};  

    Idx=max(find(v_leader==0));
    for jj=1:5
        data_v(Idx:end,jj+1)=data_v(Idx,jj+1);
        data_dx(Idx:end,jj)=data_dx(Idx,jj);
        data_Acmd(Idx:end,jj)=data_Acmd(Idx,jj);
        data_Aactual(Idx:end,jj)=data_Aactual(Idx,jj);
    end
    
    xstart=f_xnew(-10,1207);
    ystart=f_ynew(-10,1207);
    data_xgnew=f_xnew(data_xg,data_yg);
    data_ygnew=f_ynew(data_xg,data_yg);
    data_dis=sqrt( (data_xgnew-xstart).^2+(data_ygnew-ystart).^2 );
    data_dis=data_dis-data_dis(1,end);

    data_Input_set={};
    for nj=1:5
        Acmd_=data_Acmd(:,nj);
        Aactual_=data_Aactual(:,nj);
        v_=data_v(:,nj+1);
        dx_=data_dx(:,nj);
        dis_=data_dis(:,nj);
        data_calib=[tp,Acmd_,Aactual_,v_,dx_,dis_];
        data_Input_set(nj)={data_calib};
    end
    %----------------------------------------------------------------------Simulation
    for nj=2:5
        if nj==2
            data_Input=data_Input_set(nj-1:nj);
        elseif nj>2
            data_Input=data_Input_set(nj-2:nj);
        end        
        I_st=1;
        I_et=length(data_Input{1}(:,1));
        for jj=1:size(data_Input,2)
            if (nj==2 || nj==3) && jj==1
                data_Input_=data_Input{jj}(:,[1:4]);
            else
                data_Input_=data_Input{jj};
            end
            for kj=1:size(data_Input_,2)
                idx_st=min(find(~isnan(data_Input_(:,kj))));
                idx_et=max(find(~isnan(data_Input_(:,kj))));
            end
            I_st=max(I_st,idx_st);
            I_et=min(I_et,idx_et);
        end
        % 实验基准
        v_exp=data_Input{end}(:,4);
        dx_exp=data_Input{end}(:,5);
        st=max(find(v_exp(1:end-1)==0 & v_exp(2:end)>0))+1;
        et=max(find(v_exp(1:end-1)>0 & v_exp(2:end)==0));
        if isempty(I_st) || isempty(I_et)
            continue
        end
        if isempty(st) || st<I_st
            st=I_st;
        end    
        if isempty(et) || et>I_et
            et=I_et;
        end
        if nf==3 && nj==5
            et=2068;
        end
        v_exp=v_exp(st:et,:);
        dx_exp=dx_exp(st:et,:);
        data_Input_new={};
        for jj=1:size(data_Input,2)
            data_Input_=data_Input{jj}(st:et,:);
            data_Input_new(jj)={data_Input_};
        end
        data_Input=data_Input_new;
        %----------------------------------------------------------------------Exp-Det.Sim
        L_veh=4.835;
        % AV upper-level control parameters
        paraUpper=[0.4, 0.5, 0.1, 0.5, 0.4, 0];%Exp
        % AV lower-level control parameters
        paraset_AV=[78.8201836419897,2.07647787797603,9.17891305242795,4;
            58.9344898328179,1.90325205249387,7.86998881634774,4;
            95.9937146710127,2.32906700594111,10.0111919741363,4;
            43.0500120244655,1.62000868090560,6.87020881889564,4];
        para_AV=paraset_AV(nj-1,:);
        LowLevelModel_label=0;
        NoiseStrength=[0,0];
        fun=str2func(['fun_SimCFpair_Noise']);
        [vdata,xdata,adata_actual]=fun(data_Input,para_AV,paraUpper,LowLevelModel_label,NoiseStrength);
        v_sim=vdata(:,end);
        dx_sim=xdata(:,end-1)-xdata(:,end)-L_veh;
        %-------------------------------
        v_track=data_v(:,1)*3.6;
        td=fun_cal_TimeDelay(tp,100,v_track,data_v(:,2));
        I=round(td/0.05);
        v_track_shift=zeros(length(tp),1);
        v_track_shift([1+I:end])=v_track(1:end-I);
        vs_=exp_scenario(nf,2);vd_=exp_scenario(nf,3);acc_=exp_scenario(nf,4);
        % 分段
        fenge=[];
        v_=vs_-vd_/2;
        idx_11=find(v_track_shift(1:end-1)<v_ & v_track_shift(2:end)>=v_);
        idx_12=find(v_track_shift(1:end-1)>v_ & v_track_shift(2:end)<=v_);
        for j=1:length(idx_11)-1
            fenge_=round((idx_11(j)+idx_12(j))/2);
            fenge=[fenge;fenge_];
        end
        v_=vs_+vd_/2;
        idx_21=find(v_track_shift(1:end-1)<v_ & v_track_shift(2:end)>=v_);
        idx_22=find(v_track_shift(1:end-1)>v_ & v_track_shift(2:end)<=v_);
        idx_21=[idx_21;idx_12(end)];
        idx_22=[idx_11(end);idx_22];
        for j=1:length(idx_21)
            fenge_=round((idx_21(j)+idx_22(j))/2);
            fenge=[fenge;fenge_];
        end
        et_=min(et-st,fenge(end)-st);st_=fenge(1)-st;
        %Speed
        min_vexp=min(v_exp(st_:et_));min_vsim=min(v_sim(st_:et_));
        ave_vexp=nanmean(v_exp(st_:et_));ave_vsim=nanmean(v_sim(st_:et_));
        max_vexp=max(v_exp(st_:et_));max_vsim=max(v_sim(st_:et_));
        max_error=max(v_exp-v_sim);min_error=min(v_exp-v_sim);
        exp_=[min_vexp,ave_vexp,max_vexp];
        sim_=[min_vsim,ave_vsim,max_vsim];
        Stat_v(nf2,nj)={[exp_-sim_,max_error,min_error]};
        %Spacing
        min_dxexp=min(dx_exp(st_:et_));min_dxsim=min(dx_sim(st_:et_));
        ave_dxexp=nanmean(dx_exp(st_:et_));ave_dxsim=nanmean(dx_sim(st_:et_));
        max_dxexp=max(dx_exp(st_:et_));max_dxsim=max(dx_sim(st_:et_));
        max_error=max(dx_exp-dx_sim);min_error=min(dx_exp-dx_sim);
        exp_=[min_dxexp,ave_dxexp,max_dxexp];
        sim_=[min_dxsim,ave_dxsim,max_dxsim];
        Stat_dx(nf2,nj)={[exp_-sim_,max_error,min_error]};
    end
end

%% plot
vstat_set=[];
dxstat_set=[];
for nf=1:32
    for nj=2:5
        vstat_=Stat_v{nf,nj};
        dxstat_=Stat_dx{nf,nj};
        if ~isempty(vstat_)
            vstat_set=[vstat_set;vstat_];
            dxstat_set=[dxstat_set;dxstat_];
        end
    end 
end

legends={'Speed','Spacing'};
list_labels={'Max.Gap','Min.Gap'};
figure
yyaxis left
hold on
plot([0 3],[0 0],'k--')
b1=boxplot(vstat_set(:,4:5),'color','b','PlotStyle','compact');%
h = gca;
delta_x = -0.5;
h_all = findobj(h);
for i = 1:numel(h_all)
    if isprop(h_all(i), 'XData') && ~isempty(h_all(i).XData)
        xdata = h_all(i).XData;
        set(h_all(i), 'XData', xdata + delta_x);
    end
end
title('Sim-to-Real gap')
ylabel('In speed (m/s)')
set(gca,'YColor','b');
ylim([-1 1])

yyaxis right
hold on
b2=boxplot(dxstat_set(:,4:5),'color','r','PlotStyle','compact');
h = gca;
delta_x = -0.5;
h_all = findobj(h);
for i = 1:numel(h_all)
    if isprop(h_all(i), 'XData') && ~isempty(h_all(i).XData)
        xdata = h_all(i).XData;
        set(h_all(i), 'XData', xdata + delta_x);
    end
end
ylabel('In spacing (m)')
set(gca,'xtick',0.25:2.25)
set(gca,'xticklabel',list_labels)
set(gca,'YColor','r');
ylim([-3 3])
xlim([-0.25 1.75])
set(gca,'fontname','times new roman','fontsize',16)
grid on