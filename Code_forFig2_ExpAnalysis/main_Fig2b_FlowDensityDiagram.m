clc
clear
close all

% Code for Fig.2b in main text.
%% Load
path='.';
load([path,'\Dataset_v3.mat']) 
label_UseOrNot=label_abnormal==0 & para_UpperLevel(:,7)==5;
%% Flow-density measure
nf2=0;
num_Run=length(Dataset_v3);
FlowDensity_set=[];
for nf=1:num_Run
    if label_UseOrNot(nf)==0
        continue
    end
    nf2=nf2+1;
    tp=Dataset_v3{nf,2}(:,1);
    data_xg=Dataset_v3{nf,3};
    data_yg=Dataset_v3{nf,4};
    data_dx=Dataset_v3{nf,5};
    data_v=Dataset_v3{nf,6}*3.6;
    v_track=data_v(:,1);
    step=length(v_track);
    td=fun_cal_TimeDelay(tp,100,v_track,data_v(:,2));
    I=round(td/0.05);
    v_track_shift=zeros(step,1);
    v_track_shift([1+I:end])=v_track(1:end-I);
    %----------------------------------------------------------------------
    vs_=exp_scenario(nf,2);vd_=exp_scenario(nf,3);acc_=exp_scenario(nf,4);
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
    %----------------------------------------------------------------------
    L_veh=4.835;
    st=fenge(1);et=fenge(end);
    platV=nanmean(data_v(st:et,4:end),2)/3.6;%m/s
    if sum(~isnan(data_xg(:,2)))~=0
        platLen=sqrt((data_xg(:,2)-data_xg(:,5)).^2 + (data_yg(:,2)-data_yg(:,5)).^2);
    else
        platLen=sum(data_dx(:,3:5),2)+L_veh*3;
    end
    platDen=3./platLen(st:et);%/m
    platFlow=platV.*platDen*3600;
    FlowDensity_set(nf2,:)=[vs_,nanmean(platV)*3.6,nanmean(platDen),nanmean(platFlow)];
end
%% plot
figure
hold on
y=FlowDensity_set(:,4);
X=[ones(length(FlowDensity_set(:,2)),1),FlowDensity_set(:,2)];
[b,bint] = regress(y,X);
plot([0:120],b(1)+b(2)*[0:120],'k-','linewidth',1)
scatter(FlowDensity_set(:,2),FlowDensity_set(:,4),40,FlowDensity_set(:,4),'filled','markerfacealpha',0.5)
xlabel('Average speed (km/h)')
ylabel('Flow (veh/h)')
xlim([17 63])
ylim([1800 6000])
box on
grid on
set(gca,'fontname','times new roman','fontsize',18)
