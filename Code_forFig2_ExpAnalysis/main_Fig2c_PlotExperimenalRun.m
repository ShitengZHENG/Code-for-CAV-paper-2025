clc
clear
close all

% Code for Figure 2c in main text
%% Load
path='D:\000-Research work\000-Paper\paper22-CACC experiment\20250712-Transferµ½Commun Eng\Final revision\Upload_Github_20250804\Data_file';
load([path,'\Dataset_v3.mat'])
label_UseOrNot=label_abnormal==0 & para_UpperLevel(:,7)==5;
%--------------------------------------------------------------------------
nf=45;
file_no=Dataset_v3{nf,1};
tp=Dataset_v3{nf,2}(:,1);
data_dx=Dataset_v3{nf,5};
data_v=Dataset_v3{nf,6}*3.6;
v_track=data_v(:,1);
data_Acmd=Dataset_v3{nf,8};
step=length(v_track);
for jj=1:5
    Idx=max(find(data_v(:,jj)==0));
    data_v(Idx:end,jj)=0;
end
td=fun_cal_TimeDelay(tp,100,v_track,data_v(:,2));
I=round(td/0.05);
v_track_shift=zeros(step,1);
v_track_shift([1+I:end])=v_track(1:end-I);
% Speed smooth
data_=[zeros(100,5);data_v(:,2:end)/3.6;zeros(100,5)];
[acc,~]=fun_smooth_v_fft(data_,20,2);
data_Aactual=acc(101:end-100,:);
%% Plot
color_jet=[0,0,0.562500000000000;0,0.312500000000000,1;0.0625000000000000,1,0.937500000000000;0.812500000000000,1,0.187500000000000;1,0.437500000000000,0];
vs_=exp_scenario(nf,2);vd_=exp_scenario(nf,3);acc_=exp_scenario(nf,4);
tt=tp-tp(1);
%---------------------------------
fs=17;
figure
subplot(4,1,1)
hold on
p_=plot(tt,v_track_shift,'k--','linewidth',1);
plot([tt(1) tt(end)],[vs_ vs_],'b-.')
plot([tt(1) tt(end)],[vs_ vs_]+vd_,'b--')
plot([tt(1) tt(end)],[vs_ vs_]-vd_,'b--')
p_leg=p_;leg_label={'Target'};
for jj=1:5
    p_=plot(tt,data_v(:,jj+1),'r','color',color_jet(jj,:),'linewidth',1);
    p_leg=[p_leg;p_];
    leg_label=[leg_label;{['Car ',num2str(jj)]}];
end
p_leg=legend(p_leg,leg_label);
legend('boxoff')
ylabel('{\itv\rm} (km/h)')
xlim([tt(1) tt(end)])
ylim([-5 95])
set(gca,'ytick',0:30:100)
set(gca,'xticklabel',{''})
box on
set(gca,'position',[0.13 0.7215 0.775 0.166]);
set(gca,'Fontname','Times New Roman', 'FontSize', fs)

subplot(4,1,2)
hold on
max_dx=max(data_dx(:));
plot([tt(1) tt(end)],[max_dx max_dx],'b-.')
for jj=2:5
    p_=plot(tt,data_dx(:,jj),'r','color',color_jet(jj,:),'linewidth',1);
end
ylabel('{\Delta\itx\rm} (m)')
set(gca,'xticklabel',{''})
xlim([tt(1) tt(end)])
ylim([3 8.2])
set(gca,'ytick',0:8)
box on
set(gca,'position',[0.13 0.5293 0.775 0.166]);
set(gca,'Fontname','Times New Roman', 'FontSize', fs)

subplot(4,1,4)
hold on
for jj=2:5
    p_=plot(tt,data_Aactual(:,jj),'color',color_jet(jj,:),'linewidth',1);
end
ylim([-4 4])
plot([tt(1) tt(end)],[-2 -2],'b:')
plot([tt(1) tt(end)],[-1 -1],'k:')
plot([tt(1) tt(end)],[0 0],'k-.')
plot([tt(1) tt(end)],[1 1],'k:')
plot([tt(1) tt(end)],[2 2],'b:')
aaa_=[data_Aactual];
max_a=ceil(max(aaa_(:))/0.5)*0.5;min_a=floor(min(aaa_(:))/0.5)*0.5;
set(gca,'ytick',floor(min(aaa_(:))):ceil(max(aaa_(:))))
xlim([tt(1) tt(end)])
ylim([min_a max_a])
xlabel('Time (s)')
ylabel('{\ita\rm_{actual}} (m/s^{\fontsize{10}2})')
box on
set(gca,'position',[0.13 0.145 0.775 0.166]);
set(gca,'Fontname','Times New Roman', 'FontSize', fs)

subplot(4,1,3)
hold on
for nj=2:5
    v_=data_v(:,nj+1);
    Acmd_=data_Acmd(:,nj);
    Acmd_smooth=fun_smooth_fftHighPassFilter([0;Acmd_;0],20,1);
    [pks,locs]=findpeaks(-Acmd_smooth);
    idx=pks>0.5;
    tt_pk=locs(idx);
    if ~isempty(tt_pk)
        [pks2,locs2]=findpeaks(Acmd_smooth);
        locs2=[1;locs2;length(Acmd_smooth)];
        for jj=1:length(tt_pk)
            i_pk=tt_pk(jj);
            diff=i_pk-locs2;diff(diff<0)=nan;
            [~,I]=min(diff);
            st_=max(locs2(I)-5,1);
            diff=locs2-i_pk;diff(diff<0)=nan;
            [~,I]=min(diff);
            et_=min(locs2(I)+5,length(v_));
            aa_=interp1([st_,et_],Acmd_([st_,et_]),st_:et_);
            Acmd_(st_:et_)=aa_;
        end
    end
    p_=plot(tt,Acmd_,'color',color_jet(nj,:),'linewidth',1);
end
ylim([-4 4])
plot([tt(1) tt(end)],[-2 -2],'b:')
plot([tt(1) tt(end)],[-1 -1],'k:')
plot([tt(1) tt(end)],[0 0],'k-.')
plot([tt(1) tt(end)],[1 1],'k:')
plot([tt(1) tt(end)],[2 2],'b:')
set(gca,'ytick',floor(min(aaa_(:))):ceil(max(aaa_(:))))
set(gca,'xticklabel',{''})
xlim([tt(1) tt(end)])
ylim([min_a max_a])
ylabel('{\ita\rm_{cmd}} (m/s^{\fontsize{10}2})')
box on
set(gca,'position',[0.13 0.3372 0.775 0.166]);
set(gca,'Fontname','Times New Roman', 'FontSize', fs)

set(gcf,'position', [60,10,1100,700])