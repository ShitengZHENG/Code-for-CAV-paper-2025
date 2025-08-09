clc
clear
close all

% Code for Figure 3a-left in main text
%% Load
path='.';
load([path,'\Dataset_v3.mat'])

%% Run Comparison
num_Run=length(Dataset_v3);
nf2=0;
max_len=0;
tIdx_set=[];
Data_ForCompare={};
for nf=1:num_Run
    file_no=Dataset_v3{nf,1};
    if file_no~=250 && file_no~=251
        continue
    end
    nf2=nf2+1;
    tp=Dataset_v3{nf,2}(:,1);
    data_dx=Dataset_v3{nf,5};
    data_v=Dataset_v3{nf,6};
    step=length(tp);
    
    v_track=data_v(:,1)*3.6;
    td=fun_cal_TimeDelay(tp,100,v_track,data_v(:,2));
    I=round(td/0.05);
    v_track_shift=zeros(step,1);
    v_track_shift([1+I:end])=v_track(1:end-I);
    
    vs_=exp_scenario(nf,2);vd_=exp_scenario(nf,3);acc_=exp_scenario(nf,4);
    max_len=max(max_len,length(v_track_shift));
    tIdx_set(nf2,1)=max(find(v_track_shift(1:end-1)==0 & v_track_shift(2:end)>0));
    Data_ForCompare(nf2,1:5)=[{file_no},{exp_scenario(nf,2:4)},{v_track_shift},{data_v},{data_dx}];
end
%% Plot
tdiff=tIdx_set(2)-tIdx_set(1);
mycolor=[0,114,189;217,83,25]/255;
mycolor2=[.7,.7,.7;.5,.5,.5];
fs=16;
fig=figure;
st=540;et=1500;
for nj=2:5
    leg_set=[];
    for jj=1:2
        if jj==1
            load('Simdata_R27.mat')
        elseif jj==2
            load('Simdata_R28.mat')
        end
        vdata=Data_ForCompare{jj,4};
        data_dx=Data_ForCompare{jj,5};
        if tdiff>0
            if jj==1
                I=0;
            elseif jj==2
                I=tdiff;
            end
        else
            if jj==1
                I=-tdiff;
            elseif jj==2
                I=0;
            end
        end
        vj=vdata(I+1:end,nj+1)*3.6;
        dxj=data_dx(I+1:end,nj);
        tt=([1:length(vj)]-1)*0.05;
        
        vj_sim=vdata_detsim(I+1:end,nj)*3.6;
        dxj_sim=dx_detsim(I+1:end,nj);
        %-------------------------------------------------------------------------speed
        subplot(2,4,nj-1)
        hold on
        plot(tt(st:et),vj(st:et),'-','color',mycolor(jj,:),'linewidth',1)
        plot(tt(st:et),vj_sim(st:et),'-.','color',mycolor2(jj,:),'linewidth',1)
        title(['Car ',num2str(nj)])
        xlim([tt(st) tt(et)])
        ylim([12 27])
        set(gca,'ytick',10:5:30)
        box on
        grid on
        set(gca,'fontname','times new roman','fontsize',fs)
        set(gca,'xticklabel',{''})
        if nj==2
            ylabel('Speed (km/h)')
        end
        %-------------------------------------------------------------------------spacing
        subplot(2,4,nj+3)
        hold on
        p1_=plot(tt(st:et),dxj(st:et),'-','color',mycolor(jj,:),'linewidth',1,'Parent',gca);
        p2_=plot(tt(st:et),dxj_sim(st:et),'-.','color',mycolor2(jj,:),'linewidth',1,'Parent',gca);
        leg_set=[leg_set;p1_;p2_];
        xlim([tt(st) tt(et)])
        ylim([4.5 5.8])
        set(gca,'fontname','times new roman','fontsize',fs)
        box on
        grid on
        if nj==2
            ylabel('Spacing (m)')
            xlabel('Time (s)')
        end
    end
end
p_leg=legend(leg_set,'Run-1 (Exp)','Run-1 (Sim)','Run-2 (Exp)','Run-2 (Sim)');
set(p_leg,'fontname','times new roman','fontsize',fs-2)
legend('boxoff')
set(gcf,'position', [60,80,1300,420])