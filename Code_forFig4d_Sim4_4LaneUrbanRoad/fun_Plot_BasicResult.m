close all
mycmap_2=[0.188200000000000,0.207800000000000,0.576500000000000;0.189900000000000,0.211200000000000,0.578700000000000;0.191600000000000,0.214600000000000,0.581000000000000;0.193300000000000,0.217900000000000,0.583200000000000;0.195000000000000,0.221300000000000,0.585400000000000;0.196600000000000,0.224600000000000,0.587700000000000;0.198300000000000,0.228000000000000,0.589900000000000;0.200000000000000,0.231400000000000,0.592200000000000;0.171400000000000,0.271700000000000,0.619600000000000;0.142900000000000,0.312000000000000,0.647100000000000;0.114300000000000,0.352400000000000,0.674500000000000;0.0857000000000000,0.392700000000000,0.702000000000000;0.0571000000000000,0.433100000000000,0.729400000000000;0.0286000000000000,0.473400000000000,0.756900000000000;0,0.513700000000000,0.784300000000000;0,0.542300000000000,0.806200000000000;0,0.570900000000000,0.828000000000000;0,0.599400000000000,0.849900000000000;0,0.628000000000000,0.871700000000000;0,0.656600000000000,0.893600000000000;0,0.685200000000000,0.915400000000000;0,0.713700000000000,0.937300000000000;0.0218000000000000,0.717100000000000,0.901400000000000;0.0437000000000000,0.720400000000000,0.865500000000000;0.0655000000000000,0.723800000000000,0.829700000000000;0.0874000000000000,0.727200000000000,0.793800000000000;0.109200000000000,0.730500000000000,0.758000000000000;0.131100000000000,0.733900000000000,0.722100000000000;0.152900000000000,0.737300000000000,0.686300000000000;0.242600000000000,0.754600000000000,0.639200000000000;0.332200000000000,0.772000000000000,0.592200000000000;0.421800000000000,0.789400000000000,0.545100000000000;0.511500000000000,0.806700000000000,0.498000000000000;0.601100000000000,0.824100000000000,0.451000000000000;0.690800000000000,0.841500000000000,0.403900000000000;0.780400000000000,0.858800000000000,0.356900000000000;0.810600000000000,0.851000000000000,0.322700000000000;0.840900000000000,0.843100000000000,0.288500000000000;0.871100000000000,0.835300000000000,0.254300000000000;0.901400000000000,0.827500000000000,0.220200000000000;0.931700000000000,0.819600000000000,0.186000000000000;0.961900000000000,0.811800000000000,0.151800000000000;0.992200000000000,0.803900000000000,0.117600000000000;0.984900000000000,0.758500000000000,0.128300000000000;0.977600000000000,0.713200000000000,0.138900000000000;0.970300000000000,0.667800000000000,0.149600000000000;0.963000000000000,0.622400000000000,0.160200000000000;0.955700000000000,0.577000000000000,0.170900000000000;0.948500000000000,0.531700000000000,0.181500000000000;0.941200000000000,0.486300000000000,0.192200000000000;0.926600000000000,0.447600000000000,0.188800000000000;0.912000000000000,0.409000000000000,0.185400000000000;0.897500000000000,0.370300000000000,0.182100000000000;0.882900000000000,0.331700000000000,0.178700000000000;0.868300000000000,0.293000000000000,0.175400000000000;0.853800000000000,0.254300000000000,0.172000000000000;0.839200000000000,0.215700000000000,0.168600000000000;0.798300000000000,0.207800000000000,0.164700000000000;0.757400000000000,0.200000000000000,0.160800000000000;0.716500000000000,0.192200000000000,0.156900000000000;0.675600000000000,0.184300000000000,0.152900000000000;0.634700000000000,0.176500000000000,0.149000000000000;0.593800000000000,0.168600000000000,0.145100000000000;0.552900000000000,0.160800000000000,0.141200000000000];

%--------------------------------------------------------------------------Vehicle Arrival Distribution
figure
hold on
histogram(tenter_main_set,[0:0.05:5])
histogram(tenter_ramp_set,[0:0.05:5])
legend('Mainline','On-ramp')
xlabel('Entry time gap (s)')
ylabel('Count')
set(gca,'fontname','times new roman','fontsize',16)
box on
set(gca,'xtick',0:0.2:2)
xlim([0 1.4])

%--------------------------------------------------------------------------Flow&speed
default_color1=[[70,160,182];[78,102,178];[78,102,178];[70,160,182]]/255;
default_color2=[[1,1,1];[135,207,168];[254,236,159];[1,1,1]]/255;
totalQ_loop=Result_total{4};
aveSpe_loop=Result_total{5};
figure
subplot(2,1,1)
hold on
b_1=bar(aveSpe_loop,0.7);
for j=1:4
    b_1(j).FaceColor=default_color1(j,:);
    b_1(j).EdgeColor=default_color1(j,:);
end
b_2=bar([2,3,5,6,8,9],aveSpe_loop([2,3,5,6,8,9],:),0.7);
for j=1:4
    b_2(j).FaceColor=default_color2(j,:);
    b_2(j).EdgeColor=default_color2(j,:);
end
plot([0 10],[v_max v_max]*3.6,'k--')
ylabel([{'Average speed'};{'(km/h)'}])
set(gca,'xtick',1:7)
set(gca,'xticklabel',{''})
set(gca,'ytick',0:20:70)
set(gca,'fontname','times new roman','fontsize',16)
box on
set(gca,'xtick',1:10)
xlim([0.5 10.5])
ylim([0 62])
set(gca, 'position',[0.13 0.52 0.775 0.39]);

subplot(2,1,2)
hold on
yyaxis left
b_1=bar(totalQ_loop,0.7);
for j=1:4
    b_1(j).FaceColor=default_color1(j,:);
    b_1(j).EdgeColor=default_color1(j,:);
end
b_2=bar([2,3,5,6,8,9],totalQ_loop([2,3,5,6,8,9],:),0.7);
for j=1:4
    b_2(j).FaceColor=default_color2(j,:);
    b_2(j).EdgeColor=default_color2(j,:);
end
xlabel('Detector No.')
ylabel([{'Flow rate per lane'};{'(veh/h/lane)'}])
set(gca,'YColor','black');
set(gca,'ytick',0:1000:5000)
yyaxis right
p_1=plot(nansum(totalQ_loop,2),'r-s','markerfacecolor','r','linewidth',1);
legend([b_1([2,1]),b_2(2:3),p_1],'Main lane-Through','Main lane-Left/right turn','Byroad-Inflow','Byroad-Outflow','Total')
ylabel([{'Total flow rate'};{'(veh/h)'}])
xlim([0.5 10.5])
set(gca,'xtick',1:10)
set(gca,'YColor','black');
set(gca,'fontname','times new roman','fontsize',16)
box on
set(gca, 'position',[0.13 0.11 0.775 0.39]);
set(gcf,'position', [1,100,1300,420])

%--------------------------------------------------------------------------Flow&speed
figure
subplot(1,2,1)
hold on
xx=Result_FlowANDSpe(:,1);yy=pos_detector;
plot(xx,yy,'-','color',[78,102,178]/255,'linewidth',3)
xspan=[0 1.4e4];
for j=1:3
    eval(['Loc_jiaochakou_=Loc_jiaochakou_',num2str(j),';'])
    h=fill([xspan, xspan(2),xspan(1)],[[Loc_jiaochakou_,Loc_jiaochakou_] [Loc_jiaochakou_,Loc_jiaochakou_]+L_is],'c');
    set(h,'edgealpha',0,'facealpha',0.8)
end
xlim([8800 1.12e4])
set(gca,'xtick',8000:500:12000)
ylim([0 4010])
ylabel('Position (m)')
xlabel('Flow rate (veh/h)')
set(gca,'fontname','times new roman','fontsize',20)
box on
p_leg=legend([h],'Intersection');
set(p_leg,'fontname','times new roman','fontsize',20)
legend('boxoff')
set(gca, 'position',[0.13 0.145 0.39 0.77]);

subplot(1,2,2)
hold on
xx=Result_FlowANDSpe(:,2);yy=pos_detector;
plot(xx,yy,'-','color',[70,160,182]/255,'linewidth',3)
xspan=[-5 70];
for j=1:3
    eval(['Loc_jiaochakou_=Loc_jiaochakou_',num2str(j),';'])
    h=fill([xspan, xspan(2),xspan(1)],[[Loc_jiaochakou_,Loc_jiaochakou_] [Loc_jiaochakou_,Loc_jiaochakou_]+L_is],'c');
    set(h,'edgealpha',0,'facealpha',0.8)
end
set(gca, 'position',[0.52 0.145 0.39 0.77]);
xlim([-5 70])
ylim([0 4010])
set(gca,'yticklabel',{''})
xlabel('Average speed (km/h)')
set(gca,'YColor','black');
set(gca,'fontname','times new roman','fontsize',20)
box on

%--------------------------------------------------------------------------Trajectory
t0=data_vehicles(1).time(1).time-1;
t0_lt=fix(t0/signalCycle)*signalCycle;
nCycle=ceil(step/signalCycle)+1;
figure
tt_=(t0+[1:step])'*dt;
for k_lane=[1:4]
    if k_lane==1
        Idx_plot=1;
        title_str='Main lane 1';
    elseif k_lane==2
        Idx_plot=2;
        title_str='Main lane 2';
    elseif k_lane==3
        Idx_plot=3;
        title_str='Main lane 3';
    elseif k_lane==4
        Idx_plot=4;
        title_str='Main lane 4';
    elseif k_lane==51 
        Idx_plot=[37];
        title_str='Bylane 1 (Inflow)';
    elseif k_lane==11
        Idx_plot=[36];
        title_str='Bylane 1 (Outflow)';
    elseif k_lane==71
        Idx_plot=[25];
        title_str='Bylane 2 (Inflow)';
    elseif k_lane==52
        Idx_plot=[19];
        title_str='Bylane 2 (Outflow)';
    elseif k_lane==91
        Idx_plot=[13];
        title_str='Bylane 3 (Inflow)';
    elseif k_lane==72
        Idx_plot=[7];
        title_str='Bylane 3 (Outflow)';
    elseif k_lane==42
        Idx_plot=[42];
        title_str='Bylane 1 (Outflow)';
    elseif k_lane==61
        Idx_plot=[36];
        title_str='Bylane 1 (Inflow)';
    elseif k_lane==62
        Idx_plot=[30];
        title_str='Bylane 2 (Outflow)';
    elseif k_lane==81
        Idx_plot=[24];
        title_str='Bylane 2 (Inflow)';
    elseif k_lane==82
        Idx_plot=[18];
        title_str='Bylane 3 (Outflow)';
    elseif k_lane==101 
        Idx_plot=[12];
        title_str='Bylane 3 (Inflow)';
    end
    subplot(1,4,Idx_plot)
    title(title_str)
    hold on
    if k_lane==51 || k_lane==52
        laneID=5;
    elseif k_lane==61 || k_lane==62
        laneID=6;
    elseif k_lane==71 || k_lane==72
        laneID=7;
    elseif k_lane==81 || k_lane==82
        laneID=8;
    elseif k_lane==91 || k_lane==92
        laneID=9;
    elseif k_lane==101 || k_lane==102
        laneID=10;
    end
    for kj=1:total_nveh
        if k_lane==1
            idx_lane=data_laneNo(:,kj)==1 | data_laneNo(:,kj)==5 | data_laneNo(:,kj)==7 | data_laneNo(:,kj)==9;
            idx_dir=data_movdir(:,kj)==0;
        elseif k_lane==4
            idx_lane=data_laneNo(:,kj)==4 | data_laneNo(:,kj)==6 | data_laneNo(:,kj)==8 | data_laneNo(:,kj)==10;
            idx_dir=data_movdir(:,kj)==0;
        elseif k_lane==2 || k_lane==3
            idx_lane=data_laneNo(:,kj)==k_lane;
            idx_dir=data_movdir(:,kj)==0;
        elseif sum([51,71,91]==k_lane)>0
            idx_lane=data_laneNo(:,kj)==laneID;
            idx_dir=data_movdir(:,kj)==pi/2;
        elseif sum([52,72,92]==k_lane)>0
            idx_lane=data_laneNo(:,kj)==laneID;
            idx_dir=data_movdir(:,kj)==-pi/2;
        elseif sum([61,81,101]==k_lane)>0
            idx_lane=data_laneNo(:,kj)==laneID;
            idx_dir=data_movdir(:,kj)==-pi/2;
        elseif sum([62,82,102]==k_lane)>0
            idx_lane=data_laneNo(:,kj)==laneID;
            idx_dir=data_movdir(:,kj)==pi/2;
        end
        pos_=data_pos(:,kj);
        idx=idx_lane & idx_dir;
        pos_(~idx)=nan;
        plot(tt_,pos_,'-','color',[.7 .7 .7])
    end
    
    for j=1:3
        eval(['L_light=Loc_jiaochakou_',num2str(j),';']);
        if sum([1:4]==k_lane)>0
            h=fill((t0+[[1,step],[step,1]])*dt,[[L_light,L_light] [L_light,L_light]+L_is],'c');
        else
            h=fill((t0+[[1,step],[step,1]])*dt,[[L_light,L_light] [L_light,L_light]+L_is*2],'c');
        end
        set(h,'edgealpha',0,'facealpha',0.2)
        if sum([1:4]==k_lane)>0
            plot((t0+[0 nCycle*signalCycle])*dt,[L_light L_light],'r-','linewidth',2.5)
            for i=0:nCycle
                plot((t0_lt+[i*signalCycle i*signalCycle+greenTimeMain])*dt,[L_light L_light],'g-','linewidth',2.5)
                plot((t0_lt+[i*signalCycle+greenTimeMain-t_BeforeRed/dt i*signalCycle+greenTimeMain])*dt,[L_light L_light],'y-','linewidth',2.5)
            end
            plot([0 nCycle*signalCycle]*dt,[L_light L_light]-L_f,'k--','linewidth',1)
        end
        if sum([51,52,71,72,91,92]==k_lane)>0
            plot((t0+[0 nCycle*signalCycle])*dt,[L_light L_light],'r-','linewidth',2.5)
            for i=0:nCycle
                plot((t0_lt+[i*signalCycle+redTimeRamp (i+1)*signalCycle])*dt,[L_light L_light],'g-','linewidth',2.5)
                plot((t0_lt+[(i+1)*signalCycle-t_BeforeRed/dt (i+1)*signalCycle])*dt,[L_light L_light],'y-','linewidth',2.5)
            end
        end
        if sum([61,62,81,82,101,102]==k_lane)>0
            plot((t0+[0 nCycle*signalCycle])*dt,[L_light L_light]+lane_width,'r-','linewidth',2.5)
            for i=0:nCycle
                plot((t0_lt+[i*signalCycle+redTimeRamp (i+1)*signalCycle])*dt,[L_light L_light]+lane_width,'g-','linewidth',2.5)
                plot((t0_lt+[(i+1)*signalCycle-t_BeforeRed/dt (i+1)*signalCycle])*dt,[L_light L_light]+lane_width,'y-','linewidth',2.5)
            end
        end
    end
    
    if sum([1:4]==k_lane)>0
        ylim([0 L_road])
    elseif sum([51,61,52,62]==k_lane)>0
        ylim([Loc_jiaochakou_1-L_subByroad Loc_jiaochakou_1+L_is*2])
    elseif sum([71,81,72,82]==k_lane)>0
        ylim([Loc_jiaochakou_2-L_subByroad Loc_jiaochakou_2+L_is*2])
    elseif sum([91,101,92,102]==k_lane)>0
        ylim([Loc_jiaochakou_3-L_subByroad Loc_jiaochakou_3+L_is*2])
    end
    xlim([tt_(1)-dt tt_(end)])
    box on
    if k_lane==1
        ylabel('Position (m)')
    end
    if sum([2]==k_lane)>0
        xlabel('Time (s)')
    end
    if sum([71,72,81,82,91,92,101,102]==k_lane)>0
        set(gca,'xticklabel',{})
    end
end
set(gcf,'position', [1,100,1900,500])

%--------------------------------------------------------------------------trajectory-ramp
t0=data_vehicles(1).time(1).time-1;
t0_lt=fix(t0/signalCycle)*signalCycle;
nCycle=ceil(step/signalCycle);
figure
tt_=(t0+[1:step])'*dt;
for k_lane=[12,42,51,52,61,62,71,72,81,82,91,92,101,102]
    if k_lane==51 
        Idx_plot=[1];
        title_str='Bylane 1 (Inflow)';
    elseif k_lane==12
        Idx_plot=[2];
        title_str='Bylane 1 (Outflow)';
    elseif k_lane==71
        Idx_plot=[3];
        title_str='Bylane 2 (Inflow)';
    elseif k_lane==52
        Idx_plot=[4];
        title_str='Bylane 2 (Outflow)';
    elseif k_lane==91
        Idx_plot=[5];
        title_str='Bylane 3 (Inflow)';
    elseif k_lane==72
        Idx_plot=[6];
        title_str='Bylane 3 (Outflow)';
    elseif k_lane==42
        Idx_plot=[7];
        title_str='Bylane 1 (Outflow)';
    elseif k_lane==61
        Idx_plot=[8];
        title_str='Bylane 1 (Inflow)';
    elseif k_lane==62
        Idx_plot=[9];
        title_str='Bylane 2 (Outflow)';
    elseif k_lane==81
        Idx_plot=[10];
        title_str='Bylane 2 (Inflow)';
    elseif k_lane==82
        Idx_plot=[11];
        title_str='Bylane 3 (Outflow)';
    elseif k_lane==101 
        Idx_plot=[12];
        title_str='Bylane 3 (Inflow)';
    end
    subplot(2,6,Idx_plot)
    title(title_str)
    hold on    
    if k_lane==12 
        laneID=1;
    elseif k_lane==42 
        laneID=4;
    elseif k_lane==51 || k_lane==52
        laneID=5;
    elseif k_lane==61 || k_lane==62
        laneID=6;
    elseif k_lane==71 || k_lane==72
        laneID=7;
    elseif k_lane==81 || k_lane==82
        laneID=8;
    elseif k_lane==91 || k_lane==92
        laneID=9;
    elseif k_lane==101 || k_lane==102
        laneID=10;
    end
    for kj=1:total_nveh
        idx_lane=data_laneNo(:,kj)==laneID;
        if sum([51,71,91]==k_lane)>0
            idx_dir=data_movdir(:,kj)==pi/2;
        elseif sum([12,52,72]==k_lane)>0
            idx_dir=data_movdir(:,kj)==-pi/2;
        elseif sum([61,81,101]==k_lane)>0
            idx_dir=data_movdir(:,kj)==-pi/2;
        elseif sum([42,62,82]==k_lane)>0
            idx_dir=data_movdir(:,kj)==pi/2;
        end
        pos_=data_pos(:,kj);
        idx=idx_lane & idx_dir;
        pos_(~idx)=nan;
        plot(tt_,pos_,'-','color',[.7 .7 .7])
    end
    for j=1:3
        eval(['L_light=Loc_jiaochakou_',num2str(j),';']);
        if sum([1:4]==k_lane)>0
            h=fill((t0+[[1,step],[step,1]])*dt,[[L_light,L_light] [L_light,L_light]+L_is],'c');
        else
            h=fill((t0+[[1,step],[step,1]])*dt,[[L_light,L_light] [L_light,L_light]+L_is*2],'c');
        end
        set(h,'edgealpha',0,'facealpha',0.2)
        if sum([12,51,52,71,72,91]==k_lane)>0
            plot((t0+[0 nCycle*signalCycle])*dt,[L_light L_light],'r-','linewidth',2.5)
            for i=0:nCycle
                plot((t0_lt+[i*signalCycle+redTimeRamp (i+1)*signalCycle])*dt,[L_light L_light],'g-','linewidth',2.5)
                plot((t0_lt+[(i+1)*signalCycle-t_BeforeRed/dt (i+1)*signalCycle])*dt,[L_light L_light],'y-','linewidth',2.5)
            end
        end
        if sum([42,61,62,81,82,101]==k_lane)>0
            plot((t0+[0 nCycle*signalCycle])*dt,[L_light L_light]+lane_width,'r-','linewidth',2.5)
            for i=0:nCycle
                plot((t0_lt+[i*signalCycle+redTimeRamp (i+1)*signalCycle])*dt,[L_light L_light]+lane_width,'g-','linewidth',2.5)
                plot((t0_lt+[(i+1)*signalCycle-t_BeforeRed/dt (i+1)*signalCycle])*dt,[L_light L_light]+lane_width,'y-','linewidth',2.5)
            end
        end
    end
    
    if sum([51,61]==k_lane)>0
        ylim([Loc_jiaochakou_1-L_subByroad Loc_jiaochakou_1+L_is*2])
    elseif sum([12,42]==k_lane)>0
        ylim([Loc_jiaochakou_1 Loc_jiaochakou_1+L_subByroad])
    elseif sum([71,81]==k_lane)>0
        ylim([Loc_jiaochakou_2-L_subByroad Loc_jiaochakou_2+L_is*2])
    elseif sum([52,62]==k_lane)>0
        ylim([Loc_jiaochakou_2 Loc_jiaochakou_2+L_subByroad])
    elseif sum([91,101]==k_lane)>0
        ylim([Loc_jiaochakou_3-L_subByroad Loc_jiaochakou_3+L_is*2])
    elseif sum([72,82]==k_lane)>0
        ylim([Loc_jiaochakou_3 Loc_jiaochakou_3+L_subByroad])
    end
    xlim([tt_(1)-dt tt_(end)])
    box on
    if k_lane==51 || k_lane==42
        ylabel('Position (m)')
    end
    if sum([42,61,62,81,82,101]==k_lane)>0
        xlabel('Time (s)')
    end
end
set(gcf,'position', [1,100,1900,500])

%--------------------------------------------------------------------------speedheatmap
tt_set=[0:5:step*dt];
xx_set=[0:50:L_road];
AveV_perLane={};
for k_lane=1:4
    xx=data_pos;
    vv=vdata*3.6;
    tt=repmat([1:step]'*dt,1,total_nveh);
    if k_lane==1
        idx=(data_laneNo==1 | data_laneNo==5 | data_laneNo==7 | data_laneNo==9) & data_movdir==0;
    elseif k_lane==2
        idx=data_laneNo==2;
    elseif k_lane==3
        idx=data_laneNo==3;
    elseif k_lane==4
        idx=(data_laneNo==4 | data_laneNo==6 | data_laneNo==8 | data_laneNo==10) & data_movdir==0;
    end
    xx(~idx)=nan;vv(~idx)=nan;tt(~idx)=nan;
    
    aveV_=nan*zeros(length(tt_set),length(xx_set));
    for tj=1:length(tt_set)-1
        st=tt_set(tj);
        et=tt_set(tj+1);
        for kj=1:length(xx_set)-1
            sloc=xx_set(kj);
            eloc=xx_set(kj+1);
            idx=tt>=st & tt<et & xx>=sloc & xx<eloc;
            vv_=vv(idx);
            aveV_(tj,kj)=nanmean(vv_(:));
        end
    end
    aveV_(end,:)=aveV_(end-1,:);
    aveV_(:,end)=aveV_(:,end-1);
    AveV_perLane(k_lane)={aveV_};
end
%--------------------------------------------------
tt_set=t0*dt+tt_set;
figure
for k_lane=1:4
    subplot(1,4,k_lane)
    hold on
    [mesh_X, mesh_Y] = meshgrid(tt_set, xx_set);
    surf(mesh_X,mesh_Y,AveV_perLane{k_lane}')
    colormap(flipud(mycmap_2))
    caxis([0 v_max]*3.6)
    xlabel('Time (s)')
    ylabel('Position (m)')
    if k_lane==1
        zlabel('Speed (km/h)')
    end
    set(gca,'fontname','times new roman','fontsize',16)
    view(50,82)
    box on
    grid on
    title(['Main lane ',num2str(k_lane)])
    for j=1:3
        eval(['L_light=Loc_jiaochakou_',num2str(j),';']);
        plot3((t0+[0 nCycle*signalCycle])*dt,[L_light L_light],[v_max v_max]*3.6,'r-','linewidth',2.5)
        for i=0:nCycle
            plot3((t0+[i*signalCycle i*signalCycle+greenTimeMain])*dt,[L_light L_light],[v_max v_max]*3.6,'g-','linewidth',2.5)
            plot3((t0+[i*signalCycle+greenTimeMain-t_BeforeRed/dt i*signalCycle+greenTimeMain])*dt,[L_light L_light],[v_max v_max]*3.6,'y-','linewidth',2.5)
        end
    end
    if k_lane==4
        ch=colorbar('Position',[0.937608281358284 0.411494252873563 0.00779997029997226 0.515527443378905]);
        set(get(ch,'title'),'string','Unit: km/h','fontsize',13)
    end
    set(gca,'ztick',0:60:v_max*3.6)
    xlim([tt_(1)-dt tt_(end)])
    zlim([0 v_max*3.6])
    ylim([0 L_road])
    set(gca, 'position',[0.085+0.215*(k_lane-1) 0.1116 0.1633 0.7815]);
end
set(gcf,'position', [1,100,1900,300])

