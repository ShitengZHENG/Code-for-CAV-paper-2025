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
%--------------------------------------------------------------------------Speed&flow
aveSpe_zone=Result_total{1,3};
totalQ_loop=Result_total{1,4};
figure
hold on
plot(aveSpe_zone,'r-s','markerfacecolor','r','linewidth',1)
xlabel('Zone No.')
ylabel('Average speed (km/h)')
set(gca,'xtick',1:17)
set(gca,'ytick',90:2:130)
set(gca,'fontname','times new roman','fontsize',16)
box on
set(gcf,'position', [1,100,1300,300])
xlim([1 17])
ylim([110 124])

default_color=[[0,114,189];[217,83,25];[237,177,32];[126,47,142]];
default_color2=[[78,102,178];[70,160,182];[135,207,168];[254,236,159]]/255;
figure
hold on
yyaxis left
b_1=bar(totalQ_loop,1);
for j=1:4
    b_1(j).FaceColor=default_color2(j,:);
    b_1(j).EdgeColor=default_color2(j,:);
end
xlabel('Detector No.')
ylabel('Flow rate for each lane (veh/h/lane)')
set(gca,'YColor','black');
yyaxis right
p_1=plot(nansum(totalQ_loop,2),'r-s','markerfacecolor','r','linewidth',1);
ylabel('Total flow rate (veh/h)')
legend([b_1,p_1],'Main lane 1','Main lane 2','On-ramp','Off-ramp','Total')
xlim([0.5 17.5])
ylim([1.7e4 2.5e4])
set(gca,'xtick',1:17)
set(gca,'YColor','black');
set(gca,'fontname','times new roman','fontsize',16)
box on
set(gcf,'position', [1,100,1300,420])

%--------------------------------------------------------------------------Speed&flow
label_=Result_FlowANDSpe(:,3)==1;
figure
subplot(1,2,1)
hold on
xx=Result_FlowANDSpe(:,1);yy=pos_detector;
xx(~label_)=nan;yy(~label_)=nan;
plot(xx,yy,'-','color',[78,102,178]/255,'linewidth',3)
xx=Result_FlowANDSpe(:,1);yy=pos_detector;
xx(label_)=nan;yy(label_)=nan;
plot(xx,yy,'-','color',[70,160,182]/255,'linewidth',3)
xspan=[1.7e4 2.45e4 2.45e4 1.7e4];
for j=1:4
    eval(['Loc_onRamp_=Loc_onRamp_',num2str(j),';'])
    eval(['Loc_offRamp_=Loc_offRamp_',num2str(j),';'])
    h=fill(xspan,[[Loc_onRamp_,Loc_onRamp_]-x_cooperate [Loc_onRamp_,Loc_onRamp_]],'c');
    set(h,'edgealpha',0,'facealpha',0.2)
    h=fill(xspan,[[Loc_onRamp_,Loc_onRamp_] [Loc_onRamp_,Loc_onRamp_]+L_acclane],'g');
    set(h,'edgealpha',0,'facealpha',0.2)
    h=fill(xspan,[[Loc_offRamp_,Loc_offRamp_]-L_acclane [Loc_offRamp_,Loc_offRamp_]],'m');
    set(h,'edgealpha',0,'facealpha',0.2)
end
xlim([1.7e4 2.45e4])
ylim([0 6010])
ylabel('Position (m)')
xlabel('Flow rate (veh/h)')
set(gca,'fontname','times new roman','fontsize',16)
box on
legend('Main road only','Main road & Ramps')
legend('boxoff')
set(gca, 'position',[0.13 0.145 0.39 0.77]);

subplot(1,2,2)
hold on
xx=Result_FlowANDSpe(:,2);yy=pos_detector;
xx(~label_)=nan;yy(~label_)=nan;
plot(xx,yy,'-','color',[78,102,178]/255,'linewidth',3)
xx=Result_FlowANDSpe(:,2);yy=pos_detector;
xx(label_)=nan;yy(label_)=nan;
plot(xx,yy,'-','color',[70,160,182]/255,'linewidth',3)
xspan=[0 130 130 0];
for j=1:4
    eval(['Loc_onRamp_=Loc_onRamp_',num2str(j),';'])
    eval(['Loc_offRamp_=Loc_offRamp_',num2str(j),';'])
    h1=fill(xspan,[[Loc_onRamp_,Loc_onRamp_]-x_cooperate [Loc_onRamp_,Loc_onRamp_]],'c');
    set(h1,'edgealpha',0,'facealpha',0.2)
    h2=fill(xspan,[[Loc_onRamp_,Loc_onRamp_] [Loc_onRamp_,Loc_onRamp_]+L_acclane],'g');
    set(h2,'edgealpha',0,'facealpha',0.2)
    h3=fill(xspan,[[Loc_offRamp_,Loc_offRamp_]-L_acclane [Loc_offRamp_,Loc_offRamp_]],'m');
    set(h3,'edgealpha',0,'facealpha',0.2)
end
legend([h1,h2,h3],{'CZ','MZ','EZ'})
% legend('boxoff')
set(gca, 'position',[0.52 0.145 0.39 0.77]);
xlim([102 130])
ylim([0 6010])
set(gca,'yticklabel',{''})
xlabel('Average speed (km/h)')
set(gca,'YColor','black');
set(gca,'fontname','times new roman','fontsize',16)
box on

%--------------------------------------------------------------------------¹ì¼£Í¼
t0=data_vehicles(1).time(1).time-1;
figure
tt_=(t0+[1:step])'*dt;
for k_lane=1:10
    if k_lane==1
        Idx_plot=[1:3:22];
        title_str='Main lane 1';
    elseif k_lane==2
        Idx_plot=[2:3:23];
        title_str='Main lane 2';
    elseif k_lane==3
        Idx_plot=[3*8];
        title_str='On-ramp 1';
    elseif k_lane==4
        Idx_plot=[3*7];
        title_str='Off-ramp 1';
    elseif k_lane==5
        Idx_plot=[3*6];
        title_str='On-ramp 2';
    elseif k_lane==6
        Idx_plot=[3*5];
        title_str='Off-ramp 2';
    elseif k_lane==7
        Idx_plot=[3*4];
        title_str='On-ramp 3';
    elseif k_lane==8
        Idx_plot=[3*3];
        title_str='Off-ramp 3';
    elseif k_lane==9
        Idx_plot=[3*2];
        title_str='On-ramp 4';
    elseif k_lane==10
        Idx_plot=[3*1];
        title_str='Off-ramp 4';
    end
    subplot(8,3,Idx_plot)
    hold on
    if k_lane<=2
        title(title_str)
    end
    
    % »æÖÆ¹ì¼£
    for kj=1:total_nveh
        idx=data_laneNo(:,kj)==k_lane;
        pos_=data_pos(:,kj);
        pos_(~idx)=nan;
        mycolor=[.7 .7 .7];%Ö÷Â·
        plot(tt_,pos_,'-','color',mycolor)
    end
    if k_lane<=2
        for j=1:4
            eval(['Loc_onRamp_=Loc_onRamp_',num2str(j),';'])
            eval(['Loc_offRamp_=Loc_offRamp_',num2str(j),';'])
            plot((t0+[0 step])*dt,[Loc_onRamp_ Loc_onRamp_],'k--','linewidth',1)
            plot((t0+[0 step])*dt,[Loc_offRamp_ Loc_offRamp_],'k--','linewidth',1)
            h=fill((t0+[[1,step],[step,1]])*dt,[[Loc_onRamp_,Loc_onRamp_]-x_cooperate [Loc_onRamp_,Loc_onRamp_]],'c');
            set(h,'edgealpha',0,'facealpha',0.2)
            h=fill((t0+[[1,step],[step,1]])*dt,[[Loc_onRamp_,Loc_onRamp_] [Loc_onRamp_,Loc_onRamp_]+L_acclane],'g');
            set(h,'edgealpha',0,'facealpha',0.2)
            h=fill((t0+[[1,step],[step,1]])*dt,[[Loc_offRamp_,Loc_offRamp_]-L_acclane [Loc_offRamp_,Loc_offRamp_]],'m');
            set(h,'edgealpha',0,'facealpha',0.2)
        end
    else
        if k_lane==3 || k_lane==4
            j=1;
        elseif k_lane==5 || k_lane==6
            j=2;
        elseif k_lane==7 || k_lane==8
            j=3;
        elseif k_lane==9 || k_lane==10
            j=4;
        end
        eval(['Loc_onRamp_=Loc_onRamp_',num2str(j),';'])
        eval(['Loc_offRamp_=Loc_offRamp_',num2str(j),';'])
        plot((t0+[0 step])*dt,[Loc_onRamp_ Loc_onRamp_],'g--','linewidth',1)
        plot((t0+[0 step])*dt,[Loc_offRamp_ Loc_offRamp_],'m--','linewidth',1)
    end
    xlim([tt_(1)-dt tt_(end)])
    ylim([0 L_road])
    box on
    if sum([1,2,3]==k_lane)>0
        xlabel('Time (s)')
    end
    if sum([1,2,6]==k_lane)>0
        ylabel('Position (m)')
    end
    if sum([4:10]==k_lane)>0
       set(gca,'xticklabel',{''}) 
    end
    bias=30;
    if k_lane==3
        ylim_value=[Loc_onRamp_1-L_subByroad-bias,Loc_onRamp_1+L_acclane+bias];
    elseif k_lane==5
        ylim_value=[Loc_onRamp_2-L_subByroad-bias,Loc_onRamp_2+L_acclane+bias];
    elseif k_lane==7
        ylim_value=[Loc_onRamp_3-L_subByroad-bias,Loc_onRamp_3+L_acclane+bias];
    elseif k_lane==9
        ylim_value=[Loc_onRamp_4-L_subByroad-bias,Loc_onRamp_4+L_acclane+bias];
    elseif k_lane==4
        ylim_value=[Loc_offRamp_1-L_acclane-bias,Loc_offRamp_1+L_subByroad+bias];
    elseif k_lane==6
        ylim_value=[Loc_offRamp_2-L_acclane-bias,Loc_offRamp_2+L_subByroad+bias];
    elseif k_lane==8
        ylim_value=[Loc_offRamp_3-L_acclane-bias,Loc_offRamp_3+L_subByroad+bias];
    elseif k_lane==10
        ylim_value=[Loc_offRamp_4-L_acclane-bias,Loc_offRamp_4+L_subByroad+bias];
    else
        ylim_value=[0 L_road];
    end
    ylim(ylim_value)
    set(gca,'fontname','times new roman','fontsize',16)
end
set(gcf,'position', [1,100,1900,500])
% return
%--------------------------------------------------------------------------Speed heatmap
%--------------------------------------------------Average speed
tt_set=[0:5:step*dt];
xx_set=[0:50:L_road];
AveV_perLane={};
for k_lane=1:2
    xx=data_pos;
    vv=vdata*3.6;
    tt=repmat([1:step]'*dt,1,total_nveh);
    idx=data_laneNo==k_lane;
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
%--------------------------------------------------end
tt_set=t0*dt+tt_set;
figure
for k_lane=1:2
    
    subplot(1,2,k_lane)
    hold on
    [mesh_X, mesh_Y] = meshgrid(tt_set, xx_set);
    surf(mesh_X,mesh_Y,AveV_perLane{k_lane}')
    for j=1:4
        eval(['Loc_onRamp_=Loc_onRamp_',num2str(j),';'])
        eval(['Loc_offRamp_=Loc_offRamp_',num2str(j),';'])
        p1=plot3((t0+[0 step])*dt,[Loc_onRamp_ Loc_onRamp_],[120 120],'-','color',[.5 .5 .5],'linewidth',2);
        p2=plot3((t0+[0 step])*dt,[Loc_offRamp_ Loc_offRamp_],[120 120],'-','color',[.7 .7 .7],'linewidth',2);
    end
    colormap(flipud(mycmap_2))
    caxis([0 120])
    xlabel('Time (s)')
    ylabel('Position (m)')
    zlabel('Speed (km/h)')
    set(gca,'fontname','times new roman','fontsize',16)
    view(50,82)
    box on
    grid on
    title(['Main lane ',num2str(k_lane)])
    
    if k_lane==2
        ch=colorbar('Position',[0.916880754380754 0.445238095238095 0.0134763884763889 0.481783601014374]);
        set(get(ch,'title'),'string','Unit: km/h','fontsize',13)
    end
    set(gca,'ztick',0:60:120)
    xlim([tt_(1)-dt tt_(end)])
    zlim([0 120])
    ylim([0 L_road])
end
p_leg=legend([p1,p2],'Merging start','Exiting end');
set(p_leg,'fontname','times new roman','fontsize',13)
set(gcf,'position', [1,100,1300,420])

%--------------------------------------------------------------------------Speedheatmap-ramp
%--------------------------------------------------
AveV_perLane={};
for k_lane=3:10
    xx=data_pos;
    vv=vdata*3.6;
    tt=repmat([1:step]'*dt,1,total_nveh);
    idx=data_laneNo==k_lane;
    xx(~idx)=nan;vv(~idx)=nan;tt(~idx)=nan;
    
    tt_set=[0:5:step*dt];
    if k_lane==3
        xx_set=[Loc_onRamp_1-L_subByroad:10:Loc_onRamp_1+L_acclane];
    elseif k_lane==5
        xx_set=[Loc_onRamp_2-L_subByroad:10:Loc_onRamp_2+L_acclane];
    elseif k_lane==7
        xx_set=[Loc_onRamp_3-L_subByroad:10:Loc_onRamp_3+L_acclane];
    elseif k_lane==9
        xx_set=[Loc_onRamp_4-L_subByroad:10:Loc_onRamp_4+L_acclane];
    elseif k_lane==4
        xx_set=[Loc_offRamp_1-L_acclane:10:Loc_offRamp_1+L_subByroad];
    elseif k_lane==6
        xx_set=[Loc_offRamp_2-L_acclane:10:Loc_offRamp_2+L_subByroad];
    elseif k_lane==8
        xx_set=[Loc_offRamp_3-L_acclane:10:Loc_offRamp_3+L_subByroad];
    elseif k_lane==10
        xx_set=[Loc_offRamp_4-L_acclane:10:Loc_offRamp_4+L_subByroad];
    end
    
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
for k_lane=3:10
    if k_lane==3
        Idx_plot=1;
        title_str='On-ramp 1';
    elseif k_lane==5
        Idx_plot=2;
        title_str='On-ramp 2';
    elseif k_lane==7
        Idx_plot=3;
        title_str='On-ramp 3';
    elseif k_lane==9
        Idx_plot=4;
        title_str='On-ramp 4';
    elseif k_lane==4
        Idx_plot=5;
        title_str='Off-ramp 1';
    elseif k_lane==6
        Idx_plot=6;
        title_str='Off-ramp 2';
    elseif k_lane==8
        Idx_plot=7;
        title_str='Off-ramp 3';
    elseif k_lane==10
        Idx_plot=8;
        title_str='Off-ramp 4';
    end       
    subplot(2,4,Idx_plot)
    hold on
    if k_lane==3
        xx_set=[Loc_onRamp_1-L_subByroad:10:Loc_onRamp_1+L_acclane];
    elseif k_lane==5
        xx_set=[Loc_onRamp_2-L_subByroad:10:Loc_onRamp_2+L_acclane];
    elseif k_lane==7
        xx_set=[Loc_onRamp_3-L_subByroad:10:Loc_onRamp_3+L_acclane];
    elseif k_lane==9
        xx_set=[Loc_onRamp_4-L_subByroad:10:Loc_onRamp_4+L_acclane];
    elseif k_lane==4
        xx_set=[Loc_offRamp_1-L_acclane:10:Loc_offRamp_1+L_subByroad];
    elseif k_lane==6
        xx_set=[Loc_offRamp_2-L_acclane:10:Loc_offRamp_2+L_subByroad];
    elseif k_lane==8
        xx_set=[Loc_offRamp_3-L_acclane:10:Loc_offRamp_3+L_subByroad];
    elseif k_lane==10
        xx_set=[Loc_offRamp_4-L_acclane:10:Loc_offRamp_4+L_subByroad];
    end
    [mesh_X, mesh_Y] = meshgrid(tt_set, xx_set);
    surf(mesh_X,mesh_Y,AveV_perLane{k_lane}')
    for j=1:4
        eval(['Loc_onRamp_=Loc_onRamp_',num2str(j),';'])
        eval(['Loc_offRamp_=Loc_offRamp_',num2str(j),';'])
        plot3((t0+[0 step])*dt,[Loc_onRamp_ Loc_onRamp_],[120 120],'-','color',[.5 .5 .5],'linewidth',2)
        plot3((t0+[0 step])*dt,[Loc_offRamp_ Loc_offRamp_],[120 120],'-','color',[.7 .7 .7],'linewidth',2)
    end
    colormap(flipud(mycmap_2))
    caxis([0 120])
    set(gca,'fontname','times new roman','fontsize',16)
    view(50,82)
    box on
    grid on
    title(title_str)
    if k_lane==1
        title(['Outer lane'])
    elseif k_lane==2
        title(['Inner lane'])
    end
    if Idx_plot==2
        ch=colorbar('Position',[0.916880754380754 0.445238095238095 0.0134763884763889 0.481783601014374]);
        set(get(ch,'title'),'string','Unit: km/h','fontsize',13)
    end
    xlabel('Time (s)')
    ylabel('Position (m)')
    zlabel('Speed (km/h)')
    set(gca,'ztick',[0,120])
    xlim([tt_(1)-dt tt_(end)])
    zlim([0 120])
    ylim([xx_set(1) xx_set(end)])
end
set(gcf,'position', [1,50,1900,600])
