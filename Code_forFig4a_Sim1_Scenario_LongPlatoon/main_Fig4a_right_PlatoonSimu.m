clc
clear
close all

% Code for Figure 4a-right panel in main text
% Code for Figure S.22 in SI
%% scene setting
scene_set=[10,0,1,1;
    20,0,1,1;
    30,0,1,1;
    40,0,1,1;
    50,0,1,1;
    60,0,1,1;
    70,0,1,1;
    80,0,1,1;
    90,0,1,1;
    100,0,1,1;
    110,0,1,1;
    120,0,1,1];
pertur_type=repmat({'Both'},length(scene_set),1);
%% Simulation
curve_SpeedvsFlow=[];
data_SpeedvsFlow={};
for npre=1:5
    % AV control para
    %----------------------------------------------------Upper
    if npre==1
        paraUpper=[0.6722    0.2467    0.9551    0.1235    1.5];
    elseif npre==2
        paraUpper=[0.6265    0.3703    0.4437    0.2889    0.6676    0.0736    1];
    elseif npre==3
        paraUpper=[0.7026    0.4377    0.3068    0.2359    0.2194    0.1090    0.6125    0.0575    1];
    elseif npre==4
        paraUpper=[0.6802    0.3690    0.0321    0.4185    0.2587    0.1706      0.1218    0.2082    0.4448    0.0516     1];
    elseif npre==5
        paraUpper=[0.8567    0.9122    0.2174    0.0482    0.0001    0.2801    0.1235    0.4235    0.0372    0.0815    0.3039    0.0450    1];
    end
    %----------------------------------------------------Lower
    para_AV=[78.8201836419897,2.07647787797603,9.17891305242795,4;
        58.9344898328179,1.90325205249387,7.86998881634774,4;
        95.9937146710127,2.32906700594111,10.0111919741363,4;
        43.0500120244655,1.62000868090560,6.87020881889564,4];%0.109090487888431/0.110201265925165/0.113709913393937/0.125978748863393
    para_AV=repmat(para_AV,25,1);
    para_AV=[zeros(1,size(para_AV,2));para_AV];
    %-------------------------------------------------
    FlowDensity_set=nan*zeros(size(scene_set,1),4);
    for ns=1:size(scene_set,1)
        timestep=0.05;
        scene_=scene_set(ns,:);
        vs=scene_(1);%km/h
        vd=scene_(2);%km/h
        a_acc=scene_(3);%m/s2
        a_dec=scene_(4);%m/s2
        tgap=60;%s
        nd=2;
        data_Input=fun_generate_leadv(vs,vd,a_acc,a_dec,tgap,2,pertur_type{ns},timestep);
        LowLevelModel_label=0;
        NoiseStrength=[0.8556    0.0123];
        num_cishu=20;
        FlowDensity_=nan*zeros(num_cishu,4);
        for cishu=1:num_cishu
            if npre==1
                [vdata,xdata,adata,acmd]=fun_SimAVTraffic_S0aNoise(data_Input,para_AV,paraUpper,LowLevelModel_label,NoiseStrength);
            elseif npre==2
                [vdata,xdata,adata,acmd]=fun_SimAVTraffic_S1aNoise(data_Input,para_AV,paraUpper,LowLevelModel_label,NoiseStrength);
            elseif npre==3
                [vdata,xdata,adata,acmd]=fun_SimAVTraffic_S3aNoise(data_Input,para_AV,paraUpper,LowLevelModel_label,NoiseStrength);
            elseif npre==4
                [vdata,xdata,adata,acmd]=fun_SimAVTraffic_S4aNoise(data_Input,para_AV,paraUpper,LowLevelModel_label,NoiseStrength);
            elseif npre==5
                [vdata,xdata,adata,acmd]=fun_SimAVTraffic_S5aNoise(data_Input,para_AV,paraUpper,LowLevelModel_label,NoiseStrength);
            end
            %----------------------
            vdata=vdata(:,2:end);
            xdata=xdata(:,2:end);
            adata=adata(:,2:end);
            acmd=acmd(:,2:end);
            %----------------------
            L_veh=4.835;
            tp=([1:length(vdata)]-1)*0.05;
            data_dx=[xdata(:,end),xdata(:,1:end-1)]-xdata-L_veh;
            %------------------------------------------- Flow-density
            platV=nanmean(vdata(:,1+npre:end),2);%m/s
            platLen=xdata(:,npre)-xdata(:,end);
            nveh=size(vdata,2);
            platDen=(nveh-npre)./platLen;%/m
            platFlow=platV.*platDen*3600;
            FlowDensity_(cishu,:)=[vs,nanmean(platV)*3.6,nanmean(platDen),nanmean(platFlow)];
        end
        FlowDensity_set(ns,:)=nanmean(FlowDensity_,1);
    end
    
    y=FlowDensity_set(:,4);
    X=[ones(length(FlowDensity_set(:,2)),1),FlowDensity_set(:,2)];
    [b,bint] = regress(y,X);
    curve_SpeedvsFlow(npre,:)=b;
    data_SpeedvsFlow(npre,1)={FlowDensity_set(:,[2,4])};
    %% Plot-Fig.4a
    figure
    hold on
    plot([0:125],b(1)+b(2)*[0:125],'k-','linewidth',1)
    scatter(FlowDensity_set(:,2),FlowDensity_set(:,4),40,FlowDensity_set(:,4),'filled','markerfacealpha',0.5)
    xlabel('Average speed (km/h)')
    ylabel('Flow (veh/h)')
    xlim([7 123])
    set(gca,'ytick',0:2000:16000)
    ylim([1000 15000])
    box on
    grid on
    set(gca,'fontname','times new roman','fontsize',16)
end
%% Plot-Fig.S.22
figure
hold on
mycolor={'k','r','g','b','m'};
leg_=[];leg_label={};
for npre=1:5
    data_=data_SpeedvsFlow{npre};
    p_=plot(data_(:,1),data_(:,2),'-o','color',mycolor{npre},'linewidth',1.5);
    leg_=[leg_;p_];
    leg_label=[leg_label;{['Commun. ',num2str(npre),' Veh.']}];
end
p_leg=legend(leg_,leg_label);
set(p_leg,'location','northwest')
xlabel('Average speed (km/h)')
ylabel('Flow rate (veh/h)')
xlim([7 123])
set(gca,'ytick',0:2000:18000)
ylim([1000 18000])
box on
grid on
set(gca,'fontname','times new roman','fontsize',16)