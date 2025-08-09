clc
clear
close all

% Code for Figure 5 in main text
%%
for npre=1:5
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
    %----------------------------------------------------
    para_AV=[78.8201836419897,2.07647787797603,9.17891305242795,4;
        58.9344898328179,1.90325205249387,7.86998881634774,4;
        95.9937146710127,2.32906700594111,10.0111919741363,4;
        43.0500120244655,1.62000868090560,6.87020881889564,4];
    para_AV=repmat(para_AV,250,1);
    %% Plot-Figure 5a
    tg_set=[0.1235,0.0736,0.0575,0.0516,0.0450];
    %---------------------------
    npre=1:5;
    log_npre=log10(npre);
    log_tg=log10(tg_set);
    [b,bint,r,rint,stats]=regress(log_tg',[ones(5,1),log_npre']);
    %---------------------------
    figure
    loglog(1:length(tg_set),tg_set,'rs','markerfacecolor','r','linewidth',1)
    hold on
    p_=loglog([1,5],10.^(b(1)+b(2)*log_npre([1,end])),'-','linewidth',1.5);
    legend(p_,'Linear regression')
    set(gca,'xtick',0:6)
    xlim([0.9 5.2])
    ylim([0.042 0.13])
    xlabel('Number of communication vehicles')
    ylabel('Time gap (s)')
    box on
    grid on
    set(gca,'fontname','times new roman','fontsize',16)
    
    %% Simulation
    scene_set=[120,20,1,1];
    for ns=1:size(scene_set,1)
        scene_=scene_set(ns,:);
        vs=scene_(1);%km/h
        vd=scene_(2);%km/h
        a_acc=scene_(3);%m/s2
        a_dec=scene_(4);%m/s2
        tgap=100;%s
        nd=2;
        
        data_Input=fun_generate_leadv(vs,vd,a_acc,a_dec,tgap,nd,'Both');
        LowLevelModel_label=0;
        NoiseStrength=[0.8556    0.0123];
        platsize_set=[1,2,5,10,20,50,100,200,500,1000];
        kps=0;
        FlowDensity_set=nan*zeros(length(platsize_set),4);
        for platsize=platsize_set
            kps=kps+1;
            fun_=str2func(['fun_SimMultiPlatoon_npre',num2str(npre)]);
            [no_platLeader,vdata,xdata,adata,acmd]=fun_(data_Input,para_AV,paraUpper,LowLevelModel_label,NoiseStrength,platsize);
            L_veh=4.835;
            tp=([1:length(vdata)]-1)*0.05;
            data_dx=[inf*xdata(:,end),xdata(:,1:end-1)]-xdata-L_veh;
            %------------------------------------------- Flow-density
            platV=nanmean(vdata(:,2:end),2);%m/s
            platLen=xdata(:,1)-xdata(:,end);
            nveh=size(vdata,2);
            platDen=(nveh-1)./platLen;%/m
            platFlow=platV.*platDen*3600;
            FlowDensity_=[vs,nanmean(platV)*3.6,nanmean(platDen),nanmean(platFlow)];
            FlowDensity_set(kps,:)=FlowDensity_;
        end
    end
    save(['MultiPlat_npre',num2str(npre),'.mat'],'paraUpper','npre','platsize_set','FlowDensity_set')
end
%% Plot-Compare
figure
mycolor={'k','r','g','b','m'};
leg_=[];leg_label={};
for npre=1:5
    load(['MultiPlat_npre',num2str(npre),'.mat'])
    base_=FlowDensity_set(end,4);
    loss_rate=(FlowDensity_set(:,4)-base_)./base_*100;
    p_=semilogx(platsize_set,FlowDensity_set(:,4),'-s','color',mycolor{npre},'markerfacecolor',mycolor{npre},'linewidth',1.5);hold on;
    leg_=[leg_;p_];
    leg_label=[leg_label;{['Commun. ',num2str(npre),' Veh.']}];
end
p_leg=legend(leg_,leg_label);
set(p_leg,'location','southeast')
xlabel('Platoon size')
ylabel('Traffic capacity (veh/h)')
xlim([0 max(platsize_set)])
ylim([min(FlowDensity_set(:,4))-500 max(FlowDensity_set(:,4))+500])
set(gca,'Fontname','Times New Roman', 'FontSize', 15)
grid on
box on