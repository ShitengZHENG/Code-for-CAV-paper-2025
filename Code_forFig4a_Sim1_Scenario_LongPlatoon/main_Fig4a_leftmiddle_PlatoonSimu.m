clc
clear
close all

% Code for Figure 4a-left panel and middle panel in main text
% Code for Figures S.21,S.23-26 in SI
%% scene setting
scene_set=[];
for vd=[0:10:110]
    for vs=10:10:120
        if vs>vd
            scene_set=[scene_set;[vs,vd]];
        end
    end
end
pertur_type=repmat({'Both'},length(scene_set),1);
scene_set=[scene_set,1*ones(length(scene_set),1),1*ones(length(scene_set),1)];%Mild
scene_set=[scene_set,3*ones(length(scene_set),1),7*ones(length(scene_set),1)];%Aggressive
%% Simulation
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
    RatevNormInf_all=nan*zeros(length(para_AV)-npre,size(scene_set,1));
    simudata=[];
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
        Rate_vNormInf_set=nan*zeros(length(para_AV)-npre,num_cishu);
        mindx_=inf;maxdx_=0;
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
            %------------------------------------------- Norm-v-Inf
            normInf_v=max(abs(normalize_v),[],1);
            rate_vNormInf=normInf_v(npre:end)/normInf_v(npre);
            Rate_vNormInf_set(:,cishu)=rate_vNormInf;
            %-------------------------------------------dx
            mindx_=min(mindx_,min(min(data_dx(:,1+npre:end))));
            maxdx_=max(maxdx_,max(max(data_dx(:,1+npre:end))));
        end
        simudata(ns,:)=[scene_,mindx_,maxdx_];
        rate_vNormInf=nanmean(Rate_vNormInf_set,2);
        RatevNormInf_all(:,ns)=rate_vNormInf;
    end
    %% Plot
    %----------------------------------------------------------------------Spacing
    figure
    hold on
    [x,y]=meshgrid(0:10:110,10:10:120);
    z_max=nan*zeros(12,12);
    z_min=nan*zeros(12,12);
    kvd=0;
    for vd=0:10:110
        kvd=kvd+1;
        kvs=0;
        for vs=10:10:120
            kvs=kvs+1;
            I=find(simudata(:,1)==vs & simudata(:,2)==vd);
            if ~isempty(I)
                z_min(kvd,kvs)=simudata(I,end-1);
                z_max(kvd,kvs)=simudata(I,end);
            end
        end
    end
    surf(x,y,z_min')
    surf(x,y,z_max')
    ylim([10 120])
    set(gca,'xtick',10:20:120)
    set(gca,'ytick',20:20:120)
    set(gca,'ztick',0:15)
    view(39,13)
    xlabel('{\itv_{\fontsize{10}d}\rm} (km/h)')
    ylabel('{\itv_{\fontsize{10}s}\rm} (km/h)')
    zlabel('Spacing (m)')
    set(gca,'fontsize',14)
    grid on
    box off
    
    %----------------------------------------------------------------------Stability
    kvd=0;
    for vd=[10,30,50]
        kvd=kvd+1;
        RatevNormInf=[nan*zeros(npre-1,size(RatevNormInf_all,2));RatevNormInf_all];
        figure
        hold on
        idx=scene_set(:,2)==vd;
        [x,y]=meshgrid(vd+10:10:120,npre+1:4:100);
        z=RatevNormInf(npre+1:4:100,idx);
        surf(x,y,z)
        view(122,7)
        zlabel('\itL\rm_{\fontsize{10}\infty,\it\fontsize{10}n\rm} / \itL\rm_{\fontsize{10}\infty,\fontsize{10}2}')
        ylabel('Position in platoon')
        xlabel('{\itv_{\fontsize{10}s}\rm} (km/h)')
        xlim([vd+10 130])
        if npre==1 || npre==3 || npre==4
            zlim([0.3 1.05])
        elseif npre==2
            zlim([0.3 1.05])
        elseif npre==5
            zlim([0.4 1.05])
        end
        set(gca,'ytick',0:10:400)
        set(gca,'xtick',vd+10:20:120)
        set(gca,'XDir','reverse')
        grid on
        title(['{\itv_{\fontsize{10}d}\rm} = ',num2str(vd),' km/h'])
        set(gca,'fontname','times new roman','fontsize',14)
    end
end