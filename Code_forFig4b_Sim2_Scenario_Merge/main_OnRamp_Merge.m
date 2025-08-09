clc
clear
close all

% Code for simulation of on-ramp merging scenario
% Code for Figure 4b in main text and  Section 4.3 in SI.
%% Parameter
%----------------------------------------------------AV-Upper controller
paraUpper=[0.6265    0.3703    0.4437    0.2889    0.6676    0.0736    1];
kv1_=paraUpper(1);kv2_=paraUpper(2);kg_=paraUpper(3);ka1_=paraUpper(4);ka2_=paraUpper(5);Tg_=paraUpper(6);
Gmin0=paraUpper(7);
L_veh=4.835;
Gmin=L_veh+Gmin0;

%----------------------------------------------------AV-Lower controller
max_vehnum=1000;
para_AV=[78.8201836419897,2.07647787797603,9.17891305242795,4;
    58.9344898328179,1.90325205249387,7.86998881634774,4;
    95.9937146710127,2.32906700594111,10.0111919741363,4;
    43.0500120244655,1.62000868090560,6.87020881889564,4];
para_AV=repmat(para_AV,max_vehnum/4,1);
%----------------------------------------------------Stochasticity
NoiseStrength=[0.8556    0.0123];
kappa=NoiseStrength(1);sigma=NoiseStrength(2);
%% 
L_road=2000;%m
x_merge=L_road*0.5;%m
L_cooperate=200;%m
L_lc=100;%m
a_comf=3;
x_lc_end=x_merge+L_cooperate+L_lc;

dt=0.05;
det_T=dt;
step=3600/dt;
%% main code
v_max=120/3.6;
start_loc=0;
v_NewMain=120/3.6;
start_loc_ramp=0;
v_NewRamp=v_NewMain;
%------------------------------------Main road
tg_main=Tg_;%s
lb_tg=0.2;ub_tg=0.5;p=0.8;

rng(3);
s11 = rng;
rnd1=rand(step,1);
rng(4);
s12 = rng;
platTimegap1=randi(round([lb_tg,ub_tg]/dt),max_vehnum,1);
%------------------------------------Ramp
tg_ramp=Tg_;%s
lb_tg=1.5;ub_tg=2.0;p=0.8;

rng(5);
s21 = rng;
rnd2=rand(step,1);
rng(6);
s22 = rng;
platTimegap2=randi(round([lb_tg,ub_tg]/dt),max_vehnum,1);
%------------------------------------Flow rate estimate
dx_=Tg_*v_max+Gmin;
flow_max_theo=1/dx_*v_max*3600;%veh/h
flow_main_test=3600/(mean(platTimegap1)*0.05);%veh/h
flow_ramp_test=3600/(mean(platTimegap2)*0.05);%veh/h
flow_max_theo-(flow_main_test+flow_ramp_test)
%-----------------------------------------------------------Initialize
nveh=2;
xdata=zeros(step,nveh);
vdata=zeros(step,nveh);
adata=zeros(step,nveh);
acmd=zeros(step,nveh);
noise=zeros(step,nveh);
data_laneNo=nan*zeros(step,nveh);
Idx_ShadowCar=cell(1,nveh);
num_flag=-1*ones(1,nveh);

rng(7);
s_ksi = rng;
rnd_ksi=randn(step,max_vehnum);

enter_veh=[1,2];t_enter=[1,2];
t_enter_main=[1];vehnum_main=1;
t_enter_ramp=[2];vehnum_ramp=1;
VehIdx_now=enter_veh;vehIdx=length(VehIdx_now);

xdata(1,1)=start_loc;
vdata(1,1)=v_NewMain;
adata(1,1)=0;
acmd(1,1)=0;
noise(1,1)=0;
data_laneNo(1,1)=1;

xdata(2,2)=start_loc_ramp;
vdata(2,2)=v_NewRamp;
adata(2,2)=0;
acmd(2,2)=0;
noise(2,2)=0;
data_laneNo(2,2)=-1;
%% main code
kk1=1;tgap1=0;
kk2=1;tgap2=0;
IdxSeries_out=[];
vehSeries_enterMerge=[];
t_enterMerge=[];
for t=1:step-1
    nveh=length(enter_veh);
    I_main=find(data_laneNo(t,:)==1);
    [~,I]=min(xdata(t,I_main));
    Idx_last=I_main(I);
    if isempty(Idx_last)
        v_NewMain_=v_NewMain;
        x_NewMain_=start_loc;
    else
        v_NewMain_=vdata(t,Idx_last);
        gap_init=tg_main*v_NewMain_+Gmin;
        x_NewMain_=xdata(t,Idx_last)-gap_init;
    end
    if x_NewMain_>=start_loc && t-t_enter_main(vehnum_main)>=tgap1
        vehIdx=vehIdx+1;
        enter_veh=[enter_veh,vehIdx];
        t_enter=[t_enter,t];
        VehIdx_now=enter_veh;
        if tgap1==0
            xdata(t,vehIdx)=x_NewMain_;
            vdata(t,vehIdx)=v_NewMain_;
        else
            xdata(t,vehIdx)=start_loc;
            dx=xdata(t,Idx_last)-start_loc;
            vdata(t,vehIdx)=min((dx-Gmin)/tg_main,v_max);
        end
        adata(t,vehIdx)=0;
        acmd(t,vehIdx)=0;
        noise(t,vehIdx)=0;
        data_laneNo(t,vehIdx)=1;
        Idx_ShadowCar(1,vehIdx)=cell(1);
        num_flag(1,vehIdx)=-1;
        nveh=nveh+1;
        t_enter_main=[t_enter_main;t];
        vehnum_main=vehnum_main+1;

        if rnd1(t)<p
            tgap1=platTimegap1(kk1);
            kk1=kk1+1;
        else
            tgap1=0;
        end
    end

    %----------------------------------------------------------------------
    I_ramp=find(data_laneNo(t,:)==-1);
    [~,I]=min(xdata(t,I_ramp));
    Idx_last=I_ramp(I);
    if isempty(Idx_last)
        v_NewRamp_=v_NewRamp;
        x_NewRamp_=start_loc_ramp;
    else
        v_NewRamp_=vdata(t,Idx_last);
        gap_init=tg_ramp*v_NewRamp_+Gmin;
        x_NewRamp_=xdata(t,Idx_last)-gap_init;
    end
    if x_NewRamp_>=start_loc_ramp && t-t_enter_ramp(vehnum_ramp)>=tgap2
        vehIdx=vehIdx+1;
        enter_veh=[enter_veh,vehIdx];
        t_enter=[t_enter,t];
        VehIdx_now=enter_veh;
        if tgap2==0
            xdata(t,vehIdx)=x_NewRamp_;
            vdata(t,vehIdx)=v_NewRamp_;
        else
            xdata(t,vehIdx)=start_loc_ramp;
            dx=xdata(t,Idx_last)-start_loc_ramp;
            vdata(t,vehIdx)=min((dx-Gmin)/tg_ramp,v_max);
        end        
        adata(t,vehIdx)=0;acmd(t,vehIdx)=0;
        noise(t,vehIdx)=0;
        data_laneNo(t,vehIdx)=-1;
        Idx_ShadowCar(1,vehIdx)=cell(1);
        num_flag(1,vehIdx)=-1;
        nveh=nveh+1;
        t_enter_ramp=[t_enter_ramp;t];
        vehnum_ramp=vehnum_ramp+1;

        if rnd1(t)<p
            tgap2=platTimegap2(kk2);
            kk2=kk2+1;
        else
            tgap2=0;
        end
    end
    
    %----------------------------------------------------------------------
    if t<=2
        dd=t-1;
    else
        dd=1;
    end
    [~,veh_set]=sort(xdata(t,VehIdx_now),'descend');
    for kj=veh_set
        j=VehIdx_now(kj);
        laneNo_=data_laneNo(t,j);
        if isnan(laneNo_)
            continue
        end
        xj=xdata(t,j);
        %==================================================================CommandAcceleration
        %--------------------------------------------------FIFO
        if num_flag(j)==-1 && xdata(t,j)>x_merge && xdata(t,j)<=x_lc_end
            Idx=xdata(t,vehSeries_enterMerge)<L_road;
            Idx_pre=vehSeries_enterMerge(Idx);
            npre=length(Idx_pre);
            if npre==0
                Idx_ShadowCar(j)={[]};
            elseif npre==1
                Idx_ShadowCar(j)={Idx_pre(end)};
            elseif npre>1
                Idx_ShadowCar(j)={Idx_pre(end-1:end)};
            end
            num_flag(j)=num_flag(j)+1;
            vehSeries_enterMerge=[vehSeries_enterMerge;j];
            t_enterMerge=[t_enterMerge;t];
            [~,I]=sort(t_enterMerge);
            vehSeries_enterMerge=vehSeries_enterMerge(I);
        end
        %--------------------------------------------------
        if xdata(t,j)>x_merge && xdata(t,j)<=x_lc_end
            decide_label_set=[1,2];
        else
            decide_label_set=[1];
        end
        for label_=decide_label_set
            if label_==1 
                if laneNo_==1 || laneNo_==0 
                    Idx_lane=find(data_laneNo(t,:)==laneNo_);
                    Idx_lane(Idx_lane==j)=[];
                    x_lane=xdata(t,Idx_lane);
                    xdiff=x_lane-xj;Idx_lane(xdiff<0)=[];xdiff(xdiff<0)=[];
                    [~,I]=sort(xdiff,'descend');
                    Idx_pre=Idx_lane(I);
                elseif laneNo_==-1
                    Idx_lane=find(data_laneNo(t,:)==laneNo_ | data_laneNo(t,:)==0);
                    Idx_lane(Idx_lane==j)=[];
                    x_lane=xdata(t,Idx_lane);
                    xdiff=x_lane-xj;Idx_lane(xdiff<0)=[];xdiff(xdiff<0)=[];
                    [~,I]=sort(xdiff,'descend');
                    Idx_pre=Idx_lane(I);
                end
                Idx_current_pre=Idx_pre;
            elseif label_==2
                Idx_pre=Idx_ShadowCar{j};
                [~,I]=sort(xdata(t,Idx_pre),'descend');
                Idx_pre=Idx_pre(I);
            end
            npre=length(Idx_pre);
            % Upper-level control
            acmd_=0;
            if npre==0 
                if vdata(t,j)<v_NewMain
                    acmd_=a_comf;
                end
            else
                I_pre1=Idx_pre(end);
                dx=xdata(t,I_pre1)-xdata(t,j);
                dv1=vdata(t-dd,I_pre1)-vdata(t,j);
                dx_desired=Tg_*vdata(t,j)+Gmin;
                I_pre=I_pre1;
                if npre==1
                    acmd_=(ka2_+ka1_)*acmd(t-dd,I_pre1)+(kv2_+kv1_)*dv1+kg_*(dx-dx_desired);
                else
                    I_pre2=Idx_pre(end-1);
                    dv2=vdata(t-dd,I_pre2)-vdata(t,j);
                    acmd_=ka2_*acmd(t-dd,I_pre2)+ka1_*acmd(t-dd,I_pre1)+kv2_*dv2+kv1_*dv1+kg_*(dx-dx_desired);
                end
            end
            if label_==1
                acmd_ego=acmd_;
            else
                acmd_virtual=acmd_;
            end
        end
        
        if length(decide_label_set)==1
            acmd(t+1,j)=acmd_ego;
        else
            if laneNo_==0 
                dx=x_lc_end-xdata(t,j);
                dv1=-vdata(t,j);
                dx_desired=Tg_*vdata(t,j)+Gmin;
                acmd_obstacle=0.3*(dx-dx_desired)+0.3*dv1;
                acmd_obstacle=a_comf;
                acmd(t+1,j)=min([acmd_obstacle,acmd_ego,acmd_virtual]);
            else
                acmd(t+1,j)=min(acmd_ego,acmd_virtual);
            end
        end
        % Safety control
        asafe_=a_comf;
        aneed_=(v_max-vdata(t,j))/dt;
        acmd(t+1,j)=max(-a_comf,min([asafe_,aneed_,acmd(t+1,j),a_comf]));

        %==================================================================Lower-level control
        X=para_AV(j,:);
        K=X(1);theta=X(2);omega=X(3);td=X(4);
        if t-t_enter(j)+1<=td
            adata(t+1,j)=acmd(t+1,j);
        else
            eq_left=K*acmd(t-td,j);
            adata(t+1,j)=(eq_left-omega^2*adata(t,j)+theta*omega*adata(t-1,j)/(2*det_T)-(-2*adata(t,j)+adata(t-1,j))/(det_T^2))*(det_T^2)/(1+theta*omega/2*det_T);
        end
        % Stochasticity
        noise(t+1,j)=(1-kappa*dt)*noise(t,j)+sigma*rnd_ksi(t,j)*(dt^0.5);
        adata(t+1,j)=adata(t+1,j)+noise(t+1,j);
        % Update
        vvv=max(0,vdata(t,j)+adata(t+1,j)*dt);
        vdata(t+1,j)=min(v_max,vvv);
        xdata(t+1,j)=xdata(t,j)+vdata(t+1,j)*dt;
        adata(t+1,j)=(vdata(t+1,j)-vdata(t,j))/dt;
        %==================================================================LaneNo Update
        data_laneNo(t+1,j)=data_laneNo(t,j);
        if laneNo_==-1 && xdata(t+1,j)>=x_merge 
            data_laneNo(t+1,j)=0;
        end
        if laneNo_==0 && xdata(t+1,j)>=x_merge+L_cooperate 
            I=find(vehSeries_enterMerge==j);
            I_pre1=vehSeries_enterMerge(I-1);
            I_fol=vehSeries_enterMerge(I+1);
            dx_desired_ego=Tg_*vdata(t,j)+Gmin;
            dx_desired_fol=Tg_*vdata(t,I_fol)+Gmin;
            dx_ego=xdata(t,I_pre1)-xdata(t,j);
            dx_fol=xdata(t,j)-xdata(t,I_fol);
            dv_ego=vdata(t,I_pre1)-vdata(t,j);
            dv_fol=vdata(t,j)-vdata(t,I_fol);
            TTC_ego=dx_ego/(-dv_ego);TTC_fol=dx_fol/(-dv_fol);
            if dx_ego>=L_veh && dx_fol>=L_veh && ...
                    (TTC_ego<0 || TTC_ego>1.0) && (TTC_fol<0 || TTC_fol>1.0)
                data_laneNo(t+1,j)=1;
            end
        end
    end
    
    % Boundary
    Idx_out=find(xdata(t+1,:)>=L_road);
    if ~isempty(Idx_out)
        vehNow_out=find(enter_veh==Idx_out);
        enter_veh(vehNow_out)=[];
        VehIdx_now=enter_veh;
        vdata(t+2:end,Idx_out)=nan;xdata(t+2:end,Idx_out)=nan;
        adata(t+2:end,Idx_out)=nan;acmd(t+2:end,Idx_out)=nan;
        noise(t+2:end,Idx_out)=nan;data_laneNo(t+2:end,Idx_out)=nan;
        t0=t_enter(Idx_out);
        vdata(1:t0-1,Idx_out)=nan;xdata(1:t0-1,Idx_out)=nan;
        adata(1:t0-1,Idx_out)=nan;acmd(1:t0-1,Idx_out)=nan;
        noise(1:t0-1,Idx_out)=nan;data_laneNo(1:t0-1,Idx_out)=nan;
        IdxSeries_out=[IdxSeries_out;Idx_out];
    end
    
    if vehIdx>=max_vehnum-1
        break
    end
end
nveh=enter_veh(end);
%--------------------------------------------------------------------------
vehIdx_main=[];
vehIdx_ramp=[];
for j=1:nveh
    if sum(data_laneNo(:,j)==0)>0
        vehIdx_ramp=[vehIdx_ramp;j];
    else
        vehIdx_main=[vehIdx_main;j];
    end    
end
%% plot
%---------------------------------------------------
figure
hold on
tenter_main=(t_enter_main(3:end)-t_enter_main(2:end-1))*0.05;
tenter_ramp=(t_enter_ramp(3:end)-t_enter_ramp(2:end-1))*0.05;
histogram(tenter_main,[0:0.05:2])
histogram(tenter_ramp,[0:0.05:2])
legend('Mainline','On-ramp')
xlabel('Entry time gap (s)')
ylabel('Count')
set(gca,'fontname','times new roman','fontsize',16)
box on
set(gca,'xtick',0:0.2:2)
xlim([0.2 1.55])
%--------------------------------------------------------------------------Speedheat map
mycmap_2=[0.188200000000000,0.207800000000000,0.576500000000000;0.189900000000000,0.211200000000000,0.578700000000000;0.191600000000000,0.214600000000000,0.581000000000000;0.193300000000000,0.217900000000000,0.583200000000000;0.195000000000000,0.221300000000000,0.585400000000000;0.196600000000000,0.224600000000000,0.587700000000000;0.198300000000000,0.228000000000000,0.589900000000000;0.200000000000000,0.231400000000000,0.592200000000000;0.171400000000000,0.271700000000000,0.619600000000000;0.142900000000000,0.312000000000000,0.647100000000000;0.114300000000000,0.352400000000000,0.674500000000000;0.0857000000000000,0.392700000000000,0.702000000000000;0.0571000000000000,0.433100000000000,0.729400000000000;0.0286000000000000,0.473400000000000,0.756900000000000;0,0.513700000000000,0.784300000000000;0,0.542300000000000,0.806200000000000;0,0.570900000000000,0.828000000000000;0,0.599400000000000,0.849900000000000;0,0.628000000000000,0.871700000000000;0,0.656600000000000,0.893600000000000;0,0.685200000000000,0.915400000000000;0,0.713700000000000,0.937300000000000;0.0218000000000000,0.717100000000000,0.901400000000000;0.0437000000000000,0.720400000000000,0.865500000000000;0.0655000000000000,0.723800000000000,0.829700000000000;0.0874000000000000,0.727200000000000,0.793800000000000;0.109200000000000,0.730500000000000,0.758000000000000;0.131100000000000,0.733900000000000,0.722100000000000;0.152900000000000,0.737300000000000,0.686300000000000;0.242600000000000,0.754600000000000,0.639200000000000;0.332200000000000,0.772000000000000,0.592200000000000;0.421800000000000,0.789400000000000,0.545100000000000;0.511500000000000,0.806700000000000,0.498000000000000;0.601100000000000,0.824100000000000,0.451000000000000;0.690800000000000,0.841500000000000,0.403900000000000;0.780400000000000,0.858800000000000,0.356900000000000;0.810600000000000,0.851000000000000,0.322700000000000;0.840900000000000,0.843100000000000,0.288500000000000;0.871100000000000,0.835300000000000,0.254300000000000;0.901400000000000,0.827500000000000,0.220200000000000;0.931700000000000,0.819600000000000,0.186000000000000;0.961900000000000,0.811800000000000,0.151800000000000;0.992200000000000,0.803900000000000,0.117600000000000;0.984900000000000,0.758500000000000,0.128300000000000;0.977600000000000,0.713200000000000,0.138900000000000;0.970300000000000,0.667800000000000,0.149600000000000;0.963000000000000,0.622400000000000,0.160200000000000;0.955700000000000,0.577000000000000,0.170900000000000;0.948500000000000,0.531700000000000,0.181500000000000;0.941200000000000,0.486300000000000,0.192200000000000;0.926600000000000,0.447600000000000,0.188800000000000;0.912000000000000,0.409000000000000,0.185400000000000;0.897500000000000,0.370300000000000,0.182100000000000;0.882900000000000,0.331700000000000,0.178700000000000;0.868300000000000,0.293000000000000,0.175400000000000;0.853800000000000,0.254300000000000,0.172000000000000;0.839200000000000,0.215700000000000,0.168600000000000;0.798300000000000,0.207800000000000,0.164700000000000;0.757400000000000,0.200000000000000,0.160800000000000;0.716500000000000,0.192200000000000,0.156900000000000;0.675600000000000,0.184300000000000,0.152900000000000;0.634700000000000,0.176500000000000,0.149000000000000;0.593800000000000,0.168600000000000,0.145100000000000;0.552900000000000,0.160800000000000,0.141200000000000];
xx=xdata(1:t,:);
vv=vdata(1:t,:)*3.6;
tt=repmat([1:t]'*0.05',1,size(xx,2));

figure
hold on
surf(tt,xx,vv)
shading interp
xlim([tt(1) tt(end)])
ylim([0 L_road])
zlim([80 120])
xlim([50 tt(end)])
ylim([800 1500])
colormap(flipud(mycmap_2))
caxis([90 120])
xlabel('Time (s)')
ylabel('Position (m)')
zlabel('Speed (km/h)')
ch=colorbar('Position',[0.916880754380754 0.445238095238095 0.0134763884763889 0.481783601014374]);
set(get(ch,'title'),'string','Unit: km/h','fontsize',13)
set(gca,'fontname','times new roman','fontsize',16)
view(50,82)
box on
grid on

%---------------------------------------------------Trajectory diagram
xx=xdata(1:t,:);
vv=vdata(1:t,:)*3.6;
LaneNo_=data_laneNo(1:t,:);
t_cc_set=[];
figure
hold on
tt=[1:t]'*0.05;
plot(tt,xx,'-','color',[.7 .7 .7],'linewidth',0.6)
for j=[139:144]
    if ismember(j,vehIdx_main)
        p1=plot(tt,xx(:,j),'k-','linewidth',0.6);
    elseif ismember(j,vehIdx_ramp)
        Idx=find(LaneNo_(:,j)==-1);
        t_cc=max(Idx);
        Idx=find(LaneNo_(:,j)==-1 | LaneNo_(:,j)==0);
        p2=plot(tt(Idx),xx(Idx,j),'r-','linewidth',1.2);
        t_lc=max(Idx);
        Idx=[max(Idx);find(LaneNo_(:,j)==1)];
        p3=plot(tt(Idx),xx(Idx,j),'b-','linewidth',1.2);
        t_cc_set=[t_cc_set;t_cc,t_lc];
    end
end
h=fill([tt([1,end])' fliplr(tt([1,end])')],[[x_merge,x_merge] [x_merge x_merge]+L_cooperate],'c');
set(h,'edgealpha',0,'facealpha',0.2)
h=fill([tt([1,end])' fliplr(tt([1,end])')],[[x_merge x_merge]+L_cooperate [x_lc_end x_lc_end]],'g');
set(h,'edgealpha',0,'facealpha',0.2)
p1=plot(0,0,'k-','linewidth',1.2);
p_leg=legend([p1,p2,p3],'Main lane','On ramp (Before merging)','On ramp (After merging)');
set(p_leg,'location','southeast')
xlim([tt(1) tt(end)])
xlim([65 80])
ylim([0 L_road])
ylim([900 1400])
box on
xlabel('Time (s)')
ylabel('Positon (m)')
set(gca,'fontname','times new roman','fontsize',16)
box on

%---------------------------------------------------Speed profile
mycolor={'b','r'};
figure
hold on
leg_=[];
kj=0;
for j=[139:144]
    if ismember(j,vehIdx_main)
        p_=plot(tt,vv(:,j),'-','linewidth',1.1);
        leg_=[leg_;p_];
    elseif ismember(j,vehIdx_ramp)
        kj=kj+1;
        p_=plot(tt,vv(:,j),'-','color',mycolor{kj},'linewidth',1.2);
        leg_=[leg_;p_];
    end
end
kj=0;
for j=[141,142]
    kj=kj+1;
    t_cc=t_cc_set(kj,1);
    t_lc=t_cc_set(kj,2);
    p_=scatter(tt(t_cc),vv(t_cc,j),30,mycolor{kj},'filled','MarkerFaceAlpha',0.5);
    p_=scatter(tt(t_lc),vv(t_lc,j),30,mycolor{kj},'filled','MarkerFaceAlpha',0.5);
end
xlim([65 80])
ylim([100 122])
p_leg=legend(leg_,'Predecessor 2','Predecessor 1','Merger 1','Merger 2','Follower 1','Follower 2');
set(p_leg,'location','southeast')
xlabel('Time (s)')
ylabel('Speed (km/h)')
set(gca,'fontname','times new roman','fontsize',16)
box on

%% Order
xdata=xdata(:,IdxSeries_out);
vdata=vdata(:,IdxSeries_out);
adata=adata(:,IdxSeries_out);
acmd=acmd(:,IdxSeries_out);
noise=noise(:,IdxSeries_out);
data_laneNo=data_laneNo(:,IdxSeries_out);
nveh=length(IdxSeries_out);
vehIdx_main=[];
vehIdx_ramp=[];
for j=1:nveh
    if sum(data_laneNo(:,j)==0)>0
        vehIdx_ramp=[vehIdx_ramp;j];
    else
        vehIdx_main=[vehIdx_main;j];
    end    
end

%----------------------------Average travel delay
td_set=[];
for j=1:nveh
    I=find(~isnan(xdata(:,j)));
    et=max(I);st=min(I);
    t_travel=(et-st)*0.05;
    t_theo=(xdata(et,j)-xdata(st,j))/v_max;
    td_=t_travel-t_theo;
    td_set=[td_set;td_];
end
ave_delay=mean(td_set);
%----------------------------Average duration
dur_set=[];
for j=1:nveh
    st=find(xdata(1:end-1,j)<x_merge+L_cooperate & xdata(2:end,j)>=x_merge+L_cooperate);
    et=find(data_laneNo(1:end-1,j)==0 & data_laneNo(2:end,j)==1);
    t_dur=(et-st)*0.05;
    dur_set=[dur_set;t_dur];
end
ave_dur=mean(dur_set);
%----------------------------Average speed
xx=xdata;

vv=vdata;
vv(xx>x_merge)=nan;
aveV_1=nanmean(vv(:))*3.6;

vv=vdata;
vv(xx<x_merge | xx>x_lc_end)=nan;
aveV_2=nanmean(vv(:))*3.6;

vv=vdata;
vv(xx<=x_lc_end)=nan;
aveV_3=nanmean(vv(:))*3.6;

ave_Result=[ave_delay,aveV_1,aveV_2,aveV_3,ave_dur]
%% Flow rate
dx_=Tg_*v_max+Gmin;
flow_max_theo=1/dx_*v_max*3600;%veh/h

%--------------------------------------------------------------------------Upstream of the main road
xloc=500;
tg_main_ave=mean(t_enter_main(2:end)-t_enter_main(1:end-1))*dt;
dx_=tg_main_ave*v_max+Gmin;
flow_main_theo=1/dx_*v_max*3600;%veh/h
flow_main_theo2=(length(t_enter_main)-1)/((t_enter_main(end)-t_enter_main(1))*dt)*3600;%veh/h
xdata_main=xdata(:,vehIdx_main);
vdata_main=vdata(:,vehIdx_main);
t_set=[];v_set=[];
vehnum_main=size(xdata_main,2);
for j=1:vehnum_main
    tI=find(xdata_main(1:step-1,j)<xloc & xdata_main(2:step,j)>=xloc);
    if isempty(tI)
        continue
    end
    v_set(j)=vdata_main(tI,j);
    t_set(j)=tI;
end
flow=3600*vehnum_main/((t_set(end)-t_set(1))*dt);%veh/h
flow_main_loop=flow;
%--------------------------------------------------------------------------Upstream of the ramp
dx_=tg_ramp*v_max+Gmin;
flow_ramp_theo=1/dx_*v_max*3600;%veh/h
flow_ramp_theo2=(length(t_enter_ramp)-1)/((t_enter_ramp(end)-t_enter_ramp(1))*dt)*3600;%veh/h
xdata_ramp=xdata(:,vehIdx_ramp);
vdata_ramp=vdata(:,vehIdx_ramp);
t_set=[];v_set=[];
vehnum_ramp=size(xdata_ramp,2);
for j=1:vehnum_ramp
    tI=find(xdata_ramp(1:step-1,j)<xloc & xdata_ramp(2:step,j)>=xloc);
    if isempty(tI)
        continue
    end
    v_set(j)=vdata_ramp(tI,j);
    t_set(j)=tI;
end
flow=3600*vehnum_ramp/((t_set(end)-t_set(1))*dt);%veh/h
flow_ramp_loop=flow;
%--------------------------------------------------------------------------Downstream of the main road
xloc=1800;
t_set=[];v_set=[];
for j=1:nveh
    tI=find(xdata(1:step-1,j)<xloc & xdata(2:step,j)>=xloc);
    if isempty(tI)
        continue
    end
    v_set(j)=vdata(tI,j);
    t_set(j)=tI;
end
flow=3600*nveh/((t_set(end)-t_set(1))*dt);%veh/h
flow_ramp_loopdown=flow;

flow_result=[flow_main_theo2,flow_main_loop,nan;
             flow_ramp_theo2,flow_ramp_loop,nan;
             flow_max_theo,flow_ramp_loopdown,flow_main_loop+flow_ramp_loop];
