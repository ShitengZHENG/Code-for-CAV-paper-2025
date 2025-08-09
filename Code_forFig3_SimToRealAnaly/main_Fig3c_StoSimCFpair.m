clc
clear
close all

% Code for Figure 3c-right in main text
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
%% Input data
nf=2;
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
for jj=5
    data_v(Idx:end,jj+1)=data_v(Idx,jj+1);
    data_dx(Idx:end,jj)=data_dx(Idx,jj);
    data_Acmd(Idx:end,jj)=data_Acmd(Idx,jj);
    data_Aactual(Idx:end,jj)=data_Aactual(Idx,jj);
end
% 里程生成
xstart=f_xnew(-10,1207);
ystart=f_ynew(-10,1207);
data_xgnew=f_xnew(data_xg,data_yg);
data_ygnew=f_ynew(data_xg,data_yg);
data_dis=sqrt( (data_xgnew-xstart).^2+(data_ygnew-ystart).^2 );
data_dis=data_dis-data_dis(1,end);

% data_Input
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
%----------------------------------------------------------------------Experiment
data_Input=data_Input_set(nj-2:nj);
v_exp=data_Input{end}(:,4);
dx_exp=data_Input{end}(:,5);
st=max(find(v_exp(1:end-1)==0 & v_exp(2:end)>0))+1;
et=max(find(v_exp(1:end-1)>0 & v_exp(2:end)==0));
v_exp=v_exp(st:et,:);
dx_exp=dx_exp(st:et,:);
%% Simulation
nj=5;
L_veh=4.835;
% AV upper-level control parameters
paraUpper=[0.4, 0.5, 0.1, 0.5, 0.4, 0];%Exp
% AV lower-level control parameters
paraset_AV=[78.8201836419897,2.07647787797603,9.17891305242795,4;
    58.9344898328179,1.90325205249387,7.86998881634774,4;
    95.9937146710127,2.32906700594111,10.0111919741363,4;
    43.0500120244655,1.62000868090560,6.87020881889564,4];
para_AV=paraset_AV(nj-1,:);
%----------------------------------------------------------------------Exp-Det.Sim
LowLevelModel_label=0;
NoiseStrength=[0,0];
fun=str2func(['fun_SimCFpair_Noise']);
[vdata,xdata,adata_actual]=fun(data_Input,para_AV,paraUpper,LowLevelModel_label,NoiseStrength);
vdata=vdata(st:et,:);
xdata=xdata(st:et,:);
% 偏差分布
dx_sim=xdata(:,end-1)-xdata(:,end)-L_veh;
v_sim=vdata(:,end);
error_dx=dx_exp-dx_sim;
error_v=v_exp-v_sim;
%----------------------------------------------------------------------Sto.Sim-Det.Sim
NoiseStrength=[0.8556    0.0123];
fun=str2func(['fun_SimCFpair_Noise']);
num_cishu=100;simdata_set={};
for cishu=1:num_cishu
    [vdata_noise,xdata_noise,adata_actual_noise]=fun(data_Input,para_AV,paraUpper,LowLevelModel_label,NoiseStrength);
    vdata_noise=vdata_noise(st:et,:);
    xdata_noise=xdata_noise(st:et,:);
    dx_noise=xdata_noise(:,end-1)-xdata_noise(:,end)-L_veh;
    v_noise=vdata_noise(:,end);
    aActual_noise=adata_actual_noise(:,end);
    simdata_set(cishu,1:3)=[{v_noise},{dx_noise},{aActual_noise}];
end
% 置信区间计算
vvv_nj=[];dx_nj=[];
for cishu=1:num_cishu
    vvv_=simdata_set{cishu,1};
    dx_=simdata_set{cishu,2};
    vvv_nj=[vvv_nj,vvv_];
    dx_nj=[dx_nj,dx_];
end
error_dxNoise=dx_nj-dx_sim;
error_vNoise=vvv_nj-v_sim;
vdata_lb=prctile(vvv_nj,10,2);
vdata_ub=prctile(vvv_nj,90,2);
dx_lb=prctile(dx_nj,10,2);
dx_ub=prctile(dx_nj,90,2);
%------------------------------------------------------------------Paperused_f2-c5
step=length(v_exp);
tt=([1:step]-1)*0.05;
figure
subplot(2,1,1)
hold on
plot(tt,v_exp*3.6,'k-','linewidth',1)
h=fill([tt fliplr(tt)],[vdata_lb' fliplr(vdata_ub')]*3.6,'b');
set(h,'edgealpha',0,'facealpha',0.2)
legend('Exp.','Sim. (95% CI)')
ylabel('Speed (km/h)')
xlim([tt(1) tt(end)])
set(gca,'fontname','times new roman','fontsize',16)
set(gca,'xticklabel',{''})
set(gca,'ytick',0:10:40)
set(gca,'yticklabel',0:10:40)
box on
subplot(2,1,2)
hold on
plot(tt,dx_exp,'k-','linewidth',1)
h=fill([tt fliplr(tt)],[dx_lb' fliplr(dx_ub')],'b');
set(h,'edgealpha',0,'facealpha',0.2)
xlim([tt(1) tt(end)])
xlabel('Time (s)')
ylabel('Spacing (m)')
set(gca,'fontname','times new roman','fontsize',16)
box on
ylim([3.5 6.8])
