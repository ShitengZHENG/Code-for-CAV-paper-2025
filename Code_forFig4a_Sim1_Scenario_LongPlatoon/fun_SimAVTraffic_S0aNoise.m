function [vdata,xdata,adata,acmd]=fun_SimAVTraffic_S0aNoise(data_Input,para_AV,paraUpper,LowLevelModel_label,NoiseStrength)

L_veh=4.835;
kv1_=paraUpper(1);kg_=paraUpper(2);ka1_=paraUpper(3);Tg_=paraUpper(4);Gmin0_=paraUpper(5);
Gmin=L_veh+Gmin0_;

dt=0.05;
nveh=size(para_AV,1);
%% main code
tp=data_Input(:,1);
tp=tp-tp(1);
step=length(tp);
% Initialize
xdata=zeros(step,nveh);
vdata=zeros(step,nveh);
adata=zeros(step,nveh);
acmd=zeros(step,nveh);
% Leader
acmd(:,1)=data_Input(:,2);
adata(:,1)=data_Input(:,3);
vdata(:,1)=data_Input(:,4);

% Initial t
vdata(1,:)=vdata(1,1);
if nveh>5
    gap_init=Gmin+vdata(1,1)*Tg_;
    xdata(1,:)=gap_init*(nveh-1):-gap_init:0;
elseif nveh==5
    gap_init=data_Input(1,5:end);
    idx=isnan(gap_init);
    gap_init(idx)=nanmean(gap_init);
    xdata(1,end)=0;
    for j=nveh:-1:2
        xdata(1,j-1)=xdata(1,j)+gap_init(j)+L_veh;
    end
elseif nveh==2
    gap_init=data_Input(1,5);
    xdata(1,end)=0;
    for j=nveh:-1:2
        xdata(1,j-1)=xdata(1,j)+gap_init+L_veh;
    end
end

% main code
noise=zeros(step,nveh);
rnd_ksi=randn(step,nveh);
kappa=NoiseStrength(1);sigma=NoiseStrength(2);
det_T=dt;
for t=1:step-1
    if t<=2
        dd=t-1;
    else
        dd=1;
    end
    noise(t+1,:)=(1-kappa*dt)*noise(t,:)+sigma*rnd_ksi(t,:)*(dt^0.5);
    dx=xdata(t,[end,1:end-1])-xdata(t,:);
    dv1=vdata(t-dd,1:end-1)-vdata(t,2:end);
    dv1=[nan,dv1];
    for j=2:nveh
        X=para_AV(j,:);
        dx_desired=Tg_*vdata(t,j)+Gmin;
        acmd(t+1,j)=ka1_*acmd(t-dd,j-1)+kv1_*dv1(j)+kg_*(dx(j)-dx_desired);
        if LowLevelModel_label==0
            K=X(1);theta=X(2);omega=X(3);td=X(4);
            if t<td+1
                adata(t+1,j)=0;
            else
                eq_left=K*acmd(t-td,j);
                adata(t+1,j)=(eq_left-omega^2*adata(t,j)+theta*omega*adata(t-1,j)/(2*det_T)-(-2*adata(t,j)+adata(t-1,j))/(det_T^2))*(det_T^2)/(1+theta*omega/2*det_T);
            end
        end
    end
    adata(t+1,2:end)=adata(t+1,2:end)+noise(t+1,2:end);
    % Update
    vdata(t+1,2:end)=max(0,vdata(t,2:end)+adata(t+1,2:end)*dt);
    xdata(t+1,:)=xdata(t,:)+vdata(t+1,:)*dt;
end