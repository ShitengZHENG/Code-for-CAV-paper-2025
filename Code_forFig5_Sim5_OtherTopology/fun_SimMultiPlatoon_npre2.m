function [no_platLeader,vdata,xdata,adata,acmd]=fun_SimMultiPlatoon_npre2(data_Input,para_AV,paraUpper,LowLevelModel_label,NoiseStrength,platsize)

L_veh=4.835;
kv1_=paraUpper(1);kv2_=paraUpper(2);kg_=paraUpper(3);ka1_=paraUpper(4);ka2_=paraUpper(5);Tg_=paraUpper(6);Gmin0_=paraUpper(7);
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
no_platLeader=[];
vdata(1,:)=vdata(1,1);
for nj=nveh:-1:2
    j=mod(nj,platsize);
    if mod(nj,platsize)==0
        j=platsize;
    end
    if j==1
        no_platLeader=[no_platLeader;nj];
        Tg_p=2;%s
        gap_init=Gmin+vdata(1,nj)*Tg_p;
    else
        gap_init=Gmin+vdata(1,1)*Tg_;
    end
    xdata(1,nj-1)=xdata(1,nj)+gap_init;
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
    dv2=vdata(t-dd,1:end-2)-vdata(t,3:end);
    dv1=[nan,dv1];
    dv2=[nan,nan,dv2];
    for nj=2:nveh
        j=mod(nj,platsize);
        if mod(nj,platsize)==0
            j=platsize;
        end
        X=para_AV(j,:);
        if j==1
            kv_p=0.5;kg_p=0.5;Tg_p=2;%s
            dx_desired=Tg_p*vdata(t,nj)+Gmin;
            acmd(t+1,nj)=kv_p*dv1(nj)+kg_p*(dx(nj)-dx_desired);
        elseif j==2
            dx_desired=Tg_*vdata(t,nj)+Gmin;
            acmd(t+1,nj)=ka2_*acmd(t-dd,nj-1)+ka1_*acmd(t-dd,nj-1)+kv2_*dv1(nj)+kv1_*dv1(nj)+kg_*(dx(nj)-dx_desired);
        else
            dx_desired=Tg_*vdata(t,nj)+Gmin;
            acmd(t+1,nj)=ka2_*acmd(t-dd,nj-2)+ka1_*acmd(t-dd,nj-1)+kv2_*dv2(nj)+kv1_*dv1(nj)+kg_*(dx(nj)-dx_desired);
        end
        if LowLevelModel_label==0
            K=X(1);theta=X(2);omega=X(3);td=X(4);
            if t<td+1
                adata(t+1,nj)=0;
            else
                eq_left=K*acmd(t-td,nj);
                adata(t+1,nj)=(eq_left-omega^2*adata(t,nj)+theta*omega*adata(t-1,nj)/(2*det_T)-(-2*adata(t,nj)+adata(t-1,nj))/(det_T^2))*(det_T^2)/(1+theta*omega/2*det_T);
            end
        end
    end
    adata(t+1,2:end)=adata(t+1,2:end)+noise(t+1,2:end);
    % Update
    vdata(t+1,2:end)=max(0,vdata(t,2:end)+adata(t+1,2:end)*dt);
    xdata(t+1,:)=xdata(t,:)+vdata(t+1,:)*dt;
end