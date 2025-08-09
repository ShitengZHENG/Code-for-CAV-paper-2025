function [vdata,xdata,adata]=fun_SimCFpair_Noise(data_Input,para_,paraUpper,LowLevelModel_label,NoiseStrength)

dt=0.05;

% AV-Upper controller
L_veh=4.835;
Gmin=L_veh+5;
kv1_=paraUpper(1);kv2_=paraUpper(2);kg_=paraUpper(3);ka1_=paraUpper(4);ka2_=paraUpper(5);Tg_=paraUpper(6);

% AV-Lower controller
if length(data_Input)==2
    para_AV=[zeros(1,size(para_,2));para_];
elseif length(data_Input)==3
    para_AV=[zeros(2,size(para_,2));para_];
end
nveh=size(para_AV,1);
%% main code
tp=data_Input{1}(:,1);
tp=tp-tp(1);
step=length(tp);
% Initialize
xdata=zeros(step,nveh);
vdata=zeros(step,nveh);
adata=zeros(step,nveh);
acmd=zeros(step,nveh);
% Leader
for jj=1:nveh-1
    data_Input_=data_Input{jj};
    acmd(:,jj)=data_Input_(:,2);
    adata(:,jj)=data_Input_(:,3);
    vdata(:,jj)=data_Input_(:,4);
end
% Initial t
data_Input_F=data_Input{end};
vdata(1,end)=data_Input_F(1,4);
gap_init=data_Input_F(1,5);
xdata(1,end)=0;
for j=nveh:-1:2
    xdata(1,j-1)=xdata(1,j)+gap_init+L_veh;
end
for t=1:step-1
    xdata(t+1,:)=xdata(t,:)+vdata(t+1,:)*dt;
end

% main code
noise=zeros(step,nveh);
rnd_ksi=randn(step,nveh);
kappa=NoiseStrength(1);sigma=NoiseStrength(2);
det_T=dt;
for t=1:step-1
    if t<=2
        dd_a=t-1;
    else
        dd_a=1;
    end
    dd_v=dd_a;
    noise(t+1,:)=(1-kappa*dt)*noise(t,:)+sigma*rnd_ksi(t,:)*(dt^0.5);
    dx=xdata(t,[end,1:end-1])-xdata(t,:);
    dv1=vdata(t-dd_v,1:end-1)-vdata(t,2:end);
    dv2=vdata(t-dd_v,1:end-2)-vdata(t,3:end);
    dv1=[nan,dv1];
    dv2=[nan,nan,dv2];
    for j=nveh
        X=para_AV(j,:);
        % ÉÏ²ã¿ØÖÆ
        if j==2
            dx_desired=Tg_*vdata(t,j)+Gmin;
            acmd(t+1,j)=ka2_*acmd(t-dd_a,j-1)+ka1_*acmd(t-dd_a,j-1)+kv2_*dv1(j)+kv1_*dv1(j)+kg_*(dx(j)-dx_desired);
        elseif j==3
            dx_desired=Tg_*vdata(t,j)+Gmin;
            acmd(t+1,j)=ka2_*acmd(t-dd_a,j-2)+ka1_*acmd(t-dd_a,j-1)+kv2_*dv2(j)+kv1_*dv1(j)+kg_*(dx(j)-dx_desired);
        end
        if LowLevelModel_label==0
            K=X(1);theta=X(2);omega=X(3);td=X(4);
            if t<td+1
                adata(t+1,j)=data_Input{j}(t+1,3);
            else
                eq_left=K*acmd(t-td,j);
                adata(t+1,j)=(eq_left-omega^2*adata(t,j)+theta*omega*adata(t-1,j)/(2*det_T)-(-2*adata(t,j)+adata(t-1,j))/(det_T^2))*(det_T^2)/(1+theta*omega/2*det_T);
            end
        end
        adata(t+1,j)=adata(t+1,j)+noise(t+1,j);
        % Update
        vdata(t+1,j)=max(0,vdata(t,j)+adata(t+1,j)*dt);
        xdata(t+1,j)=xdata(t,j)+vdata(t+1,j)*dt;
    end
end
