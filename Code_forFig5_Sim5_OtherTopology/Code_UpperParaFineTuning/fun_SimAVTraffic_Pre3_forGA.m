function objfun=fun_SimAVTraffic_Pre3_forGA(paraUpper,para_AV,LowLevelModel_label)

timestep=0.05;
L_veh=4.835;
kv1_=paraUpper(1);kv2_=paraUpper(2);kv3_=paraUpper(3);
kg_=paraUpper(4);
ka1_=paraUpper(5);ka2_=paraUpper(6);ka3_=paraUpper(7);
Tg_=paraUpper(8);
Gmin0=5;
Gmin=L_veh+Gmin0;

dt=timestep;
nveh=size(para_AV,1);
%% Scene setting
fun_pertur_scenario_forGA;
scene_set=[];
for vd=[10:20:50]
    for vs=20:20:120
        if vs>vd
            scene_set=[scene_set;[vs,vd]];
        end
    end
end
scene_set=[[scene_set;scene_set],[[3*ones(length(scene_set),1),7*ones(length(scene_set),1)];[3*ones(length(scene_set),1),7*ones(length(scene_set),1)]]];%¼¤½ø
pertur_type=[repmat({'Upper'},length(scene_set)/2,1);repmat({'Lower'},length(scene_set)/2,1)];
%% main code
mindiff_dx=inf;
objfun_set=zeros(4+length(scene_set),1);
for ns=1:4+length(scene_set)
    % Scene 
    if ns<=4
        data_Input=pertur_set{ns};
    elseif ns>4
        type_=pertur_type{ns-4};
        scene_=scene_set(ns-4,:);
        vs=scene_(1);%km/h
        vd=scene_(2);%km/h
        a_acc=scene_(3);%m/s2
        a_dec=scene_(4);%m/s2
        data_Input=fun_generate_leadv(vs,vd,a_acc,a_dec,50,2,type_,timestep);
    end
    % Scene read
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
    gap_init=Gmin+vdata(1,1)*Tg_;
    xdata(1,:)=gap_init*(nveh-1):-gap_init:0;
    
    % main code
    det_T=dt;
    for t=1:step-1
        if t<=2
            dd=t-1;
        else
            dd=1;
        end
        dx=xdata(t,[end,1:end-1])-xdata(t,:);
        dv1=vdata(t-dd,1:end-1)-vdata(t,2:end);
        dv2=vdata(t-dd,1:end-2)-vdata(t,3:end);
        dv3=vdata(t-dd,1:end-3)-vdata(t,4:end);
        dv1=[nan,dv1];
        dv2=[nan,nan,dv2];
        dv3=[nan,nan,nan,dv3];
        for j=2:nveh
            X=para_AV(j,:);
            if j==2
                dx_desired=Tg_*vdata(t,j)+Gmin;
                acmd(t+1,j)=ka3_*acmd(t-dd,j-1)+ka2_*acmd(t-dd,j-1)+ka1_*acmd(t-dd,j-1)+...
                            kv3_*dv1(j)+kv2_*dv1(j)+kv1_*dv1(j)+...
                            kg_*(dx(j)-dx_desired);
            elseif j==3
                dx_desired=Tg_*vdata(t,j)+Gmin;
                acmd(t+1,j)=ka3_*acmd(t-dd,j-2)+ka2_*acmd(t-dd,j-2)+ka1_*acmd(t-dd,j-1)+...
                            kv3_*dv2(j)+kv2_*dv2(j)+kv1_*dv1(j)+...
                            kg_*(dx(j)-dx_desired);
            else
                dx_desired=Tg_*vdata(t,j)+Gmin;
                acmd(t+1,j)=ka3_*acmd(t-dd,j-3)+ka2_*acmd(t-dd,j-2)+ka1_*acmd(t-dd,j-1)+...
                            kv3_*dv3(j)+kv2_*dv2(j)+kv1_*dv1(j)+...
                            kg_*(dx(j)-dx_desired);
            end
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
        % Update
        vdata(t+1,2:end)=max(0,vdata(t,2:end)+adata(t+1,2:end)*dt);
        xdata(t+1,:)=xdata(t,:)+vdata(t+1,:)*dt;
    end
    fun_StabilityCriteria;
end
Gmin0_=Gmin0-mindiff_dx;
objfun=sum(objfun_set)+Tg_*7260+121*Gmin0_;