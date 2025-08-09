function min_objfun=fun_GA_NoiseCalibrate(X,Dataset_v3)

NoiseStrength=X;

LowLevelModel_label=0;
% AV upper-level control parameters
paraUpper=[0.4, 0.5, 0.1, 0.5, 0.4, 0];%Exp
% AV lower-level control parameters
paraset_AV=[78.8201836419897,2.07647787797603,9.17891305242795,4;
    58.9344898328179,1.90325205249387,7.86998881634774,4;
    95.9937146710127,2.32906700594111,10.0111919741363,4;
    43.0500120244655,1.62000868090560,6.87020881889564,4];
%%
nf2=0;
num_Run=length(Dataset_v3);
min_objfun_=nan*zeros(num_Run,4);
for nf=1:num_Run
    nf2=nf2+1;
    dataset_=Dataset_v3(nf,:);
    tp=dataset_{2}(:,1);
    data_dx=dataset_{5};
    data_v=dataset_{6};
    v_leader=data_v(:,2);
    data_Acmd=dataset_{8};  
    data_Aactual=dataset_{9};  

    Idx=max(find(v_leader==0));
    for jj=1:5
        data_v(Idx:end,jj+1)=data_v(Idx,jj+1);
        data_dx(Idx:end,jj)=data_dx(Idx,jj);
        data_Acmd(Idx:end,jj)=data_Acmd(Idx,jj);
        data_Aactual(Idx:end,jj)=data_Aactual(Idx,jj);
    end
    for nj=2:5
        if sum(~isnan(data_v(:,nj)))==0 || sum(~isnan(data_v(:,nj+1)))==0 || sum(~isnan(data_dx(:,nj)))==0
            continue
        end
        data_Input=[tp,data_Acmd(:,nj-1),data_Aactual(:,nj-1),data_v(:,nj),data_dx(:,nj)];
        
        data_dx_exp=data_dx(:,nj-1:nj);
        data_v_exp=data_v(:,nj:nj+1);
        para_AV=[zeros(1,size(paraset_AV,2));paraset_AV(nj-1,:)];
        %----------------------------------------------------------------------Exp-Det.Sim
        L_veh=4.835;
        fun=str2func(['fun_SimAVTraffic_S1a']);
        [vdata,xdata,adata_actual]=fun(data_Input,para_AV,paraUpper,LowLevelModel_label);
        dx_sim=[xdata(:,end),xdata(:,1:end-1)]-xdata-L_veh;
        v_exp=data_v_exp(:,2:end);
        v_sim=vdata(:,2:end);
        error_dx=data_dx_exp(:,2:end)-dx_sim(:,2:end);
        error_v=v_exp-v_sim;
        %----------------------------------------------------------------------Sto.Sim-Det.Sim
        simdata_set={};
        fun=str2func(['fun_SimAVTraffic_S1aNoise']);
        num_cishu=20;
        for cishu=1:num_cishu
            [vdata_noise,xdata_noise,adata_actual_noise]=fun(data_Input,para_AV,paraUpper,LowLevelModel_label,NoiseStrength);
            dx_noise=xdata_noise(:,1:end-1)-xdata_noise(:,2:end)-L_veh;
            simdata_set(cishu,1:3)=[{vdata_noise},{dx_noise},{adata_actual_noise}];
        end
        vvv_nj=[];dx_nj=[];
        for cishu=1:num_cishu
            vvv_=simdata_set{cishu,1}(:,2);
            dx_=simdata_set{cishu,2};
            vvv_nj=[vvv_nj,vvv_];
            dx_nj=[dx_nj,dx_];
        end
        error_dxNoise=dx_nj-dx_sim(:,2);
        error_vNoise=vvv_nj-v_sim;

        obj_=[];
        for jj=1:num_cishu
            [~,~,KSSTAT_dx] = kstest2(error_dxNoise(:,jj),error_dx);
            [~,~,KSSTAT_v] = kstest2(error_vNoise(:,jj),error_v);
            obj_(jj,1)=KSSTAT_dx+KSSTAT_v;
        end
        [min_obj,I]=min(obj_);
        min_objfun_(nf2,nj-1)=min_obj;
    end
end
min_objfun=nanmean(min_objfun_(:));