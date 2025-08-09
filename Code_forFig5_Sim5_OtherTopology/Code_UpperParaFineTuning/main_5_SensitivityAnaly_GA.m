clc
clear
close all
%%
timestep=0.05;
para_AV=[78.8201836419897,2.07647787797603,9.17891305242795,4;
    58.9344898328179,1.90325205249387,7.86998881634774,4;
    95.9937146710127,2.32906700594111,10.0111919741363,4;
    43.0500120244655,1.62000868090560,6.87020881889564,4];
para_AV=repmat(para_AV,100,1);
para_AV=[zeros(1,size(para_AV,2));para_AV];
%% GA
for npre=1:5
    if npre==1
        LB=[0, 0.1, 0,     0];
        UB=[2,  2,  2, ub_tg];
    elseif npre==2
        LB=[0, 0,  0.1,  0, 0,     0];
        UB=[1, 1,  2.0,  1, 1, ub_tg];
    elseif npre==3
        LB=[0, 0, 0,  0.1,  0, 0, 0, 0];
        UB=[1, 1, 1,  2,  1, 1, 1, ub_tg];
    elseif npre==4
        LB=[0, 0, 0, 0,   0.1,  0, 0, 0, 0,  0];
        UB=[1, 1, 1, 1,   2,   1, 1, 1, 1,  ub_tg];
    elseif npre==5
        LB=[0, 0, 0, 0, 0,  0.1,  0, 0, 0, 0, 0,  0];
        UB=[1, 1, 1, 1, 1,   2,   1, 1, 1, 1, 1,  ub_tg];
    end
    
    % Standard genetic algorithm
    NVARS=length(LB);
    options=gaoptimset();
    options=gaoptimset(options,'UseParallel',1);
    options.Generations=300;
    options.PopulationSize=30;
    options.TolFun=1e-4;
    options=gaoptimset(options,'PlotFcns',{@gaplotbestf});
    if npre==1
        fun=@(X)fun_SimAVTraffic_Pre1_forGA(X,para_AV,LowLevelModel_label);
    elseif npre==2
        fun=@(X)fun_SimAVTraffic_Pre2_forGA(X,para_AV,LowLevelModel_label);
    elseif npre==3
        fun=@(X)fun_SimAVTraffic_Pre3_forGA(X,para_AV,LowLevelModel_label);
    elseif npre==4
        fun=@(X)fun_SimAVTraffic_Pre4_forGA(X,para_AV,LowLevelModel_label);
    elseif npre==5
        fun=@(X)fun_SimAVTraffic_Pre5_forGA(X,para_AV,LowLevelModel_label);
    end
    [X,FVAL,EXITFLAG,OUTPUT,population,scores]=ga(fun,NVARS,[],[],[],[],LB,UB,[],[],options);
    [FVAL,Idx]=min(scores);
    tg_=X(end)
end