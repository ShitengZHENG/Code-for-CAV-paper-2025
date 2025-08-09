clc
clear
close all
%% Load
path='.';
load([path,'\Dataset_v3.mat'])  
label_UseOrNot=label_abnormal==0 & para_UpperLevel(:,7)==5;
idx=label_UseOrNot==0;
Dataset_v3(idx,:)=[];
num_Run=length(Dataset_v3);
for nf=1:num_Run
    dataset_=Dataset_v3(nf,:);
    fun_dealNaN;
    Dataset_v3(nf,:)=dataset_;
end
%% GA
LB=[0.01, 1];
UB=[0.01, 1];
NVARS=length(LB);
options = gaoptimset();
options=gaoptimset(options,'UseParallel',1);
options.Generations=300; 
options.PopulationSize=50;
options.TolFun=1e-4;
options=gaoptimset(options,'PlotFcns',{@gaplotbestf});
fun = @(X)fun_GA_NoiseCalibrate(X,Dataset_v3);
[X,FVAL,EXITFLAG,OUTPUT,population,scores]=ga(fun,NVARS,[],[],[],[],LB,UB,[],[],options);
[FVAL,Idx]=min(scores);
X=population(Idx,:);
