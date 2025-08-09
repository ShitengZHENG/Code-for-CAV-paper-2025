function td=fun_cal_TimeDelay(time,tmax,data1,data2)
idx=find(~isnan(data1));
st1=min(idx);et1=max(idx);
idx=find(~isnan(data2));
st2=min(idx);et2=max(idx);
st=max(st1,st2);et=min(et1,et2);
data1=data1(st:et);
data2=data2(st:et);
for j=1:2
    eval(['data_=data',num2str(j),';'])
    idx=isnan(data_);
    if sum(idx)==0
        continue
    end
    series=1:length(data_);
    series(idx)=[];data_(idx)=[];
    value=interp1(series,data_,1:length(data_));
    eval(['data',num2str(j),'=value;'])
end
coef=zeros(51,1);
for t=0:tmax
    s1=data1(1:end-t);
    s2=data2(1+t:end);
    coef(t+1)=fun_cal_Pearson(s1,s2);
end
[~,I]=max(coef);
td=time(I)-time(1);
