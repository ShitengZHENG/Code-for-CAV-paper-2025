function [acc,v_smooth]=fun_smooth_v_fft(v,Fs,high_f)

index_nan=isnan(v);
v_smooth=zeros(size(v));

for i=1:size(v,2)
    x=v(:,i);
    if sum(isnan(x))==0
        continue
    end
    Idx=find(~isnan(x));
    x_new=interp1(Idx,x(Idx),1:length(x));
    v(:,i)=x_new;
end
for i=1:size(v,2)
    x=v(:,i);
    len=length(x);
    if sum(~isnan(x))<=10
        continue
    end
    
    t=0;stt=1;ett=len;
    while t<len
        if sum(isnan(x))==0
            break;
        end
        st=min(find(isnan(x)))-1;
        tt=find(~isnan(x));
        tt(tt<=st)=nan;
        et=min(tt);
        if ~isnan(st) && isnan(et)
            ett=st;
            x=x(stt:ett);
            break
        elseif isnan(st) && ~isnan(et)
            stt=et;
            t=et;
            continue
        end
        t=et;
        if st==0
            st=1;x_st=1;
            pp=spline([st et],[x_st x(et)],st:et);
        else
            pp=spline([st et],[x(st) x(et)],st:et);
        end
        x(st:et)=pp;
        if stt~=1 || ett~=len
            x=x(stt:ett);
        end
    end

    L = length(x);
    interval=floor(high_f*L/Fs+1);
    X=fft(x);
    H=[ones(interval,1);zeros(L-2*interval,1);ones(interval,1)];
    Y= X.* H;
    y=ifft(Y);
    v_smooth(stt:ett,i)=real(y);
    if sum(~isnan(v_smooth(:,i)))==0
        error('filter: nan')
    end
end

v_smooth(v_smooth<0)=0;
v_smooth(v==0)=0;
v_smooth(index_nan)=nan;
acc=zeros(size(v));
acc(index_nan)=nan;
acc(2:end,:)=(v_smooth(2:end,:)-v_smooth(1:end-1,:))/(1/Fs);