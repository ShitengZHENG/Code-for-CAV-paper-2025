
vvv_ego=dataset_{6};
vvv_c5=dataset_{7};
for jj=2:size(dataset_,2)   
    data0_=dataset_{jj};
    for jjj=1:size(data0_,2)
        data_=data0_(:,jjj);
        st=min(find(~isnan(data_)));
        et=max(find(~isnan(data_)));
        if isempty(st) && isempty(et)
            continue
        end
        %----
        if size(data0_,2)==5
            dd=1;
        elseif size(data0_,2)==6
            dd=0;
        end
        if vvv_c5(st,jjj+dd)==0 || vvv_ego(st,jjj+dd)==0
            data0_(1:st,jjj)=data0_(st,jjj);
        end
        if vvv_c5(et,jjj+dd)==0 || vvv_ego(et,jjj+dd)==0
            data0_(et:end,jjj)=data0_(et,jjj);
        end
    end
    
    dataset_(jj)={data0_};
end