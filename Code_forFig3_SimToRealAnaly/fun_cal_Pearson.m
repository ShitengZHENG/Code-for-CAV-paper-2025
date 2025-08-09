function Index=fun_cal_Pearson(x,y)

a=nansum(   (x-nanmean(x)) .* (y-nanmean(y)) );
b=sqrt( nansum((x-nanmean(x)).^2) * nansum((y-nanmean(y)).^2) );
Index=a/b;