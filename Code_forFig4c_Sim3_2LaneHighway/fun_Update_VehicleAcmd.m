function acmd_=fun_Update_VehicleAcmd(npre,dx,dv1,dv2,acmd_dd1,acmd_dd2,paraUpper,vj,a_Exit,a_Signal,simu_setting)

dt=simu_setting(1);
v_max=simu_setting(2);
L_veh=simu_setting(3);
a_comf=simu_setting(4);
%==================================================================
acmd_=0;
if npre==0 
    if vj<v_max
        acmd_=a_comf;
    end
else
    kv1_=paraUpper(1);kv2_=paraUpper(2);kg_=paraUpper(3);ka1_=paraUpper(4);ka2_=paraUpper(5);Tg_=paraUpper(6);
    Gmin0=paraUpper(7);
    Gmin=L_veh+Gmin0;
    dx_desired=Tg_*vj+Gmin;
    acmd_=ka2_*acmd_dd2+ka1_*acmd_dd1+kv2_*dv2+kv1_*dv1+kg_*(dx-dx_desired);
end

%==================================================================
a_limit=(v_max-vj)/dt;
acmd_=max(-10,min([acmd_,a_Signal,a_Exit,a_limit,a_comf]));