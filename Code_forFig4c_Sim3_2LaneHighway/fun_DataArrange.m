step=numel(data_vehicles);
veh_finalstatus=data_vehicles(step).time;
total_nveh=max([veh_finalstatus.No]);
data_pos=nan*zeros(step,total_nveh);
data_dx=nan*zeros(step,total_nveh);
vdata=nan*zeros(step,total_nveh);
adata=nan*zeros(step,total_nveh);
acmd=nan*zeros(step,total_nveh);
data_laneNo=nan*zeros(step,total_nveh);
label_OutOfSystem=ones(step,total_nveh);

w_veh=2.1489;%vehicle width

for t=1:step
    veh_status=data_vehicles(t).time;
    nveh=numel(veh_status);
    for kj=1:nveh
        nokj_=veh_status(kj).No;
        laneID_=veh_status(kj).lane;
        pos_kj=veh_status(kj).pos;
        data_pos(t,nokj_)=pos_kj;
        data_dx(t,nokj_)=veh_status(kj).IVS;
        vdata(t,nokj_)=veh_status(kj).spe;
        adata(t,nokj_)=veh_status(kj).acc;
        acmd(t,nokj_)=veh_status(kj).acmd;
        data_laneNo(t,nokj_)=laneID_;
        if veh_status(kj).OutOfSystem==0
            label_OutOfSystem(t,nokj_)=0;
        end
    end
end
