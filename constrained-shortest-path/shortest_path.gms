$Ontext
Shortest Pth Problem using Column Generation
created by Qian Zhu
2021-01-010

$offtext

set i node /1*6/

    p path or column /1*10/
    p_RMP(p) the initial feasible solution /1/
    p_update(p) /1/;

alias(i,j);

parameter cost(i,j) cost for each link /
    1.2 1
    1.3 10
    3.2 1
    2.4 1
    3.5 12
    2.5 2
    3.4 5
    4.5 10
    4.6 1
    5.6 2/;
parameter time(i,j) time for each link /
    1.2 10
    1.3 3
    3.2 2
    2.4 1
    3.5 3
    2.5 3
    3.4 7
    4.5 1
    4.6 7
    5.6 2/;
parameter origin(i)
          destination(i)
          intermediate(i);
origin('1')=1;
destination('6')=1;
intermediate(i)=(1-origin(i))*(1-destination(i));

parameter path_cost(p) /
    1 24/;
parameter path_time(p) /
    1 8/;

positive variable lambda(p);
variable z_master;
equations obj_master
          time_con
          path_con;
obj_master.. z_master=e=sum(p_RMP,path_cost(p_RMP)*lambda(p_RMP));
time_con..sum(p_RMP,path_time(p_RMP)*lambda(p_RMP))=l=14;
path_con..sum(p_RMP,lambda(p_RMP))=e=1;
model RMP /obj_master,time_con,path_con/;
*Solve RMP using lp minimizing z_master;

******************the above model is RMP, the following is sub-problem

parameter pi_1 multiplier of constriant time_con;
parameter pi_0 multiplier of constriant path_con;

binary variable y(i,j);
variable z_sub;
equations obj_sub
          comm_flow_on_node_origin(i)
          comm_flow_on_node_intermediate(i)
          comm_flow_on_node_destination(i);
obj_sub.. z_sub=e=sum((i,j),(cost(i,j)-pi_1*time(i,j))*y(i,j))-pi_0;
comm_flow_on_node_origin(i)$(origin(i)=1)..sum(j$(cost(i,j)>0.1),y(i,j))=e=1;
comm_flow_on_node_destination(i)$(destination(i)=1)..sum(j$(cost(j,i)>0.1),y(j,i))=e=1;
comm_flow_on_node_intermediate(i)$(intermediate(i)=1)..sum(j$(cost(i,j)>0.1),y(i,j))-sum(j$(cost(j,i)>0.1),y(j,i))=e=0;
model SUB /obj_sub,comm_flow_on_node_origin,comm_flow_on_node_destination,comm_flow_on_node_intermediate/;
*Solve RMP using lp minimizing z_master;

******************the above is sub-problem, the following is iteration
set iter /1*30/;
parameter condition;
condition=-1;
loop(iter$(condition<0),
****step1 solve the master problem
   Solve RMP using lp minimizing z_master;
****step 2 solve the sub-problem
   pi_1=time_con.m;
   pi_0=path_con.m;
   Solve SUB using mip minimizing z_sub;
   if(z_sub.l<0,
      condition=-1;
   else
      condition=1;
   );
****step 3
    p_update(p)=p_update(p-1);
    p_RMP(p_update)=yes;
    path_cost(p_update)=sum((i,j),cost(i,j)*y.l(i,j));
    path_time(p_update)=sum((i,j),time(i,j)*y.l(i,j));
);

display lambda.l,z_master.l,y.l;




