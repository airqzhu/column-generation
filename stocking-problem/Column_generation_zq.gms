
$Ontext
Stocking Problem using Column Generation
created by Qian Zhu
2021-01-07

$offtext
set j columns /1*10/
    j_RMP(j) /1,2,3/
*j_RMPÊÇjµÄ×Ó¼¯£¬°üÀ¨1£,2£,3 Èý¸öelements
    j_update(j) /3/;

set i clients /1*3/;

table a(i,j) the number of cutting for ith clietns on jth pattern
          1   2    3
       1  1   0    0
       2  0   1    0
       3  0   0    1 ;

parameter d(i) the cutting number of ith clients/
    1  44
    2  3
    3  38/;

variables
    z_master the total number of stocks used;

positive variable x(j) the cutting number of jth pattern£»

equations obj_master
          demand(i);
obj_master..z_master=e= sum(j_RMP,x(j_RMP));
demand(i)..sum(j_RMP,a(i,j_RMP)*x(j_RMP))=g= d(i);
model RMP /obj_master,demand/;
*Solve RMP using lp minimizing z_master;

******************the above model is RMP, the following is sub-problem

parameter l(i) the cutting length of the ith lients/
    1  81
    2  70
    3  68/;
parameter max_L the maximum lenght of each stock /218/;
parameter multiplier(i);

integer variable y(i) the cutting number of the ith clients's requirement;
variable z_sub the objective value of the sub problem;

equations obj_sub
          length;
obj_sub..  z_sub=e= sum(i,multiplier(i)*y(i));
length.. sum(i, l(i)*y(i)) =l= max_L;
model SUB /obj_sub,length/;

******************the above is sub-problem, the following is iteration


set iter /1*30/;
scalar done /0/;
scalar best /9999/;
loop(iter$(not done),
****step 1: solve a RMP
    Solve RMP using lp minimizing z_master;

    if (z_master.l<best,
        best=z_master.l;
    else
        done=1;
    );
****step 2: solve a sub-problem
    multiplier(i)=demand.m(i);
    Solve SUB using mip maximizing z_sub;

****step 3: solve a sub-problem
    j_update(j) = j_update(j-1);
    j_RMP(j_update)=yes;
    a(i,j_update) = y.l(i);

);
display z_master.l,x.l,a;





