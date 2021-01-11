
from gurobipy import *
import numpy as np

TypesDemand=[3,7,9,16]
QuantityDemand=[25,30,14,8]
LengthUsable=20

MainProbRelax=Model()
SubProb=Model()
patterns=np.array([
    [6,0,0,0],
    [0,2,0,0],
    [0,0,2,0],
    [0,0,0,1]])

#主问题
x=MainProbRelax.addVars(len(TypesDemand),obj=1.0,vtype=GRB.CONTINUOUS,name='x')
Main_con=MainProbRelax.addConstrs((x[i] >= QuantityDemand[i] for i in range(len(TypesDemand))),name='con')
MainProbRelax.optimize()


#子问题
DualVar=MainProbRelax.getAttr(GRB.Attr.Pi,MainProbRelax.getConstrs())
y=SubProb.addVars(len(TypesDemand),obj=DualVar,vtype=GRB.INTEGER,name='y')
SubProb.addConstr(quicksum(y[i]*TypesDemand[i] for i in range(len(TypesDemand)))<=LengthUsable,name='sub_con')
SubProb.setAttr(GRB.Attr.ModelSense,-1)
SubProb.optimize()

iter_no=4
while SubProb.objval>1:
    ColumnCoeff=SubProb.getAttr('X',SubProb.getVars())
    ColumnCoeff_T=np.array(ColumnCoeff)
    ColumnCoeff_T=ColumnCoeff_T.reshape(ColumnCoeff_T.shape[0],1)
    patterns=np.append(patterns,ColumnCoeff_T,axis=1)

    column = Column(ColumnCoeff, MainProbRelax.getConstrs())

    #MainProbRelax.addVar(obj=1.0,vtype=GRB.CONTINUOUS,column=column,name=("x[{}]".format(iter_no)))
    MainProbRelax.addVar(obj=1.0, vtype=GRB.CONTINUOUS, column=column, name='x'+str(iter_no))
    MainProbRelax.optimize()

    for i in range(len(TypesDemand)):
        y[i].obj=Main_con[i].pi

    SubProb.optimize()
    iter_no+=1

for v in MainProbRelax.getVars():
    print(v.varName,'=',v.x)

print(patterns)

MainProbRelax.update()
MainProbRelax.write('test.lp')
SubProb.update()
SubProb.write('est.lp')