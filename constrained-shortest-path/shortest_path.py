import pandas as pd
from gurobipy import *
import numpy as np
import math
import time
import matplotlib.pyplot as plt

g_node_list=[]
g_link_list=[]
g_demand_list=[]
g_node_Id_ListNum_dict={}
g_node_ListNum_Id_dict={}
g_link_Id_ListNum_dict={}
g_link_ListNum_Id_dict={}
g_link_FromTo_ListNum_dict={}

g_node_No=-1
g_link_No=-1
g_demand_No=-1
time_budget=14
pi_1=0
path_set_list=[[0,2,3,5],[0,2,1,4,5]]
obj_value_list=[]
class Node():
    def __init__(self):
        self.node_id=0
        self.outgoing_node_list=[]
        self.incoming_node_list=[]

class Link():
    def __init__(self):
        self.link_id=0
        self.from_node_id=0
        self.to_node_id=0
        self.time=0
        self.lanes=0
        self.free_speed=0
        self.capacity=0
        self.cost=0
        self.com_cost=0

    def CompositCost(self):
        self.com_cost=self.cost-pi_1*self.time

class Demand():
    def __init__(self):
        self.o_zone_id=0
        self.d_zone_id=0
        self.volume=0


def ReadData():
    print('I am reading data, take your time.')
    global g_node_No
    global g_link_No
    global g_demand_No


    with open('node.csv') as file_object:
        lines=file_object.readlines()

        for line in lines[1:]:
            line=line.strip().split(',')
            node=Node()

            try:
                node.node_id=int(line[1])
            except:
                print('Python can not read node.csv successfuly, please check!')
            else:
                g_node_No+=1
                g_node_list.append(node)
                g_node_Id_ListNum_dict[node.node_id]=g_node_No
                g_node_ListNum_Id_dict[g_node_No] = node.node_id

                if g_node_No % 100 ==0:
                    print('the number of nodes has been read is: {}'.format(g_node_No))


    with open('link.csv') as file_object:
        lines=file_object.readlines()
        for line in lines[1:]:
            line=line.strip().split(',')
            link=Link()

            try:
                link.link_id=int(line[1])
                link.from_node_id=int(line[3])
                link.to_node_id=int(line[4])
                link.time=float(line[6])
                link.lanes=int(line[7])
                link.free_speed=float(line[8])
                link.capacity=float(line[9])
                link.cost=float(line[15])
            except:
                print('Python can not read link.csv successfuly, please check!')
            else:
                g_link_No+=1
                g_link_list.append(link)
                g_link_Id_ListNum_dict[link.link_id] = g_link_No
                g_link_ListNum_Id_dict[g_link_No] = link.link_id
                g_link_FromTo_ListNum_dict[(link.from_node_id,link.to_node_id)]=g_link_No
                from_node_ListNumb=g_node_Id_ListNum_dict[link.from_node_id]
                to_node_ListNumb = g_node_Id_ListNum_dict[link.to_node_id]
                g_node_list[from_node_ListNumb].outgoing_node_list.append(link.to_node_id)
                g_node_list[to_node_ListNumb].incoming_node_list.append(link.from_node_id)


                if g_link_No % 100 ==0:
                    print('the number of links has been read is: {}'.format(g_link_No))

    with open('demand.csv') as file_object:
        lines=file_object.readlines()
        for line in lines[1:]:
            line=line.strip().split(',')
            demand=Demand()

            try:
                demand.o_zone_id=int(line[0])
                demand.d_zone_id=int(line[1])
                demand.volume=float(line[2])
            except:
                print('Python can not read demand.csv successfuly, please check!')
            else:
                g_demand_No += 1
                g_demand_list.append(demand)
                if g_demand_No%100==0:

                    print('the number of demand has been read is: {}'.format(g_demand_No))


def LabelCorrecting(pi_1,origin, destination):

    for link in g_link_list:
        link.CompositCost()

    distance = [9999] * (g_node_No + 1)
    pred = [0] * (g_node_No + 1)
    origin_ListNum = g_node_Id_ListNum_dict[origin]
    distance[origin_ListNum] = 0

    dynamic_list = [origin]

    while len(dynamic_list) != 0:
        CurNode = dynamic_list.pop(0)
        CurNode_ListNum = g_node_Id_ListNum_dict[CurNode]
        for to_node in g_node_list[CurNode_ListNum].outgoing_node_list:
            to_node_ListNum = g_node_Id_ListNum_dict[to_node]

            link = (CurNode, to_node)
            link_ListNum = g_link_FromTo_ListNum_dict[link]

            if distance[to_node_ListNum] > distance[CurNode_ListNum] + g_link_list[link_ListNum].com_cost:
                distance[to_node_ListNum] = distance[CurNode_ListNum] + g_link_list[link_ListNum].com_cost
                pred[to_node_ListNum] = CurNode

                if to_node not in dynamic_list:
                    dynamic_list.append(to_node)

    path = []
    path_list=[destination]
    ToNode = destination
    FromNode = 9999
    while FromNode != origin:
        ToNode_ListNum = g_node_Id_ListNum_dict[ToNode]
        FromNode = pred[ToNode_ListNum]
        link = (FromNode, ToNode)
        path.insert(0, link)
        ToNode = FromNode
        path_list.insert(0, FromNode)
    destination_ListNum = g_node_Id_ListNum_dict[destination]
    path_CompositCost = distance[destination_ListNum]

    path_cost = 0
    path_time = 0
    for link in path:
        link_ListNum = g_link_FromTo_ListNum_dict[link]
        path_cost += g_link_list[link_ListNum].cost
        path_time += g_link_list[link_ListNum].time

    return path_list,path_CompositCost, path_cost, path_time


path_cost_list=[24,15]
path_time_list=[8,10]
reduced_cost=-1
def ColumnGeneration(origin,destination):
    global pi_1
    MainProbRelax=Model()
    SubProb=Model()

    #Main-problem
    x=MainProbRelax.addVars(len(path_cost_list),vtype=GRB.CONTINUOUS,name='x')
    time_con=MainProbRelax.addConstr(quicksum(path_time_list[i]*x[i] for i in range(len(path_cost_list)))<=14,name='con_1')
    path_con=MainProbRelax.addConstr(quicksum(x[i] for i in range(len(path_cost_list)))==1,name='con_2')
    MainProbRelax.setObjective(quicksum(path_cost_list[i]*x[i] for i in range(len(path_cost_list))),sense=GRB.MINIMIZE)
    MainProbRelax.optimize()
    obj_value_list.append(MainProbRelax.objVal)
    #Sub-problem
    DualVar = MainProbRelax.getAttr(GRB.Attr.Pi, MainProbRelax.getConstrs())
    pi_1=DualVar[0]
    pi_0=DualVar[1]

    path_list,path_CompositCost, path_cost, path_time=LabelCorrecting(pi_1,origin, destination)
    reduced_cost=path_CompositCost-pi_0

    iter_no=2
    while reduced_cost<0:
        path_set_list.append(path_list)

        ColumnCoeff=[path_time,1]
        column=Column(ColumnCoeff,MainProbRelax.getConstrs())

        MainProbRelax.addVar(obj=path_cost,vtype=GRB.CONTINUOUS,column=column,name='x'+str(iter_no))
        MainProbRelax.optimize()
        obj_value_list.append(MainProbRelax.objVal)
        pi_1=time_con.pi
        pi_0=path_con.pi
        path_list,path_CompositCost, path_cost, path_time = LabelCorrecting(pi_1, origin, destination)
        reduced_cost = path_CompositCost - pi_0
        iter_no+=1
    print('-----------------------------------')
    print('The optimal variable values is...')
    for v in MainProbRelax.getVars():
        print(v.varName, '=', v.x)
    print('-----------------------------------')
    print('The final path column pool is...')
    print(path_set_list)
    print('-----------------------------------')
    print('The optimal objective function value is...')
    print(MainProbRelax.objVal)

if __name__=='__main__':
    ReadData()
    ColumnGeneration(0,5)

    plt.figure()
    plt.plot(np.arange(1,len(obj_value_list)+1),obj_value_list)
    plt.xlabel('Iteration number')
    plt.ylabel('Objective function value')
    new_ticks = range(1, len(obj_value_list)+1, 1)
    plt.xticks(new_ticks)
    plt.show()