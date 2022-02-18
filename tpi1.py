#Pedro Lopes 97827

#People I discussed with:
#Vicente Costa 98515
#Joao Borges 98155
#Filipe Gonçalves 98083
#Gonçalo Machado 98359

from tree_search import *
from cidades import *
from math import floor

class MyNode(SearchNode):
    def __init__(self,state,parent,depth=None,cost=None,heuristic=None):
        super().__init__(state,parent)
        self.depth = depth
        self.cost = cost
        self.heuristic = heuristic
        self.eval = floor(cost + heuristic)
        self.children = []

class MyTree(SearchTree):

    def __init__(self,problem, strategy='breadth',seed=0):
        super().__init__(problem,strategy,seed)
        #           state,             parent,  depth,  cost, heuristic
        root = MyNode(problem.initial, None,    0,      0,    self.problem.domain.heuristic(problem.initial, problem.goal))
        self.all_nodes = [root]
        self.open_nodes = [0]
        self.strategy = strategy
        self.solution = None
        self.non_terminals = 0
        self.curr_pseudo_rand_number = seed  # values between 0 and 99; only used for rand_depth search
        self.used_shortcuts = []

    def astar_add_to_open(self,lnewnodes):
        self.open_nodes.extend(lnewnodes)
        self.open_nodes = sorted(self.open_nodes, key=lambda x: self.all_nodes[x].cost + self.all_nodes[x].heuristic)

    def propagate_eval_upwards(self,node):
        if node.children != []:
            minimo = min([self.all_nodes[n].eval for n in node.children])
            node.eval = minimo
        if node.parent != None:
            node_parent = self.all_nodes[node.parent]
            self.propagate_eval_upwards(node_parent)

    def cityInNode(self, newnode):
        node_states = [ node.state for node in self.all_nodes ]
        repeated = [True if st == newnode.state else False for st in node_states ]

        if True not in repeated:
            return False

        node = self.all_nodes[repeated.index(True)]
        if node.cost > newnode.cost:
            index = self.all_nodes.index(node)
            self.all_nodes[index] = newnode

        return True

    def search2(self,atmostonce=False):
        while self.open_nodes != []:
            nodeID = self.open_nodes.pop(0)
            node = self.all_nodes[nodeID]
            if self.problem.goal_test(node.state):
                self.solution = node
                self.terminals = len(self.open_nodes)+1
                self.solution_path = self.get_path(node)
                return self.get_path(node)
            lnewnodes = []
            self.non_terminals += 1
            for a in self.problem.domain.actions(node.state):
                newstate = self.problem.domain.result(node.state,a)
                if newstate not in self.get_path(node):
                    #newnode = SearchNode(newstate,nodeID)
                    cost = self.problem.domain.cost(node.state, a)
                    heuristic = self.problem.domain.heuristic(newstate, self.problem.goal)
                    my_new_node = MyNode(newstate, nodeID , node.depth + 1, node.cost + cost, heuristic )
                    if atmostonce == False or ( atmostonce == True and self.cityInNode(my_new_node) == False):
                        self.all_nodes.append(my_new_node)
                        node.children.append(len(self.all_nodes) - 1)
                        lnewnodes.append(len(self.all_nodes) - 1)
            self.propagate_eval_upwards(node)
            self.add_to_open(lnewnodes)
        return None

    def repeated_random_depth(self,numattempts=3,atmostonce=False):
        temp = None
        for i in range(numattempts):
            new_tree = MyTree(self.problem, self.strategy,seed=i)
            new_tree_result = new_tree.search2()
            if not temp:
                temp = new_tree
            if temp.solution.cost > new_tree.solution.cost:
                temp = new_tree
        self.solution_tree = temp
        return new_tree_result


    def make_shortcuts(self):
        idx = 0
        shorter_path = []
        found_shorcut = False
        while idx < len(self.solution_path):

            if not found_shorcut:
                shorter_path.append(self.solution_path[idx]) # Braga
            actions = self.problem.domain.actions(self.solution_path[idx]) #Braga
            for idxC2 in range(len(self.solution_path)-1, idx + 1, -1): #
                secondC = self.solution_path[idxC2]
                for action in actions:
                    if secondC in action:
                        idx = idxC2 - 1
                        self.used_shortcuts.append(action)
                        found_shorcut = True
                        shorter_path.append(action[1])
                        break
                    else:
                        found_shorcut = False
            idx += 1
        return shorter_path









class MyCities(Cidades):
    def maximum_tree_size(self,depth):   # assuming there is no loop prevention
        dic = {}
        for (e1,e2,d) in self.connections:
            if e1 not in dic.keys():
                dic[e1] = [state for state in self.connections if e1 in state]
            if e2 not in dic.keys():
                dic[e2] = [state for state in self.connections if e2 in state]

        avg = sum(len(x) for x in dic.values())/len(dic.keys())
        return sum(avg**dep for dep in range(0, depth+1))
