from tree.tree_node import TreeNode

class Decision(TreeNode):
    #returns true or false
    def execute(self, robot, team_data):
        raise NotImplementedError("please implement")


#TODO when close to the net can use probabilities to decide whether to shoot or pass
