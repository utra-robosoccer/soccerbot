from tree.tree_node import TreeNode

#TODO put each action the robot is performing in team_data under each robot

class Action(TreeNode):
    def __init__(self):
        super().__init__(None)
        return

    def execute(self, robot, team_data):
        raise NotImplementedError("please implement")
