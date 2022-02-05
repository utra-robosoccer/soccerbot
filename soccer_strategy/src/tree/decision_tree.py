# build a tree with tree and actions
# have run method
from tree.tree_node import TreeNode
from actions.action import Action


class DecisionTree:
    def __init__(self, root: TreeNode):
        self.root = root

    def execute(self, robot, team_data):
        curr = self.root
        while not isinstance(curr, Action):
            if curr.execute(robot, team_data):
                curr = curr.children[0]
            else:
                curr = curr.children[1]
        # action
        curr.execute(robot, team_data)
