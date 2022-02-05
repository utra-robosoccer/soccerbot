from typing import List

class TreeNode:
    def __init__(self, children: List):
        self.children = children

    def execute(self, robot, team_data):
        raise NotImplementedError("please implement")

