class TreeNode:
    def __init__(self, children: list):
        self.children = children

    def execute(self, robot, team_data):
        raise NotImplementedError("please implement")


class Action(TreeNode):
    def __init__(self):
        super().__init__(None)
        return

    def execute(self, robot, team_data):
        raise NotImplementedError("please implement")
