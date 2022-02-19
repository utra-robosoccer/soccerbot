from decisions.decision import Decision
import numpy as np

class IsClosestToBall(Decision):
    def execute(self, robot, team_data):
        if team_data.ball.is_known():
            #TODO take into account rotation
            ball_position = np.array(team_data.ball.position)
            a = [np.linalg.norm(ball_position - robot.position[:2]) for robot in team_data.robots.values()]
            closest = team_data.robots[np.argmin(a) + 1].player_id
            if robot.robot_id == closest:
                return True
        return False
