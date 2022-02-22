from vispy.color import Color
from vispy import scene
import math
import numpy as np
from robot_controlled_2d import RobotControlled2D

white = Color("#ecf0f1")
gray = Color("#121212")
red = Color("#e74c3c")
blue = Color("#2980b9")
orange = Color("#e88834")
green = Color("#50bb00")
black = Color("#000000")


# Rules and Dimensions https://cdn.robocup.org/hl/wp/2021/04/V-HL21_Rules_changesMarked.pdf
class Scene:
    def __init__(self, robots, ball):
        self.canvas = scene.SceneCanvas(keys='interactive')
        self.canvas.size = 800, 400
        self.view = self.canvas.central_widget.add_view()
        self.view.bgcolor = black
        # self.view.camera = 'turntable'
        self.view.camera = scene.cameras.panzoom.PanZoomCamera(rect=(-5, -3.5, 10, 7))
        self.canvas.show()
        self.draw_field()
        self.init_actors(robots, ball)

    def draw_field(self):
        field = scene.Rectangle(center=(0, 0), width=10, height=7,
                                color=green, parent=self.view.scene)
        center_circle = scene.Ellipse(center=(0, 0), radius=1.3 / 2, color=green, border_color=white, border_width=2,
                                      parent=self.view.scene)
        center_line = scene.Line(pos=np.array([[0, 3.5], [0, -3.5]]), width=2, color=white, parent=self.view.scene)
        goals = scene.Line(pos=np.array([[-4.5, 1.3], [-4.5, -1.3], [4.5, 1.3], [4.5, -1.3]]), connect='segments',
                           width=2, color=white, parent=self.view.scene)

    def init_actors(self, robots, ball):
        self.ball = scene.Ellipse(center=(ball.position[0], ball.position[1]), radius=0.1,
                                  color=blue, parent=self.view.scene)
        self.robots = []
        for robot in robots:
            color = red if robot.team == RobotControlled2D.Team.OPPONENT else white
            self.robots.append({"body": scene.Ellipse(center=(ball.position[0], ball.position[1]), radius=0.1,
                                                      color=color,
                                                      parent=self.view.scene),
                                "arrow": scene.Arrow(pos=np.array([[0, 0], [0, 0]]), width=1, color=color,
                                                     parent=self.view.scene),
                                "path": scene.Arrow(pos=None, width=1, color=color, connect='strip',
                                                    parent=self.view.scene),
                                "vision_cone": scene.Arrow(pos=None, width=1, color=color, connect='strip',
                                                           parent=self.view.scene),
                                "id": scene.Text(text="id", parent=self.view.scene, pos=(0, 0))})

        self.field_vectors = scene.Arrow(pos=None, width=1, color=blue, connect='segments', arrows=None,
                                         arrow_color=blue, arrow_type='triangle_30', arrow_size=7,
                                         parent=self.view.scene)

    def update(self, robots, ball):
        for i in range(len(robots)):
            x = robots[i].position[0]
            y = robots[i].position[1]
            self.robots[i]['body'].center = (x, y)
            self.robots[i]['id'].pos = (x, y)
            self.robots[i]['id'].text = str(robots[i].robot_id)

            theta = robots[i].position[2]
            arrow_len = 0.3
            arrow_end_x = math.cos(theta) * arrow_len
            arrow_end_y = math.sin(theta) * arrow_len
            direction = np.array([arrow_end_x, arrow_end_y])
            direction = direction / np.linalg.norm(direction)
            self.robots[i]['arrow'].set_data(pos=np.array([[x, y], [x + arrow_end_x, y + arrow_end_y]]))

            robot_pos = np.array([x, y])
            theta = -RobotControlled2D.ObservationConstants.FOV / 2
            c, s = np.cos(theta), np.sin(theta)
            R = np.array(((c, -s), (s, c)))
            vision_cone_right = np.matmul(R, direction) * RobotControlled2D.ObservationConstants.VISION_RANGE
            theta = RobotControlled2D.ObservationConstants.FOV / 2
            c, s = np.cos(theta), np.sin(theta)
            R = np.array(((c, -s), (s, c)))
            vision_cone_left = np.matmul(R, direction) * RobotControlled2D.ObservationConstants.VISION_RANGE
            self.robots[i]['vision_cone'].set_data(
                pos=np.array([robot_pos + vision_cone_left, robot_pos, vision_cone_right + robot_pos]))

            if robots[i].path is not None:
                verts = []
                for j in range(0, 11):
                    path_vert = robots[i].path.poseAtRatio(j / 10).position
                    verts.append([path_vert[0], path_vert[1]])
                self.robots[i]['path'].set_data(pos=np.array(verts))

        if ball.position is not None:
            x = ball.position[0]
            y = ball.position[1]
            dx = ball.velocity[0]
            dy = ball.velocity[1]
            self.ball.center = (x, y)

        self.canvas.update()

    def plot_vectors(self, field_vectors):
        arrows = []
        pos = []
        for k in range(0, len(field_vectors)):
            if field_vectors[k] is not None:
                for i in range(0, len(field_vectors[k][0])):
                    x1 = field_vectors[k][0][i][0]
                    y1 = field_vectors[k][0][i][1]
                    x2 = x1 + field_vectors[k][1][i][0]
                    y2 = y1 + field_vectors[k][1][i][1]
                    arrows.append([x1, y1, x2, y2])
                    pos.append([x1, y1])
                    pos.append([x2, y2])
        self.field_vectors.set_data(pos=np.array(pos), arrows=np.array(arrows))

