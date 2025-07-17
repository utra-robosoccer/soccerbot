import socket

from construct import ConstError, Container

from game_controller_interface.gamestate import (
    GAME_CONTROLLER_RESPONSE_VERSION,
    GameState,
    ReturnData,
)


class GameControllerClient:
    def __init__(self, team_id=12, robot_id=4):
        self.team_id = team_id
        self.robot_id = robot_id
        self.recv_port = 3838
        self.send_port = 3939

        self.recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.recv_sock.settimeout(1)
        self.recv_sock.bind(("0.0.0.0", self.recv_port))

        self.send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def receive_once(self):
        data, addr = self.recv_sock.recvfrom(GameState.sizeof())
        state = GameState.parse(data)
        self.answer(addr)
        return state

    def answer(self, addr):
        response = Container(
            header=b"RGrt",
            version=GAME_CONTROLLER_RESPONSE_VERSION,
            team=self.team_id,
            player=self.robot_id,
            message=2,
        )
        self.send_sock.sendto(ReturnData.build(response), (addr[0], self.send_port))
        print(f"Replied to GameController at {addr[0]}")


def main():
    gc = GameControllerClient(team_id=10, robot_id=1)
    while True:
        try:
            state = gc.receive_once()
            print("received gamestate:", state)
        except Exception as ex:
            print("error", ex)


if __name__ == "__main__":
    main()
