from soccer_strategy.src.strategy.strategy import Strategy


class StationaryStrategy(Strategy):
    def update_next_strategy(self, friendly, opponent, ball):
        return