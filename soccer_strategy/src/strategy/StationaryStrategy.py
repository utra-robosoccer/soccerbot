from soccer_strategy.src.strategy.Strategy import Strategy


class StationaryStrategy(Strategy):
    def update_next_strategy(self, friendly, opponent, ball):
        return