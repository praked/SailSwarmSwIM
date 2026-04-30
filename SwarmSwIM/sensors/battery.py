import numpy as np

class Battery:

    def __init__(self, initial_charge=100.0, consumption_rate=1.0, agent=None, turn_consumption_rate=0.01):
        self.charge = initial_charge
        self.consumption_rate = consumption_rate
        self.agent = agent
        self.turn_consumption_rate = turn_consumption_rate
        self.last_heading = agent.psi if agent is not None else None
        self.last_turn_deg = 0.0

    @staticmethod
    def _signed_heading_delta_deg(current, previous):
        return ((current - previous + 180.0) % 360.0) - 180.0

    def update(self, time_step):
        # Simulate battery consumption
        self.charge -= self.consumption_rate * time_step
        if self.agent is not None:
            current_heading = float(self.agent.psi)
            if self.last_heading is None:
                turn_deg = 0.0
            else:
                turn_deg = self._signed_heading_delta_deg(current_heading, self.last_heading)
            self.last_turn_deg = turn_deg
            self.charge -= self.turn_consumption_rate * abs(turn_deg)
            self.last_heading = current_heading
        self.charge = max(self.charge, 0)  # Ensure charge doesn't go negative

    def get_charge(self):
        return self.charge