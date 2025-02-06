import numpy as np
import matplotlib.pyplot as plt

class GaitScheduler:
    def __init__(self, gait_cycle=1.0, horizon=16, start_time=0, gait='stand'):
        self.gait_cycle = gait_cycle    # in sec
        self.duty_factor = 0.8          # portion of cycle spent in stance
        self.t_stance = self.duty_factor * self.gait_cycle
        self.t_swing = self.gait_cycle - self.t_stance

        print(f"gait cycle {self.gait_cycle}")
        print(f"swing time {self.t_swing}")
        print(f"stance time {self.t_stance}")

        self.t_start = start_time
        self.horizon = horizon

        self.current_gait = 'trot'
        self.phase_offset = {
                   #  FL   FR   RL   RR
            'trot' : [0.0, 0.5, 0.5, 0.0],     
            'stand': [0.0, 0.0, 0.0, 0.0]     
        }

        self.current_phase = 0.0    # current_phase /in [0,1)
        self.phase_map = [0.0, 0.0, 0.0, 0.0]   # FL, FR, RL, RR
        self.contact_schedule = np.zeros((4, horizon))

    def update_phase(self, t):
        dt = (t - self.t_start).nanoseconds / 1e9
        self.current_phase = (dt % self.gait_cycle) / self.gait_cycle

        for leg in range(4):
            stance_start = self.phase_offset[self.current_gait][leg]
            # self.phase_map[leg] = (self.current_phase + stance_start) % 1.0
            self.phase_map[leg] = 2 if leg == 0 else 0
        
        self.get_contact_schedule()

        # print(f"0 {self.get_leg_state(0)}, 1 {self.get_leg_state(1)}, 2 {self.get_leg_state(2)}, 3 {self.get_leg_state(3)}")
        # print(self.contact_schedule)

    def get_leg_state(self, leg_idx):
        return "stance" if self.phase_map[leg_idx] < self.duty_factor else "swing"

    def get_contact_schedule(self):
    
        for timestep in range(self.horizon):
            future_phase = (self.current_phase + timestep/self.horizon) % 1.0

            for leg in range(4):
                stance_start = self.phase_offset[self.current_gait][leg]
                leg_phase = (future_phase - stance_start) % 1.0
                # self.contact_schedule[leg, timestep] = 1.0 if leg_phase < self.duty_factor else 0.0
                self.contact_schedule[leg, timestep] = 1.0 if leg !=0 else 0.0
