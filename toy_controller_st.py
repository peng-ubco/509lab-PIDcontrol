from sim_toy import run_sim

# Simulator scenario settings.
sim_opt = {}
sim_opt['FIG_SIZE'] = [8, 8]


class Controller:
    def __init__(self):
        self.dt = 0.2
        # stopping pose [x_pos, y_pos, theta]
        self.stop_pose = [10, 0, 0]  # then, try to set as [10, 2, 1.5]

    def control(self, state):
        # Assume we know current state
        # Get the current state of the car
        pos_x = state[0]  # X Location [m]
        pos_y = state[1]  # Y Location [m]
        psi = state[2]  # Angle [rad]
        v = state[3]  # Speed [m/s]

        # Try: how the pedal and steering works by setting different values
        pedal = 1.  # Max: 1, Min: -1
        steering = 0.  # Max; 0.8, Min: -0.8

        '''your code starts here'''
        '''***********************************************'''

        '''***********************************************'''

        return [pedal, steering]

run_sim(sim_opt, Controller)
