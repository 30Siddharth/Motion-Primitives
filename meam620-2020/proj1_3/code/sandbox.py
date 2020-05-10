import inspect
import json
import matplotlib as mpl
from matplotlib.lines import Line2D
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation
import time

from flightsim.animate import animate
from flightsim.axes3ds import Axes3Ds
from flightsim.crazyflie_params import quad_params
from flightsim.simulate import Quadrotor, simulate, ExitStatus
from flightsim.world import World

from proj1_3.code.occupancy_map import OccupancyMap
from proj1_3.code.se3_control import SE3Control
from proj1_3.code.world_traj import WorldTraj

# Improve figure display on high DPI screens.
# mpl.rcParams['figure.dpi'] = 200

# Choose a test example file. You should write your own example files too!
filename = '../util/test_maze.json'
# Videoname = 'MyMap.mp4'

# Load the test example.
file = Path(inspect.getsourcefile(lambda:0)).parent.resolve() / '..' / 'util' / filename
world = World.from_file(file)          # World boundary and obstacles.
start  = world.world['start']          # Start point, shape=(3,)
goal   = world.world['goal']           # Goal point, shape=(3,)
# This object defines the quadrotor dynamical model and should not be changed.
robot_radius = 0.25

#TODO: Check this if reqd
initial_state = {'x': start,
				 'v': (0, 0, 0),
				 'q': (0, 0, 0, 1), # [i,j,k,w]
				 'w': (0, 0, 0)}



