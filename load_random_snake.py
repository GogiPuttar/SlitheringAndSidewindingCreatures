import mujoco
import mujoco_viewer
import numpy as np
import random

from generate_random_bitstring import generate_random_bitstring
from generate_snake_xml import generate_snake_xml

# Hyperparameters
body_length = random.randint(4, 10)
spin_prob = 0.3
gain = 40
angle_range = 45
frequency = np.random.uniform(0.005, 0.02)

model_path = 'models/snake_random.xml'
body_plan = generate_random_bitstring(body_length, spin_prob)
generate_snake_xml(body_plan, model_path, gain, angle_range)
body_length = body_plan.size
# body_length = 4

model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

# create the viewer object
viewer = mujoco_viewer.MujocoViewer(model, data)

# Apply torque to the motors
num_actuators = model.nu
# torque = np.array([1.0, 1.0, 1.0, 1.0])
torque = np.ones(body_length)
data.ctrl[:num_actuators] = torque

switched = False
switched = True
# simulate and render
for i in range(10000):
    if viewer.is_alive:
        if i%(int(1.0/frequency)) == 0:
            if switched:
                # torque = np.array([0.1, 0.1, 0.1, 0.1])
                torque = np.ones(body_length)*0.1
                switched = False
            else:
                # torque = np.array([-0.1, -0.1, -0.1, -0.1])
                torque = np.ones(body_length)*-0.1
                switched = True
        data.ctrl[:num_actuators] = torque
        mujoco.mj_step(model, data)
        # print(data.qpos[data.qpos.size - 4: data.qpos.size])
        print(data.qpos[0: 2])
        viewer.render()
    else:
        break

# close
viewer.close()