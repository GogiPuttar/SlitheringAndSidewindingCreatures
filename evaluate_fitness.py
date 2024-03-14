import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from generate_random_bitstring import generate_random_bitstring
from generate_snake_xml import generate_snake_xml

# Function to evaluate fitness (dummy function for illustration)
def evaluate_fitness(creature):

    # Hyperparameters
    timedur = int(1e4)

    model_path = 'models/snake_temporary.xml'
    generate_snake_xml(creature["body_plan"], model_path, creature["motor_gain"], creature["joint_range"])
    body_length = creature["body_plan"].size
    
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    # Apply torque to the motors
    num_actuators = model.nu
    torque = np.ones(body_length)
    data.ctrl[:num_actuators] = torque

    switched = False
    switched = True
    # simulate and render
    roll_penalty = 0
    pitch_penalty = 0
    penalty_weight = 5e-5 / float(timedur)
    for i in range(timedur):

        if i%(int(1.0/creature["frequency"])) == 0:

            if switched:
                torque = np.ones(body_length)*0.1
                switched = False

            else:
                torque = np.ones(body_length)*-0.1
                switched = True
        data.ctrl[:num_actuators] = torque
        mujoco.mj_step(model, data)

        quat = data.qpos[3:7]
        roll, pitch, yaw = quat2eul(quat)

        roll_penalty += max(0, np.abs(np.degrees(roll)) - 90)
        pitch_penalty += max(0, np.abs(np.degrees(pitch)) - 90)

    # print((roll_penalty + pitch_penalty) * penalty_weight)


    # fitness function = square of displacement
    fitness = max(0, (data.qpos[0]**2 + data.qpos[1]**2)**0.5 - (roll_penalty + pitch_penalty) * penalty_weight)
    # print(fitness)

    return fitness

def quat2eul(q):
    # Convert quaternion to Euler angles (roll, pitch, yaw)
    r = Rotation.from_quat(q)
    euler = r.as_euler('xyz', degrees=True)  # 'xyz' specifies the order of rotation axes
    return euler