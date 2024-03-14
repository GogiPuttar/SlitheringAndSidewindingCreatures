import mujoco
import mujoco_viewer
import numpy as np
import random

from generate_snake_xml import generate_snake_xml

def render_creature(creature):

    # Hyperparameters
    body_length = random.randint(4, 10)

    model_path = 'models/snake_render.xml'
    generate_snake_xml(creature["body_plan"], model_path, creature["motor_gain"], creature["joint_range"])
    body_length = creature["body_plan"].size

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
            if i%(int(1.0/creature["frequency"])) == 0:
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
            # print(data.qpos[0:2], data.qpos[3:7])
            viewer.render()
        else:
            break

    # close
    viewer.close()

    return True