import mujoco
import mujoco_viewer
import numpy as np

model = mujoco.MjModel.from_xml_path('models/snake_render.xml')
data = mujoco.MjData(model)

# create the viewer object
viewer = mujoco_viewer.MujocoViewer(model, data)

# Apply torque to the motors
num_actuators = model.nu
torque = np.array([1.0, 1.0, 1.0, 1.0])
data.ctrl[:num_actuators] = torque

switched = False
switched = True
# simulate and render
for i in range(10000):
    if viewer.is_alive:
        if i%200 == 0:
            if switched:
                torque = np.array([0.1, 0.1, 0.1, 0.1])
                switched = False
            else:
                torque = np.array([-0.1, -0.1, -0.1, -0.1])
                switched = True
        data.ctrl[:num_actuators] = torque
        mujoco.mj_step(model, data)
        viewer.render()
    else:
        break

# close
viewer.close()