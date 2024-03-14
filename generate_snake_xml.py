def generate_snake_xml(bitstring, file_path, gain, angle_range):
    
    headers = """
    <mujoco model="Snake">
    <option timestep="0.01"/>

    <asset>
        <texture type="skybox" builtin="gradient" rgb1=".3 .5 .7" rgb2="0 0 0" width="512" height="512"/>
        <texture name="body" type="cube" builtin="flat" mark="cross" width="127" height="1278"
                 rgb1="0.8 0.6 0.4" rgb2="0.8 0.6 0.4" markrgb="1 1 1" random="0.01"/>
        <material name="body" texture="body" texuniform="true" rgba="0.8 0.6 .4 1"/>
        <texture name="grid" type="2d" builtin="checker" width="512" height="512" rgb1=".1 .2 .3" rgb2=".2 .3 .4"/>
        <material name="grid" texture="grid" texrepeat="1 1" texuniform="true" reflectance=".2"/>
    </asset>

    <default>
        <motor ctrlrange="-1 1" ctrllimited="true"/>
        <default class="body">
            <geom type="capsule" condim="6" friction=".7" solimp=".9 .99 .003" solref=".015 1" material="body"/>
            <joint type="hinge" damping=".2" stiffness="1" armature=".01" limited="true" solimplimit="0 .99 .01"/>
          </default>
    </default>

    <visual>
        <map force="0.1" zfar="30"/>
        <rgba haze="0.9 0.9 0.15 1"/>
        <quality shadowsize="4096"/>
        <global offwidth="800" offheight="800"/>
    </visual>
    """

    worldbody_start = """
    <worldbody>
        <geom name="floor" size="0 0 .05" type="plane" material="grid" condim="3"/>
        <light name="spotlight" mode="targetbodycom" target="head"
            diffuse=".8 .8 .8" specular="0.3 0.3 0.3" pos="0 -20 4" cutoff="10"/>

        <body name="head" pos="0 0 0.5" childclass="body">
            <light name="top" pos="0 0 2" mode="trackcom"/>
            <camera name="back" pos="-3 0 1" xyaxes="0 -1 0 1 0 2" mode="trackcom"/>
            <camera name="side" pos="0 -3 1" xyaxes="1 0 0 0 1 2" mode="trackcom"/>
            <freejoint name="root"/>
            <geom name="head" fromto="0 0 0 0.1 0 0" size=".06"/>
    """
    worldbody_segments = ""

    worldbody_end = """
    </body>
    """

    actuator_start = """
    <actuator>
    """

    motors = ""

    actuator_end = """
    </actuator>
    """

    end = """
    </mujoco>
    """

    # Generate body and actuators model
    for index, element in enumerate(bitstring):

        if element == 0:

            worldbody_segments += f"""
            <body name="segment_{index}" pos="0.1 0 0 ">
                <geom name="segment_{index}" fromto="0 0 0 0.1 0 0" size=".05"/>
                <joint name="curl_{index}" pos="0 0 0" axis="0 1 0" range="-{angle_range} {angle_range}"/>
            """

            motors += f"""
            <motor name="curl_{index}"  gear="{gain}"  joint="curl_{index}"/>
            """

        elif element == 1:

            worldbody_segments += f"""
            <body name="segment_{index}" pos="0.1 0 0 ">
                <geom name="segment_{index}" fromto="0 0 0 0.1 0 0" size=".05"/>
                <joint name="twist_{index}" pos="0 0 0" axis="1 0 0" range="-10 10"/>
            """

            motors += f"""
            <motor name="twist_{index}"  gear="{3*gain}"  joint="twist_{index}"/>
            """
        worldbody_end += """
        </body>
        """

    worldbody_end += """
    </worldbody>
    """

    worldbody = worldbody_start + worldbody_segments + worldbody_end
    actuator = actuator_start + motors + actuator_end
    xml_string = headers + worldbody + actuator + end

    try:
        with open(file_path, "w") as xml_file:
            xml_file.write(xml_string)
        # print(f"XML file saved successfully at {file_path}")
    except Exception as e:
        print(f"Error occurred while saving XML file: {e}")