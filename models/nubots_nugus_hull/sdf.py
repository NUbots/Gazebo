import os
import sys
import xml.etree.ElementTree as ET
import xml.dom.minidom as minidom

import yaml

from math import pi
from time import localtime, strftime


def prettify(element):
    """    Returns prettified string of sdf    """
    reparsed = minidom.parseString(ET.tostring(element, "utf-8"))
    return reparsed.toprettyxml(indent="  ")


def get_xyzrpy(d):
    """ Returns positional (x,y,z) and rotational (roll, pitch, yaw) component string """
    return "{} {} {} {} {} {}".format(
        d["x"], d["y"], d["z"], d["roll"], d["pitch"], d["yaw"]
    )


def get_xyz(d):
    """ Returns positional (x,y,z) component string """
    return "{} {} {}".format(d["x"], d["y"], d["z"])


def pose_sdf(parent, pose_data):
    """ Creates pose sdf element, as a child of the parent param """
    pose = ET.SubElement(parent, "pose", {"frame": ""})
    pose.text = " ".join(["{}".format(x) for x in pose_data])


def inertial_sdf(parent, link_data):
    """ Creates inertial sdf element consisting of pose, mass and inertia elements """
    inertial = ET.SubElement(parent, "inertial")
    ## Pose
    pose_sdf(inertial, link_data["pose"])
    ## Mass
    mass = ET.SubElement(inertial, "mass")
    mass.text = "{}".format(link_data["mass"])
    ## Inertia
    inertia = ET.SubElement(inertial, "inertia")
    for v in ["ixx", "ixy", "ixz", "iyy", "iyz", "izz"]:
        e = ET.SubElement(inertia, v)
        e.text = "{}".format(link_data["tensor"][v])


def geometry_sdf(parent, geometry_data):
    """ Creates geometry sdf element consisting of mesh element and relevant mesh information """
    geometry = ET.SubElement(parent, "geometry")
    # Mesh
    mesh = ET.SubElement(geometry, "mesh")
    # Scale
    scale = ET.SubElement(mesh, "scale")
    scale.text = " ".join(["{}".format(x) for x in geometry_data["scale"]])
    # URI
    uri = ET.SubElement(mesh, "uri")
    uri.text = geometry_data["uri"]


def collision_sdf(parent, collision_data):
    """ Creates collision sdf element consisting of pose, geometry, and surface elements """
    collision = ET.SubElement(parent, "collision", {"name": collision_data["name"]})
    ## Pose
    pose_sdf(collision, collision_data["pose"])
    ## Geometry
    geometry_sdf(collision, collision_data["geometry"])
    ## Surface
    surface = ET.SubElement(collision, "surface")
    contact = ET.SubElement(surface, "contact")
    contact_ode = ET.SubElement(contact, "ode")
    # Friction
    friction = ET.SubElement(surface, "friction")
    friction_ode = ET.SubElement(friction, "ode")
    for f in ["mu", "mu2", "slip1", "slip2"]:
        e = ET.SubElement(friction_ode, f)
        e.text = "{}".format(collision_data["friction"][f])
    # Torsional
    torsional = ET.SubElement(friction, "torsional")
    coeff = ET.SubElement(torsional, "coefficient")
    coeff.text = "{}".format(collision_data["friction"]["torsional"]["coefficient"])
    patch_radius = ET.SubElement(torsional, "patch_radius")
    patch_radius.text = "{}".format(
        collision_data["friction"]["torsional"]["patch_radius"]
    )


def visual_sdf(parent, visual_data):
    """ Creates visual sdf element consisting of pose, geometry and material elements """
    visual = ET.SubElement(parent, "visual", {"name": visual_data["name"]})
    ## Pose
    pose_sdf(visual, visual_data["pose"])
    ## Geometry
    geometry_sdf(visual, visual_data["geometry"])
    ## Material
    script = ET.SubElement(ET.SubElement(visual, "material"), "script")
    name = ET.SubElement(script, "name")
    name.text = visual_data["material"]["name"]
    uri = ET.SubElement(script, "uri")
    uri.text = visual_data["material"]["uri"]


def sensor_sdf(parent, sensor_data):
    """ Creates sensor sdf element for IMU data """
    sensor = ET.SubElement(
        parent, "sensor", {"name": sensor_data["name"], "type": "imu"}
    )
    ## Pose
    pose_sdf(sensor, sensor_data["pose"])
    ## Always sensing
    always_on = ET.SubElement(sensor, "always_on")
    always_on.text = "1"
    ## Visualise IMU
    visualise = ET.SubElement(sensor, "visualize")
    visualise.text = "1"
    ## Update rate
    update_rate = ET.SubElement(sensor, "update_rate")
    update_rate.text = "{}".format(sensor_data["update_rate"])
    ## imu
    imu = ET.SubElement(sensor, "imu")
    # Orientation reference frame
    orf = ET.SubElement(imu, "orientation_reference_frame")
    #   Localisation
    localisation = ET.SubElement(orf, "localization")
    localisation.text = "GRAV_UP"
    #   Gravity direction
    grav_dir = ET.SubElement(orf, "grav_dir_x")
    grav_dir.text = "1 0 0"
    # Gyroscope noise
    ang_vel = ET.SubElement(imu, "angular_velocity")
    ind = 0
    for ax in ["x", "y", "z"]:
        se = ET.SubElement(ang_vel, ax)
        noise = ET.SubElement(se, "noise", {"type": "gaussian"})
        mean = ET.SubElement(noise, "mean")
        mean.text = "{}".format(sensor_data["gyro_noise"]["mean"][ind])
        ind += 1
    # Accelerometer noise
    lin_acc = ET.SubElement(imu, "linear_acceleration")
    ind = 0
    for ax in ["x", "y", "z"]:
        se = ET.SubElement(lin_acc, ax)
        noise = ET.SubElement(se, "noise", {"type": "gaussian"})
        mean = ET.SubElement(noise, "mean")
        mean.text = "{}".format(sensor_data["accel_noise"]["mean"][ind])
        ind += 1


def link_sdf(parent, link_data):
    """ Creates link sdf element consisting of pose, inertial, collision and visual elements """
    parent.append(ET.Comment("LINK: {}".format(link_data["name"])))
    link = ET.SubElement(parent, "link", {"name": link_data["name"]})
    pose_sdf(link, link_data["pose"])
    inertial_sdf(link, link_data["inertial"])
    collision_sdf(link, link_data["collision"])
    visual_sdf(link, link_data["visual"])
    gravity = ET.SubElement(link, "gravity")
    gravity.text = "1"
    # self collision
    self_collide = ET.SubElement(link, "self_collide")
    if "sensor" in link_data:
        sensor_sdf(link, link_data["sensor"])


def joint_sdf(parent, joint_data):
    parent.append(ET.Comment("JOINT: {}".format(joint_data["name"])))
    joint = ET.SubElement(
        parent, "joint", {"name": joint_data["name"], "type": "revolute"}
    )
    ## Child
    child = ET.SubElement(joint, "child")
    child.text = joint_data["child"]
    ## Parent
    parent = ET.SubElement(joint, "parent")
    parent.text = joint_data["parent"]
    ## Axis
    axis = ET.SubElement(joint, "axis")
    xyz = ET.SubElement(axis, "xyz")
    xyz.text = " ".join(["{}".format(x) for x in joint_data["axis"]["xyz"]])
    ## Limit
    limit = ET.SubElement(axis, "limit")
    # Positional limits
    lower = ET.SubElement(limit, "lower")
    lower.text = "{}".format(-pi)
    upper = ET.SubElement(limit, "upper")
    upper.text = "{}".format(pi)
    # Velocity limits
    velocity = ET.SubElement(limit, "velocity")
    velocity.text = "{}".format(joint_data["axis"]["limit"]["velocity"])
    # Effort limits
    effort = ET.SubElement(limit, "effort")
    effort.text = "{}".format(joint_data["axis"]["limit"]["effort"])
    ## Dynamics
    dynamics = ET.SubElement(axis, "dynamics")
    # Friction
    friction = ET.SubElement(dynamics, "friction")
    friction.text = "{}".format(joint_data["axis"]["dynamics"]["friction"])
    # Damping
    damping = ET.SubElement(dynamics, "damping")
    damping.text = "{}".format(joint_data["axis"]["dynamics"]["damping"])
    spring_reference = ET.SubElement(dynamics, "spring_reference")
    spring_reference.text = "{}".format(
        joint_data["axis"]["dynamics"]["spring_reference"]
    )
    spring_stiffness = ET.SubElement(dynamics, "spring_stiffness")
    spring_stiffness.text = "{}".format(
        joint_data["axis"]["dynamics"]["spring_stiffness"]
    )

    ## Parent model frame
    parent_model_frame = ET.SubElement(axis, "use_parent_model_frame")
    parent_model_frame.text = "1"


def main():
    assert len(sys.argv) == 2, "[ERR] Usage: {} <output sdf path>".format(sys.argv[0])

    # Read NUgus configuration from yaml
    with open(
        os.path.join(os.path.dirname(os.path.realpath(__file__)), "NUgus.yaml"), "r"
    ) as df:
        cfg = yaml.load(df)

    # Define both types of servo
    mx64 = {
        "dynamics": cfg["joint"]["dynamics"],
        "limit": cfg["joint"]["mx64"]["limit"],
    }
    mx106 = {
        "dynamics": cfg["joint"]["dynamics"],
        "limit": cfg["joint"]["mx106"]["limit"],
    }

    # Define IMU sensor
    sensor = {
        "name": "imu_sensor",
        "pose": cfg["sensor"]["pose"],
        "update_rate": cfg["sensor"]["update_rate"],
        "gyro_noise": cfg["sensor"]["gyro_noise"],
        "accel_noise": cfg["sensor"]["accel_noise"],
    }

    # Construct sdf tree
    # sdf_tree = get_sdf(...)
    sdf_tree = ET.Element("sdf", {"version": "1.6"})
    sdf_tree.append(ET.Comment("NUgus SDF Model"))
    sdf_tree.append(
        ET.Comment(
            "Last updated: {}".format(strftime("%d %b %Y %H:%M:%S", localtime()))
        )
    )
    # Construct model with initial pose of 0
    model = ET.SubElement(sdf_tree, "model", {"name": "nubots_nugus_hull"})
    pose = pose_sdf(model, [0, 0, 0, 0, 0, 0])

    for link in cfg["links"]:
        name = list(link.keys())[0]
        d = link[name]
        link_data = {
            "name": name,
            "pose": d["link_pose"],
            "inertial": {
                "pose": d["inertial_pose"],
                "mass": d["mass"],
                "tensor": {
                    "ixx": d["inertia"][0][0],
                    "ixy": d["inertia"][0][1],
                    "ixz": d["inertia"][0][2],
                    "iyy": d["inertia"][1][1],
                    "iyz": d["inertia"][1][2],
                    "izz": d["inertia"][2][2],
                },
            },
            "collision": {
                "name": name + "_collision",
                "pose": d["display_pose"],
                "geometry": {"scale": [1, 1, 1], "uri": cfg["mesh_path"] + d["mesh"]},
                "friction": cfg["link"]["friction"],
            },
            "visual": {
                "name": name + "_visual",
                "pose": d["display_pose"],
                "geometry": {"scale": [1, 1, 1], "uri": cfg["mesh_path"] + d["mesh"]},
                "material": {"name": d["colour"], "uri": cfg["mat_path"]},
            },
        }

        if "sensor" in d:
            link_data.update({"sensor": sensor})

        link_sdf(model, link_data)

    for joint in cfg["joints"]:
        name = list(joint.keys())[0]
        d = joint[name]
        joint_data = {"name": name, "parent": d["parent"], "child": d["child"]}
        # Set joint type
        assert (
            d["type"] == "mx64" or d["type"] == "mx106"
        ), "[ERROR] Invalid joint type found ({})".format(d["type"])
        axis = mx64 if d["type"] == "mx64" else mx106
        axis.update({"xyz": d["axis"]})
        joint_data.update({"axis": axis})
        # Add joint to sdf
        joint_sdf(model, joint_data)

    # Set output path of the result sdf
    sdf_path = sys.argv[1]

    # Write sdf to file
    with open(sdf_path, "w") as df:
        df.write(prettify(sdf_tree))

    print("[INFO] sdf written to {}".format(sdf_path))


if __name__ == "__main__":
    main()
