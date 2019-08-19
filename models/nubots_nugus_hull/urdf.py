import os
import sys
import xml.etree.ElementTree as ET
import xml.dom.minidom as minidom

import yaml

from math import pi
from time import localtime, strftime

STL_PATH = "mesh/"


def prettify(element):
    """    Returns prettified string of urdf    """
    reparsed = minidom.parseString(ET.tostring(element, "utf-8"))
    return reparsed.toprettyxml(indent="  ")


def origin_urdf(parent, origin_data):
    xyz = origin_data[:3]
    rpy = [0 for x in range(3)] if len(origin_data) == 3 else origin_data[3:]

    """ Creates pose urdf element, as a child of the parent param """
    origin = ET.SubElement(
        parent,
        "origin",
        {
            "xyz": " ".join(["{}".format(x) for x in xyz]),
            "rpy": " ".join(["{}".format(x) for x in rpy]),
        },
    )


def inertial_urdf(parent, link_data):
    """ Creates inertial urdf element consisting of pose, mass and inertia elements """
    inertial = ET.SubElement(parent, "inertial")
    ## Pose
    origin_urdf(inertial, link_data["pose"])
    ## Mass
    mass = ET.SubElement(inertial, "mass", {"value": "{}".format(link_data["mass"])})
    ## Inertia
    inertia_dict = {}
    for v in ["ixx", "ixy", "ixz", "iyy", "iyz", "izz"]:
        inertia_dict.update({v: "{}".format(link_data["tensor"][v])})
    inertia = ET.SubElement(inertial, "inertia", inertia_dict)


def geometry_urdf(parent, geometry_data):
    """ Creates geometry urdf element consisting of mesh element and relevant mesh information """
    geometry = ET.SubElement(parent, "geometry")
    # Mesh
    mesh = ET.SubElement(
        geometry,
        "mesh",
        {
            "filename": geometry_data["uri"],
            "scale": " ".join(["{}".format(x) for x in geometry_data["scale"]]),
        },
    )


def visual_urdf(parent, visual_data):
    visual = ET.SubElement(parent, "visual", {"name": visual_data["name"]})
    origin_urdf(visual, visual_data["pose"])
    ET.SubElement(
        ET.SubElement(visual, "geometry"),
        "mesh",
        {
            "filename": visual_data["geometry"]["uri"],
            "scale": " ".join(
                ["{}".format(x) for x in visual_data["geometry"]["scale"]]
            ),
        },
    )


def link_urdf(parent, link_data):
    """ Creates link urdf element consisting of pose, inertial, collision and visual elements """
    parent.append(ET.Comment("LINK: {}".format(link_data["name"])))
    link = ET.SubElement(parent, "link", {"name": link_data["name"]})

    add_lists = lambda x, y: [x[v] + y[v] for v in range(len(x))]

    # rel_inert = add_lists(link_data["inertial"]["pose"], link_data["pose"])
    # rel_inert = add_lists(link_data["inertial"]["pose"], link_data["pose"])

    # link_data["inertial"].update({"pose": rel_inert})
    inertial_urdf(link, link_data["inertial"])
    visual_urdf(link, link_data["visual"])


def links_urdf(parent, cfg):
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
                "pose": [0, 0, 0, 0, 0, 0],
                "geometry": {"scale": [1, 1, 1], "uri": STL_PATH + d["mesh"]},
                "friction": cfg["link"]["friction"],
            },
            "visual": {
                "name": name + "_visual",
                "pose": d["link_pose"],
                "geometry": {"scale": [1, 1, 1], "uri": STL_PATH + d["mesh"]},
                "material": {"name": d["colour"], "uri": cfg["mat_path"]},
            },
        }

        link_urdf(parent, link_data)


def joint_urdf(parent, joint_data):
    parent.append(ET.Comment("JOINT: {}".format(joint_data["name"])))
    joint = ET.SubElement(
        parent, "joint", {"name": joint_data["name"], "type": "revolute"}
    )

    # relative_vector = [
    #     joint_data["origins"][1][x] - joint_data["origins"][0][x] for x in range(3)
    # ]

    origin_urdf(joint, [0, 0, 0, 0, 0, 0])

    ## Child/ Parent
    for v in ["child", "parent"]:
        ET.SubElement(joint, v, {"link": joint_data[v]})
    ## Axis

    urdf_axis = [
        joint_data["axis"]["xyz"][0],
        joint_data["axis"]["xyz"][1],
        joint_data["axis"]["xyz"][2],
    ]
    axis = ET.SubElement(
        joint, "axis", {"xyz": " ".join(["{}".format(x) for x in urdf_axis])}
    )

    ## Dynamics
    dynamics = ET.SubElement(
        axis,
        "dynamics",
        {
            "damping": "{}".format(joint_data["axis"]["dynamics"]["damping"]),
            "friction": "{}".format(joint_data["axis"]["dynamics"]["friction"]),
        },
    )

    ## Limit
    limit = ET.SubElement(
        axis,
        "limit",
        {
            "lower": "{}".format(-pi),
            "upper": "{}".format(pi),
            "effort": "{}".format(joint_data["axis"]["limit"]["effort"]),
            "velocity": "{}".format(joint_data["axis"]["limit"]["velocity"]),
        },
    )


def joints_urdf(parent, cfg):
    # Define both types of servo
    mx64 = {
        "dynamics": cfg["joint"]["dynamics"],
        "limit": cfg["joint"]["mx64"]["limit"],
    }
    mx106 = {
        "dynamics": cfg["joint"]["dynamics"],
        "limit": cfg["joint"]["mx106"]["limit"],
    }

    for joint in cfg["joints"]:
        name = list(joint.keys())[0]
        d = joint[name]

        origins = []
        for v in ["parent", "child"]:
            origins.append(
                [x for x in cfg["links"] if list(x.keys())[0] == d[v]][0][d[v]][
                    "link_pose"
                ]
            )

        joint_data = {
            "name": name,
            "parent": d["parent"],
            "child": d["child"],
            "origins": origins,
        }
        # Set joint type
        assert (
            d["type"] == "mx64" or d["type"] == "mx106"
        ), "[ERROR] Invalid joint type found ({})".format(d["type"])
        axis = mx64 if d["type"] == "mx64" else mx106
        axis.update({"xyz": d["axis"]})
        joint_data.update({"axis": axis})
        # Add joint to urdf
        joint_urdf(parent, joint_data)


def main():
    assert len(sys.argv) == 2, "[ERR] Usage: {} <output urdf path>".format(sys.argv[0])

    # Read NUgus configuration from yaml
    with open(
        os.path.join(os.path.dirname(os.path.realpath(__file__)), "NUgus.yaml"), "r"
    ) as df:
        # Specify loader type for python 2 to avoid warning
        if sys.version_info[0] < 3:
            cfg = yaml.load(df, Loader=yaml.FullLoader)
        else:
            cfg = yaml.load(df)

    # Construct urdf tree
    urdf_tree = ET.Element("robot", {"name": "nubots_nugus_hull"})
    urdf_tree.append(ET.Comment("NUgus URDF Model"))
    urdf_tree.append(
        ET.Comment(
            "Last updated: {}".format(strftime("%d %b %Y %H:%M:%S", localtime()))
        )
    )

    # Add links to urdf
    links_urdf(urdf_tree, cfg)
    # Add joints to urdf
    joints_urdf(urdf_tree, cfg)

    # Set output path of the result urdf
    urdf_path = sys.argv[1]

    # Write urdf to file
    with open(urdf_path, "w") as df:
        df.write(prettify(urdf_tree))

    print("[INFO] urdf written to {}".format(urdf_path))


if __name__ == "__main__":
    main()
