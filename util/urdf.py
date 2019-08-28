import os
import sys
import xml.etree.ElementTree as ET
import xml.dom.minidom as minidom

import yaml
import numpy as np

from math import pi
from time import localtime, strftime
from transformations import *

STL_PATH = "mesh/"


def prettify(element):
    """    Returns prettified string of urdf    """
    reparsed = minidom.parseString(ET.tostring(element, "utf-8"))
    return reparsed.toprettyxml(indent="  ")


class URDF(object):
    """docstring for URDF"""

    def __init__(self, cfg):
        super(URDF, self).__init__()
        self.cfg = cfg

        # Construct urdf tree
        self.urdf_tree = ET.Element("robot", {"name": "nubots_nugus_hull"})
        self.urdf_tree.append(ET.Comment("NUgus URDF Model"))
        self.urdf_tree.append(
            ET.Comment(
                "Last updated: {}".format(strftime("%d %b %Y %H:%M:%S", localtime()))
            )
        )

        # Add links to urdf
        self.links_urdf(self.urdf_tree, cfg)
        # Add joints to urdf
        self.joints_urdf(self.urdf_tree, cfg)

    def pose2origin(parent, pose):
        xyz, rpy = homogeneous2translation_rpy(pose)

    # def calculate_absolute_pose(
    #     self, pose, worldMVparent=numpy.identity(4, dtype=numpy.float64)
    # ):
    #     worldMVmodel = tf.concatenate_matrices(worldMVparent, pose)
    #     pose_world = worldMVmodel

    # for link in cfg.links:

    def origin_urdf(self, parent, origin_data):
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

    def inertial_urdf(self, parent, link_data):
        """ Creates inertial urdf element consisting of pose, mass and inertia elements """
        inertial = ET.SubElement(parent, "inertial")
        ## Pose
        self.origin_urdf(inertial, link_data["pose"])
        # self.origin_urdf(inertial, [0, 0, 0, 0, 0, 0])
        ## Mass
        mass = ET.SubElement(
            inertial, "mass", {"value": "{}".format(link_data["mass"])}
        )
        ## Inertia
        inertia_dict = {}
        for v in ["ixx", "ixy", "ixz", "iyy", "iyz", "izz"]:
            inertia_dict.update({v: "{}".format(link_data["tensor"][v])})
        inertia = ET.SubElement(inertial, "inertia", inertia_dict)

    def geometry_urdf(self, parent, geometry_data):
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

    def visual_urdf(self, parent, visual_data):
        visual = ET.SubElement(parent, "visual", {"name": visual_data["name"]})
        # self.origin_urdf(visual, [0, 0, 0, 0, 0, 0])
        self.origin_urdf(visual, visual_data["pose"])
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

    def link_urdf(self, parent, link_data):
        """ Creates link urdf element consisting of pose, inertial, collision and visual elements """
        parent.append(ET.Comment("LINK: {}".format(link_data["name"])))
        link = ET.SubElement(parent, "link", {"name": link_data["name"]})

        add_lists = lambda x, y: [x[v] + y[v] for v in range(len(x))]

        # rel_inert = add_lists(link_data["inertial"]["pose"], link_data["pose"])
        # rel_inert = add_lists(link_data["inertial"]["pose"], link_data["pose"])

        # link_data["inertial"].update({"pose": rel_inert})
        self.inertial_urdf(link, link_data["inertial"])
        self.visual_urdf(link, link_data["visual"])

    def links_urdf(self, parent, cfg):
        for link in cfg["links"]:
            name = list(link.keys())[0]
            d = link[name]
            link_data = {
                "name": name,
                "pose": d["link_pose"],
                "inertial": {
                    "pose": [0, 0, 0, 0, 0, 0],
                    # "pose": d["inertial_pose"],
                    # "pose": d["link_pose"],
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
                    "pose": d["link_pose"],
                    "geometry": {"scale": [1, 1, 1], "uri": STL_PATH + d["mesh"]},
                    "friction": cfg["link"]["friction"],
                },
                "visual": {
                    "name": name + "_visual",
                    "pose": d["link_pose"],
                    # "pose": [
                    #     0,
                    #     0,
                    #     0,
                    #     d["link_pose"][3],
                    #     d["link_pose"][4],
                    #     d["link_pose"][5],
                    # ],
                    "geometry": {"scale": [1, 1, 1], "uri": STL_PATH + d["mesh"]},
                    "material": {"name": d["colour"], "uri": cfg["mat_path"]},
                },
            }

            self.link_urdf(parent, link_data)

    def get_link(self, name):
        return [
            self.cfg["links"][x][name]
            for x in range(len(self.cfg["links"]))
            if name in self.cfg["links"][x]
        ][0]

    def get_parent_link(self, link_name):
        # Find connected joint to link to parent
        for j in self.cfg["joints"]:
            name = list(j.keys())[0]
            joint_child = j[name]["child"]
            # If joint is connected to our current link, return the parent
            if joint_child == link_name:
                return j[name]["parent"], self.get_link(j[name]["parent"])
        return None, None

    def get_transform(self, pose):
        return pose_string2homogeneous(
            "{} {} {}  {} {} {}".format(
                pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]
            )
        )

    def get_relative_transform(self, link_name, pose):
        # Get parent link to calculate transform
        parent_name, parent_link = self.get_parent_link(link_name)
        if not parent_link:
            return pose
        # return get_relative_transform(parent_transform()
        parent_transform = self.get_transform(parent_link["link_pose"])
        # print(link_name, "->", parent_name)
        # Get next joint
        return self.get_relative_transform(
            parent_name, concatenate_matrices(pose, parent_transform)
        )

    def joint_urdf(self, parent, joint_data):
        parent.append(ET.Comment("JOINT: {}".format(joint_data["name"])))
        joint = ET.SubElement(
            parent, "joint", {"name": joint_data["name"], "type": "revolute"}
        )

        # Get child links recursively
        parent_link = self.get_link(joint_data["parent"])
        pose_transform = self.get_transform(parent_link["link_pose"])

        relative_transform = self.get_relative_transform(
            joint_data["child"], pose_transform
        )
        # final_transform = concatenate_matrices(
        #     inverse_matrix(relative_transform), identity_matrix()
        # )

        xyz = translation_from_matrix(pose_transform)
        rpy = euler_from_matrix(pose_transform)
        xyzrpy = np.concatenate((xyz[:], rpy[:]))
        # print(list(xyz), list(rpy), list(xyzrpy))

        # self.origin_urdf(joint, [0, 0, 0, 0, 0, 0])
        # self.origin_urdf(joint, xyzrpy)
        # child_link = self.get_link(joint_data["child"])
        # self.origin_urdf(joint, child_link["link_pose"])

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

    def joints_urdf(self, parent, cfg):
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
            self.joint_urdf(parent, joint_data)


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

    urdf = URDF(cfg)

    # Set output path of the result urdf
    urdf_path = sys.argv[1]

    # Write urdf to file
    with open(urdf_path, "w") as df:
        df.write(prettify(urdf.urdf_tree))

    print("[INFO] urdf written to {}".format(urdf_path))


if __name__ == "__main__":
    main()
