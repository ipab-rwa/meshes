#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
stl2ros_pkg.py — Génère un package ROS minimal à partir d'un STL.
Le package contiendra :
- package.xml et CMakeLists.txt
- dossier meshes/ avec le STL
- dossier urdf/ avec un URDF valide (1 link, visual+collision)
- dossier config/ avec un SRDF minimal
"""

import argparse
import os
import shutil
import sys
import xml.etree.ElementTree as ET

def make_urdf(robot_name: str,
              mesh_rel_path: str,
              link_name: str = "base_link",
              mesh_origin=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
              mesh_scale=(1.0, 1.0, 1.0),
              mass=1.0):
    inertia = (1e-3, 0.0, 0.0, 1e-3, 0.0, 1e-3)
    com = (0.0, 0.0, 0.0)

    robot = ET.Element("robot", name=robot_name)
    link = ET.SubElement(robot, "link", name=link_name)

    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "origin", xyz=f"{com[0]} {com[1]} {com[2]}", rpy="0 0 0")
    ET.SubElement(inertial, "mass", value=f"{mass:.9g}")
    ixx, ixy, ixz, iyy, iyz, izz = inertia
    ET.SubElement(inertial, "inertia",
                  ixx=f"{ixx:.9g}", ixy=f"{ixy:.9g}", ixz=f"{ixz:.9g}",
                  iyy=f"{iyy:.9g}", iyz=f"{iyz:.9g}", izz=f"{izz:.9g}")

    visual = ET.SubElement(link, "visual")
    ET.SubElement(visual, "origin",
                  xyz=f"{mesh_origin[0]} {mesh_origin[1]} {mesh_origin[2]}",
                  rpy=f"{mesh_origin[3]} {mesh_origin[4]} {mesh_origin[5]}")
    geom_v = ET.SubElement(visual, "geometry")
    ET.SubElement(geom_v, "mesh", filename=f"package://{robot_name}/{mesh_rel_path}",
                  scale=f"{mesh_scale[0]} {mesh_scale[1]} {mesh_scale[2]}")
    material = ET.SubElement(visual, "material", name="grey")
    ET.SubElement(material, "color", rgba="0.7 0.7 0.7 1.0")

    collision = ET.SubElement(link, "collision")
    ET.SubElement(collision, "origin",
                  xyz=f"{mesh_origin[0]} {mesh_origin[1]} {mesh_origin[2]}",
                  rpy=f"{mesh_origin[3]} {mesh_origin[4]} {mesh_origin[5]}")
    geom_c = ET.SubElement(collision, "geometry")
    ET.SubElement(geom_c, "mesh", filename=f"package://{robot_name}/{mesh_rel_path}",
                  scale=f"{mesh_scale[0]} {mesh_scale[1]} {mesh_scale[2]}")

    tree = ET.ElementTree(robot)
    ET.indent(tree, space="  ", level=0)
    return tree

def make_srdf(robot_name: str):
    return f"""<?xml version="1.0" ?>
<robot name="{robot_name}">
  <!-- SRDF minimal, aucun groupe ou collision désactivée -->
</robot>
"""

def make_package_xml(robot_name: str):
    return f"""<?xml version="1.0"?>
<package format="2">
  <name>{robot_name}</name>
  <version>0.0.1</version>
  <description>Package auto-généré pour {robot_name}</description>
  <maintainer email="user@example.com">Auto Generator</maintainer>
  <license>BSD</license>
  <buildtool_depend>catkin</buildtool_depend>
  <depend>roscpp</depend>
  <depend>urdf</depend>
  <depend>srdfdom</depend>
</package>
"""

def make_cmakelists(robot_name: str):
    return f"""cmake_minimum_required(VERSION 3.0.2)
project({robot_name})

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY urdf meshes config
  DESTINATION ${{CATKIN_PACKAGE_SHARE_DESTINATION}}
)
"""

def main():
    parser = argparse.ArgumentParser(description="Génère un package ROS minimal à partir d'un STL.")
    parser.add_argument("stl", help="Chemin vers le fichier STL.")
    parser.add_argument("--robot-name", default="my_robot", help="Nom du robot / package.")
    parser.add_argument("--link-name", default="base_link", help="Nom du link.")
    parser.add_argument("--output-dir", default=".", help="Dossier où créer le package.")
    args = parser.parse_args()

    stl_path = os.path.abspath(args.stl)
    if not os.path.isfile(stl_path):
        sys.exit(f"Erreur : fichier introuvable {stl_path}")

    pkg_dir = os.path.join(args.output_dir, args.robot_name)
    meshes_dir = os.path.join(pkg_dir, "meshes")
    urdf_dir = os.path.join(pkg_dir, "urdf")
    config_dir = os.path.join(pkg_dir, "config")
    os.makedirs(meshes_dir, exist_ok=True)
    os.makedirs(urdf_dir, exist_ok=True)
    os.makedirs(config_dir, exist_ok=True)

    # Copie du mesh
    mesh_filename = os.path.basename(stl_path)
    mesh_dest_path = os.path.join(meshes_dir, mesh_filename)
    shutil.copyfile(stl_path, mesh_dest_path)

    # Génération URDF
    urdf_tree = make_urdf(
        robot_name=args.robot_name,
        mesh_rel_path=f"meshes/{mesh_filename}",
        link_name=args.link_name
    )
    urdf_tree.write(os.path.join(urdf_dir, f"{args.robot_name}.urdf"),
                    encoding="utf-8", xml_declaration=True)

    # Génération SRDF
    with open(os.path.join(config_dir, f"{args.robot_name}.srdf"), "w", encoding="utf-8") as f:
        f.write(make_srdf(args.robot_name))

    # Génération package.xml et CMakeLists.txt
    with open(os.path.join(pkg_dir, "package.xml"), "w", encoding="utf-8") as f:
        f.write(make_package_xml(args.robot_name))
    with open(os.path.join(pkg_dir, "CMakeLists.txt"), "w", encoding="utf-8") as f:
        f.write(make_cmakelists(args.robot_name))

    print(f"[OK] Package ROS généré dans : {pkg_dir}")

if __name__ == "__main__":
    main()
