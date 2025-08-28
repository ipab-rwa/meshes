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


def make_urdf(robot_name: str, mesh_rel_path: str, link_name: str = "base_link"):
    inertia = (1e-3, 0.0, 0.0, 1e-3, 0.0, 1e-3)
    com = (0.0, 0.0, 0.0)
    mass = 1.0

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
    ET.SubElement(visual, "origin", xyz="0 0 0", rpy="0 0 0")
    geom_v = ET.SubElement(visual, "geometry")
    ET.SubElement(geom_v, "mesh", filename=f"package://{robot_name}/{mesh_rel_path}",
                  scale="1 1 1")
    material = ET.SubElement(visual, "material", name="grey")
    ET.SubElement(material, "color", rgba="0.7 0.7 0.7 1.0")

    collision = ET.SubElement(link, "collision")
    ET.SubElement(collision, "origin", xyz="0 0 0", rpy="0 0 0")
    geom_c = ET.SubElement(collision, "geometry")
    ET.SubElement(geom_c, "mesh", filename=f"package://{robot_name}/{mesh_rel_path}",
                  scale="1 1 1")

    tree = ET.ElementTree(robot)
    ET.indent(tree, space="  ", level=0)
    return tree

def make_srdf(robot_name: str):
    return f"""<?xml version="1.0" ?>
<robot name="{robot_name}">
  <!-- SRDF minimal -->
</robot>
"""

def make_package_xml(robot_name: str):
    return f"""<?xml version="1.0"?>
<package format="2">
  <name>{robot_name}</name>
  <version>0.0.1</version>
  <description>Auto-generated package for {robot_name}</description>
  <maintainer email="user@example.com">Auto Generator</maintainer>
  <license>BSD</license>
  <buildtool_depend>catkin</buildtool_depend>
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


def safe_copy(src, dst):
    """Copie un fichier uniquement si la destination est différente ou n'existe pas."""
    if os.path.exists(dst):
        # Vérifie si c'est le même fichier physique
        try:
            if os.path.samefile(src, dst):
                print(f"[SKIP] Fichier identique, pas de copie : {dst}")
                return
        except FileNotFoundError:
            pass  # si dst n'existe pas encore
    shutil.copyfile(src, dst)

def generate_packages_for_all_stls(start_rel_path: str):
    start_path = os.path.abspath(start_rel_path)
    output_root = os.path.abspath(os.path.join(start_path, "./urdf"))
    os.makedirs(output_root, exist_ok=True)

    for root, _, files in os.walk(start_path):
        for file in files:
            if file.lower().endswith(".stl"):
                stl_path = os.path.join(root, file)
                robot_name = os.path.splitext(file)[0].lower().replace(" ", "_")
                pkg_dir = os.path.join(output_root, robot_name)

                meshes_dir = os.path.join(pkg_dir, "meshes")
                urdf_dir = os.path.join(pkg_dir, "urdf")
                config_dir = os.path.join(pkg_dir, "config")
                os.makedirs(meshes_dir, exist_ok=True)
                os.makedirs(urdf_dir, exist_ok=True)
                os.makedirs(config_dir, exist_ok=True)

                mesh_filename = os.path.basename(stl_path)
                safe_copy(stl_path, os.path.join(meshes_dir, mesh_filename))

                urdf_tree = make_urdf(
                    robot_name=robot_name,
                    mesh_rel_path=f"meshes/{mesh_filename}"
                )
                urdf_tree.write(os.path.join(urdf_dir, f"{robot_name}.urdf"),
                                encoding="utf-8", xml_declaration=True)

                with open(os.path.join(config_dir, f"{robot_name}.srdf"), "w") as f:
                    f.write(make_srdf(robot_name))

                with open(os.path.join(pkg_dir, "package.xml"), "w") as f:
                    f.write(make_package_xml(robot_name))
                with open(os.path.join(pkg_dir, "CMakeLists.txt"), "w") as f:
                    f.write(make_cmakelists(robot_name))

                print(f"[OK] Package créé : {pkg_dir}")

# Exemple d'appel :
# generate_packages_for_all_stls("mes_stl")



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
    safe_copy(stl_path, mesh_dest)

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
    generate_packages_for_all_stls("../")
