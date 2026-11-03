#!/usr/bin/env python3
"""Generate the 5 x 5 m test arena world and AprilTag pole models.

Reads config/world_params.yaml and writes, inside this package's source tree:

  worlds/test_arena.sdf           - the Gazebo world
  models/apriltag_pole_<id>/      - one pole model per tag (pole, base, plate, tag)
  config/apriltags.yaml           - ground-truth tag poses in the world frame

Run it from anywhere after editing world_params.yaml, then rebuild the package:

  python3 parc_test_world/scripts/generate_world.py
  colcon build --packages-select parc_test_world
"""

import math
import shutil
from pathlib import Path

import cv2
import numpy as np
import yaml

PKG_DIR = Path(__file__).resolve().parent.parent
PARAMS_FILE = PKG_DIR / "config" / "world_params.yaml"
WORLD_FILE = PKG_DIR / "worlds" / "test_arena.sdf"
MODELS_DIR = PKG_DIR / "models"
TAGS_FILE = PKG_DIR / "config" / "apriltags.yaml"

TAG_FAMILIES = {"tag36h11": cv2.aruco.DICT_APRILTAG_36h11}

# Pixels per tag cell in the generated texture. A tag36h11 marker is 8 cells wide
# (including the black border) plus a white margin of apriltag.margin_cells on each side.
PX_PER_CELL = 64


def fmt(*values):
    """Format numbers for SDF, dropping noise like -0.0 and long fractions."""
    return " ".join(f"{v + 0.0:.6g}" for v in values)


def make_tag_texture(family, tag_id, path, margin_cells):
    """Write a tag image with a white margin of margin_cells, as it would be printed."""
    dictionary = cv2.aruco.getPredefinedDictionary(TAG_FAMILIES[family])
    marker_cells = dictionary.markerSize + 2  # data bits + black border
    # OpenCV < 4.7 calls this drawMarker
    generate = getattr(cv2.aruco, "generateImageMarker", None) or cv2.aruco.drawMarker
    marker = generate(dictionary, tag_id, marker_cells * PX_PER_CELL, borderBits=1)
    pad = margin_cells * PX_PER_CELL
    image = cv2.copyMakeBorder(
        marker, pad, pad, pad, pad,
        cv2.BORDER_CONSTANT, value=255,
    )
    cv2.imwrite(str(path), image)
    return marker_cells


def write_tag_mesh(mesh_dir, texture_name):
    """Write a 1 x 1 m textured quad in the YZ plane facing +x.

    Seen from the front (looking along -x), image right is +y and image up is +z,
    so the tag is not mirrored. The quad is scaled to size in the SDF.
    """
    (mesh_dir / "tag.mtl").write_text(
        "newmtl tag\n"
        "Ka 1 1 1\n"
        "Kd 1 1 1\n"
        "Ks 0 0 0\n"
        "d 1\n"
        "illum 1\n"
        f"map_Kd {texture_name}\n"
    )
    (mesh_dir / "tag.obj").write_text(
        "mtllib tag.mtl\n"
        "o tag\n"
        "v 0 -0.5 -0.5\n"
        "v 0  0.5 -0.5\n"
        "v 0  0.5  0.5\n"
        "v 0 -0.5  0.5\n"
        "vt 0 0\n"
        "vt 1 0\n"
        "vt 1 1\n"
        "vt 0 1\n"
        "vn 1 0 0\n"
        "usemtl tag\n"
        "f 1/1/1 2/2/1 3/3/1 4/4/1\n"
    )


def cylinder_element(kind, name, radius, length, z, color=None):
    material = ""
    if color:
        material = (
            f"<material><ambient>{color}</ambient><diffuse>{color}</diffuse>"
            "<specular>0.1 0.1 0.1 1</specular></material>"
        )
    return f"""
      <{kind} name="{name}">
        <pose>0 0 {fmt(z)} 0 0 0</pose>
        <geometry><cylinder><radius>{fmt(radius)}</radius><length>{fmt(length)}</length></cylinder></geometry>
        {material}
      </{kind}>"""


def write_pole_model(params, tag_id):
    """Create models/apriltag_pole_<id>. Model origin is the base centre on the floor; the tag faces +x."""
    pole = params["pole"]
    tag = params["apriltag"]
    name = f"apriltag_pole_{tag_id}"
    model_dir = MODELS_DIR / name
    mesh_dir = model_dir / "meshes"
    if model_dir.exists():
        shutil.rmtree(model_dir)
    mesh_dir.mkdir(parents=True)

    texture_name = f"{tag['family']}_{tag_id}.png"
    margin_cells = tag.get("margin_cells", 1)
    marker_cells = make_tag_texture(tag["family"], tag_id, mesh_dir / texture_name, margin_cells)
    write_tag_mesh(mesh_dir, texture_name)

    radius = pole["diameter"] / 2
    printed_size = tag["tag_size"] * (marker_cells + 2 * margin_cells) / marker_cells
    plate_t = pole["plate_thickness"]
    plate_x = radius + plate_t / 2
    tag_x = radius + plate_t + 0.0005  # just proud of the plate to avoid z-fighting
    tag_z = pole["tag_center_height"]

    if tag_z + printed_size / 2 > pole["height"]:
        print(f"warning: tag {tag_id} sticks out above the top of its pole")

    pole_color = "0.25 0.25 0.27 1"
    elements = "".join([
        cylinder_element("collision", "base_collision", pole["base_diameter"] / 2,
                         pole["base_height"], pole["base_height"] / 2),
        cylinder_element("visual", "base_visual", pole["base_diameter"] / 2,
                         pole["base_height"], pole["base_height"] / 2, pole_color),
        cylinder_element("collision", "pole_collision", radius, pole["height"], pole["height"] / 2),
        cylinder_element("visual", "pole_visual", radius, pole["height"], pole["height"] / 2,
                         pole_color),
    ])

    (model_dir / "model.sdf").write_text(f"""<?xml version="1.0"?>
<!-- Generated by scripts/generate_world.py - edit config/world_params.yaml instead. -->
<sdf version="1.10">
  <model name="{name}">
    <static>true</static>
    <link name="link">{elements}
      <collision name="plate_collision">
        <pose>{fmt(plate_x, 0, tag_z)} 0 0 0</pose>
        <geometry><box><size>{fmt(plate_t, printed_size, printed_size)}</size></box></geometry>
      </collision>
      <visual name="plate_visual">
        <pose>{fmt(plate_x, 0, tag_z)} 0 0 0</pose>
        <geometry><box><size>{fmt(plate_t, printed_size, printed_size)}</size></box></geometry>
        <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0 0 0 1</specular></material>
      </visual>
      <visual name="tag_visual">
        <pose>{fmt(tag_x, 0, tag_z)} 0 0 0</pose>
        <geometry>
          <mesh>
            <uri>model://{name}/meshes/tag.obj</uri>
            <scale>{fmt(1, printed_size, printed_size)}</scale>
          </mesh>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
""")
    (model_dir / "model.config").write_text(f"""<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.10">model.sdf</sdf>
  <description>Pole with AprilTag {tag['family']} id {tag_id}. Generated by scripts/generate_world.py.</description>
</model>
""")
    return tag_x, tag_z


def include(uri, name, x, y, z, yaw):
    return f"""
    <include>
      <uri>model://{uri}</uri>
      <name>{name}</name>
      <pose>{fmt(x, y, z)} 0 0 {fmt(yaw)}</pose>
    </include>"""


def barrier_model(arena):
    half = arena["size"] / 2
    t = arena["barrier_thickness"]
    h = arena["barrier_height"]
    outer = arena["size"] + 2 * t
    walls = {
        "north": (0, half + t / 2, outer, t),
        "south": (0, -half - t / 2, outer, t),
        "east": (half + t / 2, 0, t, arena["size"]),
        "west": (-half - t / 2, 0, t, arena["size"]),
    }
    color = "0.55 0.38 0.22 1"
    parts = ""
    for side, (x, y, sx, sy) in walls.items():
        geometry = f"<geometry><box><size>{fmt(sx, sy, h)}</size></box></geometry>"
        parts += f"""
        <collision name="{side}_collision">
          <pose>{fmt(x, y, h / 2)} 0 0 0</pose>
          {geometry}
        </collision>
        <visual name="{side}_visual">
          <pose>{fmt(x, y, h / 2)} 0 0 0</pose>
          {geometry}
          <material><ambient>{color}</ambient><diffuse>{color}</diffuse><specular>0.1 0.1 0.1 1</specular></material>
        </visual>"""
    return f"""
    <model name="arena_barrier">
      <static>true</static>
      <link name="link">{parts}
      </link>
    </model>"""


def main():
    params = yaml.safe_load(PARAMS_FILE.read_text())
    arena = params["arena"]
    tag = params["apriltag"]
    if tag["family"] not in TAG_FAMILIES:
        raise SystemExit(f"Unsupported tag family {tag['family']!r}; supported: {list(TAG_FAMILIES)}")

    MODELS_DIR.mkdir(exist_ok=True)
    WORLD_FILE.parent.mkdir(exist_ok=True)

    # Remove pole models left over from tag IDs that are no longer used.
    wanted = {f"apriltag_pole_{p['id']}" for p in params["poles"]}
    for old in MODELS_DIR.glob("apriltag_pole_*"):
        if old.name not in wanted:
            shutil.rmtree(old)

    entities = ""
    tag_poses = []
    for p in params["poles"]:
        yaw = p.get("yaw", math.atan2(-p["y"], -p["x"]))
        tag_x, tag_z = write_pole_model(params, p["id"])
        entities += include(f"apriltag_pole_{p['id']}", f"apriltag_pole_{p['id']}",
                            p["x"], p["y"], 0, yaw)
        tag_poses.append({
            "id": p["id"],
            "x": round(p["x"] + tag_x * math.cos(yaw), 4),
            "y": round(p["y"] + tag_x * math.sin(yaw), 4),
            "z": round(tag_z, 4),
            "facing_yaw": round(yaw, 4),
        })

    tables = {t["name"]: t for t in params["tables"]}
    for t in params["tables"]:
        entities += include("cafe_table", t["name"], t["x"], t["y"], 0, t.get("yaw", 0.0))
    for c in params["chairs"]:
        yaw = c.get("yaw", 0.0)
        if "face" in c:
            # The WoodenChair model faces +x (backrest on -x).
            target = tables[c["face"]]
            yaw = math.atan2(target["y"] - c["y"], target["x"] - c["x"])
        entities += include("WoodenChair", c["name"], c["x"], c["y"], 0, yaw)

    size = arena["size"]
    WORLD_FILE.write_text(f"""<?xml version="1.0"?>
<!-- Generated by scripts/generate_world.py - edit config/world_params.yaml instead. -->
<sdf version="1.10">
  <world name="test_arena">
    <physics name="1ms" type="ignored">
      <max_step_size>0.01</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>

    <scene>
      <ambient>0.5 0.5 0.5 1</ambient>
      <background>0.75 0.78 0.82 1</background>
      <shadows>true</shadows>
    </scene>

    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1</intensity>
      <direction>-0.5 0.1 -0.9</direction>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
    </light>

    <!-- Shadowless fill light from the opposite side, like indoor lighting, so tags
         facing away from the sun are not left in shade. -->
    <light name="fill" type="directional">
      <cast_shadows>false</cast_shadows>
      <intensity>0.7</intensity>
      <direction>0.5 -0.1 -0.6</direction>
      <diffuse>0.7 0.7 0.7 1</diffuse>
      <specular>0.05 0.05 0.05 1</specular>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>30 30</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>30 30</size></plane></geometry>
          <material><ambient>0.45 0.45 0.45 1</ambient><diffuse>0.45 0.45 0.45 1</diffuse><specular>0.1 0.1 0.1 1</specular></material>
        </visual>
        <!-- Lighter floor inside the {fmt(size)} x {fmt(size)} m arena (visual only). -->
        <visual name="arena_floor">
          <pose>0 0 0.001 0 0 0</pose>
          <geometry><plane><normal>0 0 1</normal><size>{fmt(size, size)}</size></plane></geometry>
          <material><ambient>0.82 0.8 0.76 1</ambient><diffuse>0.82 0.8 0.76 1</diffuse><specular>0.1 0.1 0.1 1</specular></material>
        </visual>
      </link>
    </model>
{barrier_model(arena)}
{entities}
  </world>
</sdf>
""")

    TAGS_FILE.write_text(
        "# Generated by scripts/generate_world.py - ground-truth AprilTag poses in the world frame.\n"
        "# x, y, z: centre of the tag face. facing_yaw: direction the tag faces (rad, from +x).\n"
        + yaml.safe_dump({
            "family": tag["family"],
            "size": tag["tag_size"],
            "tags": tag_poses,
        }, sort_keys=False)
    )

    print(f"Wrote {WORLD_FILE.relative_to(PKG_DIR)}, {len(tag_poses)} pole models, "
          f"and {TAGS_FILE.relative_to(PKG_DIR)}")


if __name__ == "__main__":
    main()
