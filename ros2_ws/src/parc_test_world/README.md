# parc_test_world

A 6 x 7 m test arena for the CAYTU SITO-E robot that mimics the real test environment:
two cafe tables with three chairs tucked in at each, four poles with AprilTags (`tag36h11`, IDs 1, 4, 7, 9 as used by the referee)
in the corners facing the centre, and a low barrier around the edge.

## Launch

```bash
colcon build --packages-select parc_test_world
source install/setup.bash
ros2 launch parc_test_world test_world.launch.py
```

Launch arguments:

| Argument | Default | Description |
|---|---|---|
| `x`, `y`, `z`, `yaw` | from `config/world_params.yaml` | Robot spawn pose |
| `rviz` | `true` | Start RViz |
| `use_sim_time` | `true` | Use the Gazebo clock |

The robot, bridge (`parc_robot_bringup/config/gz_bridge.yaml`) and RViz config are reused
from `parc_robot_bringup`, so topics are the same as in the task world.

## Layout

The arena is centred on the world origin (inner edges at x = ±3.0 m, y = ±3.5 m).

```
 y
 ^   T4 (-2.5, 3)                    T1 (2.5, 3)
 |        chair
 |  chair [table_a] chair
 |         (-1, 0.9)
 |  R -> ----- open corridor (y ≈ ±0.45) -----
 |                       chair [table_b] chair
 |                              (1, -0.9)
 |                               chair
 |   T7 (-2.5, -3)                   T9 (2.5, -3)
 +-------------------------------------------> x
```

`R` is the default robot spawn at (-1.8, 0), facing +x.

## Changing dimensions

Pole diameter, pole height, tag size and height, barrier size, and furniture and pole
positions are all in `config/world_params.yaml`. Most of them were eyeballed and are
marked as such. After editing that file, regenerate and rebuild:

```bash
python3 parc_test_world/scripts/generate_world.py   # needs python3-opencv
colcon build --packages-select parc_test_world
```

This rewrites `worlds/test_arena.sdf`, `models/apriltag_pole_*` and `config/apriltags.yaml`.
Don't edit those by hand.

## AprilTags

- Family `tag36h11`, black square 0.165 m (the `size` to give your detector). Each tag
  is printed with a two-cell white margin on a white plate, so the plate is 0.248 m square
  (`apriltag.margin_cells` in `config/world_params.yaml`).
- `config/apriltags.yaml` lists the ground-truth world position and facing direction of
  every tag, for checking localisation against.
- The simulated top camera is 640 x 480. At that resolution a tag is roughly 38 px wide
  at 1.4 m, which is near the lower limit for reliable decoding.
