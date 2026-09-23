# Erratic-like robot with a planar ranger.

define ros2_laser ranger
(
  sensor(
    range [ 0.00 20.0 ]
    fov 360.0
    samples 720
  )
  color "blue"
  size [ 0.05 0.05 0.05 ]
)

define ros2_robot position
(
  size [ 0.50 0.40 0.25 ]
  origin [ 0 0 0 0 ]
  gui_nose 1
  drive "diff"
  localization "odom"
  localization_origin [ 0 0 0 0 ]

  ros2_laser(
    name "laser"
    pose [ 0.22 0 0.20 0.3 ]
  )
)
