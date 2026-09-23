define rslidar ranger
(
  sensor
  (
    pose [ 0.0 0.0 0.5 0.0 ]
    size [ 0.1 0.1 0.1 ]
    range [ 0.0 42.0 ]
    fov 360.0
    samples 720
  )
  color "red"
  size [ 0.05 0.05 0.05 ]
)

define ros2_robot position
(
  size [ 0.612 0.580 0.245 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  drive "omni"
  localization "odom"
  localization_origin [ 0.0 0.0 0.0 0.0 ]

  rslidar
  (
    name "laser"
    pose [ 0.0 0.0 0.0 0.0 ]
  )
)
