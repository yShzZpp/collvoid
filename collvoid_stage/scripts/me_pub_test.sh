#!/bin/bash
rostopic pub /position_share_me collvoid_msgs/PoseTwistWithCovariance "
header: 
  seq: 91976
  stamp: 
    secs: 1722594329
    nsecs: 252557307
  frame_id: \"/map\"
robot_id: \"86\"
radius: 0.0
holo_robot: False
controlled: True
holonomic_velocity: 
  x: 0.0
  y: 0.0
  z: 0.0
pose: 
  pose: 
    position: 
      x: 13.3649123811
      y: 0.00859520007612
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: 0.997221536348
      w: 0.0744930026437
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
twist: 
  twist: 
    linear: 
      x: 1.0
      y: 0.0
      z: 0.0
    angular: 
      x: 0.0
      y: 0.0
      z: 1.0
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
footprint: 
  header: 
    seq: 0
    stamp: 
      secs: 1722594329
      nsecs: 267129771
    frame_id: \"/base_link\"
  polygon: 
    points: 
      - 
        x: 0.40000000596
        y: 0.259999990463
        z: 0.0
      - 
        x: -0.40000000596
        y: 0.259999990463
        z: 0.0
      - 
        x: -0.40000000596
        y: -0.259999990463
        z: 0.0
      - 
        x: 0.40000000596
        y: -0.259999990463
        z: 0.0" -r1
