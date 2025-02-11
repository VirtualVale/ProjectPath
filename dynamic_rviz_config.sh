#!/bin/bash
#this script produces a rviz config file for a dynamic number of robots
#TODO change the location of the config file to a dynamic path
echo "ready for creating the config"
number=$1
r_value=(0 0 1 0 1 1 0 1 0)
g_value=(0 0 0 1 1 0 1 1 0)
b_value=(0 1 0 0 0 1 1 1 0)
#static part
echo "Panels:
  - Class: rviz/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /TF1/Frames1
        - /TF1/Tree1
      Splitter Ratio: 0.5
    Tree Height: 843
  - Class: rviz/Selection
    Name: Selection
  - Class: rviz/Tool Properties
    Expanded:
      - /2D Pose Estimate1
      - /2D Nav Goal1
    Name: Tool Properties
    Splitter Ratio: 0.588679016
  - Class: rviz/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5
  - Class: rviz/Time
    Experimental: false
    Name: Time
    SyncMode: 0
    SyncSource: ""
Toolbars:
  toolButtonStyle: 2
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.0299999993
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 20
      Reference Frame: <Fixed Frame>
      Value: true
    - Class: rviz/TF
      Enabled: false
      Frame Timeout: 15
      Frames:
        All Enabled: false
      Marker Scale: 1
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: false
      Tree:
        {}
      Update Interval: 0
      Value: false
    - Class: rviz/Image
      Enabled: false
      Image Topic: /raspicam_node/image
      Max Value: 1
      Median window: 5
      Min Value: 0
      Name: Image
      Normalize Range: true
      Queue Size: 2
      Transport Hint: compressed
      Unreliable: false
      Value: false
    - Alpha: 0.699999988
      Class: rviz/Map
      Color Scheme: map
      Draw Behind: false
      Enabled: true
      Name: Map
      Topic: /map
      Unreliable: false
      Use Timestamp: false
      Value: true" > /home/valo/catkin_ws/src/timeflow/path_planning_system/config/dynamic.rviz
#loop to create the part for every resource
counter=1
while [ $counter -le $number ]
do
  echo "${counter}"
  echo "    - Class: rviz/Group
      Displays:
        - Class: rviz/Group
          Displays:
            - Alpha: 1
              Class: rviz/Polygon
              Color: 0; 0; 0
              Enabled: true
              Name: Polygon
              Topic: /tb3_${counter}/move_base/local_costmap/footprint
              Unreliable: false
              Value: true
            - Alpha: 0.699999988
              Class: rviz/Map
              Color Scheme: costmap
              Draw Behind: false
              Enabled: true
              Name: Costmap
              Topic: /tb3_${counter}/move_base/local_costmap/costmap
              Unreliable: false
              Use Timestamp: false
              Value: true
            - Alpha: 1
              Buffer Length: 1
              Class: rviz/Path
              Color: $((${r_value[$counter]} * 255)); $((${g_value[$counter]} * 255)); $((${b_value[$counter]} * 255))
              Enabled: true
              Head Diameter: 0.300000012
              Head Length: 0.200000003
              Length: 0.300000012
              Line Style: Lines
              Line Width: 0.0299999993
              Name: Planner
              Offset:
                X: 0
                Y: 0
                Z: 0
              Pose Color: $((${r_value[$counter]}+2)); ${g_value[$counter]}; ${b_value[$counter]}
              Pose Style: None
              Radius: 0.0299999993
              Shaft Diameter: 0.100000001
              Shaft Length: 0.100000001
              Topic: /tb3_${counter}/move_base/DWAPlannerROS/local_plan
              Unreliable: false
              Value: true
          Enabled: true
          Name: Local Map
        - Class: rviz/Group
          Displays:
            - Alpha: 0.699999988
              Class: rviz/Map
              Color Scheme: costmap
              Draw Behind: true
              Enabled: true
              Name: Costmap
              Topic: /tb3_${counter}/move_base/global_costmap/costmap
              Unreliable: false
              Use Timestamp: false
              Value: true
            - Alpha: 1
              Buffer Length: 1
              Class: rviz/Path
              Color: $((${r_value[$counter]} * 205)); $((${g_value[$counter]} * 205)); $((${b_value[$counter]} * 205))
              Enabled: true
              Head Diameter: 0.300000012
              Head Length: 0.200000003
              Length: 0.300000012
              Line Style: Lines
              Line Width: 0.0299999993
              Name: Planner
              Offset:
                X: 0
                Y: 0
                Z: 0
              Pose Color: $((${r_value[$counter]}+4)); ${g_value[$counter]}; ${b_value[$counter]}
              Pose Style: None
              Radius: 0.0299999993
              Shaft Diameter: 0.100000001
              Shaft Length: 0.100000001
              Topic: /tb3_${counter}/move_base/DWAPlannerROS/global_plan
              Unreliable: false
              Value: true
          Enabled: true
          Name: Global Map
        - Alpha: 1
          Arrow Length: 0.0500000007
          Axes Length: 0.300000012
          Axes Radius: 0.00999999978
          Class: rviz/PoseArray
          Color: $((${r_value[$counter]} * 255)); $((${g_value[$counter]} * 255)); $((${b_value[$counter]} * 255))
          Enabled: true
          Head Length: 0.0700000003
          Head Radius: 0.0299999993
          Name: Amcl Particles
          Shaft Length: 0.230000004
          Shaft Radius: 0.00999999978
          Shape: Arrow (Flat)
          Topic: /tb3_${counter}/particlecloud
          Unreliable: false
          Value: true
        - Alpha: 1
          Autocompute Intensity Bounds: true
          Autocompute Value Bounds:
            Max Value: 10
            Min Value: -10
            Value: true
          Axis: Z
          Channel Name: intensity
          Class: rviz/LaserScan
          Color: $((${r_value[$counter]} * 255)); $((${g_value[$counter]} * 255)); $((${b_value[$counter]} * 255))
          Color Transformer: FlatColor
          Decay Time: 0
          Enabled: true
          Invert Rainbow: false
          Max Color: 255; 255; 255
          Max Intensity: 13069
          Min Color: 0; 0; 0
          Min Intensity: 28
          Name: LaserScan
          Position Transformer: XYZ
          Queue Size: 10
          Selectable: true
          Size (Pixels): 3
          Size (m): 0.0300000012
          Style: Flat Squares
          Topic: /tb3_${counter}/scan
          Unreliable: false
          Use Fixed Frame: true
          Use rainbow: true
          Value: true
        - Alpha: 1
          Buffer Length: 1
          Class: rviz/Path
          Color: $((${r_value[$counter]} * 139)); $((${g_value[$counter]} * 139)); $((${b_value[$counter]} * 139))
          Enabled: true
          Head Diameter: 0.300000012
          Head Length: 0.200000003
          Length: 0.300000012
          Line Style: Lines
          Line Width: 0.0299999993
          Name: Planner Plan
          Offset:
            X: 0
            Y: 0
            Z: 0
          Pose Color: $((${r_value[$counter]}+8)); ${g_value[$counter]}; ${b_value[$counter]}
          Pose Style: None
          Radius: 0.0299999993
          Shaft Diameter: 0.100000001
          Shaft Length: 0.100000001
          Topic: /tb3_${counter}/move_base/NavfnROS/plan
          Unreliable: false
          Value: true
        - Alpha: 1
          Class: rviz/RobotModel
          Collision Enabled: false
          Enabled: true
          Links:
            All Links Enabled: true
            Expand Joint Details: false
            Expand Link Details: false
            Expand Tree: false
            Link Tree Style: Links in Alphabetic Order
          Name: RobotModel
          Robot Description: /tb3_${counter}/robot_description
          TF Prefix: tb3_${counter}
          Update Interval: 0
          Value: true
          Visual Enabled: true
        - Alpha: 1
          Axes Length: 1
          Axes Radius: 0.100000001
          Class: rviz/Pose
          Color: $((${r_value[$counter]} * 139)); $((${g_value[$counter]} * 139)); $((${b_value[$counter]} * 139))
          Enabled: true
          Head Length: 0.300000012
          Head Radius: 0.100000001
          Name: Goal
          Shaft Length: 0.5
          Shaft Radius: 0.0500000007
          Shape: Arrow
          Topic: /tb3_${counter}/move_base/current_goal
          Unreliable: false
          Value: true
      Enabled: true
      Name: tb3_${counter}" >> /home/valo/catkin_ws/src/timeflow/path_planning_system/config/dynamic.rviz
        ((counter++))
done

#static part II
echo "  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Default Light: true
    Fixed Frame: map
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz/MoveCamera
    - Class: rviz/Interact
      Hide Inactive Objects: true
    - Class: rviz/Select
    - Class: rviz/SetInitialPose
      Topic: /initialpose
    - Class: rviz/SetGoal
      Topic: tb3_1/move_base_simple/goal
    - Class: rviz/Measure
  Value: true
  Views:
    Current:
      Angle: -1.92579532
      Class: rviz/TopDownOrtho
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.0599999987
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.00999999978
      Scale: 146.759033
      Target Frame: <Fixed Frame>
      Value: TopDownOrtho (rviz)
      X: 3.32984638
      Y: 1.58498001
    Saved: ~
Window Geometry:
  Displays:
    collapsed: false
  Height: 1056
  Hide Left Dock: false
  Hide Right Dock: true
  Image:
    collapsed: false
  QMainWindow State: 000000ff00000000fd00000004000000000000016a000003dafc0200000007fb0000001200530065006c0065006300740069006f006e00000001e10000009b0000006100fffffffb0000001e0054006f006f006c002000500072006f007000650072007400690065007302000001ed000001df00000185000000a3fb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb000000100044006900730070006c0061007900730100000028000003da000000d700fffffffb0000000a0049006d0061006700650000000336000000cc0000001600fffffffb0000000a0049006d0061006700650000000330000000ce0000000000000000000000010000010f000003a0fc0200000003fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000000a005600690065007700730000000043000003a0000000ad00fffffffb0000001200530065006c0065006300740069006f006e010000025a000000b200000000000000000000000200000490000000a9fc0100000001fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000004a00000003efc0100000002fb0000000800540069006d00650000000000000004a00000030000fffffffb0000000800540069006d00650100000000000004500000000000000000000005cf000003da00000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000
  Selection:
    collapsed: false
  Time:
    collapsed: false
  Tool Properties:
    collapsed: false
  Views:
    collapsed: true
  Width: 1855
  X: 65
  Y: 24
" >> /home/valo/catkin_ws/src/timeflow/path_planning_system/config/dynamic.rviz

echo "finished.
dynamic rviz config ready"