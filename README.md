# hsr_humble_docker

[hsr_ros2_doc](https://github.com/hsr-project/hsr_ros2_doc/tree/humble)

## Setup

## シミュレーションの起動
### Gazebo
Gazeboシミュレータを起動します． 以下は起動launchの例で，使うロボット，ワールドに応じて適切なlaunchファイルを利用してください．
```
ros2 launch hsrb_gazebo_launch hsrb_apartment_no_objects_world.launch.py
```

### Rviz Simulation
センサ系のシミュレーションはありません．簡単な動作確認には利用可能です．
#### HSR-B
```
ros2 launch hsrb_rviz_simulator hsrb_rviz_simulator.launch.py
```

#### HSR-C
```
ros2 launch hsrb_rviz_simulator hsrc_rviz_simulator.launch.py
```

## MoveIt
### サンプルプログラム
シミュレータ等を起動後，demo.launch.pyを実行します．ロボットに応じた適切なlaunchファイルを利用してください． 以下は，HSR-Bの例です．
```
ros2 launch hsrb_moveit_config hsrb_demo.launch.py
```
HSR-Cの場合は
```
ros2 launch hsrb_moveit_config hsrc_demo.launch.py
```

### GUIでの操作
RvizのMotionPlanningプラグインから指令値を投げてください．

### C++からのMoveGroupの利用
次のコマンドで，サンプルプログラムを実行できます．
```
ros2 launch hsrb_moveit_config hsrb_example.launch.py example_name:=moveit_fk_demo
```
準備されているサンプルプログラムは以下の通りです．
- moveit_fk_demo
- moveit_ik_demo
- moveit_gripper_demo
- moveit_constraints_demo

## RosNav2
### Mapping
まず，地図作成用のノードを立ち上げます．
```
ros2 launch hsrb_mapping slam_toolbox_mapping.launch.py
```
Rvizで地図作成の進捗を確認します．
```
rviz2 -d src/hsrb_rosnav/hsrb_mapping/rviz/hsr_slam_toolbox_mapping.rviz
```
地図が十分作成されたら，保存します．
```
ros2 run nav2_map_server map_saver_cli -f map --ros-args -p save_map_timeout:=100000.0
```
これを実行することで，実行したディレクトリに2つのファイルが保存されます．
- map.pgm
- map.yaml

### Navigation
ナビゲーション用のプログラムを立ち上げます．
```
ros2 launch hsrb_rosnav_config navigation_launch.py map:=/full/path/to/map.yaml use_sim_time:=True initial_orientation_xyzw:=[0,0,0,1]
```
Rvizを使って自己位置を初期化し，ナビゲーションの確認を行います．
```
rviz2 -d src/hsrb_rosnav/hsrb_rosnav_config/rviz/hsr_navigation2.rviz
```
1. RViz 上部にある 2D Pose Estimate を利用して，自己位置を初期化してください
2. RViz 上部にある Navigation2 Goal を利用して，目標位置を指令してください

## Python インタフェース
```
ros2 run hsrb_interface_py ihsrb.py
```
以下はコマンド例です．詳細は，各関数のhelpを確認ください．
```
whole_body.move_to_go()
whole_body.move_to_neutral()
whole_body.move_to_joint_positions({'arm_lift_joint': 0.1})
whole_body.gaze_point(point=geometry.Vector3(x=1.0, y=-0.5, z=0.3), ref_frame_id='base_link')
whole_body.move_end_effector_pose([geometry.pose(z=1.0), geometry.pose(z=0.8)], ref_frame_id='hand_palm_link')
whole_body.move_end_effector_by_line((0, 0, 1), 0.3)
whole_body.move_end_effector_by_arc(geometry.pose(y=0.45, z=0.08, ej=math.radians(90.0)), math.radians(60.0), ref_frame_id='hand_palm_link')
gripper.command(1.2)
gripper.apply_force(1.0)
gripper.set_distance(0.05)
gripper.command(0.0)
poses = [geometry.pose(x=1.0, y=0.0, ek=0.0), geometry.pose(x=1.0, y=1.0, ek=math.pi)]
omni_base.go_abs(1.0, -2.0, 0.0, 300.0)
omni_base.go_rel(0.0, 1.0, 0.0, 100.0)
omni_base.follow_trajectory(poses, ref_frame_id='base_footprint')
goal = omni_base.create_follow_trajectory_goal(poses, time_from_starts=[10, 30], ref_frame_id='base_footprint')
omni_base.execute(goal)
omni_base.is_succeeded()
omni_base.cancel_goal()
tts.say(u'Hello')
```
