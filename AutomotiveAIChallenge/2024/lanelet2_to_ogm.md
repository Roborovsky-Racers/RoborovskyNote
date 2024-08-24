---
title: Roborovsky Note
---

# lanelet2 を Occupancy Grid Map 形式に変換する

## 本記事の要点

* Autoware で使用されている lanelet2 形式 (拡張子 `.osm`) の地図データを ROS などで良く使用する Occupancy Grid Map 形式 (拡張子 `.pgm` と `.yaml`) に変換するツールを作成しました。
* ソースコードは [こちら](https://github.com/Roborovsky-Racers/lanelet2_to_ogm) に公開中


## 地図形式の変換を実行する

ソースコードを clone します。
```
git clone https://github.com/Roborovsky-Racers/lanelet2_to_ogm.git
```

lanelet2 形式のマップを `lanelet2/map/lanelet2_map.osm` に配置します。
なお、デフォルトで `AutomotiveAIChallenge/aichallenge-2024` リポジトリにアップロードされている [lanelet2 マップ](https://github.com/AutomotiveAIChallenge/aichallenge-2024/blob/v2024.7.19.0/aichallenge/workspace/src/aichallenge_submit/aichallenge_submit_launch/map/lanelet2_map.osm) を配置してあるため、他のマップを使う必要がなければそのままで問題ありません。

ディレクトリを移動し、変換を実行します。
```
cd lanelet2_to_ogm
make
```

成功した場合、 `lanelet2_to_ogm/map/` の下に、 `occupancy_grid_map.pgm` と `occupancy_grid_map.yaml` が生成されます。
失敗した場合は、docker と docker compose が正常にインストールされているか確認してください。

## ROS 2 で Occupancy Grid Map を使用する

ここでは簡単のため、 `aichallenge_submit_launch` パッケージを編集する前提で話を進めます。
ご使用の環境に合わせて適宜読み替えてください。

まず、上記で生成した `occupancy_grid_map.pgm` と `occupancy_grid_map.yaml` ファイルを`aichallenge_submit_launch/map/` 下に配置します。

次に以下のような launch ファイルを `aichallenge_submit_launch/launch/map_server.launch.py` に作成します。

```python
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter, PushRosNamespace

def generate_launch_description():
    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    map_yaml_file = LaunchConfiguration('map_yaml_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    lifecycle_nodes = ['map_server']

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the map server')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map yaml file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the map server stack')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')

    # Specify the actions
    bringup_cmd_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),

        SetParameter("use_sim_time", use_sim_time),
        Node(
            package='nav2_map_server',
            executable='map_server',
            output='both',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[{'yaml_filename': map_yaml_file}],
            arguments=['--ros-args', '--log-level', log_level],
            ),

        Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager',
                output='both',
                emulate_tty=True,  # https://github.com/ros2/launch/issues/188
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'autostart': autostart},
                            {'node_names': lifecycle_nodes}])
    ])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group)

    return ld
```

このlaunch内では、 `nav2_map_server` と `nav2_lifecycle_manager` を使用していますが、
aichallenge2024の標準Docker環境ではインストールされていないため、 `aichallenge_submit_launch/package.xml` に以下の記述を追加しておきます。
```xml
  <exec_depend>nav2_map_server</exec_depend>
  <exec_depend>nav2_lifecycle_manager</exec_depend>
```

この状態で、aichallenge2024のコンテナ内で以下のコマンドを実行することで自動的に上記２つのパッケージがインストールされるかと思います。
```
source /autoware/install/setup.bash
cd /aichallenge/workspace
rosdep update
rosdep install -y -r -i --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

また、配置したOccupancy Grid Map や launch ファイルは、そのままでは colcon workspace の install ディレクトリにインストールされていないので、一度 `colcon build` を実行してください。

その後、 `run_evaluation.bash` などを実行した後、以下のコマンドで `map_server` を実行します。
```
ros2 launch aichallenge_submit_launch map_server.launch.py map_yaml_file:=$(ros2 pkg prefix --share aichallenge_submit_launch)/map/occupancy_grid_map.yaml use_namespace:=true namespace:=/ogm
```
これにより、 `/ogm/map` topic にマップデータが publish されます。
`ros2 topic echo /ogm/map` などで正常にマップデータがpublishされているか確認してみてください。

また、rviz2 上で上記 topic を Add することで、Occupancy Grid Map を可視化することができます。
rviz2 上で下の画像のように "No map received" の Warning が出ている場合は、 `map_server` シャットダウンした後にもう一度起動すると正しく表示できるかと思います。

<img src="https://github.com/Roborovsky-Racers/RoborovskyNote/blob/main/AutomotiveAIChallenge/2024/.images/lanelet2_to_ogm/rviz_warn.png?raw=true" width="500px" />

<img src="https://github.com/Roborovsky-Racers/RoborovskyNote/blob/main/AutomotiveAIChallenge/2024/.images/lanelet2_to_ogm/map_display.png?raw=true" width="500px" />

この理由は、`map_server` は起動時に一度しかマップデータを publish せず、rviz2 がマップデータの subscribe を開始した時点では既にマップデータは publish されていないため、rviz2 がマップデータを受信できないためです。
そのため、 `map_server` を再起動すればもう一度マップデータが publish され、rviz2 上に表示されるようになります。

---
<img src="https://github.com/Roborovsky-Racers/RoborovskyNote/blob/main/.images/roborovsky_logo.png?raw=true" width="75" />

[Roborovsky Note トップページ](https://roborovsky-racers.github.io/RoborovskyNote/)
