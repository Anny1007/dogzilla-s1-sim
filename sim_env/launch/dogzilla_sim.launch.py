"""DOGZILLA S1 自主避障模擬 — 一鍵啟動 project.pptx 的完整軟體管線 (容器內使用)

    LiDAR (/scan) --> 地面點過濾 (/scan_filtered) --> SLAM Toolbox (map + 位姿) --> Nav2 全域路徑規劃
                                                        |
                          controller:=dwb / rpp  -> Nav2 局部控制器 --\
                          controller:=rl         -> RL 避障控制器   ----> /cmd_vel_raw --> 運動安全濾波 --> /cmd_vel --> CHAMP 步態控制 --> 12 顆關節
                                                                                        (Gazebo 裡的 DOGZILLA)

用法:
    ros2 launch /launch/dogzilla_sim.launch.py                       # 預設 DWB + Gazebo 畫面 + RViz
    ros2 launch /launch/dogzilla_sim.launch.py controller:=rl        # RL 局部避障 (Nav2 只負責全域路徑)
    ros2 launch /launch/dogzilla_sim.launch.py gui:=false rviz:=false  # 無頭模式 (跑 benchmark 用)

目標點: 在 RViz 用「2D Goal Pose」點一下即可 (三種 controller 模式都支援)。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# model:=s1      Yahboom 官方 DOGZILLA S1 URDF (預設; 由 description/make_s1_official.py 產生, 修正內容見該檔)
# model:=s1_box  依官方尺寸手工建的方塊模型 (舊版, 對照用)
# model:=champ   舊的 CHAMP/XGO 放大模型 (約 S1 的 2 倍大, 對照用)
MODELS = {
    's1':     dict(description='/description/dogzilla_s1_official.urdf.xacro', gait='/config/gait_s1.yaml',
                   spawn_z='0.10', scan_height='0.18',
                   # 官方模型: laser_link 在 base_link x=-0.06 (make_s1_official.py 的 LIDAR_X); 機身 x -0.106~0.103, 髖+腿外緣 |y| 約 0.075
                   # LiDAR 在 x=-0.0167 時是 [-0.10, 0.16, 0.08] (實測機身回波在 x 0.116~0.147 m, 漏網的點會讓 RL 安全保護
                   # 以為正前方有障礙物而卡住); LiDAR 後移 0.043 m, 前後緣跟著移
                   self_box='[-0.06, 0.21, 0.08]'),
    's1_box': dict(description='/description/dogzilla_s1.urdf.xacro', gait='/config/gait_s1.yaml',
                   spawn_z='0.125', scan_height='0.16',
                   self_box='[-0.12, 0.16, 0.08]'),   # 機身+髖關節+腿 (laser_link 座標), 見 scan_ground_filter.py
    'champ': dict(description='/description/dogzilla_sim.urdf.xacro', gait=None,
                  spawn_z='0.3', scan_height='0.39', self_box='[0.0, 0.0, 0.0]'),
}


def pick(model, key, default=None):
    """依 model 參數在 launch 時取出 MODELS 裡的值 (PythonExpression 版的 MODELS[model][key])。"""
    table = {m: (v[key] if v[key] is not None else default) for m, v in MODELS.items()}
    return PythonExpression([repr(table), "['", model, "']"])


def generate_launch_description():
    world = LaunchConfiguration('world')
    controller = LaunchConfiguration('controller')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    model = LaunchConfiguration('model')
    is_rl = PythonExpression(["'", controller, "' == 'rl'"])
    params_file = PythonExpression(["'/nav2_params_rpp.yaml' if '", controller, "' == 'rpp' else '/nav2_params.yaml'"])

    champ_config = FindPackageShare('champ_config')
    champ_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('champ_bringup'), 'launch', 'bringup.launch.py'])),
        launch_arguments={
            'description_path': pick(model, 'description'),
            'joints_map_path': PathJoinSubstitution([champ_config, 'config/joints/joints.yaml']),
            'links_map_path': PathJoinSubstitution([champ_config, 'config/links/links.yaml']),
            'gait_config_path': pick(model, 'gait', '/ws/install/champ_config/share/champ_config/config/gait/gait.yaml'),
            'use_sim_time': 'true',
            'robot_name': 'champ',
            'gazebo': 'true',
            'rviz': 'false',
            'joint_controller_topic': 'joint_group_effort_controller/joint_trajectory',
            'hardware_connected': 'false',
            'publish_foot_contacts': 'false',   # gazebo:=true 時控制器不發佈, 由下方 contact_sensor 提供
            'close_loop_odom': 'true',
        }.items(),
    )
    # Gazebo (取代 champ_gazebo/launch/gazebo.launch.py, 流程相同, 差別只在「暫停啟動」):
    # S1 舵機只有 0.44 N·m, 控制器載入前腿是軟的, 物理一跑機身就趴下而且撐不起來。
    # 所以 gzserver 以 -u (暫停) 啟動, 以站姿生成機器狗 (xacro 內 initial_value), 控制器設定好後才 unpause。
    gz_params = '/ws/install/champ_gazebo/share/champ_gazebo/config/gazebo.yaml'
    gazebo = [
        ExecuteProcess(cmd=['gzserver', '-u', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so',
                            world, '--ros-args', '--params-file', gz_params], output='screen'),
        ExecuteProcess(cmd=['gzclient'], output='screen', condition=IfCondition(gui)),
        Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
             arguments=['-entity', 'champ', '-topic', '/robot_description', '-robot_namespace', '',
                        '-x', '0.0', '-y', '0.0', '-z', pick(model, 'spawn_z'), '-R', '0', '-P', '0', '-Y', '0.0']),
        ExecuteProcess(cmd=['ros2', 'control', 'load_controller', '--set-state', 'inactive', 'joint_states_controller'],
                       output='screen'),
        ExecuteProcess(cmd=['ros2', 'control', 'load_controller', '--set-state', 'inactive', 'joint_group_effort_controller'],
                       output='screen'),
        # (list_controllers 輸出帶 ANSI 顏色碼, 不能用 grep -w 比對)
        # 控制器切換要靠模擬更新迴圈, 暫停時送出會卡住直到逾時 (5 s)。所以先在背景送出啟動請求,
        # 再 unpause: 物理一開始跑的第一個更新就完成切換, 腿只軟掉幾毫秒, 而機器狗是以站姿生成的。
        ExecuteProcess(output='screen', cmd=['bash', '-c',
            'until [ "$(ros2 control list_controllers 2>/dev/null | grep -c inactive)" -ge 2 ]; do sleep 0.5; done; '
            '(ros2 control switch_controllers --activate joint_states_controller joint_group_effort_controller &); '
            'sleep 1; ros2 service call /unpause_physics std_srvs/srv/Empty > /dev/null; sleep 3; '
            'n=$(ros2 control list_controllers 2>/dev/null | grep -v inactive | grep -c active); '
            'echo "[unpause] 開始物理模擬, active 控制器數 = $n (應為 2)"']),
    ]

    # 腳掌接觸感測 (/foot_contacts): state_estimation 需要它才能算腿部里程計 /odom/raw。
    # champ_gazebo 的 launch 把它註解掉了 (懷疑它造成 gzserver 崩潰, 實際元兇是 trimesh 碰撞, 見 description/),
    # 少了它 /odom 速度恆為 0, SLAM 只能純靠掃描比對。
    contact_sensor = Node(package='champ_gazebo', executable='contact_sensor', output='screen',
                          parameters=[{'use_sim_time': True}, PathJoinSubstitution([champ_config, 'config/links/links.yaml'])])

    # LiDAR 地面點過濾: 步態晃動讓掃描平面打到地面, 依 IMU 傾角濾掉 -> /scan_filtered (SLAM / Nav2 / RL 都用它)
    scan_filter = Node(executable='/launch/scan_ground_filter.py', name='scan_ground_filter', output='screen',
                       parameters=[{'scan_height': ParameterValue(pick(model, 'scan_height'), value_type=float),
                                    'self_box': ParameterValue(pick(model, 'self_box'))}])

    # SLAM: LiDAR + 里程計 -> /map 與 map->odom
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'])),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': '/config/slam.yaml',   # 模擬用設定, 見檔內說明
        }.items(),
    )

    # Nav2 (DWB / RPP): 完整導航堆疊, 規劃 + 局部控制 + 恢復行為
    nav2_full = IncludeLaunchDescription(
        PythonLaunchDescriptionSource('/launch/nav2_navigation.launch.py'),   # 只改了輸出話題 -> /cmd_vel_raw
        launch_arguments={'use_sim_time': 'true', 'params_file': params_file, 'autostart': 'true'}.items(),
        condition=UnlessCondition(is_rl),
    )
    # RL 模式: Nav2 只開 planner_server 提供全域路徑, 局部避障與速度指令全交給 RL 節點;
    # 不開 bt_navigator / controller_server, 避免兩邊同時搶 /cmd_vel。
    nav2_planner_only = [
        Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen',
             parameters=['/nav2_params.yaml', {'use_sim_time': True}], condition=IfCondition(is_rl)),
        # lifecycle_manager 晚 3 秒開: 與 planner_server 同時啟動時, FastDDS 常在服務用戶端配對前就回應
        # change_state 而逾時 ("failed to send response ... timeout"), manager 就永遠卡在 Configuring。
        TimerAction(period=3.0, actions=[
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation',
                 output='screen', condition=IfCondition(is_rl),
                 parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': ['planner_server']}])]),
        Node(executable='/rl/rl_controller.py', name='rl_controller', output='screen', condition=IfCondition(is_rl),
             parameters=[{'policy': LaunchConfiguration('rl_policy'), 'shield': LaunchConfiguration('rl_shield'),
                         'vel_obs': LaunchConfiguration('rl_vel_obs')}],
             remappings=[('/scan', '/scan_filtered'), ('/cmd_vel', '/cmd_vel_raw')]),
    ]

    # Nav2 看門狗：偶爾 lifecycle_manager 啟動時某個節點 (如 velocity_smoother) 2 秒內沒回應,
    # 就會 "Aborting bringup" 整組停在未啟動狀態。啟動後檢查一次, 沒 active 就再下 STARTUP。
    nav2_watchdog = ExecuteProcess(output='screen', cmd=['bash', '-c',
        'timeout 15 ros2 service call /lifecycle_manager_navigation/is_active std_srvs/srv/Trigger | grep -q "success=True" '
        '&& echo "[nav2_watchdog] Nav2 active" '
        '|| { echo "[nav2_watchdog] Nav2 未啟動, 重新 STARTUP"; '
        'timeout 60 ros2 service call /lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 0}"; }'])

    # 運動安全濾波 /cmd_vel_raw -> /cmd_vel: 轉彎降速、限制轉向變化率 (見 cmd_vel_safety.py, DWB 邊走邊急轉曾把 S1 甩翻)
    cmd_vel_safety = Node(executable='/launch/cmd_vel_safety.py', name='cmd_vel_safety', output='screen')

    rviz_node = Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
                     arguments=['-d', '/launch/dogzilla_sim.rviz'], parameters=[{'use_sim_time': True}],
                     condition=IfCondition(rviz))

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='/worlds/corridor.world',
                              description='/worlds/corridor.world (窄通道 + 死巷, 預設) | /worlds/clutter.world (密集障礙物) | /worlds/project3.world (project.pptx 第 1 頁場景) | /worlds/obstacle_test.world (舊場地)'),
        DeclareLaunchArgument('controller', default_value='dwb', description='dwb | rpp | rl'),
        DeclareLaunchArgument('rl_policy', default_value='/rl/models/policy.npz',
                              description='controller:=rl 時使用的權重檔, 例 /rl/models/my_run/best/policy.npz'),
        DeclareLaunchArgument('rl_shield', default_value='auto',
                              description='RL 安全保護: auto (依權重檔) | on | off'),
        DeclareLaunchArgument('rl_vel_obs', default_value='model',
                              description='RL 觀測裡的目前速度: model (由指令推算, 與訓練一致) | odom (腿部里程計, 會隨步伐擺動)'),
        DeclareLaunchArgument('gui', default_value='true', description='開 Gazebo 畫面'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('model', default_value='s1', description='s1 (官方 URDF) | s1_box | champ'),
        # 用 GroupAction 限定作用域: Humble 的 IncludeLaunchDescription 參數會洩漏到上層,
        # 否則傳給 champ_bringup 的 rviz:=false 會蓋掉本檔的 rviz 參數, RViz 永遠不會開。
        GroupAction([champ_bringup]),
        *gazebo,
        contact_sensor,
        scan_filter,
        cmd_vel_safety,
        # 等機器狗站穩、控制器啟動後再開 SLAM / 導航
        TimerAction(period=12.0, actions=[slam]),
        TimerAction(period=18.0, actions=[nav2_full, *nav2_planner_only]),
        TimerAction(period=35.0, actions=[nav2_watchdog]),
        rviz_node,
    ])
