# Box Controller 参数服务器使用说明

## 参数文件
配置文件位置：`config/box_controller_params.yaml`

## 主要参数说明

### 运动参数
- `constant_linear_velocity`: 恒定线速度 (m/s)，默认0.5
- `lookahead_distance`: 前视距离 (m)，默认1.0

### PID控制器参数
- `pid_kp`: 比例增益，默认1.0
- `pid_ki`: 积分增益，默认0.1  
- `pid_kd`: 微分增益，默认0.05

### 高级参数
- `max_angular_velocity`: 最大角速度限制 (rad/s)，默认2.0
- `integral_limit`: 积分限幅值，默认1.0
- `path_points_limit`: 路径点数量限制，默认200
- `debug_frequency`: 调试信息打印频率，默认20

## 使用方法

### 1. 启动控制器（自动加载参数）
```bash
roslaunch control box_controller.launch
```

### 2. 实时调参
```bash
# 查看当前参数
./scripts/tune_params.sh

# 修改PID参数
./scripts/tune_params.sh pid_kp 1.5
./scripts/tune_params.sh pid_ki 0.2
./scripts/tune_params.sh pid_kd 0.1

# 修改运动参数
./scripts/tune_params.sh constant_linear_velocity 0.8
./scripts/tune_params.sh lookahead_distance 1.2
```

### 3. 直接使用rosparam命令
```bash
# 查看参数
rosparam get /pid_kp

# 设置参数
rosparam set /pid_kp 1.5

# 重启节点使参数生效
rosnode kill /box_controller
roslaunch control box_controller.launch
```

### 4. 修改配置文件（永久生效）
编辑 `config/box_controller_params.yaml` 文件，然后重启launch文件。

## 调参建议

1. **比例增益 (Kp)**: 控制响应速度，过大会震荡
2. **积分增益 (Ki)**: 消除稳态误差，过大会超调
3. **微分增益 (Kd)**: 减少超调，提高稳定性
4. **前视距离**: 影响转弯预判，过小反应迟钝，过大转弯不够
5. **线速度**: 影响整体行驶速度

## 注意事项
- 修改参数后需要重启控制器节点才能生效
- 建议先在仿真环境中调试参数
- 记录好的参数组合，避免重复调试
