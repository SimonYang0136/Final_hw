#!/bin/bash
# Box Controller 参数调试脚本
# 用法: ./tune_params.sh [参数名] [参数值]

echo "=== Box Controller 参数调试工具 ==="

if [ $# -eq 0 ]; then
    echo "当前参数值："
    echo "constant_linear_velocity: $(rosparam get /constant_linear_velocity)"
    echo "lookahead_distance: $(rosparam get /lookahead_distance)"
    echo "pid_kp: $(rosparam get /pid_kp)"
    echo "pid_ki: $(rosparam get /pid_ki)"
    echo "pid_kd: $(rosparam get /pid_kd)"
    echo "max_angular_velocity: $(rosparam get /max_angular_velocity)"
    echo "integral_limit: $(rosparam get /integral_limit)"
    echo ""
    echo "使用方法："
    echo "  $0 pid_kp 1.5        # 设置比例增益为1.5"
    echo "  $0 constant_linear_velocity 0.8  # 设置线速度为0.8"
    echo ""
    echo "可调参数："
    echo "  constant_linear_velocity  - 恒定线速度"
    echo "  lookahead_distance       - 前视距离"  
    echo "  pid_kp                   - PID比例增益"
    echo "  pid_ki                   - PID积分增益"
    echo "  pid_kd                   - PID微分增益"
    echo "  max_angular_velocity     - 最大角速度"
    echo "  integral_limit           - 积分限幅值"
elif [ $# -eq 2 ]; then
    param_name=$1
    param_value=$2
    echo "设置参数 $param_name = $param_value"
    rosparam set /$param_name $param_value
    echo "参数已更新！重启控制器节点以生效："
    echo "  rosnode kill /box_controller"
    echo "  roslaunch control box_controller.launch"
else
    echo "错误：参数数量不正确"
    echo "用法: $0 [参数名] [参数值]"
fi
