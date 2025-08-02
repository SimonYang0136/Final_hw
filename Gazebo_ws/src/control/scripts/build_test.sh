#!/bin/bash
# 编译测试脚本

echo "=== 开始编译控制器包 ==="

# 进入工作空间
cd /home/simon/Desktop/Final/Final_hw/Gazebo_ws

# 清理之前的编译
echo "清理之前的编译文件..."
rm -rf build/ devel/

# 编译
echo "开始编译..."
catkin_make

# 检查编译结果
if [ $? -eq 0 ]; then
    echo "✅ 编译成功！"
    echo ""
    echo "生成的可执行文件："
    find devel/lib -name "box_controller" -type f 2>/dev/null || echo "❌ 未找到box_controller可执行文件"
    echo ""
    echo "使用方法："
    echo "  source devel/setup.bash"
    echo "  rosrun control box_controller"
    echo "  或者"
    echo "  roslaunch control box_controller.launch"
else
    echo "❌ 编译失败！请检查错误信息。"
    exit 1
fi
