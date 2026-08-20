#!/usr/bin/env python3
"""
px4_ros2_check.py - 检查 PX4 ↔ ROS 2 (uXRCE-DDS) 链路状态

用法：
    # 1) 先启动 MicroXRCEAgent
    # 2) 在 Pixhawk 上确保 uxrce_dds_client 已启动
    # 3) 运行本脚本

期望输出：
    - 显示 /fmu/out/* 话题列表
    - 显示每个话题的发布速率
"""
import subprocess
import sys
import time


def run(cmd: str):
    return subprocess.run(cmd, shell=True, capture_output=True, text=True)


def main():
    print('检查 ROS 2 环境 ...')
    out = run('ros2 topic list 2>&1 | grep -i fmu')
    if not out.stdout:
        print('❌ 没有 /fmu/* 话题！请检查：')
        print('   1) MicroXRCEAgent 是否已启动？')
        print('   2) Pixhawk 6C 端 uxrce_dds_client 是否已运行？')
        print('   3) 两端 IP/端口是否匹配？')
        sys.exit(1)

    print('✅ 找到以下 PX4 话题：')
    for line in out.stdout.strip().splitlines():
        print(f'   - {line}')

    print()
    print('测量话题发布速率（3 秒采样）...')
    topics = ['/fmu/out/vehicle_odometry',
              '/fmu/out/sensor_combined',
              '/fmu/out/vehicle_status']
    for topic in topics:
        out = run(f"timeout 3 ros2 topic hz {topic} 2>&1 | head -20")
        rate = '?'
        for line in out.stdout.splitlines():
            if 'average rate' in line:
                rate = line.split(':')[-1].strip()
        print(f'   {topic:<40} rate = {rate}')


if __name__ == '__main__':
    main()