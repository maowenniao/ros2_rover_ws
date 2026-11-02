#!/usr/bin/env python3
"""浏览器遥控。只向 ROS 发布 /cmd_vel、/estop、/odrive_enable，不占用 CAN。

默认只监听 127.0.0.1。需要局域网访问时显式传 --host，并放在受信任网络里。
"""
from __future__ import annotations

import argparse
import threading
import time

import rclpy
from flask import Flask, jsonify, render_template, request
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from rover_description.geometry import declare_geometry

app = Flask(__name__)
_node = None
_lock = threading.Lock()


def _latched() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


class WebTeleopNode(Node):
    def __init__(self) -> None:
        super().__init__('web_controller')
        self.declare_parameter('command_topic', '/cmd_vel')
        self.declare_parameter('publish_hz', 10.0)
        self.declare_parameter('command_timeout', 0.4)
        declare_geometry(self)
        topic = str(self.get_parameter('command_topic').value)
        self.max_linear_vel = float(self.get_parameter('max_linear_vel').value)
        self.max_angular_vel = float(self.get_parameter('max_angular_vel').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.track_width = float(self.get_parameter('track_width').value)
        self.command_timeout = float(self.get_parameter('command_timeout').value)
        self._pub = self.create_publisher(Twist, topic, 10)
        self._estop_pub = self.create_publisher(Bool, '/estop', _latched())
        self._enable_pub = self.create_publisher(Bool, '/odrive_enable', _latched())
        self.create_subscription(JointState, '/wheel_state', self._on_state, 10)
        self._vx = 0.0
        self._wz = 0.0
        self._last_http = 0.0
        self._wheels = {}
        self.create_timer(1.0 / float(self.get_parameter('publish_hz').value), self._tick)
        self.get_logger().info(f'网页遥控发布 {topic}，不直接访问 CAN')

    def set_velocity(self, vx: float, wz: float) -> None:
        vx = max(-self.max_linear_vel, min(self.max_linear_vel, float(vx)))
        wz = max(-self.max_angular_vel, min(self.max_angular_vel, float(wz)))
        with _lock:
            self._vx = vx
            self._wz = wz
            self._last_http = time.monotonic()

    def stop(self) -> None:
        with _lock:
            self._vx = 0.0
            self._wz = 0.0
            self._last_http = 0.0

    def estop(self) -> None:
        self.stop()
        self._estop_pub.publish(Bool(data=True))
        self._enable_pub.publish(Bool(data=False))

    def enable_motors(self) -> None:
        self._estop_pub.publish(Bool(data=False))
        self._enable_pub.publish(Bool(data=True))

    def snapshot(self) -> dict:
        with _lock:
            wheels = dict(self._wheels)
            age = None if self._last_http == 0.0 else time.monotonic() - self._last_http
        return {
            'wheels': wheels,
            'command_age': age,
            'max_linear_vel': self.max_linear_vel,
            'max_angular_vel': self.max_angular_vel,
            'wheel_radius': self.wheel_radius,
            'track_width': self.track_width,
        }

    def _on_state(self, msg: JointState) -> None:
        wheels = {}
        for index, name in enumerate(msg.name):
            wheels[name] = {
                'position': float(msg.position[index]) if index < len(msg.position) else 0.0,
                'velocity': float(msg.velocity[index]) if index < len(msg.velocity) else 0.0,
            }
        with _lock:
            self._wheels = wheels

    def _tick(self) -> None:
        with _lock:
            fresh = self._last_http > 0.0 and (time.monotonic() - self._last_http) <= self.command_timeout
            vx = self._vx if fresh else 0.0
            wz = self._wz if fresh else 0.0
        msg = Twist()
        msg.linear.x = vx
        msg.angular.z = wz
        self._pub.publish(msg)


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/api/limits')
def limits():
    if _node is None:
        return jsonify({'error': 'ROS 节点未就绪'}), 503
    snap = _node.snapshot()
    return jsonify({
        'max_linear_vel': snap['max_linear_vel'],
        'max_angular_vel': snap['max_angular_vel'],
        'wheel_radius': snap['wheel_radius'],
        'track_width': snap['track_width'],
    })


@app.route('/api/status')
def status():
    if _node is None:
        return jsonify({'error': 'ROS 节点未就绪'}), 503
    return jsonify(_node.snapshot()['wheels'])


@app.route('/api/velocity', methods=['POST'])
def velocity():
    if _node is None:
        return jsonify({'error': 'ROS 节点未就绪'}), 503
    data = request.get_json(force=True, silent=True) or {}
    _node.set_velocity(float(data.get('vx', 0.0)), float(data.get('wz', 0.0)))
    return jsonify({'status': 'ok'})


@app.route('/api/stop', methods=['POST'])
def stop():
    if _node is None:
        return jsonify({'error': 'ROS 节点未就绪'}), 503
    _node.stop()
    return jsonify({'status': 'stopped'})


@app.route('/api/emergency_stop', methods=['POST'])
def emergency_stop():
    if _node is None:
        return jsonify({'error': 'ROS 节点未就绪'}), 503
    _node.estop()
    return jsonify({'status': 'estop'})


@app.route('/api/enable', methods=['POST'])
def enable():
    if _node is None:
        return jsonify({'error': 'ROS 节点未就绪'}), 503
    _node.enable_motors()
    return jsonify({'status': 'enabled'})


@app.route('/api/mode', methods=['POST'])
def mode():
    data = request.get_json(force=True, silent=True) or {}
    if data.get('mode') == 'position':
        return jsonify({'error': '位置模式未实现，请使用速度模式'}), 400
    return jsonify({'status': 'ok', 'mode': 'velocity'})


def main() -> None:
    global _node
    parser = argparse.ArgumentParser(description='发布 /cmd_vel 的网页遥控')
    parser.add_argument('--host', default='127.0.0.1', help='默认只监听本机')
    parser.add_argument('--port', '-p', type=int, default=5000)
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    _node = WebTeleopNode()
    executor = SingleThreadedExecutor()
    executor.add_node(_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    _node.get_logger().info(f'网页控制 http://{args.host}:{args.port}')
    try:
        app.run(host=args.host, port=args.port, threaded=True, use_reloader=False)
    finally:
        _node.stop()
        executor.shutdown()
        _node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
