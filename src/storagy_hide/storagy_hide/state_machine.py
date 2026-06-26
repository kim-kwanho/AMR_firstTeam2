#!/usr/bin/env python3
"""R1 — 위장 상태제어 FSM (숨는팀 통합 허브).

상태: FREEZE → WAKE → GUIDE → RETURN → DOCK → FREEZE

담당(R1):
  - SetLamp(LED) / Emotion(OLED) 서비스 서버 구현
  - FREEZE 진입 시 모터 잠금(/cmd_vel 0) + 안내팀 주행 양보(/wander_enabled=false) + LED/OLED off
  - 안내팀 신호(/hide/takeover_start, /hide/mission_done)와 R2 도킹 완료(/hide/dock_done) 구독
  - 현재 상태를 /hide/state 로 발행(안내팀 대시보드 연동)

개발 중에는 안내팀 신호가 없어도 더미 publisher 로 테스트:
  ros2 topic pub --once /hide/takeover_start std_msgs/Bool "{data: true}"
  ros2 topic pub --once /hide/mission_done   std_msgs/Bool "{data: true}"
"""
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker # 시각화 토픽추가

from storagy_interfaces.srv import SetLamp, Emotion


class State(Enum):
    FREEZE = 'FREEZE'   # 위장: 모터 잠금 / LED·OLED off
    WAKE = 'WAKE'       # 기상: LED·OLED 점등
    GUIDE = 'GUIDE'     # 안내팀이 주행 제어 (숨는팀은 사람감지만)
    RETURN = 'RETURN'   # 구석 대기석으로 복귀 (R2 Nav2)
    DOCK = 'DOCK'       # ArUco 정밀 도킹 (R2)


class HideStateMachine(Node):
    def __init__(self):
        super().__init__('hide_state_machine')

        latched = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE, depth=1)

        # --- 발행 ---
        self.state_pub = self.create_publisher(String, '/hide/state', latched)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.wander_pub = self.create_publisher(Bool, '/wander_enabled', latched)
        # --- 시각화 토픽 추가 ---
        self.led_pub = self.create_publisher(Marker, '/hide/led_marker', 1)
        self.oled_pub = self.create_publisher(String, '/hide/oled_text', 1)

        # --- 구독 (안내팀 인계 신호 + R2 도킹 완료) ---
        self.create_subscription(Bool, '/hide/takeover_start', self._on_takeover, 10)
        self.create_subscription(Bool, '/hide/mission_done', self._on_mission_done, 10)
        self.create_subscription(Bool, '/hide/dock_done', self._on_dock_done, 10)

        # --- 서비스 서버 (LED / OLED) ---
        self.lamp_srv = self.create_service(SetLamp, '/hide/set_lamp', self._on_set_lamp)
        self.emotion_srv = self.create_service(Emotion, '/hide/set_emotion', self._on_set_emotion)

        self.state = State.FREEZE
        self._enter_freeze()

        # 0.5s 주기로 현재 상태 발행 + 동작 유지
        self.create_timer(0.5, self._loop)
        self.get_logger().info('R1 HideStateMachine 시작 (초기 상태: FREEZE)')

    # ------------------------------------------------------------------ 전이
    def _set_state(self, state: State):
        if state != self.state:
            self.get_logger().info(f'상태 전이: {self.state.value} -> {state.value}')
        self.state = state

    def _on_takeover(self, msg: Bool):
        if msg.data and self.state == State.FREEZE:
            self._wake()

    def _on_mission_done(self, msg: Bool):
        if msg.data and self.state in (State.GUIDE, State.WAKE):
            self._return_to_hideout()

    def _on_dock_done(self, msg: Bool):
        if msg.data and self.state in (State.DOCK, State.RETURN):
            self._enter_freeze()

    # ------------------------------------------------------------- 상태 동작
    def _enter_freeze(self):
        self._set_state(State.FREEZE)
        self._yield_navigation()          # 안내팀 배회/주행 끄기
        self.cmd_pub.publish(Twist())     # 모터 잠금(정지)
        self._lamp_off()
        self._oled_idle()

    def _wake(self):
        self._set_state(State.WAKE)
        self._lamp_on()
        self._oled_wake()
        # TODO(R1): 입구로 Nav2 이동 트리거 or 안내팀에 제어 위임 후 GUIDE 전이
        self._set_state(State.GUIDE)

    def _return_to_hideout(self):
        self._set_state(State.RETURN)
        # TODO(R2 연동): hideout 좌표로 Nav2 goal 발행. 근접 시 DOCK 전이.
        self._set_state(State.DOCK)

    # ------------------------------------------------------------ LED / OLED
    def _on_set_lamp(self, req, resp):
        # TODO(R1): 시뮬 표현(Marker/로그) or HW(MCU 시리얼)로 매핑
        self.get_logger().info(
            f'[SetLamp] mode={req.mode} rgba=({req.color.r:.2f},{req.color.g:.2f},'
            f'{req.color.b:.2f},{req.color.a:.2f}) time={req.time}')
        
        if req.mode == 0:
            self._publish_led(0.0, 0.0, 0.0, 0.0)
        else:
            self._publish_led(req.color.r, req.color.g, req.color.b, req.color.a)
    
        resp.result = True
        return resp
    
    def _publish_led(self, r, g, b, a=1.0):
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'hide_led'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.z = 0.5
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.15
        marker.scale.y = 0.15
        marker.scale.z = 0.15
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a
        self.led_pub.publish(marker)

    def _on_set_emotion(self, req, resp):
        # TODO(R1): OLED 표정 표시(시뮬 이미지 토픽 or HW)
        self.oled_pub.publish(String(data=req.emotion))
        self.get_logger().info(f'[Emotion] emotion={req.emotion}')
        resp.response = f'ok:{req.emotion}'
        return resp
    
    def _lamp_off(self):
        self._publish_led(0.0, 0.0, 0.0, 0.0)
        self.get_logger().info('LED OFF (위장)', throttle_duration_sec=5.0)

    def _lamp_on(self):
        self._publish_led(0.0, 1.0, 0.0, 1.0)
        self.get_logger().info('LED ON (기상)')

    def _oled_idle(self):
       # self.get_logger().info('OLED 인형 눈/정지화면 (위장)', throttle_duration_sec=5.0)
        self.oled_pub.publish(String(data='doll_eyes'))

    def _oled_wake(self):
      #  self.get_logger().info('OLED 점등 (기상)')
        self.oled_pub.publish(String(data='awake'))

    # --------------------------------------------------------------- 주행 양보
    def _yield_navigation(self):
        msg = Bool()
        msg.data = False
        self.wander_pub.publish(msg)

    # ------------------------------------------------------------------- loop
    def _loop(self):
        self.state_pub.publish(String(data=self.state.value))
        if self.state == State.FREEZE:
            self.cmd_pub.publish(Twist())  # 위장 중 모터 잠금 유지


def main(args=None):
    rclpy.init(args=args)
    node = HideStateMachine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
