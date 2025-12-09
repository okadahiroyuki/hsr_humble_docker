# hsr_patrol_smach/patrol_state_machine.py

import math
from typing import Dict, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

import smach

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


# ----------------------------------------
#  ユーティリティ：Yaw からクォータニオン生成
# ----------------------------------------
def yaw_to_quaternion_msg(yaw: float):
    """Yaw[rad] から geometry_msgs/Quaternion を作る（ロール・ピッチは0）。"""
    from geometry_msgs.msg import Quaternion

    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def make_pose_stamped(
    node: Node,
    x: float,
    y: float,
    yaw_deg: float,
    frame_id: str = "map",
) -> PoseStamped:
    """2D Pose（x, y, yaw_deg）から PoseStamped を生成."""
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = node.get_clock().now().to_msg()

    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0

    yaw_rad = math.radians(yaw_deg)
    pose.pose.orientation = yaw_to_quaternion_msg(yaw_rad)

    return pose


# ----------------------------------------
#  Nav2 NavigateToPose を叩くステート
# ----------------------------------------
class Nav2PatrolState(smach.State):
    """
    Nav2 の NavigateToPose アクションを呼び出す Smach ステート。

    outcomes:
        - 'succeeded' : ゴール到達
        - 'aborted'   : サーバ未起動 / ゴール拒否 / 失敗 など
    """

    def __init__(
        self,
        node: Node,
        room_name: str,
        goal_pose: PoseStamped,
        action_name: str = "navigate_to_pose",
        server_wait_timeout_sec: float = 5.0,
        nav_timeout_sec: float = 120.0,
    ):
        smach.State.__init__(self, outcomes=["succeeded", "aborted"])
        self._node = node
        self._room_name = room_name
        self._goal_pose = goal_pose
        self._action_name = action_name
        self._server_wait_timeout_sec = server_wait_timeout_sec
        self._nav_timeout_sec = nav_timeout_sec

        # NavigateToPose のアクションクライアント
        self._action_client = ActionClient(
            node, NavigateToPose, self._action_name
        )

    def execute(self, userdata):
        log = self._node.get_logger()
        log.info(f"[{self._room_name}] へ移動開始します")

        # 1. サーバ待ち
        if not self._action_client.wait_for_server(
            timeout_sec=self._server_wait_timeout_sec
        ):
            log.error(
                f"Nav2 アクションサーバ '{self._action_name}' が見つかりません"
            )
            return "aborted"

        # 2. Goal メッセージ作成
        goal_msg = NavigateToPose.Goal()
        # 時刻を更新しておく
        self._goal_pose.header.stamp = self._node.get_clock().now().to_msg()
        goal_msg.pose = self._goal_pose

        # 3. ゴール送信
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self._node, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            log.error(f"[{self._room_name}] へのゴールが Nav2 に拒否されました")
            return "aborted"

        log.info(f"[{self._room_name}] へのゴールが Nav2 に受理されました")

        # 4. 結果待ち
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self._node, get_result_future, timeout_sec=self._nav_timeout_sec
        )

        if not get_result_future.done():
            log.error(
                f"[{self._room_name}] への移動がタイムアウトしました"
            )
            # キャンセルするならここで goal_handle.cancel_goal_async()
            return "aborted"

        result = get_result_future.result()
        status = result.status  # GoalStatus の enum
        # result.result には Nav2 側の結果メッセージが入っている

        if status == GoalStatus.STATUS_SUCCEEDED:
            log.info(f"[{self._room_name}] に到着しました 🎉")
            return "succeeded"
        else:
            log.error(
                f"[{self._room_name}] への移動に失敗しました (status={status})"
            )
            return "aborted"


# ----------------------------------------
#  ノード定義 ＋ ステートマシン本体
# ----------------------------------------
class PatrolNode(Node):
    def __init__(self):
        super().__init__("hsr_patrol_smach")


def main():
    rclpy.init()
    node = PatrolNode()

    # ===============================
    # 巡回する部屋のゴール座標定義
    # （★実環境に合わせて書き換えてください）
    # ===============================
    # 例: x[m], y[m], yaw[deg] （map座標系）
    room_goals: Dict[str, Tuple[float, float, float]] = {
        "Room1": (1.0, 0.0, 0.0),
        "Room2": (1.0, 2.0, 90.0),
        "Room3": (0.0, 2.0, 180.0),
    }

    # PoseStamped に変換
    room_goal_poses: Dict[str, PoseStamped] = {
        name: make_pose_stamped(node, x, y, yaw_deg, frame_id="map")
        for name, (x, y, yaw_deg) in room_goals.items()
    }

    # ステートマシン定義
    sm = smach.StateMachine(outcomes=["PATROL_DONE", "PATROL_ABORTED"])

    with sm:
        # Room1
        smach.StateMachine.add(
            "ROOM1",
            Nav2PatrolState(
                node,
                "Room1",
                room_goal_poses["Room1"],
            ),
            transitions={
                "succeeded": "ROOM2",
                "aborted": "PATROL_ABORTED",
            },
        )

        # Room2
        smach.StateMachine.add(
            "ROOM2",
            Nav2PatrolState(
                node,
                "Room2",
                room_goal_poses["Room2"],
            ),
            transitions={
                "succeeded": "ROOM3",
                "aborted": "PATROL_ABORTED",
            },
        )

        # Room3
        smach.StateMachine.add(
            "ROOM3",
            Nav2PatrolState(
                node,
                "Room3",
                room_goal_poses["Room3"],
            ),
            transitions={
                "succeeded": "PATROL_DONE",
                "aborted": "PATROL_ABORTED",
            },
        )

    node.get_logger().info("Nav2 を使った部屋巡回ステートマシンを開始します")

    outcome = sm.execute()
    node.get_logger().info(f"巡回終了 outcome = {outcome}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
