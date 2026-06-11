from tello_driver.mission_node import MissionNode


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


def make_mission_node():
    node = MissionNode.__new__(MissionNode)
    node.target_class_name = 'landing_base'
    node.align_yaw_gain = 25.0
    node.max_yaw_cmd = 20.0
    node.base_map = []
    return node


def test_compute_yaw_cmd_uses_visual_servo_sign_and_clamps():
    node = make_mission_node()

    assert node._compute_yaw_cmd(0.4) == 10.0
    assert node._compute_yaw_cmd(-0.4) == -10.0
    assert node._compute_yaw_cmd(2.0) == 20.0
    assert node._compute_yaw_cmd(-2.0) == -20.0


def test_select_base_from_map_prefers_target_class_and_best_score():
    node = make_mission_node()
    node.base_map = [
        {
            'id': 'base_1',
            'class_name': 'takeoff_base',
            'best_confidence': 0.99,
            'best_area_ratio': 0.80,
            'observations': 20,
        },
        {
            'id': 'base_2',
            'class_name': 'landing_base',
            'best_confidence': 0.80,
            'best_area_ratio': 0.20,
            'observations': 2,
        },
        {
            'id': 'base_3',
            'class_name': 'landing_base',
            'best_confidence': 0.90,
            'best_area_ratio': 0.15,
            'observations': 1,
        },
    ]

    selected = node._select_base_from_map()

    assert selected['id'] == 'base_3'


def test_publish_or_preview_cmd_does_not_publish_in_dry_run():
    node = make_mission_node()
    node.dry_run = True
    node.cmd_vel_pub = FakePublisher()
    cmd = MissionNode._build_twist(linear_x=10.0, yaw=-3.0)

    node._publish_or_preview_cmd(cmd)

    assert node.cmd_vel_pub.messages == []
    assert node.last_cmd_preview == {
        'linear_x': 10.0,
        'linear_y': 0.0,
        'linear_z': 0.0,
        'angular_z': -3.0,
    }


def test_publish_or_preview_cmd_publishes_when_not_dry_run():
    node = make_mission_node()
    node.dry_run = False
    node.cmd_vel_pub = FakePublisher()
    cmd = MissionNode._build_twist(linear_x=10.0, yaw=-3.0)

    node._publish_or_preview_cmd(cmd)

    assert len(node.cmd_vel_pub.messages) == 1
    assert node.cmd_vel_pub.messages[0].linear.x == 10.0
    assert node.cmd_vel_pub.messages[0].angular.z == -3.0
