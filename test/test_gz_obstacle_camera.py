import xml.etree.ElementTree as ET
from tools.gz.obstacle_camera import add_camera


def test_camera_is_rendered_sensor_and_preserves_collision_geometry():
    root = ET.fromstring('<sdf><world name="test"><model name="pinky"><link name="base"><collision name="body"><geometry><box><size>.1 .1 .1</size></box></geometry></collision></link></model></world></sdf>')
    before = ET.tostring(root.find('.//collision'))
    add_camera(root)
    assert ET.tostring(root.find('.//collision')) == before
    camera = root.find(".//sensor[@type='camera']")
    assert camera.findtext('topic') == '/pinky/rendered_camera'
    assert camera.findtext('camera/image/width') == '320'
    add_camera(root)
    assert len(root.findall(".//sensor[@type='camera']")) == 1
