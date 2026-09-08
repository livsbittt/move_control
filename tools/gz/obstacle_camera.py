"""Opt-in rendered camera for the exact-world simulation; no invented images."""
import xml.etree.ElementTree as ET


def add_camera(root):
    base = root.find(".//model[@name='pinky']/link[@name='base']")
    if base is None:
        raise ValueError('Expected Pinky base link')
    if base.find("sensor[@name='obstacle_camera']") is not None:
        return
    sensor = ET.SubElement(base, 'sensor', name='obstacle_camera', type='camera')
    for name, value in [('pose', '.055 0 .08 0 .25 0'), ('always_on', 'true'),
                        ('update_rate', '10'), ('topic', '/pinky/rendered_camera')]:
        ET.SubElement(sensor, name).text = value
    camera = ET.SubElement(sensor, 'camera')
    ET.SubElement(camera, 'horizontal_fov').text = '1.2'
    image = ET.SubElement(camera, 'image')
    for name, value in [('width', '320'), ('height', '240'), ('format', 'R8G8B8')]:
        ET.SubElement(image, name).text = value
    clip = ET.SubElement(camera, 'clip')
    ET.SubElement(clip, 'near').text = '.02'
    ET.SubElement(clip, 'far').text = '8'
