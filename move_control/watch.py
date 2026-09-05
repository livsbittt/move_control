"""Subject: node interruptions. Pure graph check — no ROS.

Local stack must be one of each required node. Exclusive topics
must be owned by the local node, not a remote gazebo/bridge twin.
"""
from collections import Counter
from dataclasses import dataclass, field


REQUIRED = (
    'sllidar_node',
    'pinky_bringup',
    'pinky_sensor_adc',
    'led_service_server',
    'pinky_imu_bno055',
    'camera_detect_node',
    'safety_node',
    'wander_node',
)

OPTIONAL = ('lcd_node', 'web_node')

# topic -> the one local node that may publish it
EXCLUSIVE = {
    '/cmd_vel': 'safety_node',
    '/cmd_vel_raw': 'wander_node',
    '/scan': 'sllidar_node',
}

# Local extras that are not a fight (safety zeros /cmd_vel_raw on e-stop).
ALLOWED = {
    '/cmd_vel_raw': frozenset({'safety_node', 'web_node'}),
}

FOREIGN = frozenset({
    'pinky_control',
    'pinky_move',
    'pinky_map',
    'pinky_explore',
    'parameter_bridge',
    'image_bridge',
})

IGNORE_NODES = frozenset({
    'watch_node',
    'graph_probe',
    'transform_listener_impl',
})


@dataclass(frozen=True)
class Issue:
    kind: str
    topic: str
    node: str
    detail: str


@dataclass
class Report:
    ok: bool
    issues: list = field(default_factory=list)
    nodes: dict = field(default_factory=dict)

    def line(self) -> str:
        if self.ok:
            return 'ok'
        return '; '.join(i.detail for i in self.issues)


def _bare(name: str) -> str:
    n = (name or '').strip()
    if n.startswith('/'):
        n = n[1:]
    if '/' in n:
        n = n.rsplit('/', 1)[-1]
    return n


def inspect(node_names, pubs_by_topic) -> Report:
    """node_names: iterable of node name strings.
    pubs_by_topic: {topic: [publisher node names]}.
    """
    counts = Counter(_bare(n) for n in node_names if _bare(n) not in IGNORE_NODES)
    issues = []
    for name in REQUIRED:
        n = counts.get(name, 0)
        if n == 0:
            issues.append(Issue('missing', '', name, f'missing {name}'))
        elif n > 1:
            issues.append(Issue('duplicate', '', name, f'duplicate {name} x{n}'))
    for name in OPTIONAL:
        n = counts.get(name, 0)
        if n > 1:
            issues.append(Issue('duplicate', '', name, f'duplicate {name} x{n}'))
    for topic, owner in EXCLUSIVE.items():
        pubs = [_bare(p) for p in (pubs_by_topic.get(topic) or [])]
        local = [p for p in pubs if p == owner]
        foreign = [p for p in pubs if p in FOREIGN]
        allow = ALLOWED.get(topic, frozenset())
        extra = [
            p for p in pubs
            if p not in FOREIGN and p != owner and p not in IGNORE_NODES and p not in allow
        ]
        if not local:
            issues.append(Issue('missing_pub', topic, owner, f'{topic} has no {owner}'))
        if len(local) > 1:
            issues.append(Issue('duplicate', topic, owner, f'{topic} {owner} x{len(local)}'))
        if foreign:
            who = ','.join(sorted(set(foreign)))
            issues.append(Issue('foreign', topic, who, f'{topic} interrupted by {who}'))
        if extra:
            who = ','.join(sorted(set(extra)))
            issues.append(Issue('extra', topic, who, f'{topic} extra {who}'))
    return Report(ok=not issues, issues=issues, nodes=dict(counts))
