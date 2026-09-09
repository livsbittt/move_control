"""Emit sensor readiness transitions without flooding logs with sample updates."""


class StartupDiagnostics:
    def __init__(self):
        self.previous = None

    def update(self, report):
        sensors = report.get('sensors', {})
        states = tuple(sorted((name, item.get('status', 'unknown'))
                              for name, item in sensors.items()))
        signature = (report.get('phase'), report.get('mode', 'calibration'),
                     bool(report.get('ready')), states)
        if signature == self.previous:
            return None
        self.previous = signature
        return {'event': 'startup_state', 'phase': signature[0],
                'mode': signature[1], 'ready': signature[2],
                'sensors': dict(states),
                'details': {name: item.get('detail', '') for name, item in sensors.items()
                            if item.get('status') != 'ok'},
                'message': report.get('message', '')}
