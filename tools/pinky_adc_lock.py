"""Install cooperative transaction locks for the verified Pinky ADC drivers.

Run on Pinky as its owner before rebuilding pinky_sensor_adc and restarting
ADC, battery_publisher and battery-buzzer. Original files are preserved.
"""
from pathlib import Path
import hashlib
import json

LOCK = '/home/pinky/.local/state/move_control/adc-i2c-1-08.lock'


def patch_battery(source):
    if 'def _read_adc_channel_locked' in source:
        return source
    needle = '    def _read_adc_channel(self, register_cmd):\n'
    if source.count(needle) != 1:
        raise ValueError('Unrecognized Battery implementation')
    wrapper = (
        '    def _read_adc_channel(self, register_cmd):\n'
        '        # Shared with pinky_sensor_adc: select/wait/read is one transaction.\n'
        '        import fcntl\n'
        f'        with open({LOCK!r}, "r") as lock:\n'
        '            fcntl.flock(lock, fcntl.LOCK_EX)\n'
        '            return self._read_adc_channel_locked(register_cmd)\n\n'
        '    def _read_adc_channel_locked(self, register_cmd):\n')
    return source.replace(needle, wrapper)


def patch_adc(source):
    if 'struct AdcTransactionLock' in source:
        return source
    needle = '        void timer_callback()\n        {\n'
    if source.count(needle) != 1:
        raise ValueError('Unrecognized ADC implementation')
    guard = f'''#include <sys/file.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdexcept>

struct AdcTransactionLock {{
    int fd;
    AdcTransactionLock() {{
        fd = open("{LOCK}", O_RDONLY | O_CLOEXEC);
        if (fd < 0) throw std::runtime_error("ADC transaction lock unavailable");
        if (flock(fd, LOCK_EX) != 0) {{
            close(fd);
            throw std::runtime_error("ADC transaction lock failed");
        }}
    }}
    ~AdcTransactionLock() {{ flock(fd, LOCK_UN); close(fd); }}
}};

'''
    return guard + source.replace(needle, needle + '            AdcTransactionLock transaction;\n')


def main():
    import inspect
    from pinkylib import Battery
    lock = Path(LOCK)
    lock.parent.mkdir(parents=True, exist_ok=True)
    lock.touch(exist_ok=True)
    lock.chmod(0o644)
    targets = [
        (Path(inspect.getfile(Battery)), patch_battery),
        (Path('/home/pinky/pinky_pro/src/pinky_pro/pinky_sensor_adc/src/main_node.cpp'), patch_adc),
    ]
    changes = [(path, path.read_text(), transform(path.read_text())) for path, transform in targets]
    for path, original, updated in changes:
        backup = path.with_name(path.name + '.before-adc-lock')
        if not backup.exists():
            backup.write_text(original)
        path.write_text(updated)
        print(json.dumps({'path': str(path), 'sha256': hashlib.sha256(updated.encode()).hexdigest()}))


if __name__ == '__main__':
    main()
