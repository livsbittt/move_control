import ast
from pinky_adc_lock import patch_battery, patch_adc


def test_battery_wraps_whole_transaction_and_is_idempotent():
    source = 'class Battery:\n    def _read_adc_channel(self, register_cmd):\n        return self.bus.read(register_cmd)\n'
    result = patch_battery(source)
    ast.parse(result)
    assert 'fcntl.flock(lock, fcntl.LOCK_EX)' in result
    assert 'return self._read_adc_channel_locked(register_cmd)' in result
    assert patch_battery(result) == result


def test_adc_guard_covers_callback_and_releases_on_exit():
    source = '        void timer_callback()\n        {\n            read_adc();\n        }\n'
    result = patch_adc(source)
    assert 'AdcTransactionLock transaction;\n            read_adc();' in result
    assert '~AdcTransactionLock() { flock(fd, LOCK_UN); close(fd); }' in result
    assert patch_adc(result) == result
