"""Real browser with isolated HTTP fixtures; never contacts or drives a robot."""
import json
from pathlib import Path
from playwright.sync_api import sync_playwright, expect


def main():
    root = Path(__file__).resolve().parents[1]
    html = (root / 'web/dashboard.html').read_text(encoding='utf-8').replace('__BACKEND_PORT__', '28162')
    state = {'estop': False, 'map': [10, 10, .02, 0, 0, 1], 'pose': [.1, .1, 0],
             'pose_available': True, 'map_control': {'available': True, 'paused': False},
             'sensors': {}, 'calibration_ready': False,
             'calibration': {'phase': 'collecting', 'ready': False, 'auto_motion': True, 'sensors': {}}}
    requests, errors = [], []
    with sync_playwright() as p:
        browser = p.chromium.launch(headless=True)
        page = browser.new_page(viewport={'width': 1440, 'height': 1000})
        page.on('pageerror', lambda error: errors.append(str(error)))

        def route(r):
            url = r.request.url
            if r.request.method == 'POST':
                requests.append((url.split(':28162')[-1], r.request.post_data))
                r.fulfill(status=200, content_type='application/json', body='{"ok":true}')
            elif '/state.json' in url:
                r.fulfill(content_type='application/json', body=json.dumps(state))
            elif url.endswith('/'):
                r.fulfill(content_type='text/html', body=html)
            else:
                r.fulfill(status=404, body='')
        page.route('**/*', route)
        page.goto('http://127.0.0.1:28161/', wait_until='domcontentloaded')
        expect(page.locator('#wanderstart')).to_be_disabled()
        expect(page.locator('#navexplore')).to_be_disabled()
        state['calibration'].update(phase='ready', ready=True)
        state['calibration_ready'] = True
        expect(page.locator('#navexplore')).to_be_enabled()
        expect(page.locator('#calibrationsummary')).to_contain_text('완료')
        assert not requests, 'Readiness must never submit a driving command'
        for selector, mode in [('#navexplore','explore'), ('#navcoverage','coverage'), ('#wanderstart','start')]:
            page.locator(selector).click()
            page.wait_for_timeout(100)
            assert requests[-1] == ('/wander', mode), requests
        state['calibration'].update(phase='failed', ready=False)
        state['calibration_ready'] = False
        expect(page.locator('#navexplore')).to_be_disabled()
        expect(page.locator('#calibrationsummary')).to_contain_text('정지됨')
        page.locator('[data-calibration="retry"]').click()
        page.wait_for_timeout(100)
        assert requests[-1] == ('/calibration', 'retry')
        state['calibration'].update(phase='ready', ready=True)
        state['calibration_ready'] = True
        state['estop'] = True
        expect(page.locator('#wanderstart')).to_be_disabled()
        expect(page.locator('#navcoverage')).to_be_disabled()
        assert not errors, errors
        (root / 'artifacts').mkdir(exist_ok=True)
        page.screenshot(path=str(root / 'artifacts/integrated-ui-browser.png'), full_page=True)
        browser.close()
    print('PASS: ready remains stopped, explicit three-mode clicks, failure/retry, estop and no JS errors')


if __name__ == '__main__':
    main()
