"""Capture the real isolated simulation dashboard without issuing control requests."""
import argparse
import json
from pathlib import Path
from playwright.sync_api import sync_playwright, TimeoutError as PlaywrightTimeout


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--allow-in-progress', action='store_true')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    with sync_playwright() as p:
        browser = p.chromium.launch(headless=True, channel='msedge')
        page = browser.new_page(viewport={'width': 1600, 'height': 1100}, device_scale_factor=1)
        errors = []
        page.on('pageerror', lambda error: errors.append(str(error)))
        page.goto('http://localhost:28761', wait_until='domcontentloaded')
        try:
            page.wait_for_load_state('networkidle', timeout=2000)
        except PlaywrightTimeout:
            pass  # The live dashboard polls faster than networkidle's 500 ms gap.
        if not args.allow_in_progress:
            page.wait_for_function("document.getElementById('calibrationstatus').textContent.includes('교정 완료')", timeout=15000)
        state = page.request.get('http://localhost:28762/state.json').json()
        (args.output/'dashboard_state.json').write_text(json.dumps(state, ensure_ascii=False, indent=2), encoding='utf-8')
        page.screenshot(path=str(args.output/'dashboard.png'), full_page=True)
        page.get_by_text('센서별 상세값', exact=True).click()
        page.locator('details').filter(has=page.locator('#calibrationsensors')).screenshot(
            path=str(args.output/'rotation_details.png'))
        (args.output/'browser_errors.json').write_text(json.dumps(errors), encoding='utf-8')
        print(json.dumps({'errors':errors, 'map':state.get('map'), 'calibration_ready':state.get('calibration_ready')}))
        browser.close()


if __name__ == '__main__':
    main()
