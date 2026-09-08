"""Browser -> actual HTTP -> WanderNode; fixture has no motor plant."""
import json
from pathlib import Path
from playwright.sync_api import sync_playwright, expect


def main():
    out = Path(__file__).resolve().parents[1]/'docs/validation/navigation-session-web'
    out.mkdir(parents=True, exist_ok=True)
    errors = []
    with sync_playwright() as p:
        browser = p.chromium.launch(headless=True, channel='msedge')
        page = browser.new_page(viewport={'width':1600,'height':1100})
        page.on('pageerror', lambda e: errors.append(str(e)))
        page.goto('http://localhost:28861', wait_until='domcontentloaded')
        expect(page.locator('#navsessionstart')).to_be_enabled(timeout=15000)
        page.locator('#navstrategy').select_option('nearest')
        page.locator('#navduration').fill('10')
        page.locator('#navstall').fill('10')
        page.locator('#navsessionstart').click()
        expect(page.locator('#navsessionstatus')).to_contain_text('실행 중', timeout=5000)
        expect(page.locator('#navsessionstatus')).to_contain_text('가까운 곳 우선')
        expect(page.locator('#navstrategy')).to_be_disabled()
        page.screenshot(path=str(out/'running.png'), full_page=True)
        # The browser disconnects; the real node still owns the deadline.
        page.close()
        page = browser.new_page(viewport={'width':1600,'height':1100})
        page.wait_for_timeout(11000)
        page.goto('http://localhost:28861', wait_until='domcontentloaded')
        expect(page.locator('#navsessionstatus')).to_contain_text('전체 시간 제한으로 정지', timeout=5000)
        state = page.request.get('http://localhost:28862/state.json').json()
        assert state['wander'] == 'stop:time_limit', state['wander']
        assert state['navigation_session']['active'] is False
        page.locator('#navsessionstatus').scroll_into_view_if_needed()
        page.screenshot(path=str(out/'timeout.png'), full_page=True)
        (out/'state.json').write_text(json.dumps(state, ensure_ascii=False, indent=2), encoding='utf8')
        (out/'browser.json').write_text(json.dumps({'errors':errors, 'test':'real HTTP/ROS; paused ROS clock; no motor plant'}), encoding='utf8')
        assert not errors, errors
        browser.close()
    print('PASS: nearest selection, node acknowledgement, browser disconnect, wall-time stop')


if __name__ == '__main__':
    main()
