"""Chromium -> real WebNode HTTP -> isolated ROS fixture -> readback."""
import json
from pathlib import Path
import subprocess
import os
from playwright.sync_api import sync_playwright, expect


def scenario(value):
    helper = Path(__file__).with_name('dashboard_scenario.sh').resolve()
    command = ['bash', str(helper), value]
    if os.name == 'nt':
        linux_path = '/mnt/'+helper.drive[0].lower()+helper.as_posix()[2:]
        command = ['wsl', '-d', 'Ubuntu', '--', 'bash', linux_path, value]
    subprocess.run(command, check=True, capture_output=True)


def main():
    out = Path(__file__).resolve().parents[1]/'docs/validation/dashboard-operations'
    out.mkdir(parents=True, exist_ok=True)
    errors, requests = [], []
    with sync_playwright() as p:
        browser = p.chromium.launch(headless=True)
        page = browser.new_page(viewport={'width':1440,'height':1000})
        page.on('pageerror', lambda e: errors.append(str(e)))
        page.on('request', lambda r: requests.append(r.url))
        page.goto('http://localhost:28361', wait_until='domcontentloaded')
        expect(page.locator('#batteryvalue')).to_have_text('72%', timeout=15000)
        expect(page.locator('#batterydetail')).to_contain_text('충전 중')
        expect(page.locator('#overviewcal')).to_have_text('적용 완료')
        expect(page.locator('#navexplore')).to_be_enabled(timeout=15000)
        page.wait_for_timeout(1800)
        page.screenshot(path=str(out/'desktop-after.png'))
        page.locator('#wanderstart').click()
        expect(page.locator('#commandstatus')).to_contain_text('주행 시작', timeout=8000)
        expect(page.locator('#overviewmotion')).to_have_text('주행 중')
        page.locator('[data-post="/wander"][data-val="stop"]').click()
        expect(page.locator('#commandstatus')).to_contain_text('주행 정지')
        page.locator('[data-post="/estop"][data-val="stop"]').click()
        expect(page.locator('#commandstatus')).to_contain_text('비상정지 설정')
        expect(page.locator('#wanderstart')).to_be_disabled()
        page.locator('[data-post="/estop"][data-val="release"]').click()
        expect(page.locator('#commandstatus')).to_contain_text('비상정지 해제')
        expect(page.locator('#overviewmotion')).to_have_text('대기 중')
        page.locator('[data-calibration="abort"]').click()
        expect(page.locator('#calibrationstatus')).to_contain_text('중단', timeout=8000)
        expect(page.locator('#wanderstart')).to_be_disabled()
        page.locator('[data-calibration="retry"]').click()
        expect(page.locator('#overviewcal')).to_have_text('적용 완료', timeout=8000)
        page.locator('#mappingtoggle').click()
        expect(page.locator('#mappingstatus')).to_have_text('지도 수집 일시정지', timeout=10000)
        expect(page.locator('#navexplore')).to_be_disabled()
        page.locator('#mappingtoggle').click()
        expect(page.locator('#mappingstatus')).to_have_text('지도 수집 중', timeout=10000)
        expect(page.locator('#overviewmotion')).to_have_text('대기 중')
        expect(page.locator('#previewstatus')).to_have_text('영상 수신 중', timeout=10000)
        page.locator('#cameratoggle').uncheck()
        expect(page.locator('#previewstatus')).to_contain_text('영상 요청 중지됨')
        page.wait_for_timeout(400)
        before = sum('/camera.jpg' in r for r in requests)
        page.wait_for_timeout(1200)
        assert sum('/camera.jpg' in r for r in requests) == before
        page.reload()
        expect(page.locator('#cameratoggle')).not_to_be_checked()
        page.locator('#cameratoggle').check()
        expect(page.locator('#previewstatus')).to_have_text('영상 수신 중', timeout=10000)
        page.locator('#displaycard').scroll_into_view_if_needed()
        page.screenshot(path=str(out/'controls-after.png'))
        scenario('unknown')
        expect(page.locator('#batteryvalue')).to_have_text('정보 없음', timeout=10000)
        expect(page.locator('#batterydetail')).to_contain_text('7.80 V')
        scenario('silent')
        expect(page.locator('#batterydetail')).to_have_text('배터리 수신 지연', timeout=12000)
        scenario('refuse_pause')
        expect(page.locator('#mappingtoggle')).to_be_enabled(timeout=10000)
        page.locator('#mappingtoggle').click()
        expect(page.locator('#mappingerror')).to_contain_text('rejected', timeout=10000)
        scenario('live')
        expect(page.locator('#batteryvalue')).to_have_text('72%', timeout=10000)
        page.set_viewport_size({'width':390,'height':844})
        page.reload()
        expect(page.locator('#batteryvalue')).to_have_text('72%', timeout=10000)
        assert page.evaluate('document.documentElement.scrollWidth <= window.innerWidth')
        page.screenshot(path=str(out/'mobile-after.png'), full_page=True)
        # Browser-offline must remove readiness, not retain the last green state.
        page.context.set_offline(True)
        expect(page.locator('#overviewmotion')).to_have_text('연결 끊김', timeout=7000)
        expect(page.locator('#wanderstart')).to_be_disabled()
        assert not errors, errors
        browser.close()
    (out/'browser-test.json').write_text(json.dumps({'passed':True, 'page_errors':errors,
        'scope':'HTTP + ROS fixture, domain 231; no physical robot',
        'checks':['battery live/unknown/stale','start/stop readback','estop/release readback',
                  'mapping pause/resume and service refusal','camera requests off/on and preference reload',
                  'mobile overflow','offline readiness removal']}, indent=2), encoding='utf-8')
    print('PASS: real HTTP/ROS round trips and browser controls')


if __name__ == '__main__':
    main()
