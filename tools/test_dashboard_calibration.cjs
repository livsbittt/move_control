const {test} = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');

test('sensor hold retains calibration but disables modes until runtime recovery', () => {
  const d = dashboard();
  const state = {estop:false,map:[1],map_control:{paused:false},pose_available:true,
    calibration_ready:false,calibration:{phase:'sensor_hold',ready:false,
      calibration_verified:true,settings_applied:true,message:'lidar: awaiting fresh scan'}};
  d.renderCalibration(state);
  d.renderNavigation(state);
  assert.match(d.elements.get('calibrationstatus').textContent,/보정 유지 · 센서 재확인 중/);
  assert.match(d.elements.get('calibrationsummary').textContent,/저장된 보정 유지/);
  assert.match(d.elements.get('navready').textContent,/lidar: awaiting fresh scan/);
  assert.doesNotMatch(d.elements.get('navready').textContent,/자동 보정을 완료/);
  for (const id of ['wanderstart','navexplore','navcoverage']) assert.equal(d.elements.get(id).disabled,true);
  const before = d.requests.length;
  state.calibration_ready=true;
  state.calibration.ready=true;
  state.calibration.phase='ready';
  state.calibration.message='';
  d.renderCalibration(state);
  d.renderNavigation(state);
  for (const id of ['wanderstart','navexplore','navcoverage']) assert.equal(d.elements.get(id).disabled,false);
  assert.equal(d.requests.length,before, 'Health recovery must not submit a mode command');
});

test('calibration displays runtime directional space and selected stroke', () => {
  const d = dashboard();
  d.renderCalibration({calibration:{phase:'waiting_motion',motion_clearance:{
    available_front_m:.15,required_front_m:.145,available_rear_m:.117,
    required_rear_m:.095,target_m:.02,front_stop_m:.111,rear_stop_m:.087
  }}});
  const text = d.elements.get('calibrationsensors').textContent;
  assert.ok(text.includes('15.0cm / 필요 14.5cm'));
  assert.ok(text.includes('11.7cm / 필요 9.5cm'));
  assert.ok(text.includes('이동 목표 2.0cm'));
});

test('footprint clearance is labeled as travel outside the chassis', () => {
  const d = dashboard();
  d.renderCalibration({calibration:{phase:'waiting_motion',motion_clearance:{
    translation_mode:true,available_travel_m:.04,required_travel_m:.038,
    reverse_travel_m:.02,required_reverse_travel_m:.008,target_m:.03,
    front_stop_m:.07,rear_stop_m:.07
  }}});
  const text = d.elements.get('calibrationsensors').textContent;
  assert.ok(text.includes('차체 외곽'));
  assert.ok(text.includes('4.0cm / 필요 3.8cm'));
});

test('estop and sensor qualification are displayed separately', () => {
  const d = dashboard();
  d.renderCalibration({estop:true,calibration:{phase:'collecting',message:'Waiting for sensors'}});
  assert.ok(d.elements.get('calibrationstatus').textContent.includes('센서 검사도 진행 중'));
  assert.ok(d.elements.get('calibrationstatus').textContent.includes('비상정지 걸림'));
  d.renderCalibration({estop:false,calibration:{phase:'collecting',message:'Waiting for sensors'}});
  assert.ok(!d.elements.get('calibrationstatus').textContent.includes('비상정지 걸림'));
});

function dashboard() {
  const elements = new Map();
  const noop = () => {};
  const element = id => {
    if (!elements.has(id)) elements.set(id, {
      disabled:false, style:{}, textContent:'', width:0, height:0,
      classList:{add:noop, remove:noop, toggle:noop}, parentElement:{classList:{toggle:noop}},
      addEventListener:noop, getContext:() => ({clearRect:noop}),
      getBoundingClientRect:() => ({width:0,height:0}),
    });
    return elements.get(id);
  };
  const requests = [];
  const context = vm.createContext({
    document:{getElementById:element, querySelectorAll:() => []},
    window:{addEventListener:noop, confirm:() => false},
    location:{port:'28161',protocol:'http:',hostname:'robot'},
    ResizeObserver:class {observe() {}}, Image:class {},
    fetch:async (url, options) => {requests.push([url, options]); return {ok:true,json:async () => null};},
    setInterval:() => 1, clearInterval:noop, setTimeout:noop,
    requestAnimationFrame:noop, HTMLInputElement:class {},
  });
  const html = fs.readFileSync(process.env.DASHBOARD_HTML || path.join(__dirname, '../web/dashboard.html'), 'utf8');
  const source = html.match(/<script>([\s\S]*?)<\/script>/)[1];
  vm.runInContext(source.replace(/\}\)\(\);\s*$/, 'globalThis.exposed={S,renderCalibration,renderNavigation,calibrationAction,ktick,LIM,chLimits};})();'), context);
  return {...context.exposed, elements, requests};
}

test('gauges use live front and rear limits and identify stale fallback', () => {
  const d = dashboard();
  d.S.data={limits:{stop:.12,clear:.14},motion_limits_fresh:true,
    motion_limits:{front_stop_m:.07,front_clear_m:.08,rear_stop_m:.075,rear_clear_m:.085}};
  assert.equal(d.LIM().stop,.07);
  assert.equal(d.chLimits({k:'rear'}).stop,.075);
  d.S.data.motion_limits_fresh=false;
  assert.equal(d.LIM().stop,.12);
  assert.ok(!d.LIM().live);
});

test('calibration gates reactive and map navigation; sensor detail stays visible', () => {
  const d = dashboard();
  const state = {estop:false,map:[1],map_control:{paused:false},pose_available:true,
    calibration:{phase:'waiting_motion',ready:false,sensors:{lidar:{ok:true,status:'ok',samples:9,detail:'TF180'}}}};
  d.renderCalibration(state);
  d.renderNavigation(state);
  assert.equal(d.elements.get('wanderstart').disabled,true);
  assert.equal(d.elements.get('navexplore').disabled,true);
  assert.equal(d.elements.get('calibrationmotion').disabled,false);
  assert.match(d.elements.get('calibrationsensors').textContent,/TF180/);
  assert.equal(d.elements.get('calibrationprogress').value,1);
  state.calibration.auto_motion=true;
  d.renderCalibration(state);
  assert.equal(d.elements.get('calibrationmotion').hidden,true);
  assert.equal(d.elements.get('calibrationmotion').disabled,true);
  state.calibration.ready=true;
  state.calibration_ready=true;
  d.renderCalibration(state);
  d.renderNavigation(state);
  assert.equal(d.elements.get('wanderstart').disabled,false);
  assert.equal(d.elements.get('navexplore').disabled,false);
});

test('motion validation cannot run while estopped or mapping paused', async () => {
  const d = dashboard();
  const state = {estop:true,map_control:{paused:false},calibration:{phase:'waiting_motion'}};
  d.renderCalibration(state);
  const before=d.requests.length;
  await d.calibrationAction('validate_motion');
  assert.equal(d.requests.length,before);
  state.estop=false;
  state.map_control.paused=true;
  d.renderCalibration(state);
  assert.equal(d.elements.get('calibrationmotion').disabled,true);
});
