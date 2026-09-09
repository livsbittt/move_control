const {test} = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');

test('map navigation explains missing planner even when map and calibration are ready', () => {
  const d=dashboard();
  const state={estop:false,map:[1],map_control:{paused:false},pose_available:true,
    calibration_ready:true,calibration:{ready:true},wander:'stop',
    navigation_session_fresh:true,navigation_session:{active:false,reason:'idle'},planner_fresh:false};
  d.renderNavigation(state);
  for (const id of ['navexplore','navcoverage','navsessionstart']) assert.equal(d.elements.get(id).disabled,true);
  assert.match(d.elements.get('navready').textContent,/경로 계획 노드/);
  assert.match(d.elements.get('navsessionstatus').textContent,/경로 계획 노드/);
  state.planner_fresh=true;
  d.renderNavigation(state);
  for (const id of ['navexplore','navcoverage','navsessionstart']) assert.equal(d.elements.get(id).disabled,false);
});

test('limited sensor driving is distinct from stationary sensing and waits for readiness', async () => {
  const d = dashboard();
  const state={estop:false,map:[1],map_control:{paused:false},pose_available:true,
    calibration_ready:false,calibration:{mode:'limited_sensors',phase:'limited_sensors',
      degraded:true,excluded_sensors:['imu'],calibration_verified:false,ready:false,
      waiting_reasons:['estop']}};
  d.renderCalibration(state);
  d.renderOverview(state);
  assert.equal(d.elements.get('calibrationlimitedstatus').hidden,false);
  assert.equal(d.elements.get('calibrationpartial').hidden,true);
  assert.match(d.elements.get('calibrationlimitedstatus').textContent,/수동 완료.*IMU 제외.*저속 제한/);
  assert.match(d.elements.get('calibrationlimitedstatus').textContent,/비상정지 해제 대기/);
  assert.match(d.elements.get('overviewcal').textContent,/보정 완료\(일부 센서 모드\)/);
  assert.equal(d.elements.get('wanderstart').disabled,true);
  await d.calibrationAction('use_limited_sensors');
  assert.equal(d.requests.at(-1)[1].body,'use_limited_sensors');
  assert.equal(d.elements.get('wanderstart').disabled,true);
  state.calibration_ready=true;
  state.calibration.ready=true;
  d.renderCalibration(state);
  assert.equal(d.elements.get('wanderstart').disabled,false);
  assert.match(d.elements.get('calibrationretry').textContent,/제한 주행 종료/);
  state.estop=true;
  d.renderCalibration(state);
  assert.equal(d.elements.get('wanderstart').disabled,true);
});

test('existing settings remain visibly uncalibrated and require runtime readiness', async () => {
  const d = dashboard();
  const state={estop:false,map:[1],map_control:{paused:false},pose_available:true,
    calibration_ready:false,calibration:{mode:'existing_settings',phase:'existing_settings',
      existing_settings:true,calibration_verified:false,ready:false,operating_ready:false,
      motion_allowed:false,waiting_reasons:['lidar stale','estop']}};
  d.renderCalibration(state);
  d.renderOverview(state);
  assert.equal(d.elements.get('calibrationexistingstatus').hidden,false);
  assert.match(d.elements.get('calibrationexistingstatus').textContent,/기존 설정값.*미보정/);
  assert.match(d.elements.get('calibrationexistingstatus').textContent,/비상정지 해제 대기/);
  assert.match(d.elements.get('overviewcal').textContent,/미보정/);
  assert.equal(d.elements.get('wanderstart').disabled,true);
  await d.calibrationAction('use_existing_settings');
  assert.equal(d.requests.at(-1)[1].body,'use_existing_settings');
  assert.equal(d.elements.get('wanderstart').disabled,true);
  state.calibration.ready=true;
  state.calibration.operating_ready=true;
  state.calibration.motion_allowed=true;
  state.calibration_ready=true;
  d.renderCalibration(state);
  d.renderOverview(state);
  assert.equal(d.elements.get('wanderstart').disabled,false);
  assert.match(d.elements.get('calibrationsummary').textContent,/미보정/);
  assert.doesNotMatch(d.elements.get('overviewcaldetail').textContent,/보정값 적용 확인됨/);
  assert.match(d.elements.get('calibrationretry').textContent,/기존 설정값 모드 종료/);
  state.estop=true;
  d.renderCalibration(state);
  assert.equal(d.elements.get('wanderstart').disabled,true);
  state.estop=false;
  state.calibration_ready=false;
  d.renderCalibration(state);
  assert.equal(d.elements.get('wanderstart').disabled,true);
});

test('IMU-excluded sensing stays visibly partial and cannot enable driving', async () => {
  const d = dashboard();
  const state = {estop:false,map:[1],map_control:{paused:false},pose_available:true,
    calibration_ready:false,calibration:{phase:'sensing_only',mode:'sensing_only',partial:true,
      ready:false,calibration_verified:false,motion_allowed:false,partial_baseline_ready:true,
      excluded_sensors:['imu'],sensors:{imu:{excluded:true,ok:false,status:'excluded'}}}};
  d.renderCalibration(state);
  d.renderNavigation(state);
  assert.equal(d.elements.get('calibrationpartial').hidden,false);
  assert.match(d.elements.get('calibrationpartial').textContent,/IMU 제외.*미교정.*주행 불가/);
  assert.match(d.elements.get('calibrationsummary').textContent,/부분 센싱.*기준값 수집 완료/);
  assert.match(d.elements.get('calibrationsensors').textContent,/자세 IMU: 제외/);
  assert.match(d.elements.get('calibrationretry').textContent,/부분 센싱 종료/);
  for (const id of ['wanderstart','navexplore','navcoverage','calibrationmotion'])
    assert.equal(d.elements.get(id).disabled,true);
  await d.calibrationAction('sensing_only');
  assert.equal(d.requests.at(-1)[1].body,'sensing_only');
  state.calibration={phase:'collecting',ready:false};
  d.renderCalibration(state);
  assert.equal(d.elements.get('calibrationpartial').hidden,true);
});

test('retry sends the chosen post-calibration position without releasing stop', async () => {
  const d = dashboard();
  const initial = d.requests.length;
  for (const option of ['stay', 'return_origin']) {
    d.elements.set('calibrationafter', {value:option});
    await d.calibrationAction('retry');
    const [url, request] = d.requests.at(-1);
    assert.ok(url.endsWith('/calibration'));
    assert.equal(request.body, 'retry:' + option);
  }
  assert.equal(d.requests.length, initial + 2);
});

test('sensor hold retains calibration but disables modes until runtime recovery', () => {
  const d = dashboard();
  const state = {estop:false,map:[1],map_control:{paused:false},pose_available:true,planner_fresh:true,
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
  vm.runInContext(source.replace(/\}\)\(\);\s*$/, 'globalThis.exposed={S,renderCalibration,renderNavigation,renderOverview,calibrationAction,ktick,LIM,chLimits,plannerStatus,BIND};})();'), context);
  return {...context.exposed, elements, requests};
}

test('map clearance status separates planned motion from actual safety and output', () => {
  const d=dashboard();
  const state={estop:false,map:[1],map_control:{paused:false},pose_available:true,
    planner_fresh:true,calibration_ready:true,calibration:{ready:true},vel:[0,0],eta:12};
  for (const detail of ['robot inside obstacle clearance','map start clearance insufficient']) {
    state.gstate='planning blocked: '+detail+' eta=? v=1.4cm/s pose~tf';
    d.renderNavigation(state);
    assert.match(d.elements.get('navready').textContent,/지도 격자/);
    assert.match(d.elements.get('navready').textContent,/실제 센서 차단/);
    assert.doesNotMatch(d.plannerStatus(state),/eta=|v=|1.4/);
    assert.equal(d.BIND.find(([key])=>key==='eta')[1](state),'-');
    assert.match(d.BIND.find(([key])=>key==='vel')[1](state),/^0.0 cm\/s/);
  }
  state.gstate='escape: map start connector eta=12 v=1.4cm/s';
  d.renderNavigation(state);
  assert.match(d.elements.get('navready').textContent,/연결 경로 선택/);
  assert.doesNotMatch(d.elements.get('navready').textContent,/탈출 중|주행 재개/);
  assert.match(d.plannerStatus(state),/계획 속도=/);
  state.gstate='verified start connection: anchor accepted eta=12 v=1.4cm/s';
  d.renderNavigation(state);
  assert.match(d.elements.get('navready').textContent,/연결 경로 선택/);
  assert.doesNotMatch(d.elements.get('navready').textContent,/탈출 중|주행 재개/);
  state.estop=true;
  d.renderNavigation(state);
  assert.equal(d.elements.get('navready').textContent,'비상정지 상태');
});

test('navigation exposes zero safety output and transient replanning without invented progress', () => {
  const d=dashboard();
  const state={estop:false,map:[1],map_control:{paused:false},pose_available:true,
    planner_fresh:true,calibration_ready:true,calibration:{ready:true},
    gstate:'narrow passage: eta=10 v=1.4cm/s',wander:'stalled_restart_required',
    safety_decision:{action:'stop',reason:'bounded_sweep_unavailable',requested_v:.014,
      requested_omega:.05,safe_v:0,safe_omega:0},motion_limits_fresh:true,
    motion_limits:{can_rotate:false,rotation_scan_observed:false},
    navigation_session_fresh:true,navigation_session:{active:true,reason:'running',elapsed_s:3,
      remaining_s:17,options:{strategy:'gain',duration_s:20}}};
  d.renderNavigation(state);
  assert.match(d.elements.get('navready').textContent,/자동 재계획 전 대기/);
  assert.doesNotMatch(d.elements.get('navready').textContent,/다시 시작/);
  assert.match(d.elements.get('navmotion').textContent,/주행 요청 1.4cm\/s/);
  assert.match(d.elements.get('navmotion').textContent,/최종 출력 0.0cm\/s/);
  assert.match(d.elements.get('navmotion').textContent,/궤적 관측 부족/);
  assert.match(d.elements.get('navmotion').textContent,/관측 미완료/);
  assert.match(d.elements.get('navsessionstatus').textContent,/남은 17초/);
  state.navigation_session.execution_escape={reason:'waiting_for_candidate',active:false,
    episode_elapsed_s:4.2,remaining_s:12.8,target_remaining_m:.002};
  d.renderNavigation(state);
  assert.match(d.elements.get('navmotion').textContent,/최신 이동 후보 대기/);
  assert.match(d.elements.get('navmotion').textContent,/현재 복구 경과 4.2초/);
  assert.match(d.elements.get('navmotion').textContent,/남은 시간 12.8초/);
  assert.match(d.elements.get('navmotion').textContent,/잔여 이동 추정 0.2cm/);
  assert.doesNotMatch(d.elements.get('navready').textContent,/3회|5초/);
  state.navigation_session_fresh=false;
  d.renderNavigation(state);
  assert.doesNotMatch(d.elements.get('navmotion').textContent,/현재 복구 경과|최신 이동 후보 대기/);
  state.navigation_session_fresh=true;
  state.safety_decision={action:'allow',reason:'allow',requested_v:0,requested_omega:0,safe_v:0,safe_omega:0};
  for (const [status,label] of [['gate_rotation_blocked','회전 조건 미충족 · 대체 경로 탐색'],
    ['execution_escape_reverse','관측된 후방 공간으로 이동 · 회전 여유 재확인'],
    ['execution_escape_forward','관측된 전방 공간으로 이동 · 회전 여유 재확인'],
    ['execution_escape_complete','공간 확보 이동 완료 · 재탐색'],['execution_escape_stopped','공간 확보 이동 중단']]) {
    state.wander=status;
    d.renderNavigation(state);
    assert.equal(d.elements.get('navready').textContent,label);
    assert.match(d.elements.get('navmotion').textContent,/최종 출력 0.0cm\/s/);
  }
  state.wander='forward';state.motion_limits.rotation_scan_observed=true;
  d.renderNavigation(state);
  assert.match(d.elements.get('navready').textContent,/실제 이동이 아닙니다/);
  assert.match(d.elements.get('navmotion').textContent,/회전 여유 또는 다른 안전 조건 미충족/);
  state.motion_limits.execution_escape={candidates:[{direction:-1,target_m:.073,
    objective:'restore_rotation'},{direction:1,target_m:.031,objective:'improve_clearance'}]};
  d.renderNavigation(state);
  assert.match(d.elements.get('navmotion').textContent,/필요 이동 추정 \(이동한 거리 아님\): 후방 7.3cm/);
  assert.match(d.elements.get('navmotion').textContent,/전방 3.1cm/);
  state.motion_limits.execution_escape={candidates:[],reason:'no_candidate'};
  d.renderNavigation(state);
  assert.match(d.elements.get('navmotion').textContent,/이동 후보 없음/);
  assert.doesNotMatch(d.elements.get('navmotion').textContent,/7.3cm|3.1cm/);
  state.motion_limits.execution_escape={candidates:[{direction:-1,target_m:.073}]};
  state.safety_decision=null;state.motion_limits_fresh=false;
  d.renderNavigation(state);
  assert.match(d.elements.get('navmotion').textContent,/출력과 정지 사유 확인 불가/);
  assert.doesNotMatch(d.elements.get('navmotion').textContent,/최종 출력 0/);
  assert.doesNotMatch(d.elements.get('navmotion').textContent,/필요 이동 추정|7.3cm/);
});

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

test('body radius never inherits a swept rotation radius', () => {
  const d = dashboard();
  d.S.data={limits:{radius:.076},motion_limits_fresh:true,
    motion_limits:{rotation_radius_m:.16,body_radius_m:.1144}};
  assert.equal(d.LIM().radius,.1144);
  delete d.S.data.motion_limits.body_radius_m;
  assert.equal(d.LIM().radius,.076);
});

test('rotation estimate and live permission are separate centimeter evidence', () => {
  const d = dashboard();
  const state={calibration:{rotation:{envelope:{valid:true,center_m:[-.04,.002],
    pivot_radius_m:.13,center_uncertainty_m:.006}}},
    motion_limits_fresh:true,motion_limits:{body_radius_m:.1144,
      rotation_scan_observed:true,can_rotate:false}};
  d.renderCalibration(state);
  let text=d.elements.get('calibrationsensors').textContent;
  assert.match(text,/중심.*-4.0.*0.2cm/);
  assert.match(text,/회전 반경 13.0cm.*불확실성 0.6cm/);
  assert.match(text,/차체.*11.4cm/);
  assert.match(text,/현재 회전.*보류.*관측 완료/);
  state.motion_limits_fresh=false;
  d.renderCalibration(state);
  text=d.elements.get('calibrationsensors').textContent;
  assert.match(text,/회전 반경 13.0cm/);
  assert.match(text,/현재 회전.*최신 안전 정보 대기/);
});

test('calibration gates reactive and map navigation; sensor detail stays visible', () => {
  const d = dashboard();
  const state = {estop:false,map:[1],map_control:{paused:false},pose_available:true,planner_fresh:true,
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

test('measurement completion waits for application and reports effective geometry', () => {
  const d = dashboard();
  const state = {estop:false, calibration_ready:false,
    calibration:{phase:'ready', ready:false, sensors:{}, rotation:{done:true, legs:Array(8)}},
    safety_profile:{valid:true, commissioned:false, effective:{stop:.15,radius:.09}}};
  d.renderCalibration(state);
  assert.match(d.elements.get('calibrationstatus').textContent,/적용 확인 대기/);
  assert.match(d.elements.get('calibrationsensors').textContent,/15.0cm/);
  assert.match(d.elements.get('calibrationsensors').textContent,/8\/8/);
  assert.equal(d.elements.get('wanderstart').disabled,true);
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
