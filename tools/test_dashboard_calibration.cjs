const {test} = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');

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
  vm.runInContext(source.replace(/\}\)\(\);\s*$/, 'globalThis.exposed={S,renderCalibration,renderNavigation,calibrationAction,ktick};})();'), context);
  return {...context.exposed, elements, requests};
}

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
