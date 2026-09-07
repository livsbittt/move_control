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
      disabled: false, style: {}, textContent: '', width: 0, height: 0,
      classList: {add: noop, remove: noop, toggle: noop},
      parentElement: {classList: {toggle: noop}},
      addEventListener: noop,
      getContext: () => ({clearRect: noop}),
      getBoundingClientRect: () => ({width: 0, height: 0}),
    });
    return elements.get(id);
  };
  const images = [];
  const context = vm.createContext({
    document: {getElementById: element, querySelectorAll: () => []},
    window: {addEventListener: noop, confirm: () => false},
    location: {port: '28161', protocol: 'http:', hostname: 'robot'},
    ResizeObserver: class {observe() {}},
    Image: class {constructor() {images.push(this);}},
    fetch: async () => {throw new Error('offline');},
    setInterval: () => 1, clearInterval: noop, setTimeout: noop,
    requestAnimationFrame: noop, HTMLInputElement: class {},
  });
  const html = fs.readFileSync(path.join(__dirname, '../web/dashboard.html'), 'utf8');
  const source = html.match(/<script>([\s\S]*?)<\/script>/)[1];
  vm.runInContext(source.replace(/\}\)\(\);\s*$/, 
    'globalThis.mappingTest = {S, renderMapping, renderNavigation, mappingAction, loadMap, bucket, saveMapImage};})();'), context);
  return {context, elements, images, ...context.mappingTest};
}

test('unavailable SLAM disables controls; resume only enabled while paused', () => {
  const d = dashboard();
  d.renderMapping(null);
  assert.equal(d.elements.get('mapreset').disabled, true);
  assert.equal(d.elements.get('mapsave').disabled, true);
  assert.equal(d.elements.get('mapresume').disabled, true);
  d.renderMapping({available: true, paused: false});
  assert.equal(d.elements.get('mapreset').disabled, false);
  assert.equal(d.elements.get('mapresume').disabled, true);
  d.renderMapping({available: true, paused: true});
  assert.equal(d.elements.get('mapresume').disabled, false);
});

test('map export downloads native map PNG and rejects a reset during fetch', async () => {
  const d=dashboard();
  await new Promise(resolve => setImmediate(resolve));
  const downloads=[], blobs=[], urls=[];
  d.context.document.createElement=() => {const a={click:() => downloads.push(a)};return a;};
  d.context.URL={createObjectURL:b => {blobs.push(b);return 'blob:map';},revokeObjectURL:()=>{}};
  d.S.mapEpoch=1;
  d.S.data.map=[64,16,.02,0,0,7];
  d.S.data.map_control={available:true,paused:false};
  d.S.img={im:{},gen:7};
  d.S.view.z=8;
  const png={type:'image/png',size:123};
  d.context.fetch=async url => {urls.push(url);return {ok:true,blob:async()=>png};};
  d.renderMapping(d.S.data.map_control);
  assert.equal(d.elements.get('mapsave').disabled,false);
  await d.saveMapImage();
  assert.match(urls[0],/\/map\.png\?g=7$/);
  assert.equal(blobs[0],png);
  assert.match(downloads[0].download,/^pinky-map-.*\.png$/);
  d.context.fetch=async()=>{d.S.mapEpoch=2;return {ok:true,blob:async()=>png};};
  await d.saveMapImage();
  assert.equal(downloads.length,1);
  d.S.mappingBusy=true;
  d.renderMapping(d.S.data.map_control);
  assert.equal(d.elements.get('mapsave').disabled,true);
  d.S.mappingBusy=false;d.S.data.map=null;
  d.renderMapping(d.S.data.map_control);
  assert.equal(d.elements.get('mapsave').disabled,true);
});

test('canceling reset sends no request', async () => {
  const d = dashboard();
  let requests = 0;
  d.context.fetch = async () => {requests++;};
  d.renderMapping({available: true, paused: true});
  await d.mappingAction('reset');
  assert.equal(requests, 0);
});

test('scan front follows supplied mount TF instead of hardcoded angle', () => {
  const d = dashboard();
  const scan = {amin:0, inc:Math.PI, rs:[.6,.3], rmin:.05,rmax:40,nose_yaw:Math.PI};
  assert.equal(d.bucket(scan,-5,5),.3);
  assert.equal(d.bucket({...scan,nose_yaw:0},-5,5),.6);
  assert.equal(d.bucket({...scan,nose_yaw:null},-5,5),null);
});

test('map driving requires connection, released stop, active map and map pose', () => {
  const d = dashboard();
  const good = {estop: false, map: [10,10,.02], map_control: {paused:false}, pose_available:true,
    calibration_ready:true, calibration:{ready:true}};
  for (const bad of [null, {...good, estop:true}, {...good, map:null},
      {...good, pose_available:false}, {...good, map_control:{paused:true}}]) {
    d.renderNavigation(bad);
    assert.equal(d.elements.get('navexplore').disabled, true);
    assert.equal(d.elements.get('navcoverage').disabled, true);
  }
  d.renderNavigation(good);
  assert.equal(d.elements.get('navexplore').disabled, false);
});

test('pre-reset image cannot return after reset epoch changes', () => {
  const d = dashboard();
  d.S.mapEpoch = 1;
  d.S.data.map = [10, 10, .05, 0, 0, 2];
  d.loadMap(2);
  d.S.mapEpoch = 2;
  d.images[0].onload();
  assert.equal(d.S.img.im, null);
});

test('service failure is visible and is not reported as a successful reset', async () => {
  const d = dashboard();
  await new Promise(resolve => setImmediate(resolve));
  d.S.data.map_control = {available: true, paused: true};
  d.S.pin = {x: 1, y: 2};
  d.context.window.confirm = () => true;
  d.context.fetch = async () => ({ok: false, json: async () => ({message: 'SLAM unavailable'})});
  d.renderMapping(d.S.data.map_control);
  await d.mappingAction('reset');
  assert.equal(d.elements.get('mappingerror').textContent, 'SLAM unavailable');
  assert.equal(d.S.mappingBusy, false);
  assert.equal(d.S.pin.x, 1);
});
