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
    'globalThis.mappingTest = {S, cv, fitView, w2c, c2w, screenPoint, setViewAngle, zoomAt, syncView};})();'), context);
  return {context, elements, images, ...context.mappingTest};
}


const near = (a,b) => assert.ok(Math.abs(a-b)<1e-8, `${a} != ${b}`);
test('world cell centers land in raster pixel centers', () => {
  const d=dashboard(); d.cv.width=1000; d.cv.height=700;
  d.S.data.map=[5,4,.02,-.38,-.01,1];d.setViewAngle(0);d.fitView();
  const v=d.S.view, scale=v.k*v.z;
  const point=d.w2c(v.ox+3.5*v.res,v.oy+1.5*v.res);
  near(point[0],v.A.x+3.5*scale);
  near(point[1],v.A.y+2.5*scale);
});
test('rotated map click returns original world coordinate at several angles', () => {
  const d=dashboard(); d.cv.width=1000; d.cv.height=700;
  d.S.data.map=[63,43,.02,-.38,-.01,1];
  for(const angle of [-Math.PI/6,0,Math.PI/3]) {
    d.setViewAngle(angle); d.fitView();
    const raw=d.w2c(.27,.49); const screen=d.screenPoint(...raw);
    const world=d.c2w(...screen); near(world[0],.27); near(world[1],.49);
  }
});
test('view rotation leaves ROS pose untouched and flattens reference heading', () => {
  const d=dashboard(); d.cv.width=1000;d.cv.height=700;
  d.S.data.map=[63,43,.02,-.38,-.01,1];
  d.S.data.pose=[.27,.49,-Math.PI/6]; const before=JSON.stringify(d.S.data.pose);
  d.setViewAngle(d.S.data.pose[2]);d.fitView();
  const a=d.screenPoint(...d.w2c(.27,.49));
  const b=d.screenPoint(...d.w2c(.27+Math.cos(-Math.PI/6),.49+Math.sin(-Math.PI/6)));
  near(a[1],b[1]);assert.ok(b[0]>a[0]);assert.equal(JSON.stringify(d.S.data.pose),before);
});
test('rotated zoom keeps clicked world point fixed', () => {
  const d=dashboard();d.cv.width=1000;d.cv.height=700;
  d.S.data.map=[63,43,.02,-.38,-.01,1];d.setViewAngle(-Math.PI/6);d.fitView();
  const center=[500,350],before=d.c2w(...center);d.zoomAt(...center,2);
  const after=d.c2w(...center);near(before[0],after[0]);near(before[1],after[1]);
});
