/* app.js — the live lamp.
 *
 * exports/aibo-assembled.glb has the arm baked at one pose. web/aibo-rig.glb
 * (cad/export_web.py) instead ships every moving segment in its OWN frame,
 * sitting on its yoke pivot pointing +Z, plus rig.json describing how they
 * chain. This rebuilds the pose every frame from four joint angles, which is
 * the same maths assembly.world_items() runs once at build time.
 *
 * The GLB carries no normals -- they are computed per-face in the fragment
 * shader from screen-space derivatives, which is what let the file go from
 * 14.5 MB to 2.1 MB. Flat shading is also the correct look for FDM parts.
 */

const $ = s => document.querySelector(s);

/* ------------------------------------------------------------ maths ---- */
const M4 = {
  id: () => new Float32Array([1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]),
  mul(a, b) {
    const o = new Float32Array(16);
    for (let c = 0; c < 4; c++) for (let r = 0; r < 4; r++) {
      let s = 0;
      for (let k = 0; k < 4; k++) s += a[k * 4 + r] * b[c * 4 + k];
      o[c * 4 + r] = s;
    }
    return o;
  },
  trans(x, y, z) { const m = M4.id(); m[12] = x; m[13] = y; m[14] = z; return m; },
  rotX(d) {
    const a = d * Math.PI / 180, c = Math.cos(a), s = Math.sin(a), m = M4.id();
    m[5] = c; m[6] = s; m[9] = -s; m[10] = c; return m;
  },
  rotZ(d) {
    const a = d * Math.PI / 180, c = Math.cos(a), s = Math.sin(a), m = M4.id();
    m[0] = c; m[1] = s; m[4] = -s; m[5] = c; return m;
  },
  persp(fovy, asp, n, f) {
    const t = 1 / Math.tan(fovy / 2), m = new Float32Array(16);
    m[0] = t / asp; m[5] = t; m[10] = (f + n) / (n - f);
    m[11] = -1; m[14] = 2 * f * n / (n - f); return m;
  },
  look(eye, tgt, up) {
    const s = (a, b) => [a[0]-b[0], a[1]-b[1], a[2]-b[2]];
    const nrm = v => { const l = Math.hypot(...v) || 1; return [v[0]/l, v[1]/l, v[2]/l]; };
    const cr = (a, b) => [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];
    const z = nrm(s(eye, tgt)), x = nrm(cr(up, z)), y = cr(z, x);
    return new Float32Array([
      x[0], y[0], z[0], 0, x[1], y[1], z[1], 0, x[2], y[2], z[2], 0,
      -(x[0]*eye[0]+x[1]*eye[1]+x[2]*eye[2]),
      -(y[0]*eye[0]+y[1]*eye[1]+y[2]*eye[2]),
      -(z[0]*eye[0]+z[1]*eye[1]+z[2]*eye[2]), 1]);
  },
};

/* ------------------------------------------------------------- glb ----- */
async function loadGLB(url) {
  const buf = await (await fetch(url)).arrayBuffer();
  const dv = new DataView(buf);
  if (dv.getUint32(0, true) !== 0x46546C67) throw new Error("not a GLB");
  let off = 12, json = null, bin = null;
  while (off + 8 <= dv.byteLength) {
    const len = dv.getUint32(off, true), type = dv.getUint32(off + 4, true);
    const body = buf.slice(off + 8, off + 8 + len);
    if (type === 0x4E4F534A) json = JSON.parse(new TextDecoder().decode(body));
    else if (type === 0x004E4942) bin = body;
    off += 8 + len + ((4 - (len % 4)) % 4);
  }
  const acc = i => {
    const a = json.accessors[i], v = json.bufferViews[a.bufferView];
    const o = v.byteOffset || 0, n = a.type === "VEC3" ? 3 : 1;
    if (a.componentType === 5126) return new Float32Array(bin, o, a.count * n);
    if (a.componentType === 5123) return new Uint16Array(bin, o, a.count);
    return new Uint32Array(bin, o, a.count);
  };
  return json.nodes.map(nd => {
    const p = json.meshes[nd.mesh].primitives[0];
    const c = json.materials[p.material].pbrMetallicRoughness.baseColorFactor;
    return { name: nd.name, pos: acc(p.attributes.POSITION),
             idx: acc(p.indices), color: [c[0], c[1], c[2]] };
  });
}

/* -------------------------------------------------------------- gl ----- */
const cv = $("#c");
const gl = cv.getContext("webgl2", { antialias: true, alpha: false });
if (!gl) $("#load").textContent = "THIS BROWSER HAS NO WEBGL2";

const VS = `#version 300 es
in vec3 p; uniform mat4 mvp, mv; out vec3 vp;
void main(){ vp = (mv * vec4(p,1.)).xyz; gl_Position = mvp * vec4(p,1.); }`;

// Flat normal from derivatives -- the file ships no NORMAL attribute.
const FS = `#version 300 es
precision highp float;
in vec3 vp; uniform vec3 col; out vec4 o;
void main(){
  vec3 n = normalize(cross(dFdx(vp), dFdy(vp)));
  float key  = max(dot(n, normalize(vec3(0.45, 0.55, 0.85))), 0.0);
  float fill = max(dot(n, normalize(vec3(-0.6, -0.2, 0.35))), 0.0);
  float rim  = pow(1.0 - max(dot(n, vec3(0,0,1)), 0.0), 2.5);
  vec3 c = col * (0.30 + 0.80*key + 0.22*fill) + vec3(0.30,0.20,0.55)*rim*0.30;
  o = vec4(pow(c, vec3(1.0/2.2)), 1.0);
}`;

function sh(type, src) {
  const s = gl.createShader(type);
  gl.shaderSource(s, src); gl.compileShader(s);
  if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(s));
  return s;
}
const prog = gl.createProgram();
gl.attachShader(prog, sh(gl.VERTEX_SHADER, VS));
gl.attachShader(prog, sh(gl.FRAGMENT_SHADER, FS));
gl.linkProgram(prog); gl.useProgram(prog);
const U = { mvp: gl.getUniformLocation(prog, "mvp"),
            mv:  gl.getUniformLocation(prog, "mv"),
            col: gl.getUniformLocation(prog, "col") };
gl.enable(gl.DEPTH_TEST);

/* ------------------------------------------------------------ rig ------ */
let PARTS = [], RIG = null, ROLE = new Map();

function upload(nodes) {
  return nodes.map(n => {
    const vao = gl.createVertexArray();
    gl.bindVertexArray(vao);
    const vb = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, vb);
    gl.bufferData(gl.ARRAY_BUFFER, n.pos, gl.STATIC_DRAW);
    gl.enableVertexAttribArray(0);
    gl.vertexAttribPointer(0, 3, gl.FLOAT, false, 0, 0);
    const ib = gl.createBuffer();
    gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, ib);
    gl.bufferData(gl.ELEMENT_ARRAY_BUFFER, n.idx, gl.STATIC_DRAW);
    gl.bindVertexArray(null);
    const lo = [1e9, 1e9, 1e9], hi = [-1e9, -1e9, -1e9];
    for (let i = 0; i < n.pos.length; i += 3)
      for (let k = 0; k < 3; k++) {
        if (n.pos[i + k] < lo[k]) lo[k] = n.pos[i + k];
        if (n.pos[i + k] > hi[k]) hi[k] = n.pos[i + k];
      }
    return { name: n.name, vao, count: n.idx.length, color: n.color, lo, hi,
             itype: n.idx.BYTES_PER_ELEMENT === 2 ? gl.UNSIGNED_SHORT
                                                  : gl.UNSIGNED_INT,
             model: M4.id() };
  });
}

/* Joint angles. `cur` is what is drawn; `tgt` is where the behaviour wants
 * it. A critically-ish damped spring gets from one to the other -- that is
 * what makes a nod read as a nod rather than a lerp: it arrives fast, tips
 * a hair past, and settles. */
const J = ["base", "shoulder", "elbow", "head"];
const cur = {}, vel = {}, tgt = {};

function poseParts() {
  if (!RIG) return;
  const piv = RIG.pivot;
  let p = [piv[0], piv[1], piv[2]];
  let cum = cur.base;
  const below = RIG.yokeBelow;

  RIG.segments.forEach((seg, i) => {
    const M = M4.mul(M4.trans(p[0], p[1], p[2]), M4.rotX(cum));
    for (const nm of seg.parts.concat(seg.servo)) {
      const part = ROLE.get(nm);
      if (part) part.model = M;
    }
    const a = cum * Math.PI / 180;
    p = [p[0], p[1] - seg.length * Math.sin(a), p[2] + seg.length * Math.cos(a)];
    cum += cur[J[i + 1]];
  });
  const sh = ROLE.get(RIG.shade);
  if (sh) sh.model = M4.mul(M4.trans(p[0], p[1], p[2]), M4.rotX(cum + 180));
}

/* World bounds of the pose as drawn, from each part's 8 local AABB corners.
 * Used to FIT the camera instead of hand-tuning a distance: perk-up throws
 * the shade ~90 mm higher than neutral and sulk folds it almost onto the
 * base, so a number picked to look right at rest clips at both extremes. */
function worldBounds() {
  const lo = [1e9, 1e9, 1e9], hi = [-1e9, -1e9, -1e9];
  for (const p of PARTS) {
    const m = p.model;
    for (let c = 0; c < 8; c++) {
      const v = [c & 1 ? p.hi[0] : p.lo[0], c & 2 ? p.hi[1] : p.lo[1],
                 c & 4 ? p.hi[2] : p.lo[2]];
      for (let k = 0; k < 3; k++) {
        const w = m[k] * v[0] + m[4 + k] * v[1] + m[8 + k] * v[2] + m[12 + k];
        if (w < lo[k]) lo[k] = w;
        if (w > hi[k]) hi[k] = w;
      }
    }
  }
  return { lo, hi };
}

/* ------------------------------------------------------- behaviours ---- */
/* Each is a timeline of [t seconds, {joint: offset from neutral}]. Offsets,
 * not absolutes, so every move reads relative to the resting pose. */
const MOODS = {
  idle:    { label: "IDLE",    loop: true, keys: [
    [0.0, {}], [2.2, { shoulder: 2.5, head: -2.0 }], [4.4, {}] ] },
  nod:     { label: "NOD YES", loop: false, keys: [
    [0.00, {}], [0.16, { head: 24, shoulder: -5 }], [0.34, { head: -12 }],
    [0.50, { head: 20, shoulder: -3 }], [0.66, { head: -7 }], [0.90, {}] ] },
  curious: { label: "CURIOUS", loop: false, keys: [
    [0.00, {}], [0.30, { base: 13, shoulder: -14, elbow: 12, head: -26 }],
    [1.30, { base: 13, shoulder: -14, elbow: 12, head: -26 }], [1.75, {}] ] },
  perk:    { label: "PERK UP", loop: false, keys: [
    [0.00, {}], [0.14, { shoulder: -8, elbow: -10, head: -18 }],
    [0.30, { base: -12, shoulder: -22, elbow: -18, head: -30 }],
    [0.95, { base: -12, shoulder: -22, elbow: -18, head: -30 }], [1.5, {}] ] },
  sulk:    { label: "SULK",    loop: false, keys: [
    [0.00, {}], [0.75, { base: 16, shoulder: 20, elbow: 22, head: 34 }],
    [2.20, { base: 16, shoulder: 20, elbow: 22, head: 34 }], [3.0, {}] ] },
  scan:    { label: "SCAN",    loop: false, keys: [
    [0.00, {}], [0.45, { base: -9, head: -20 }], [1.05, { base: 11, head: 14 }],
    [1.65, { base: -6, head: -12 }], [2.2, {}] ] },
};
let mood = "idle", moodT = 0;

function sampleMood(name, t) {
  const m = MOODS[name], k = m.keys, end = k[k.length - 1][0];
  if (m.loop) t = t % end; else t = Math.min(t, end);
  let i = 0;
  while (i < k.length - 2 && t > k[i + 1][0]) i++;
  const [t0, a] = k[i], [t1, b] = k[i + 1];
  const u = t1 === t0 ? 0 : (t - t0) / (t1 - t0);
  const e = u < 0.5 ? 4*u*u*u : 1 - Math.pow(-2*u + 2, 3) / 2;   // easeInOutCubic
  const o = {};
  for (const j of J) o[j] = (a[j] || 0) + ((b[j] || 0) - (a[j] || 0)) * e;
  return o;
}

/* ------------------------------------------------------------ camera --- */
function fitCamera() {
  const save = mood, saveT = moodT;
  const lo = [1e9, 1e9, 1e9], hi = [-1e9, -1e9, -1e9];
  for (const k of Object.keys(MOODS)) {
    mood = k; moodT = 0;
    // Union along the WHOLE timeline, not at the end of it. Sampling only
    // the settled pose measured idle every time -- update() flips a
    // non-looping mood back to idle the moment its last key passes, so the
    // extremes this is supposed to frame had already been thrown away.
    const span = MOODS[k].keys[MOODS[k].keys.length - 1][0];
    for (let i = 0; i < Math.ceil(span * 60) + 12; i++) {
      update(1 / 60);
      const b = worldBounds();
      for (let j = 0; j < 3; j++) {
        lo[j] = Math.min(lo[j], b.lo[j]); hi[j] = Math.max(hi[j], b.hi[j]);
      }
      if (mood !== k) break;                 // it reverted; extremes are in
    }
  }
  mood = save; moodT = saveT;
  for (const j of J) { cur[j] = RIG.neutral[j]; vel[j] = 0; }
  update(1 / 60);
  CTR = [0, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2];
  const span = Math.max(hi[2] - lo[2], hi[0] - lo[0], hi[1] - lo[1]);
  dist = (span * 1.16) / (2 * Math.tan(FOVY / 2));
}


// Framed so the EXTREMES stay in shot -- perk-up reaches ~90 mm higher
// than neutral and sulk drops the shade almost to the base.
const FOVY = 0.62;
let CTR = [0, 6, 190];
let az = -0.62, el = 0.10, dist = 790, drag = null;
cv.addEventListener("pointerdown", e => { drag = { x: e.clientX, y: e.clientY };
  cv.setPointerCapture(e.pointerId); });
addEventListener("pointerup", () => drag = null);
addEventListener("pointermove", e => {
  if (!drag) return;
  az -= (e.clientX - drag.x) * 0.008;
  el = Math.max(-0.5, Math.min(1.1, el + (e.clientY - drag.y) * 0.006));
  drag = { x: e.clientX, y: e.clientY };
});
cv.addEventListener("wheel", e => {
  e.preventDefault();
  dist = Math.max(220, Math.min(1600, dist * (1 + Math.sign(e.deltaY) * 0.09)));
}, { passive: false });

/* -------------------------------------------------------------- loop --- */
/* One animation step. Split out of frame() so it can be driven with a fixed
 * dt from a test -- requestAnimationFrame does not tick in a headless pane,
 * which made "is it actually moving?" impossible to answer from a screenshot. */
function update(dt) {
  moodT += dt;
  const want = sampleMood(mood, moodT);
  if (!MOODS[mood].loop && moodT > MOODS[mood].keys[MOODS[mood].keys.length - 1][0]) {
    mood = "idle"; moodT = 0; syncButtons();
  }
  for (const j of J) {
    const lim = RIG ? RIG.range[j] : [-40, 40];
    const nz = RIG ? RIG.neutral[j] : 0;
    tgt[j] = nz + Math.max(lim[0], Math.min(lim[1], want[j] || 0));
    // Spring, damped just UNDER critical so it overshoots a few degrees and
    // settles. That overshoot is the whole difference between "a servo
    // moved" and "something alive moved" -- a lerp reads as neither.
    const k = 210, c = 2 * Math.sqrt(k) * 0.72;
    vel[j] += ((tgt[j] - cur[j]) * k - vel[j] * c) * dt;
    cur[j] += vel[j] * dt;
  }
  poseParts();
}

function draw() {
  const dpr = Math.min(devicePixelRatio || 1, 2);
  const w = cv.clientWidth | 0, h = cv.clientHeight | 0;
  if (cv.width !== w * dpr || cv.height !== h * dpr) {
    cv.width = w * dpr; cv.height = h * dpr;
  }
  gl.viewport(0, 0, cv.width, cv.height);
  gl.clearColor(0.933, 0.933, 0.925, 1);
  gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

  const ctr = CTR;
  const eye = [ctr[0] + dist * Math.cos(el) * Math.sin(az),
               ctr[1] + dist * Math.cos(el) * Math.cos(az),
               ctr[2] + dist * Math.sin(el)];
  const V = M4.look(eye, ctr, [0, 0, 1]);
  const P = M4.persp(FOVY, cv.width / cv.height, 10, 4000);

  gl.useProgram(prog);
  for (const p of PARTS) {
    const mv = M4.mul(V, p.model);
    gl.uniformMatrix4fv(U.mv, false, mv);
    gl.uniformMatrix4fv(U.mvp, false, M4.mul(P, mv));
    gl.uniform3fv(U.col, p.color);
    gl.bindVertexArray(p.vao);
    gl.drawElements(gl.TRIANGLES, p.count, p.itype, 0);
  }
}

let last = performance.now();
function frame(now) {
  const dt = Math.min((now - last) / 1000, 0.05); last = now;
  update(dt);
  draw();
  requestAnimationFrame(frame);
}

/* Exposed so the joint chain can be inspected from the console -- and so a
 * screenshot test can prove the thing actually moves rather than eyeballing
 * two stills taken a second apart. */
window.__aibo = {
  get pose() { return { ...cur }; },
  get mood() { return mood; },
  play(k) { if (MOODS[k]) { mood = k; moodT = 0; syncButtons(); } },
  step: update,
  draw,
  get cam() { return { CTR, dist, az, el, FOVY }; },
  bounds: worldBounds,
  /* Drive the chain with a fixed dt and report the joint track. No rAF, so
     this works headless and gives the same numbers every run. */
  sample(k, seconds = 1.0, dt = 1 / 60) {
    this.play(k);
    const out = [];
    for (let t = 0; t < seconds; t += dt) {
      update(dt);
      out.push({ t: +(t + dt).toFixed(3), ...cur });
    }
    return out;
  },
};

/* --------------------------------------------------------------- ui ---- */
function syncButtons() {
  document.querySelectorAll(".mood").forEach(b =>
    b.classList.toggle("on", b.dataset.k === mood));
}
function buildMoods() {
  $("#moods").innerHTML = Object.entries(MOODS)
    .filter(([k]) => k !== "idle")
    .map(([k, m]) => `<button class="mood" data-k="${k}">${m.label}</button>`).join("");
  document.querySelectorAll(".mood").forEach(b => b.onclick = () => {
    mood = b.dataset.k; moodT = 0; syncButtons();
  });
}

const TICKER = ["NODS WHEN YOU TALK", "SULKS WHEN IGNORED", "22 PRINTED PARTS",
  "FITS AN A1 MINI", "ONE SCREW YOU PRINT", "FOUR SERVOS", "ONE WEEKEND BUILD",
  "OPEN SOURCE CAD"];
$("#run").innerHTML = [...TICKER, ...TICKER]
  .map(t => `<span>${t} <i>✦</i></span>`).join("");

const CAPS = [
  ["01", "Nods and shakes", "Head tilt plus a shoulder counter-move. The overshoot is what sells it as alive rather than actuated."],
  ["02", "Follows the room", "INMP441 mic on the front wall picks direction; the arm leans toward whoever is talking."],
  ["03", "Wakes to a word", "ESP32-S3 runs wake-word inference on-device. Nothing leaves the desk."],
  ["04", "Talks back", "MAX98357A into a 40 × 20 driver, with the whole 749 cm³ tub as its back volume."],
  ["05", "Sulks", "Ignore it long enough and the whole arm folds down. It is a posture, not a light."],
  ["06", "Perks up", "Fast extension on all four joints — the move it makes when it notices you."],
  ["07", "One real button", "A single Cherry MX on the lid. Blue cap. That is the entire interface."],
  ["08", "Reaches 345 mm", "316 mm tall in its neutral pose, and it can put the shade anywhere in that radius."],
  ["09", "Rebuilds itself", "Every dimension is one parameter. Change the servo, re-run, re-print."],
];
$("#caps").innerHTML = CAPS.map(([n, h, p]) =>
  `<div class="cap"><div class="n">${n}</div><h3>${h}</h3><p>${p}</p></div>`).join("");

/* ------------------------------------------------------------- boot ---- */
(async function boot() {
  try {
    const [nodes, rig, spec] = await Promise.all([
      loadGLB("./aibo-rig.glb"),
      fetch("./rig.json").then(r => r.json()),
      fetch("./spec.json").then(r => r.json()),
    ]);
    RIG = rig; RIG.yokeBelow = 16.0;
    // The rig also carries the loose-parts tray for the viewer. The hero is a
    // portrait of the lamp, so they are dropped here rather than floating
    // beside it.
    const loose = new Set(rig.loose || []);
    PARTS = upload(nodes.filter(n => !loose.has(n.name)));
    PARTS.forEach(p => ROLE.set(p.name, p));
    for (const j of J) { cur[j] = RIG.neutral[j]; vel[j] = 0; tgt[j] = cur[j]; }

    const S = spec.stats;
    $("#stats").innerHTML = [
      [S.parts, "printed parts"], [S.interfaces, "interfaces"],
      [S.joints, "driven joints"], [S.plates, "print plates"],
      [S.height_mm + "mm", "tall, posed"], [S.reach_mm + "mm", "reach"],
      [S.plastic_cm3, "cm³ of plastic"], [S.m3 + "+" + S.m2, "M3 + M2 screws"],
    ].map(([b, s]) => `<div class="stat"><b>${b}</b><span>${s}</span></div>`).join("");

    $("#parts-tbl").innerHTML =
      "<tr><th>part</th><th>size mm</th><th>cm³</th><th>A1 mini</th></tr>" +
      spec.parts.map(p => `<tr><td class="nm">${p.name}</td>
        <td class="mono">${p.mm[0]} × ${p.mm[1]} × ${p.mm[2]}</td>
        <td class="mono">${p.cm3}</td>
        <td><span class="pill">${p.fits ? "fits" : "NO"}</span></td></tr>`).join("");

    $("#iface-tbl").innerHTML =
      "<tr><th>interface</th><th>what wants to separate</th><th>held by</th></tr>" +
      spec.interfaces.map(i => `<tr><td class="nm">${i.a} → ${i.b}</td>
        <td>${i.dof}</td><td>${i.held_by}</td></tr>`).join("");

    fitCamera();
    buildMoods(); syncButtons();
    revealOnScroll();
    const ld = $("#load");
    ld.style.opacity = "0";
    setTimeout(() => ld.style.display = "none", 260);
    requestAnimationFrame(frame);
  } catch (e) {
    $("#load").textContent = "COULD NOT LOAD: " + e.message;
    console.error(e);
  }
})();

/* Scroll reveal. Cards, stats and section bodies used to appear fully formed
 * the moment they crossed the fold. This bridges that with a 12 px rise and a
 * fade, staggered 55 ms across a group.
 *
 * Strictly decorative, so it is built to be harmless: elements are already
 * interactive before it runs, anything still unrevealed when the observer is
 * unavailable simply shows, and reduced-motion drops the travel in CSS. */
function revealOnScroll() {
  const groups = [
    ...document.querySelectorAll("#caps .cap"),
    ...document.querySelectorAll("#stats .stat"),
    ...document.querySelectorAll("section .eyebrow, section h2, .scroller, .wait > div"),
  ];
  if (!("IntersectionObserver" in window)) return;   // leave them visible
  groups.forEach(el => el.classList.add("reveal"));
  const io = new IntersectionObserver((entries) => {
    const hit = entries.filter(e => e.isIntersecting);
    hit.forEach((e, i) => {
      const el = e.target;
      el.style.transitionDelay = Math.min(i * 55, 220) + "ms";
      el.classList.add("in");
      io.unobserve(el);
    });
  }, { rootMargin: "0px 0px -8% 0px", threshold: 0.08 });
  groups.forEach(el => io.observe(el));
}

/* waitlist — local only, and the page says so */
const KEY = "aibo-waitlist";
const list = () => JSON.parse(localStorage.getItem(KEY) || "[]");
$("#wcount").textContent = list().length;
$("#wf").addEventListener("submit", e => {
  e.preventDefault();
  const v = $("#em").value.trim().toLowerCase();
  const L = list();
  if (v && !L.includes(v)) { L.push(v); localStorage.setItem(KEY, JSON.stringify(L)); }
  $("#wcount").textContent = L.length;
  $("#ok").classList.add("show");
  $("#em").value = "";
});
