/* app.js — the live lamp.
 *
 * exports/aibo-assembled.glb has the arm baked at one pose. web/hotaru-rig.glb
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
if (!gl) $("#skel").textContent = "This browser does not support WebGL2.";

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
  vec3 c = col * (0.22 + 0.86*key + 0.26*fill) + vec3(0.30,0.20,0.55)*rim*0.55;
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

  /* 2.0's base joint is a PAN about Z, not a tilt about X, so it cannot
   * join the cumulative rotX chain the other three form. It becomes an
   * OUTER transform on everything that turns with the disc, and the chain
   * below it starts from zero. rig.panParts is what marks a v2 rig; a v1
   * rig has none and keeps the original single-chain behaviour. */
  const panning = Array.isArray(RIG.panParts);
  const outer = panning ? M4.rotZ(cur.base) : M4.id();
  let cum = panning ? 0 : cur.base;
  if (panning) {
    for (const nm of RIG.panParts) {
      const q = ROLE.get(nm);
      if (q) q.model = outer;
    }
  }

  RIG.segments.forEach((seg, i) => {
    const M = M4.mul(outer,
                     M4.mul(M4.trans(p[0], p[1], p[2]), M4.rotX(cum)));
    for (const nm of seg.parts.concat(seg.servo)) {
      const part = ROLE.get(nm);
      if (part) part.model = M;
    }
    const a = cum * Math.PI / 180;
    p = [p[0], p[1] - seg.length * Math.sin(a), p[2] + seg.length * Math.cos(a)];
    cum += cur[J[i + 1]];
  });
  const shM = M4.mul(outer,
                     M4.mul(M4.trans(p[0], p[1], p[2]), M4.rotX(cum + 180)));
  for (const nm of (RIG.shadeParts || [RIG.shade])) {
    const sh = ROLE.get(nm);
    if (sh) sh.model = shM;
  }
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
  idle:    { label: "Idle",    loop: true, keys: [
    [0.0, {}], [2.2, { shoulder: 2.5, head: -2.0 }], [4.4, {}] ] },
  nod:     { label: "Nod", loop: false, keys: [
    [0.00, {}], [0.16, { head: 24, shoulder: -5 }], [0.34, { head: -12 }],
    [0.50, { head: 20, shoulder: -3 }], [0.66, { head: -7 }], [0.90, {}] ] },
  curious: { label: "Curious", loop: false, keys: [
    [0.00, {}], [0.30, { base: 13, shoulder: -14, elbow: 12, head: -26 }],
    [1.30, { base: 13, shoulder: -14, elbow: 12, head: -26 }], [1.75, {}] ] },
  perk:    { label: "Perk up", loop: false, keys: [
    [0.00, {}], [0.14, { shoulder: -8, elbow: -10, head: -18 }],
    [0.30, { base: -12, shoulder: -22, elbow: -18, head: -30 }],
    [0.95, { base: -12, shoulder: -22, elbow: -18, head: -30 }], [1.5, {}] ] },
  sulk:    { label: "Sulk",    loop: false, keys: [
    [0.00, {}], [0.75, { base: 16, shoulder: 20, elbow: 22, head: 34 }],
    [2.20, { base: 16, shoulder: 20, elbow: 22, head: 34 }], [3.0, {}] ] },
  scan:    { label: "Scan",    loop: false, keys: [
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
  // the stage is a tall full-viewport panel now, so leave more air
  dist = (span * 1.46) / (2 * Math.tan(FOVY / 2));
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
  gl.clearColor(0.094, 0.094, 0.094, 1);   // --surface #181818
  gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

  // The sheet occupies the bottom of the stage, so the model is framed
  // against the visible area above it rather than the whole canvas.
  // Title scrim owns the top, sheet the bottom. Centre the model in what is
  // left between them rather than in the raw canvas.
  const t = document.querySelector(".title");
  const sh = document.querySelector(".sheet");
  const top = t ? t.getBoundingClientRect().height : 0;
  const bot = sh ? sh.getBoundingClientRect().height * 0.55 : 0;
  const bias = ((top - bot) / cv.clientHeight) * 210;
  const ctr = [CTR[0], CTR[1], CTR[2] + bias];
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


/* =====================================================================
 * UI
 * ===================================================================== */

/* Play a movement by name. The conversation drives this; there is no manual
 * chip row on the character stage. */
function play(k) { if (MOODS[k]) { mood = k; moodT = 0; } }

/* ---- voice -----------------------------------------------------------
 * Pre-rendered mp3, not speechSynthesis. lisa.locomotive.ca does the same
 * thing -- its network log is a bank of lisa.intro.3.mp3 style files -- and
 * for the same reason: the browser voice is robotic, differs on every
 * machine, and on some has no usable English voice at all. cad/build_voice.py
 * renders the bank; this only ever plays audio.
 *
 * Off until asked for. Browsers block unprompted audio anyway, and a page
 * that starts talking at you is hostile. */
const voice = {
  // ON by default. It was off, which meant the visitor had to find a toggle
  // before the thing ever spoke -- so in practice nobody heard it. Browsers
  // still block audio until a real gesture, so `unlocked` gates the first
  // play and any click, tap or key anywhere opens it.
  on: true, unlocked: false, bank: null, cur: null, pending: null,
  async load() {
    try { this.bank = (await (await fetch("./voice/lines.json")).json()).lines; }
    catch { this.bank = null; }
  },
  play(key, onEnd) {
    this.stop();
    if (!this.on || !this.bank || !this.bank[key]) { onEnd && onEnd(); return; }
    if (!this.unlocked) { this.pending = key; onEnd && onEnd(); return; }
    const vs = this.bank[key];
    const pick = vs[Math.floor(Math.random() * vs.length)];
    const a = new Audio(pick.src);
    a.volume = 0.9;
    a.onended = () => onEnd && onEnd();
    a.onerror = () => onEnd && onEnd();
    this.cur = a;
    a.play().catch(() => onEnd && onEnd());
  },
  stop() { if (this.cur) { this.cur.pause(); this.cur = null; } },

  /* First gesture anywhere unlocks audio and speaks whatever is on screen.
     Listeners are `once` and cover pointer, touch and keyboard so it opens
     however the visitor arrived. */
  arm(onOpen) {
    const open = () => {
      if (this.unlocked) return;
      this.unlocked = true;
      document.body.classList.add("audio-on");
      const k = this.pending; this.pending = null;
      if (k) this.play(k);
      onOpen && onOpen();
    };
    ["pointerdown", "keydown", "touchstart"].forEach(e =>
      addEventListener(e, open, { once: true, passive: true }));
  },
};

/* ---- the conversation ----------------------------------------------
 * A graph, not a queue: every answer returns to the menu, and the way out is
 * always on screen. `say` is the voice key AND the id of the line text, so a
 * line can never drift from its audio. */
const SCRIPT = {
  intro:  { say: "intro",
            chips: [["How does it move?", "move"], ["What do I need?", "need"],
                    ["How long to print?", "print"], ["Why?", "why"],
                    ["Send me the files", "signup", true]] },
  menu:   { say: "menu", chips: null },   // chips filled from intro
  move:   { say: "move", back: true },
  need:   { say: "need", back: true },
  print:  { say: "print", back: true },
  why:    { say: "why", back: true },
};

const talk = {
  node: "intro", timer: null, answers: {}, step: null,
  el: {},

  boot() {
    this.el = { line: $("#line"), chips: $("#chips"), err: $("#conv-err") };
    $("#restart").onclick = () => { this.answers = {}; this.go("intro", "again"); };
    const on = $("#snd-on"), off = $("#snd-off");
    voice.arm(() => { const p = $("#tap"); if (p) p.remove(); });
    const set = v => {
      voice.on = v;
      on.setAttribute("aria-pressed", String(v));
      off.setAttribute("aria-pressed", String(!v));
      if (!v) voice.stop();
    };
    set(voice.on);                       // on by default
    on.onclick = () => { voice.unlocked = true; set(true); voice.play(this.lastKey); };
    off.onclick = () => set(false);
    this.go("intro");
  },

  /* Which movement goes with which line. The character reacting to its own
     words is what makes it read as one thing rather than a model next to a
     transcript. */
  moodFor(key) {
    return ({ intro: "perk", again: "perk", menu: "curious",
              move: "nod", need: "curious", print: "curious", why: "sulk",
              ask_name: "curious", ask_printer: "curious",
              ask_email: "curious", bad_email: "sulk", done: "nod"
            })[key] || "curious";
  },

  /* Type the line and start its audio together. The typing is the primary
     channel: if audio is off or fails, nothing waits on it. */
  type(text, key, after) {
    clearInterval(this.timer);
    const el = this.el.line;
    el.textContent = ""; el.classList.add("typing"); el.classList.remove("done");
    this.lastKey = key;
    play(this.moodFor(key));
    voice.play(key);
    let i = 0;
    this.timer = setInterval(() => {
      el.textContent = text.slice(0, ++i);
      if (i >= text.length) {
        clearInterval(this.timer);
        el.classList.remove("typing"); el.classList.add("done");
        after && after();
      }
    }, 16);
  },

  lineFor(key) {
    const vs = voice.bank && voice.bank[key];
    return vs ? vs[0].text : "";
  },

  chips(list) {
    this.el.chips.innerHTML = list.map(([label, to, go]) =>
      `<button class="chip${go ? " go" : ""}" type="button" data-to="${to}">${label}</button>`
    ).join("");
    const bs = [...this.el.chips.querySelectorAll(".chip")];
    bs.forEach((b, k) => {
      setTimeout(() => b.classList.add("in"), 60 * k);
      b.onclick = () => this.go(b.dataset.to);
    });
  },

  field(placeholder, type, onSubmit) {
    this.el.chips.innerHTML =
      `<input id="conv-in" type="${type}" placeholder="${placeholder}"
              aria-label="${placeholder}" autocomplete="${type === "email" ? "email" : "name"}">
       <button class="chip go" type="button" id="conv-go">Continue</button>`;
    const i = $("#conv-in"), b = $("#conv-go");
    setTimeout(() => { i.classList.add("in"); b.classList.add("in"); }, 40);
    const send = () => onSubmit(i.value);
    b.onclick = send;
    i.onkeydown = e => { if (e.key === "Enter") { e.preventDefault(); send(); } };
    i.focus({ preventScroll: true });
  },

  go(node, sayKey) {
    this.el.err.textContent = "";
    this.el.chips.innerHTML = "";
    if (node === "signup") return this.askName();
    const n = SCRIPT[node] || SCRIPT.intro;
    const key = sayKey || n.say;
    this.type(this.lineFor(key), key, () => {
      const opts = n.back
        ? [...SCRIPT.intro.chips.filter(c => c[1] !== node)]
        : SCRIPT.intro.chips;
      this.chips(opts);
    });
  },

  askName() {
    this.type(this.lineFor("ask_name"), "ask_name", () =>
      this.field("Your name", "text", v => {
        if (v.trim().length < 2) { this.el.err.textContent = "That is a little short."; return; }
        this.answers.name = v.trim();
        this.askPrinter();
      }));
  },
  askPrinter() {
    this.type(this.lineFor("ask_printer"), "ask_printer", () => {
      const opts = ["Bambu P1S", "Bambu A1 mini", "Prusa", "Something else", "No printer yet"];
      this.el.chips.innerHTML = opts.map(o =>
        `<button class="chip" type="button" data-v="${o}">${o}</button>`).join("");
      [...this.el.chips.querySelectorAll(".chip")].forEach((b, k) => {
        setTimeout(() => b.classList.add("in"), 60 * k);
        b.onclick = () => { this.answers.printer = b.dataset.v; this.askEmail(); };
      });
    });
  },
  askEmail() {
    this.type(this.lineFor("ask_email"), "ask_email", () =>
      this.field("you@example.com", "email", v => {
        if (!/^[^\s@]+@[^\s@]+\.[^\s@]{2,}$/.test(v.trim())) {
          this.el.err.textContent = this.lineFor("bad_email") || "That does not look like an email address.";
          voice.play("bad_email");
          $("#conv-in").setAttribute("aria-invalid", "true");
          return;
        }
        this.answers.email = v.trim();
        this.finish();
      }));
  },
  async finish() {
    // Local copy first, so a network failure never loses what they typed.
    const KEY = "hotaru-waitlist";
    const list = JSON.parse(localStorage.getItem(KEY) || "[]");
    if (!list.some(x => x && x.email === this.answers.email)) list.push(this.answers);
    localStorage.setItem(KEY, JSON.stringify(list));

    this.type(this.lineFor("sending"), "sending");
    this.el.chips.innerHTML = "";

    let ok = false, why = "";
    try {
      const r = await fetch("./api/subscribe", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(this.answers),
      });
      const j = await r.json().catch(() => ({}));
      ok = r.ok && j.ok;
      why = j.detail || j.error || "";
    } catch (e) { why = e.message; }

    if (ok) {
      this.type(this.lineFor("done"), "done", () => {
        this.el.chips.innerHTML =
          `<span class="chip in" style="cursor:default">Sent to ${this.answers.email}</span>
           <a class="chip go in" href="./parts.html">See every part</a>`;
      });
    } else {
      // Say what actually went wrong rather than pretending it worked.
      this.type(this.lineFor("send_failed"), "send_failed", () => {
        this.el.chips.innerHTML =
          `<button class="chip go in" type="button" id="retry">Try again</button>
           <a class="chip in" href="./parts.html">See every part</a>`;
        $("#retry").onclick = () => this.finish();
      });
      if (why) this.el.err.textContent = why;
    }
  },
};

/* ---- mood chips ---- */
function buildMoods() {
  if (!$("#moods")) return;          // the character stage has no chip row
  $("#moods").innerHTML = Object.entries(MOODS)
    .filter(([k]) => k !== "idle")
    .map(([k, m]) => `<button class="mood" type="button" data-k="${k}"
        aria-pressed="false">${m.label}</button>`).join("");
  document.querySelectorAll(".mood").forEach(b => b.onclick = () => {
    mood = b.dataset.k; moodT = 0; syncButtons();
  });
}
function syncButtons() {
  if (!document.querySelector(".mood")) return;
  document.querySelectorAll(".mood").forEach(b =>
    b.setAttribute("aria-pressed", String(b.dataset.k === mood)));
}

/* ------------------------------------------------------------- boot ---- */
(async function boot() {
  await voice.load();
  talk.boot();

  try {
    const [nodes, rig] = await Promise.all([
      loadGLB("./hotaru2-rig.glb"),
      fetch("./rig2.json").then(r => r.json()),
    ]);
    RIG = rig; RIG.yokeBelow = 16.0;
    // the rig carries the viewer's loose-parts tray; the hero is a portrait
    const loose = new Set(rig.loose || []);
    PARTS = upload(nodes.filter(n => !loose.has(n.name)));
    PARTS.forEach(p => ROLE.set(p.name, p));
    for (const j of J) { cur[j] = RIG.neutral[j]; vel[j] = 0; tgt[j] = cur[j]; }

    fitCamera();
    const sk = $("#skel");
    sk.style.opacity = "0";
    setTimeout(() => sk.style.display = "none", 420);
    requestAnimationFrame(frame);
  } catch (e) {
    $("#skel").textContent = "The model could not load. " + e.message;
    console.error(e);
  }
})();
