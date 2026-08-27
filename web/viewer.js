/* viewer.js — the parts browser.
 *
 * Reuses web/hotaru2-rig.glb rather than exports/v2/hotaru2-assembled.glb
 * (6.9 MB): the rig already has every part indexed, welded and normal-free,
 * and it carries the chain, so assembled and exploded are both just poses
 * rather than two more files to ship.
 *
 * Download links point at raw.githubusercontent.com on refs/heads/main, so
 * a future rebuild replaces what they serve instead of breaking them.
 */
import { M4, loadGLB, makeGL } from "./gl.js?v=3d7212e4";

const $ = s => document.querySelector(s);
const cv = $("#vc");
const { gl, prog, U } = makeGL(cv);

let PARTS = [], RIG = null, SPEC = null;
let az = -0.62, el = 0.14, dist = 700, drag = null;
let explode = 0, explodeTarget = 0;        // 0 = assembled, 1 = exploded
// When a plate is selected we show its parts THE WAY THEY SLICE, not the way
// they sit in the robot. The shoulder is the reason: its fix was a 180 degree
// print flip, and in assembly pose that fix is completely invisible.
let printMode = null;                      // null = assembled, else Set of names
const CTR = [0, 0, 170];

/* Subsystem grouping, same rules the model viewer uses. First match wins and
   "Other" catches the rest, so a part can never silently drop off the list. */
const RULES = [
  ["BASE",     n => /^(base|shoulder|lid|base-joint|cap-base)$/.test(n)],
  ["ARM",      n => /^(arm-|cap-(shoulder|elbow|head)|shade)/.test(n)],
  ["DRIVE",    n => /^(horn-|yoke-screw|spline-test)/.test(n)],
  ["RETAINERS",n => /^(spk-clamp|mic-tab|esp-tab|amp-tab|keycap)$/.test(n)],
  ["SERVOS",   n => /^(mg996r|sg90)-/.test(n)],
  ["BOARDS",   n => /^(esp32|amp|mic|spk|mx)-/.test(n)],
  ["LOOSE PARTS", n => (RIG.loose || []).includes(n)],
  ["OTHER",    () => true],
];

/* Explode: push each part along the arm chain, so it comes apart the way it
   goes together rather than scattering. Static base parts lift straight up
   by their stack order; arm segments slide out along their own axis. */
const LIFT = { base:0, shoulder:26, lid:52, "base-joint":86, "cap-base":110 };

function explodeOffset(name, i) {
  if (name in LIFT) return [0, 0, LIFT[name] * 1.0];
  if (/^(esp32|amp|mic|spk|mx|keycap|spk-clamp|mic-tab|esp-tab|amp-tab)/.test(name))
    return [0, 0, -70];                       // internals drop out the bottom
  if ((RIG.loose || []).includes(name)) return [90, 0, 0];
  const seg = RIG.segments.findIndex(s =>
    s.parts.includes(name) || s.servo.includes(name));
  if (seg >= 0) return [0, 0, 60 + seg * 55];
  if (name === RIG.shade) return [0, 0, 250];
  return [0, 0, 0];
}

function poseParts() {
  if (!RIG) return;
  const piv = RIG.pivot, N = RIG.neutral;
  const J = ["base", "shoulder", "elbow", "head"];

  /* Same two 2.0 rules app.js follows: the base joint is a PAN about Z that
   * wraps the whole chain, and the cone is a GROUP (shell, LED cap, ring),
   * not the single part v1's rig named. Posing only RIG.shade left the cap
   * and the ring sitting at the world origin under the base. */
  const panning = Array.isArray(RIG.panParts);
  const outer = panning ? M4.rotZ(N.base) : M4.id();
  let p = [piv[0], piv[1], piv[2]], cum = panning ? 0 : N.base;
  if (panning) {
    for (const nm of RIG.panParts) {
      const part = PARTS.find(x => x.name === nm);
      if (part) part.base = outer;
    }
  }

  RIG.segments.forEach((seg, i) => {
    const M = M4.mul(outer,
                     M4.mul(M4.trans(p[0], p[1], p[2]), M4.rotX(cum)));
    for (const nm of seg.parts.concat(seg.servo)) {
      const part = PARTS.find(x => x.name === nm);
      if (part) part.base = M;
    }
    const a = cum * Math.PI / 180;
    p = [p[0], p[1] - seg.length * Math.sin(a), p[2] + seg.length * Math.cos(a)];
    cum += N[J[i + 1]];
  });
  const shM = M4.mul(outer,
                     M4.mul(M4.trans(p[0], p[1], p[2]), M4.rotX(cum + 180)));
  for (const nm of (RIG.shadeParts || [RIG.shade])) {
    const sh = PARTS.find(x => x.name === nm);
    if (sh) sh.base = shM;
  }

  if (printMode) {
    // print pose: rotate about X by `flip`, then drop the part onto Z = 0,
    // which is exactly the transform cad/assembly.print_items() records
    for (const part of PARTS) {
      const pp = (SPEC.print_pose || {})[part.name] || { flip: 0, dz: 0 };
      part.model = M4.mul(M4.trans(0, 0, pp.dz), M4.rotX(pp.flip));
    }
    return;
  }
  for (const part of PARTS) {
    const o = explodeOffset(part.name);
    part.model = M4.mul(M4.trans(o[0] * explode, o[1] * explode, o[2] * explode),
                        part.base || M4.id());
  }
}

/* ------------------------------------------------------------- list ---- */
function buildList() {
  const bucket = new Map(RULES.map(([t]) => [t, []]));
  for (const p of PARTS) bucket.get(RULES.find(([, m]) => m(p.name))[0]).push(p);
  const dl = SPEC.downloads;
  let html = "";
  for (const [title] of RULES) {
    const items = bucket.get(title);
    if (!items.length) continue;
    html += `<div class="grp">${title} · ${items.length}</div>`;
    for (const p of items) {
      const url = dl.files[p.name];
      const rgb = p.color.map(c => Math.round(255 * Math.pow(c, 1 / 2.2))).join(",");
      html += `<div class="row" data-n="${p.name}">
        <span class="ck"></span>
        <span class="sw" style="background:rgb(${rgb})"></span>
        <span class="nm">${p.name}</span>
        ${url ? `<a class="dl" href="${url}" download
                   title="raw STL, always the newest on main">STL</a>` : ""}
      </div>`;
    }
  }
  html = `<div class="grp">EVERY PART &middot; <a class="dl" style="opacity:1;transform:none"
      href="${dl.zip}" download>ZIP ${SPEC.downloads.zip_mb} MB</a></div>` + html;
  $("#list").innerHTML = html;
  $("#list").querySelectorAll(".row").forEach(r => {
    r.addEventListener("click", e => {
      if (e.target.closest(".dl")) return;      // let the download through
      const p = PARTS.find(x => x.name === r.dataset.n);
      p.on = !p.on;
      r.classList.toggle("off", !p.on);
    });
  });
}

/* Plates. A plate is the unit somebody actually prints, so each row says what
   is on it, how tall it stands, and whether supports are needed. The supports
   flag is measured by cad/audit_printable.py and carried through spec.json --
   this page never decides it, because a second opinion about overhangs is a
   second thing to keep in sync. */
function buildPlates() {
  const P = SPEC.plates;
  if (!P || !P.list) { $("#plates").innerHTML = "<div class='grp'>NO PLATE DATA</div>"; return; }
  const bed = P.bed_mm;
  let html = `<div class="grp">${P.list.length} PLATES &middot; ${bed}MM BED</div>`;
  for (const p of P.list) {
    const [x, y, z] = p.used_mm;
    const label = p.plate.replace(/^plate-/, "").replace(/-/g, " ").toUpperCase();
    const chips = [`<span class="chip">${p.parts.length} PART${p.parts.length > 1 ? "S" : ""}</span>`,
                   `<span class="chip">${x}&times;${y}&times;${z}MM</span>`];
    if (z > bed * 0.75) chips.push(`<span class="chip tall">TALL</span>`);
    if (p.supports.length) chips.push(`<span class="chip sup">SUPPORTS</span>`);
    html += `<div class="plate" data-p="${p.plate}">
      <div class="t"><span class="n">${label}</span><span class="mb">${p.mb} MB</span></div>
      <div class="why">${p.why}</div>
      <div class="meta">${chips.join("")}
        <a class="go" href="${p.url}" download>STL</a></div>
    </div>`;
  }
  $("#plates").innerHTML = html;

  // Clicking a plate isolates exactly what is on it, so "what am I printing"
  // is answered in the viewport rather than by reading a filename.
  const byName = new Map(SPEC.plates.list.map(p => [p.plate, p.parts]));
  $("#plates").querySelectorAll(".plate").forEach(row => {
    row.addEventListener("click", e => {
      if (e.target.closest(".go")) return;          // let the download through
      const on = row.classList.contains("on");
      $("#plates").querySelectorAll(".plate").forEach(r => r.classList.remove("on"));
      const keep = on ? null : new Set(byName.get(row.dataset.p));
      if (!on) row.classList.add("on");
      for (const part of PARTS) part.on = keep ? keep.has(part.name) : true;
      printMode = keep;
      $("#modes").style.opacity = keep ? ".35" : "1";
      $("#modes").style.pointerEvents = keep ? "none" : "auto";
      $("#printbadge").hidden = !keep;
      poseParts();
      frameVisible();
      $("#list").querySelectorAll(".row").forEach(r =>
        r.classList.toggle("off", keep ? !keep.has(r.dataset.n) : false));
    });
  });
}

/* ------------------------------------------------------------ input ---- */
cv.addEventListener("pointerdown", e => {
  drag = { x: e.clientX, y: e.clientY }; cv.setPointerCapture(e.pointerId);
});
addEventListener("pointerup", () => drag = null);
addEventListener("pointermove", e => {
  if (!drag) return;
  az -= (e.clientX - drag.x) * 0.008;
  el = Math.max(-0.5, Math.min(1.2, el + (e.clientY - drag.y) * 0.006));
  drag = { x: e.clientX, y: e.clientY };
});
cv.addEventListener("wheel", e => {
  e.preventDefault();
  dist = Math.max(220, Math.min(1800, dist * (1 + Math.sign(e.deltaY) * 0.09)));
}, { passive: false });

$("#modes").addEventListener("click", e => {
  const b = e.target.closest("button"); if (!b) return;
  $("#modes").querySelectorAll("button").forEach(x =>
    x.classList.toggle("on", x === b));
  explodeTarget = b.dataset.m === "exploded" ? 1 : 0;
});

$("#tabs").addEventListener("click", e => {
  const b = e.target.closest("button"); if (!b) return;
  $("#tabs").querySelectorAll("button").forEach(x => x.classList.toggle("on", x === b));
  const plates = b.dataset.t === "plates";
  $("#plates").hidden = !plates;
  $("#list").hidden = plates;
  $("#toggleall").hidden = plates;
  if (!plates) {                       // back to PARTS: drop out of print pose
    printMode = null;
    $("#plates").querySelectorAll(".plate").forEach(r => r.classList.remove("on"));
    $("#printbadge").hidden = true;
    $("#modes").style.opacity = "1";
    $("#modes").style.pointerEvents = "auto";
    for (const p of PARTS) p.on = true;
    $("#list").querySelectorAll(".row").forEach(r => r.classList.remove("off"));
    poseParts(); frameVisible();
  }
});

$("#toggleall").addEventListener("click", () => {
  const anyOn = PARTS.some(p => p.on);
  PARTS.forEach(p => p.on = !anyOn);
  $("#list").querySelectorAll(".row").forEach(r => r.classList.toggle("off", anyOn));
  $("#toggleall").textContent = anyOn ? "SHOW ALL" : "HIDE ALL";
});

/* Refit the camera to whatever is currently visible. Selecting one plate out
   of 66 parts otherwise leaves the model a speck in the middle of the view. */
function frameVisible() {
  const vis = PARTS.filter(p => p.on);
  if (!vis.length) return;
  const lo = [1e9, 1e9, 1e9], hi = [-1e9, -1e9, -1e9];
  for (const p of vis) {
    const m = p.model;
    for (let c = 0; c < 8; c++) {
      const v = [c & 1 ? p.hi[0] : p.lo[0], c & 2 ? p.hi[1] : p.lo[1],
                 c & 4 ? p.hi[2] : p.lo[2]];
      for (let k = 0; k < 3; k++) {
        const w = m[k]*v[0] + m[4+k]*v[1] + m[8+k]*v[2] + m[12+k];
        if (w < lo[k]) lo[k] = w;
        if (w > hi[k]) hi[k] = w;
      }
    }
  }
  CTR[0] = (lo[0]+hi[0])/2; CTR[1] = (lo[1]+hi[1])/2; CTR[2] = (lo[2]+hi[2])/2;
  dist = Math.max(hi[0]-lo[0], hi[1]-lo[1], hi[2]-lo[2]) * 2.1 + 40;
}

/* ------------------------------------------------------------- loop ---- */
const REDUCED = matchMedia("(prefers-reduced-motion: reduce)").matches;
function frame() {
  // Explode is eased rather than snapped: this transition is the one piece of
  // motion on the page doing real explanatory work -- it shows which part came
  // from where. Critically damped, no overshoot; parts flying past their slot
  // and settling back would misrepresent the assembly.
  const k = REDUCED ? 0.5 : 0.12;
  explode += (explodeTarget - explode) * k;
  if (Math.abs(explodeTarget - explode) < 1e-4) explode = explodeTarget;
  poseParts();

  const dpr = Math.min(devicePixelRatio || 1, 2);
  const w = cv.clientWidth | 0, h = cv.clientHeight | 0;
  if (cv.width !== w * dpr || cv.height !== h * dpr) {
    cv.width = w * dpr; cv.height = h * dpr;
  }
  gl.viewport(0, 0, cv.width, cv.height);
  gl.clearColor(0.094, 0.094, 0.094, 1);   // --surface #181818
  gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

  const eye = [CTR[0] + dist * Math.cos(el) * Math.sin(az),
               CTR[1] + dist * Math.cos(el) * Math.cos(az),
               CTR[2] + dist * Math.sin(el)];
  const V = M4.look(eye, CTR, [0, 0, 1]);
  const P = M4.persp(0.62, cv.width / cv.height, 10, 5000);

  gl.useProgram(prog);
  for (const p of PARTS) {
    if (!p.on) continue;
    const mv = M4.mul(V, p.model);
    gl.uniformMatrix4fv(U.mv, false, mv);
    gl.uniformMatrix4fv(U.mvp, false, M4.mul(P, mv));
    gl.uniform3fv(U.col, p.color);
    gl.bindVertexArray(p.vao);
    gl.drawElements(gl.TRIANGLES, p.count, p.itype, 0);
  }
  requestAnimationFrame(frame);
}

/* ------------------------------------------------------------- boot ---- */
(async function () {
  try {
    const [nodes, rig, spec] = await Promise.all([
      loadGLB("./hotaru2-rig.glb"),
      fetch("./rig2.json").then(r => r.json()),
      fetch("./spec2.json").then(r => r.json()),
    ]);
    RIG = rig; SPEC = spec;
    PARTS = nodes.map(n => ({ ...uploadPart(n), on: true }));
    poseParts();

    const b = bounds();
    CTR[1] = (b.lo[1] + b.hi[1]) / 2; CTR[2] = (b.lo[2] + b.hi[2]) / 2;
    dist = Math.max(...[0, 1, 2].map(i => b.hi[i] - b.lo[i])) * 1.9;

    buildList();
    buildPlates();
    const dz = spec.downloads;
    $("#zip").href = dz.plates_zip || dz.zip;
    $("#zip").textContent = dz.plates_zip
      ? `DOWNLOAD PLATES · ${dz.plates_zip_mb} MB`
      : `DOWNLOAD ALL STLs · ${dz.zip_mb} MB`;
    $("#vload").style.opacity = "0";
    setTimeout(() => $("#vload").style.display = "none", 260);
    requestAnimationFrame(frame);
  } catch (e) {
    $("#vload").textContent = "COULD NOT LOAD: " + e.message;
    console.error(e);
  }
})();

function uploadPart(n) {
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
           model: M4.id(), base: M4.id() };
}

function bounds() {
  const lo = [1e9, 1e9, 1e9], hi = [-1e9, -1e9, -1e9];
  for (const p of PARTS) {
    const m = p.model;
    for (let c = 0; c < 8; c++) {
      const v = [c & 1 ? p.hi[0] : p.lo[0], c & 2 ? p.hi[1] : p.lo[1],
                 c & 4 ? p.hi[2] : p.lo[2]];
      for (let k = 0; k < 3; k++) {
        const w = m[k]*v[0] + m[4+k]*v[1] + m[8+k]*v[2] + m[12+k];
        if (w < lo[k]) lo[k] = w;
        if (w > hi[k]) hi[k] = w;
      }
    }
  }
  return { lo, hi };
}
