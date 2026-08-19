/* bandview.js — the Thenar Band, assembled and coming apart.
 *
 * One 100 kB GLB carries every part in world coordinates; band.json carries
 * the direction each one leaves along. So assembled and exploded are two ends
 * of one interpolation rather than two files, and the scroll position through
 * the hero drives it: the product turns, then takes itself apart as you read
 * about it.
 *
 * Explode is critically damped on purpose. This transition is doing
 * explanatory work (which part came from where), and parts flying past their
 * slot and springing back would misrepresent how the thing goes together.
 */
import { M4, loadGLB, makeGL } from "./gl.js";

export async function mountBandView(cv, opts = {}) {
  const { gl, prog, U } = makeGL(cv);
  const [nodes, meta] = await Promise.all([
    loadGLB("./band.glb"),
    fetch("./band.json").then(r => r.json()),
  ]);

  const info = new Map(meta.parts.map(p => [p.name, p]));

  const parts = nodes.map(n => {
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
    return { name: n.name, vao, count: n.idx.length, colour: n.color, lo, hi,
             itype: n.idx.BYTES_PER_ELEMENT === 2 ? gl.UNSIGNED_SHORT
                                                  : gl.UNSIGNED_INT,
             off: (info.get(n.name) || {}).explode || [0, 0, 0],
             dim: 0, dimT: 0, model: M4.id() };
  });

  // Two framings, not one. Framing only the exploded extent leaves the
  // assembled product small and low in shot; framing only the assembled one
  // throws parts out of frame the moment it opens. So compute both and lerp
  // the camera with the transition.
  function extent(withOffset) {
    const lo = [1e9, 1e9, 1e9], hi = [-1e9, -1e9, -1e9];
    for (const p of parts) for (let k = 0; k < 3; k++) {
      const o = withOffset ? p.off[k] : 0;
      lo[k] = Math.min(lo[k], p.lo[k] + o);
      hi[k] = Math.max(hi[k], p.hi[k] + o);
    }
    return { ctr: [0, 1, 2].map(i => (lo[i] + hi[i]) / 2),
             span: Math.max(...[0, 1, 2].map(i => hi[i] - lo[i])) };
  }
  const A = extent(false), E = extent(true);

  const reduced = matchMedia("(prefers-reduced-motion: reduce)").matches;
  const st = { explode: 0, target: 0, az: -0.7, spin: true };
  const mix = (a, b, t) => a + (b - a) * t;

  function frame(now) {
    const dpr = Math.min(devicePixelRatio || 1, 2);
    const w = cv.clientWidth | 0, h = cv.clientHeight | 0;
    if (!w || !h) { requestAnimationFrame(frame); return; }
    if (cv.width !== w * dpr || cv.height !== h * dpr) {
      cv.width = w * dpr; cv.height = h * dpr;
    }

    st.explode += (st.target - st.explode) * (reduced ? 0.5 : 0.11);
    if (st.spin && !reduced) st.az = -0.7 + now / 7000;

    gl.viewport(0, 0, cv.width, cv.height);
    gl.clearColor(0, 0, 0, 0);
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

    const e0 = st.explode;
    const ctr = [0, 1, 2].map(i => mix(A.ctr[i], E.ctr[i], e0));
    const dist = mix(A.span * 1.45, E.span * 1.35, e0);
    const el = 0.30;
    const eye = [ctr[0] + dist * Math.cos(el) * Math.sin(st.az),
                 ctr[1] + dist * Math.cos(el) * Math.cos(st.az),
                 ctr[2] + dist * Math.sin(el)];
    const V = M4.look(eye, ctr, [0, 0, 1]);
    const P = M4.persp(0.6, cv.width / cv.height, 5, 4000);

    gl.useProgram(prog);
    for (const p of parts) {
      p.dim += (p.dimT - p.dim) * 0.16;
      const e = st.explode;
      const M = M4.trans(p.off[0] * e, p.off[1] * e, p.off[2] * e);
      const mv = M4.mul(V, M);
      gl.uniformMatrix4fv(U.mv, false, mv);
      gl.uniformMatrix4fv(U.mvp, false, M4.mul(P, mv));
      // dimming is a lerp toward the page background, so a highlighted part
      // reads without needing a second shader
      const k = 1 - p.dim * 0.82;
      gl.uniform3fv(U.col, new Float32Array(p.colour.map(c => c * k)));
      gl.bindVertexArray(p.vao);
      gl.drawElements(gl.TRIANGLES, p.count, p.itype, 0);
    }
    requestAnimationFrame(frame);
  }
  requestAnimationFrame(frame);

  // drag to orbit; dragging stops the idle spin so a visitor can hold a view
  let drag = null;
  cv.addEventListener("pointerdown", e => {
    drag = e.clientX; st.spin = false; cv.setPointerCapture(e.pointerId);
  });
  addEventListener("pointerup", () => { drag = null; });
  addEventListener("pointermove", e => {
    if (drag === null) return;
    st.az -= (e.clientX - drag) * 0.009;
    drag = e.clientX;
  });

  cv.closest(".stage3d")?.classList.add("ready");

  return {
    explode(v) { st.target = Math.max(0, Math.min(1, v)); },
    highlight(name) { for (const p of parts) p.dimT = !name || p.name === name ? 0 : 1; },
    parts: meta.parts,
    mount: meta.mount,
  };
}
