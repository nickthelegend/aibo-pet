/* chain.js — the settlement ledger, running.
 *
 * "Blockchain" on a landing page is usually a picture of a glowing cube. The
 * only thing worth showing here is the actual unit of work: one accepted clip,
 * hashed, written with its terms, paid for. So this draws the ledger itself.
 *
 * Blocks accrete right to left. Each carries a payload hash, the three
 * channels the Band records, and what the contributor was paid. The amounts
 * are fractions of a cent, which is the whole reason the settlement layer has
 * to be cheap: a fee of even one cent would cost more than the payment.
 *
 * Figures are illustrative of the model, not a live feed. The page says so.
 */
const HEX = "0123456789abcdef";
const BLUE = "#4D17F5", PINK = "#FA9DCD", MUTE = "#6E6E6E";
const FG = "#FFFFFF", DIM = "#9B9B9B", LINE = "#272727";

const BW = 104, GAP = 14, SPEED = 26;   // px wide, px gap, px per second

/* Small deterministic PRNG. Blocks must look varied but must not reshuffle
   every frame, so each block gets its values once, from its own index. */
function rng(seed) {
  let s = (seed * 2654435761) >>> 0;
  return () => {
    s ^= s << 13; s >>>= 0;
    s ^= s >> 17;
    s ^= s << 5;  s >>>= 0;
    return s / 4294967296;
  };
}

function makeBlock(i) {
  const r = rng(i + 1);
  let h = "";
  for (let k = 0; k < 4; k++) h += HEX[(r() * 16) | 0];
  let t = "";
  for (let k = 0; k < 4; k++) t += HEX[(r() * 16) | 0];
  // sub cent, varied, never a round number
  const amt = 0.0018 + r() * 0.0121;
  return {
    i, hash: h + "…" + t, amt,
    ch: [0.55 + r() * 0.45, 0.35 + r() * 0.6, 0.5 + r() * 0.5],
  };
}

export function mountChain(cv) {
  const ctx = cv.getContext("2d");
  const reduced = matchMedia("(prefers-reduced-motion: reduce)").matches;
  let t0 = null, total = 0, counted = -1;

  function draw(now) {
    if (t0 === null) t0 = now;
    const dpr = Math.min(devicePixelRatio || 1, 2);
    const w = cv.clientWidth | 0, h = cv.clientHeight | 0;
    if (!w || !h) { requestAnimationFrame(draw); return; }
    if (cv.width !== w * dpr || cv.height !== h * dpr) {
      cv.width = w * dpr; cv.height = h * dpr;
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);

    const elapsed = reduced ? 34 : (now - t0) / 1000;
    const travelled = elapsed * SPEED;
    const pitch = BW + GAP;
    const newest = Math.floor(travelled / pitch);

    // running total accumulates once per block rather than per frame
    while (counted < newest) {
      counted++;
      total += makeBlock(counted).amt;
    }

    const top = 52, bh = h - top - 46;

    // the rail the blocks sit on
    ctx.strokeStyle = LINE; ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(0, top + bh + 12.5); ctx.lineTo(w, top + bh + 12.5);
    ctx.stroke();

    const count = Math.ceil(w / pitch) + 2;
    for (let k = 0; k < count; k++) {
      const idx = newest - k;
      if (idx < 0) continue;
      const b = makeBlock(idx);
      const bx = w - 40 - BW - (travelled - idx * pitch);
      if (bx + BW < -20 || bx > w + 20) continue;

      // a block flashes as it lands, then settles to its resting state
      const age = (travelled - idx * pitch) / SPEED;
      const fresh = Math.max(0, 1 - age / 0.9);

      ctx.fillStyle = fresh > 0.02 ? `rgba(77,23,245,${0.10 + fresh * 0.30})`
                                   : "rgba(31,31,31,0.9)";
      ctx.strokeStyle = fresh > 0.02 ? BLUE : LINE;
      ctx.lineWidth = 1;
      round(ctx, bx, top, BW, bh, 10);
      ctx.fill(); ctx.stroke();

      ctx.font = "600 11px Manrope, system-ui, sans-serif";
      ctx.fillStyle = fresh > 0.02 ? FG : DIM;
      ctx.fillText(b.hash, bx + 12, top + 22);

      // the three channels the Band records, as stacked bars
      const bw2 = BW - 24;
      [PINK, BLUE, MUTE].forEach((c, ci) => {
        const y = top + 34 + ci * 9;
        ctx.fillStyle = "rgba(255,255,255,.07)";
        ctx.fillRect(bx + 12, y, bw2, 4);
        ctx.fillStyle = c;
        ctx.fillRect(bx + 12, y, bw2 * b.ch[ci], 4);
      });

      ctx.font = "700 13px Manrope, system-ui, sans-serif";
      ctx.fillStyle = fresh > 0.02 ? FG : DIM;
      ctx.fillText("$" + b.amt.toFixed(4), bx + 12, top + bh - 10);
    }

    // header: what the strip is
    ctx.font = "700 10px Manrope, system-ui, sans-serif";
    ctx.fillStyle = MUTE;
    ctx.fillText("ACCEPTED CONTRIBUTIONS, SETTLED PER CLIP", 2, 16);

    const label = "PAID OUT  $" + total.toFixed(4);
    ctx.fillStyle = BLUE;
    ctx.fillText(label, w - ctx.measureText(label).width - 2, 16);

    ctx.font = "600 10px Manrope, system-ui, sans-serif";
    ctx.fillStyle = MUTE;
    ctx.fillText("ILLUSTRATIVE OF THE MODEL, NOT A LIVE FEED", 2, h - 8);

    if (!reduced) requestAnimationFrame(draw);
  }

  function round(c, x, y, w, h, r) {
    c.beginPath();
    c.moveTo(x + r, y);
    c.arcTo(x + w, y, x + w, y + h, r);
    c.arcTo(x + w, y + h, x, y + h, r);
    c.arcTo(x, y + h, x, y, r);
    c.arcTo(x, y, x + w, y, r);
    c.closePath();
  }

  requestAnimationFrame(draw);
}
