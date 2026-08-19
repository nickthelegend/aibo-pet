/* api/subscribe.js — the waitlist actually goes somewhere now.
 *
 * Until this existed the signup wrote to localStorage and nothing else, which
 * means every email Hotaru collected stayed on the visitor's own machine and
 * never reached anyone. This posts it to Resend and sends the person a
 * confirmation from hotaru@loompad.tech.
 *
 * The API key is a Vercel environment variable. It is never in the repo, and
 * it must never be sent to the browser -- which is the entire reason this is
 * a server function rather than a fetch from app.js.
 */

const FROM = "Hotaru <hotaru@loompad.tech>";
const NOTIFY = "niveshgajengi@gmail.com";   // where the signups land

const okEmail = v => /^[^\s@]+@[^\s@]+\.[^\s@]{2,}$/.test(String(v || "").trim());

function esc(s) {
  return String(s || "").replace(/[<>&"]/g, c =>
    ({ "<": "&lt;", ">": "&gt;", "&": "&amp;", '"': "&quot;" }[c]));
}

async function send(key, payload) {
  const r = await fetch("https://api.resend.com/emails", {
    method: "POST",
    headers: { Authorization: `Bearer ${key}`, "Content-Type": "application/json" },
    body: JSON.stringify(payload),
  });
  const body = await r.json().catch(() => ({}));
  return { ok: r.ok, status: r.status, body };
}

export default async function handler(req, res) {
  if (req.method !== "POST") {
    res.setHeader("Allow", "POST");
    return res.status(405).json({ error: "POST only" });
  }

  const key = process.env.RESEND_API_KEY;
  if (!key) return res.status(500).json({ error: "mail is not configured" });

  let body = req.body;
  if (typeof body === "string") { try { body = JSON.parse(body); } catch { body = {}; } }
  const name = String(body?.name || "").trim().slice(0, 80);
  const email = String(body?.email || "").trim().slice(0, 160);
  const printer = String(body?.printer || "").trim().slice(0, 80);

  if (!okEmail(email)) return res.status(400).json({ error: "bad email" });
  if (name.length < 2) return res.status(400).json({ error: "bad name" });

  const first = esc(name.split(/\s+/)[0]);

  const welcome = await send(key, {
    from: FROM,
    to: [email],
    subject: "You are on the Hotaru list",
    text:
`${name},

You are on the list.

Hotaru is a desk lamp with four servos, a microphone and a speaker. It leans
toward whoever is talking, nods when you answer it, and folds down when you
stop. 22 printed parts, and every one fits a Bambu A1 mini.

You will get one email when the files are verified on real hardware. Not a
newsletter.

In the meantime everything is already public:

  Every part and its STL   https://aibo.loompad.tech/parts
  All 22 STLs as a zip     https://raw.githubusercontent.com/nickthelegend/aibo-pet/main/exports/all-stls.zip
  The CAD                  https://github.com/nickthelegend/aibo-pet

One thing worth knowing before you print: a few component dimensions still
come from listings rather than calipers. Measure your servo and your speaker
before you commit to the tub.

Hotaru
hotaru@loompad.tech`,
    html:
`<div style="font-family:ui-sans-serif,system-ui,sans-serif;line-height:1.55;
  color:#151515;max-width:560px">
  <p style="font-size:20px;font-weight:700;margin:0 0 16px">${first}, you are on the list.</p>
  <p>Hotaru is a desk lamp with four servos, a microphone and a speaker. It
     leans toward whoever is talking, nods when you answer it, and folds down
     when you stop. 22 printed parts, and every one fits a Bambu A1 mini.</p>
  <p>You will get <strong>one email</strong> when the files are verified on real
     hardware. Not a newsletter.</p>
  <p>Everything is already public:</p>
  <ul>
    <li><a href="https://aibo.loompad.tech/parts">Every part and its STL</a></li>
    <li><a href="https://raw.githubusercontent.com/nickthelegend/aibo-pet/main/exports/all-stls.zip">All 22 STLs as a zip</a></li>
    <li><a href="https://github.com/nickthelegend/aibo-pet">The CAD, in pure Python</a></li>
  </ul>
  <p style="color:#555">One thing worth knowing before you print: a few
     component dimensions still come from listings rather than calipers.
     Measure your servo and your speaker before you commit to the tub.</p>
  <p style="color:#555;margin-top:24px">Hotaru · hotaru@loompad.tech</p>
</div>`,
  });

  // The signup notice is best effort: if it fails the person is still
  // subscribed, so it must not turn their success into an error.
  send(key, {
    from: FROM,
    to: [NOTIFY],
    subject: `Hotaru waitlist: ${name}`,
    text: `name: ${name}\nemail: ${email}\nprinter: ${printer || "not asked"}`,
  }).catch(() => {});

  if (!welcome.ok) {
    // Surface Resend's own reason -- an unverified sending domain is the
    // usual one, and a generic 500 would hide it.
    return res.status(502).json({
      error: "mail rejected",
      detail: welcome.body?.message || welcome.body?.name || `status ${welcome.status}`,
    });
  }
  return res.status(200).json({ ok: true, id: welcome.body?.id || null });
}
