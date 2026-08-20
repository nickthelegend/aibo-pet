"""pages.py — the body content for every Thenar page.

Separated from build_site.py so the chrome and the argument can be edited
without stepping on each other.
"""

def sec(eyebrow, h2, lede, body="", pad=""):
    return f'''  <section class="fwrap"{pad}>
    <p class="feyebrow">{eyebrow}</p>
    <h2 class="fh2">{h2}</h2>
    <p class="flede" style="margin-top:var(--s-300)">{lede}</p>
{body}
  </section>
'''

def tiles(items, cls="bento"):
    out = [f'    <div class="{cls}" style="margin-top:var(--s-600)">']
    for t in items:
        spec = ""
        if t.get("spec"):
            spec = '\n          <div class="spec">' + "".join(
                f'<div>{k}<b>{v}</b></div>' for k, v in t["spec"]) + '</div>'
        tag = f'<span class="tg {t.get("tagcls","")}">{t["tag"]}</span>' if t.get("tag") else ""
        out.append(f'''      <article class="tile"{t.get("style","")}>
        <h3>{t["h"]}</h3>
        <p>{t["p"]}</p>{spec}
        {f'<div class="tagbar">{tag}</div>' if tag else ''}
      </article>''')
    out.append('    </div>')
    return "\n".join(out)


# ----------------------------------------------------------------- products
PRODUCTS = sec(
  "What we build",
  "Four things, and<br>only one is finished",
  "Two instruments, one protocol, and a robot that exists to prove we can take "
  "hardware from parameters to a shipped object. Each one is labelled with what "
  "state it is actually in.",
  tiles([
    {"h":"Thenar Band","tag":"In design","p":
     "A wrist unit worn while you do ordinary work. Vision, inertial and pressure "
     "across the thenar eminence and the fingertips, sampled against one clock in "
     "the device rather than aligned in software afterwards. Four printed parts: a "
     "C shaped cuff that springs over the wrist, a vented bay under a snap fit cap, "
     "a camera on a three lug bayonet that twists 60 degrees to a hard stop, and a "
     "pressure pad on a compliant arm that stays loaded as the thumb opposes.",
     "spec":[("BORE","58 &times; 46 mm"),("BAYONET","3 &times; 34&deg;, 60&deg; twist"),
             ("STATE","Watertight CAD, nobody has worn one")]},
    {"h":"Quest 3S capture","tag":"Running","tagcls":"live","p":
     "Teleoperation on hardware anyone can buy. Six degrees of freedom on both "
     "hands at 90 Hz, head pose, and the operator's own segmentation of a task, "
     "because a person doing teleoperation naturally breaks work into attempts. "
     "This is what we actually record today, and it is deliberately the data source "
     "for everything we ship in the next twelve weeks.",
     "spec":[("TRACKING","6DoF, both hands"),("RATE","90 Hz"),
             ("GRIP FORCE","None. That is the point.")]},
    {"h":"GRASP protocol","tag":"Building","p":
     "Signed capture manifests, hash only commitments anchored on Avalanche, a "
     "public verifier anybody runs against mainnet without touching our servers, "
     "and licence settlement in USDC that binds payment to a specific corpus root "
     "and a specific version of the terms in one transaction. Built against Quest "
     "capture so it does not wait on hardware.",
     "spec":[("CHAIN","Avalanche C-Chain"),("ON CHAIN","Hashes and terms only"),
             ("VERIFIER","Public, no credentials")]},
    {"h":"Hotaru","tag":"Shipping","tagcls":"live","p":
     "An open source desk robot: four servos, a microphone and a speaker across 22 "
     "printed parts, every one on a 180 mm bed, CAD in pure Python with published "
     "STLs and interface audits anyone can rerun. It is not a data product and we "
     "do not present it as one. It is evidence about geometry, tolerancing and "
     "release discipline, which is a narrower claim than it looks.",
     "spec":[("PARTS","22"),("CAD","Pure Python, open"),("STATE","Shipped")]},
  ])
) + '''  <section class="fwrap">
    <div class="note" style="margin:0 auto">
      <b>What Hotaru does not prove.</b> Printing a servo desk robot is evidence
      about geometry, tolerancing and shipping discipline. It says nothing about
      transduction, drift, calibration across different hands, or yield, which is
      the part that actually kills force sensing wearables. Those are ahead of us.
    </div>
  </section>
'''

# ----------------------------------------------------------------- protocol
PROTOCOL = sec(
  "GRASP on Avalanche",
  "Hashes on chain.<br>Everything else off it.",
  "A buyer needs to verify a slice of capture data without trusting our internal "
  "records, and a contributor needs to withdraw without us being able to pretend "
  "they did not. That is the whole job. Here is the design, and here is the part "
  "it does not solve.",
  tiles([
    {"h":"Why the C-Chain, and not our own L1","p":
     "Because the workload does not have the shape that justifies a chain of its "
     "own, and claiming it does is the first thing a reviewer kills. Batched into "
     "an hourly anchor, our entire provenance traffic is 24 transactions a day. A "
     "dedicated L1 would cost roughly 4,100 dollars a year against about 88, but "
     "the money is not the argument: it is that an L1 needs a part time operator we "
     "do not have. The decisive reason is settlement. USDC is native and deep on "
     "the C-Chain; on our own L1 it would be a bridged representation secured by "
     "five nodes we run ourselves, which is a company with no revenue underwriting "
     "buyer funds."},
    {"h":"One leaf per accepted clip","p":
     "A 154 byte preimage hashing to a 32 byte leaf: payload hash, sensor manifest "
     "hash, a consent commitment, the terms id, capture and submission timestamps, "
     "duration, scope bits. No identity, no payload, no free text. The consent "
     "commitment uses a fresh random salt on every submission, because a stable "
     "pseudonymous identifier written on chain is one no erasure request can ever "
     "undo, and that single mistake would break the entire privacy position."},
    {"h":"A monotonic head, not a bag of roots","p":
     "The log is an append only Merkle tree in the Certificate Transparency style, "
     "living off chain and anchored hourly with its previous root and its size. "
     "Publishing a pile of independent roots would prove nothing about ordering; a "
     "monotonic head with a size is what makes a consistency proof possible. The "
     "chain anchors the log. The chain is not the log."},
    {"h":"Revocation needs a non membership proof","p":
     "Withdrawal is the hard half. A buyer has to be able to prove a clip's consent "
     "was NOT revoked as of a given root, and an inclusion list cannot express "
     "that, so revocations live in a sparse Merkle tree anchored in the same call. "
     "One extra word per anchor. The verifier reports the block at which a "
     "withdrawal became publicly knowable, which is the fact a buyer's counsel "
     "actually needs."},
    {"h":"Payment and terms in one transaction","style":' style="border-color:var(--blue)"',"p":
     "This is the only part where a chain genuinely beats the alternatives. Off "
     "chain, the claim that a buyer paid for a corpus under a specific version of "
     "the licence is our word. On chain it is a USDC transfer and a state write in "
     "one transaction that neither side can revise afterwards, pinned to the log "
     "size at the moment of sale so an inclusion proof three years later is "
     "checkable against a head that provably existed."},
    {"h":"What it does not prove","p":
     "A ledger attests record integrity after submission, not authenticity at "
     "capture. Nothing written on chain shows a stream came off a real hand rather "
     "than a replay, and a device key can be extracted. Closing that needs a secure "
     "element and a remote attestation path, and the Band CAD does not have one. "
     "Avalanche shipped a P-256 precompile that would verify exactly those "
     "signatures cheaply when the hardware exists. It does not exist."},
  ])
) + '''  <section class="fwrap">
    <p class="feyebrow">Protocol Camp, cohort 10</p>
    <h2 class="fh2">The twelve weeks</h2>
    <p class="flede" style="margin-top:var(--s-300)">
      Demo days land at week 6 and week 12, so both dates carry something a
      stranger can verify on their own laptop. Two weeks are deliberately empty,
      because a plan with no slack is a plan that has not been costed.</p>
    <div class="weeks">
      <ol>
        <li><b>1</b><span>Freeze the manifest schema. Canonical JSON, channel descriptors, golden test vectors, and a reserved grip force descriptor that is not used yet.</span></li>
        <li><b>2</b><span>Deterministic packer. Quest capture in, chunked channels and a signed manifest out. CI proves two machines produce byte identical roots.</span></li>
        <li><b>3</b><span>Registry contracts on Fuji, source verified, non upgradeable. First real corpus root committed. External reviewers confirmed in writing this week, not in week 10.</span></li>
        <li><b>4</b><span>Verifier CLI on npm. Recomputes from the spec, reads the anchor over a public RPC, prints MATCH or names the exact broken leaf.</span></li>
        <li><b>5</b><span>Static explorer, published as JSON rather than a database, because it is a convenience and never the source of truth.</span></li>
        <li class="demo"><b>6</b><span>DEMO DAY. A stranger verifies a corpus on their own machine with no credentials. Then we flip one byte and the verifier localises it.</span></li>
        <li><b>7</b><span>Licence escrow. Purchase in USDC, receipt and terms hash written in the same transaction. No contributor split: there are zero contributors.</span></li>
        <li><b>8</b><span>Revocation, the sparse tree, and the remedy when a withdrawal lands after a sale. That last part is the only genuinely new problem here.</span></li>
        <li class="buf"><b>9</b><span>Buffer. Named as buffer so it does not quietly become week 8 running late.</span></li>
        <li><b>10</b><span>Contract freeze, static analysis, invariant tests, published external findings, mainnet deploy with verified source.</span></li>
        <li class="buf"><b>11</b><span>Buffer, then a transparency log mirror so a buyer who wants no chain at all gets the same tamper evidence.</span></li>
        <li class="demo"><b>12</b><span>DEMO DAY. Mainnet, end to end, driven by people who are not us, with a written statement of what the ledger does not prove.</span></li>
      </ol>
    </div>

    <div class="note" style="margin:var(--s-600) auto 0">
      <b>What week 12 is, honestly.</b> A live purchase on mainnet for a small
      real amount is a working settlement path. It is not a market signal, and we
      will not present it as one. Revenue is zero, signed licences are zero, and
      the buyer at a demo day is somebody in the room.
    </div>
  </section>

''' + sec(
  "Scope discipline",
  "What we are not<br>building in twelve weeks",
  "Cut deliberately, because scope discipline is part of what is being judged and "
  "an over full plan is the easiest thing in the world to write.",
  tiles([
    {"h":"Not the Band as a data source","p":"Nobody has worn one and no force data will exist by December. The protocol is built on Quest capture that runs today, and the Band enters later as a registry entry rather than a contract change."},
    {"h":"Not a custom L1, no token","p":"Validator operations and a bridge are a permanent burden that does not repay itself at our volume. Settlement is USDC. A token would be the fastest way to make the provenance claim look like a pretext."},
    {"h":"Not per clip micropayments","p":"A one cent fee costs more than the payment. Contributors without wallets are paid on ordinary rails against a publicly verifiable claim, because requiring a wallet is a barrier and not a feature."},
    {"h":"Not hardware attestation","p":"It is the honest gap in the design and it is silicon and firmware work, not a software sprint. The schema reserves the field and the documentation says the gap is open."},
  ], "bento")
)

# ------------------------------------------------------------------- market
MARKET = sec(
  "Go to market",
  "Sell the audit,<br>not the network",
  "A network needs contributors we do not have and a device nobody has worn. So "
  "the first product is not data and not a network. It is a fixed fee engagement "
  "on tasks a customer has already named as failing.",
  tiles([
    {"h":"The beachhead","style":' style="border-color:var(--blue)"',"p":
     "Teams that have already committed to a multi fingered or anthropomorphic end "
     "effector, and that have at least one named task stuck below about 90 percent "
     "where the failure is a slip, a crush or a regrasp rather than a perception "
     "miss. That last clause is the filter. If the robot cannot see the object, we "
     "are the wrong call and we will say so."},
    {"h":"The Contact Audit","p":
     "A fixed fee engagement: instrumented capture on their objects, their task and "
     "their operators, delivered as labelled data plus a written failure analysis. "
     "They already know the task is failing. What they do not have is a measurement "
     "of what the hand does differently when it succeeds."},
    {"h":"Why this is winnable now","p":
     "Because it needs one instrumented pair of hands and a customer with a named "
     "problem, not a network. It converts a device in design into revenue without "
     "waiting for a supply chain, and every engagement is a labelled corpus we "
     "already have the rights to."},
    {"h":"How the first ten are reached","p":
     "Directly, by name. Manipulation failures get published: papers, issue "
     "threads, demo videos where a grasp visibly slips. That is a list of people "
     "who have already told the world which task is broken. Nobody in that list is "
     "reached by advertising."},
  ])
) + '''  <section class="fwrap">
    <p class="feyebrow">Sequencing</p>
    <h2 class="fh2">One calendar,<br>not two</h2>
    <p class="flede" style="margin-top:var(--s-300)">
      The most common way a plan like this falls apart is running a hardware
      programme and a protocol programme in the same twelve weeks with the same
      people. We are not doing that, and we would rather write down which one
      moves first.</p>
    <div class="rail" style="margin:var(--s-600) auto 0">
      <div><h3>Weeks 0 to 12, during the programme</h3>
        <p>The protocol, and nothing else. The data source is the Quest capture
          that already runs. Band work in this window is limited to printing and
          fit checking, not a validation study.</p></div>
      <div><h3>Months 4 to 6, after</h3>
        <p>First instrumented Bands and the biomechanics validation that has to
          precede any claim about measured force. This is where the hardware risk
          actually gets retired.</p></div>
      <div><h3>Months 6 to 12</h3>
        <p>Paid Contact Audits on named failing tasks. The corpus accumulates as a
          by product of engagements we were paid for rather than as a network we
          have to recruit.</p></div>
      <div><h3>The trigger for the network</h3>
        <p>More than one buyer wanting the same corpus. Until that number exceeds
          one, a shared ledger is infrastructure ahead of need, and we will say so
          rather than dress it up.</p></div>
    </div>

    <div class="note" style="margin:var(--s-600) auto 0">
      <b>The number that decides this.</b> Distinct buyers per corpus. One buyer
      is a consulting business with good tooling. Several buyers for the same
      recordings is the point at which provenance, licence terms and settlement
      stop being decoration and start being the product.
    </div>
  </section>
'''

# ------------------------------------------------------------------ company
COMPANY = sec(
  "Company",
  "Where this<br>actually stands",
  "Written plainly, because every number here is checkable and the ones that are "
  "zero are the ones people ask about.",
  tiles([
    {"h":"Shipped","tag":"Real","tagcls":"live","p":"Hotaru: an open source desk robot with public CAD, printable STLs and interface audits that must be proven to fail on broken geometry before they count as checks. Quest 3S teleoperation capture, running."},
    {"h":"In design","tag":"CAD only","p":"The Thenar Band. Watertight geometry, audited tolerances, printable today. Nobody has worn one and no data has been recorded with it."},
    {"h":"Not yet","tag":"Zero","p":"Revenue. Signed licences. Contributors. A live network. We would rather you read that here than find it in diligence."},
    {"h":"Being built","tag":"12 weeks","p":"GRASP: the provenance and settlement protocol on Avalanche, scoped to what one small team can actually ship between October and December."},
  ], "quad")
) + sec(
  "Method",
  "A check that has not<br>failed is not a check",
  "The discipline that runs through everything here: an audit only counts once it "
  "has been shown to fail on geometry that is wrong. Our bayonet checks are run "
  "against a build that is not twisted home, and if they pass there, they are "
  "reported as worthless. It is mutation testing pointed at CAD, and it is the "
  "reason we publish audits alongside the STLs instead of asking to be trusted.",
  tiles([
    {"h":"Open by default","p":"CAD in pure Python, STLs published, audits anyone can rerun. If a claim on this site is checkable, the way to check it is public."},
    {"h":"Status on every claim","p":"Every product on this site carries what state it is in. Shipping, in design, running, or zero. No exceptions, including the uncomfortable ones."},
    {"h":"We name the gaps","p":"Hardware attestation is missing. Hotaru proves geometry and not transduction. A ledger does not prove authenticity at capture. All three are on the site because a reviewer finds them anyway."},
  ], "bento")
)
