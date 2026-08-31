---
workflow: general-video
flow: automation
storyboard: no
message: "The whole lamp goes together in one order, and every step of it has been checked."
destination: youtube
aspect: 1920x1080
language: en
length: 80s
audience: "someone who has the plates printed and the servos in a bag, about to build it"
---

## Intent

A build sequence for HOTARU 2.0, one part at a time, in the order the parts
actually go together. Not a marketing reel: the viewer has the prints on the
desk and wants to know what goes where, and in what order.

The order is not invented for the video. It is the order `cad/v2_insert.py`
verifies -- the closure audit walks every part to its seat along a straight
line and fails if one cannot get there. Using any other order in the video
would be showing an assembly the design has not been checked for.

## Notes

- Visuals come from the project's own CAD, not from stock or invented
  graphics: the parts are the real meshes.
- Four MG996R and the stock horns are the only bought hardware; the base
  fasteners are M3, everything else is printed.
- Inferred, not asked (the user's standing "just build it"): destination,
  aspect and language come from remembered defaults confirmed on
  xorv-launch and polaris-launch; length and audience are my call.
