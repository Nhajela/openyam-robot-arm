# Visual choreography: "a happy wave hello to the people at the table"

- model `claude-fable-5-1`, 3 call(s), 103.29 s total, stopped: satisfied
- camera: yes (rules): rules: refers to a person
- result: VALID happy-wave-hello

| pass | valid | readability | silhouette | energy_contrast | timing | safety_margin | satisfied | latency s |
|---|---|---|---|---|---|---|---|---|
| 0 | yes (retimed) | 4 | 4 | 4 | 4 | 5 | False | 43.84 |
| 1 | yes | 4 | 4 | 4 | 4 | 3 | False | 28.11 |
| 2 | yes | 4 | 4 | 4 | 4 | 4 | True | 19.1 |

## Pass 0 (drafting)

- Draft: small anticipation turn away, then big arc turning body (j1 +0.65) and rising tall (j3 1.5) toward the audience with head turning to them (j5 -0.4) and grip opening at the peak.
- Peak held 0.6 s, then three wrist-pitch wave swings of decreasing size and varied tempo with j6 tilt accents and a grip snap on the last beat.
- Slow soft resolve back to ready over 5 s; all cubic segments sized with >10% margin on the 0.3 rad/s / 1 rad/s^2 caps.

## Pass 1 (reviewing pass 1/3)

Critique: From the audience the rise to the tall peak reads well: body turns toward them, hand climbs to 67 cm with the grip opening, and the hold registers. The anticipation dip is small (5 cm) but visible. The three wrist nods move the hand 9, 10 and 6 cm on the audience's view — legible but a bit timid for a wave; they are pure up-down with no side-to-side component. The grip snap before the final hold is a good 'word'. Peak velocity is 97% of cap because the retimer had to slow the flow phrase; the ending resolve is long (5 s) and the whole piece is 18.9 s, a little over target.

- Lengthened the flow rise (K1->K3) by ~0.4 s to pull peak velocity from 97% to ~80% of cap.
- Added a j5 side-to-side component (-0.2 / -0.55 / -0.35) to the three wave swings so the hand also sweeps across the audience's view, making each wave travel further.
- Shortened the final resolve to 4.5 s (still >10% margin) to tighten the ending.

## Pass 2 (reviewing pass 2/3)

Critique: Valid without retiming. From the audience the gesture reads as intended: a small dip away, then a long arc up and toward them into a tall, open-handed peak at 67 cm that is held; the three waves now sweep both up-down and across the view (10, 12, 7 cm), shrinking in size with a quicker last beat and a grip snap, followed by a short freeze and a slow, soft settle. Tempo is varied (CV 0.65), there are two holds and four grip values, the body leads and the head/grip arrive late on the rise. All joints sit mid-range, the hand stays high above the table, and peak velocity is at 90% of cap with 25% acceleration headroom. The only remaining softness is that the second and third waves move the hand modestly rather than dramatically, but that is dictated by the speed caps and still reads clearly.

- No changes: candidate is valid, reads clearly from the audience, and has adequate headroom.
