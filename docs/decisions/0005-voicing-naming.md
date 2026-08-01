# Locked naming — amp & speaker voicings (user-supplied, 2026-08-01)

Trademark-safe by construction: descriptive character name + category/vintage subtitle.
Convention for all future additions: `NAME — CATEGORY / YEAR`.
No brand names anywhere in code, UI, parameter IDs, presets, or docs; the reference
gear may be discussed in design notes only as "the archetype", never as a claim of emulation.

## Amp voicings
| Preset | Subtitle | Archetype character (design note only) |
|---|---|---|
| GLASS CURRENT | US CLEAN / 65 | scooped, high-headroom, fixed-bias, stiff rectifier |
| BROWN CURRENT | US WARM / 62 | earlier-breakup, softer supply, mid-forward |
| CROWN STACK | UK CRUNCH / 68 | cathode-follower tone stack, high-gain-for-era, tight |
| RIDGE LEAD | BOUTIQUE GAIN / 92 | cascaded gain, tight low end, modern boutique |
| DESERT BLOOM | CA DRIVE / 94 | multi-stage lead, scooped-to-mid switchable, class-A/AB |
| BELL ARRAY | UK CHIME / 63 | cathode-biased class A, no negative feedback, top-cut |

## Speaker voicings
| Preset | Character (design note only) |
|---|---|
| GREEN CONE 25 | early cone breakup, pronounced mids, low power handling |
| BLUE BELL 15 | alnico, bell-like top, earliest compression |
| CERAMIC 30 | modern mid-focused workhorse |
| MODERN 75 | extended top, late breakup |
| US PAPER 12 | open, papery, American voicing |
| HEAVY CERAMIC 200 | very high power handling, near-linear, minimal compression |

## OPEN QUESTIONS raised by these names (fold into the consolidated question list)
1. The trailing number: power handling in watts, or cone diameter in inches? The list reads
   as watts (25/15/30/75/200) except `US PAPER 12`, which reads as inches. This matters
   physically, not cosmetically — power handling drives *when and how the speaker compresses*
   (cone mass, magnet, voice-coil heating), while diameter drives low-frequency extension and
   directivity. If it is watts, the number is a real continuous parameter the user should be
   able to sweep, not just a label.
2. Speaker x configuration combinatorics: cab configs are separately specified as 1x1, 1x2,
   2x2, 4x2. Is any speaker voicing loadable into any configuration (6 x 4 = 24 combinations),
   or does each speaker preset carry an implied "native" cab?
