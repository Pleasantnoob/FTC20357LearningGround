# AirSort Logic Trace

## 1. Overview

AirSort uses **AprilTags 21/22/23** for the desired shot order (motif) and **color sensors c1, c2, c3** for the current ball order. It decides **fast shot** (correct color) or **slow shot** (wrong color) and feeds that to IntakeV2, which applies ShotPhysics (min vs max time in air, hood 40–70°, flywheel ≤ 1700).

---

## 2. Step-by-Step Process

### A. Motif (desired order)

| Tag ID | motif[0] | motif[1] | motif[2] | Meaning |
|--------|----------|----------|----------|---------|
| 21     | green    | purple   | purple   | 1st shot green, 2nd & 3rd purple |
| 22     | purple   | green    | purple   | 2nd shot green |
| 23     | purple   | purple   | green    | 3rd shot green |

- **motifStored**: Once any tag 21/22/23 is seen, motif is set and `motifStored = true`. We never read the tag again this run.
- Default (no tag seen): `[purple, purple, purple]`.

### B. Ball order (current pipeline)

- **c1** → slot 0 (intake side)
- **c2** → slot 1 (middle)
- **c3** → slot 2 (**at launcher**, next to shoot)

Color classification:
- `r+g+b < 80` → **unknown** (no ball)
- `g > r+30 && g > b+30` → **green**
- else → **purple**

### C. Shot index

- `shotIndex` = 0, 1, or 2 (which shot in the sequence we're on).
- **Auto-advance**: When `lastBallAtLauncher` was green/purple and now is unknown → ball left launcher → `advanceToNextShot()` (0→1, 1→2, 2 stays 2).
- **Manual**: Call `advanceToNextShot()` when you know you fired.

### D. Fast vs slow shot

- **Desired color** for current shot: `motif[shotIndex]`.
- **Ball at launcher**: `currentBallOrder[2]` = color at c3.
- **useFastShot()**: `true` iff ball at launcher is not unknown AND equals desired color.
- **getShotMode()**: `FAST_SHOT` if useFastShot, else `SLOW_SHOT`.

### E. Launcher preset (IntakeV2)

- **FAST_SHOT** → ShotPhysics.fastShotHoodAndSpeedMPS → min time in air (flattest angle that reaches goal at v≤vMax).
- **SLOW_SHOT** → ShotPhysics.slowShotHoodAndSpeedMPS → max time in air (steepest angle that reaches goal at v≤vMax).
- vMax = launch speed from 1700 ticks/s.
- IntakeV2 converts speed (m/s) to targetVel (ticks/s, negative).

---

## 3. Logic Checks

### Correct-color case

- Motif tag 21: want [G, P, P]. Shot 1: want green.
- Ball at launcher = green → **FAST_SHOT** ✓ (correct)
- Shot 2: want purple. Ball at launcher = purple → **FAST_SHOT** ✓
- Shot 3: want purple. Ball at launcher = purple → **FAST_SHOT** ✓

### Wrong-color case

- Motif tag 21: want [G, P, P]. Shot 1: want green.
- Ball at launcher = purple → **SLOW_SHOT** ✓ (wrong, lob it)
- After shoot, index → 1. Ball at launcher = next ball (e.g. green). Want purple → **SLOW_SHOT** ✓ (wrong for this slot)

### Empty launcher

- Ball at launcher = unknown → **SLOW_SHOT** (useFastShot returns false when unknown). Conservative: lob rather than risk.

### Shot detection

- `hadBall && !hasBall`: ball was present, now empty. One-shot transition = ball left.
- Auto-advance ensures we move to next shot index. Correct.

### Pipeline order

- Intake feeds → c1 → c2 → c3 → launcher. So c3 = ball about to be shot. ✓

---

## 4. Edge Cases

| Case | Behavior |
|------|----------|
| No tag ever seen | motif = [P,P,P], all shots “want purple”. Works but may mismatch field. |
| Ball falls out (no shoot) | Ball goes unknown → we may advance index. Could cause off-by-one. Mitigation: use manual advance from shoot trigger if this is common. |
| Multiple tags visible | First in detection list wins; `return` after first match. |
| Unknown at launcher | Always SLOW_SHOT. Safe fallback. |

---

## 5. Integration (MainDrive)

1. `airSort.update()` each loop (motif, ball order, shot detection, index advance).
2. If `airSortActive`: `intake.setAirSortPreset(airSort.getShotMode(), getDistanceFromOdometry())`.
3. IntakeV2.runLauncher: when `airSortEnabled && airSortPresetActive`, use `airSortHoodDeg` and `airSortTargetVel`.
4. Operator Back toggles `airSortActive` and `airSortEnabled`.
