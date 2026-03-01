# Camera Relocalization — Logic Check + Numerical Example (Distance 50 in)

## Step-by-step with equations and units

**Constants:** MM_TO_IN = 1/25.4, CAMERA_TILT_DEG = 15°, tag = (-70, -64) in, CAMERA_FORWARD_OFFSET = 6 in, CAMERA_RIGHT_OFFSET = 0.

---

### Step 1: Raw camera→tag (camera frame), mm → in

**Equation:** `x = ftcPose.x * (1/25.4)`, same for y, z.  
**Units:** ftcPose in **mm**, result in **in**.  
**Meaning:** Vector from camera to tag in camera frame (X right, Y forward, Z up).

**Numerical (distance 50 in):**  
Assume tag is 50 in straight behind camera (robot heading 0). In camera frame that is 0 right, -50 forward, 0 up. With 15° tilt, “horizontal forward” = -50 in, so we need `y*cos(15°) - z*sin(15°) = -50`. Take z = 0 ⇒ y = -50/cos(15°) ≈ -51.76 in ⇒ ftcPose.y = -51.76*25.4 ≈ -1314.7 mm. So use:
- ftcPose (mm): (0, -1314.7, 0)
- After Step 1: x = 0, y = -51.76, z = 0 **(in)**

---

### Step 2: Project to horizontal (tilt correction)

**Equations:**
- `yHorz = y*cos(tilt) - z*sin(tilt)`  (horizontal “forward”)
- `xHorz = x`  (horizontal “right”)

**Units:** in → in.  
**Meaning:** Camera→tag in the horizontal plane, in robot frame (right, forward).

**Numerical:** tilt = 15°, cos≈0.966, sin≈0.259  
- yHorz = -51.76*0.966 - 0 = **-50.0** in  
- xHorz = **0** in  
So (xHorz, yHorz) = (0, -50) in (0 right, 50 back).

---

### Step 3: Rotate to field frame

**Equations:** c = cos(h), s = sin(h)  
- `fieldDx = -xHorz*s + yHorz*c`  
- `fieldDy = xHorz*c + yHorz*s`  

**Units:** in → in.  
**Meaning:** camera→tag in field frame (field +X, +Y). RR: heading 0 ⇒ forward = (+X), right = (+Y), so (right, forward) → (fieldDx, fieldDy) = (yHorz, xHorz) when h=0.

**Numerical (h = 0):** c = 1, s = 0  
- fieldDx = 0 + (-50)*1 = **-50** in  
- fieldDy = 0 + 0 = **0** in  
So camera→tag in field = **(-50, 0)** in (50 in along -X).

---

### Step 4: Camera position = tag − (camera→tag)

**Equations:**
- `cameraX = tagFieldX - fieldDx`
- `cameraY = tagFieldY - fieldDy`  
(Code reuses `robotX`, `robotY` for this first.)

**Units:** in.  
**Meaning:** Position of camera in field frame.

**Numerical:** tag = (-70, -64), (fieldDx, fieldDy) = (-50, 0)  
- cameraX = -70 - (-50) = **-20** in  
- cameraY = -64 - 0 = **-64** in  
So **camera at (-20, -64)** in.

---

### Step 5: Robot center = camera − mounting offset (in field frame)

**Equations:**  
Forward unit vector = (cos(h), sin(h)), Right = (-sin(h), cos(h)).  
- `robotX = cameraX - CAMERA_FORWARD_OFFSET*cos(h) + CAMERA_RIGHT_OFFSET*sin(h)`  
- `robotY = cameraY - CAMERA_FORWARD_OFFSET*sin(h) - CAMERA_RIGHT_OFFSET*cos(h)`  

**Units:** in.  
**Meaning:** Camera is FORWARD_OFFSET in front of robot; so robot = camera − forward_offset * forward̂.

**Numerical (h = 0, offset 6 in forward):**  
- robotX = -20 - 6*1 + 0 = **-26** in  
- robotY = -64 - 0 - 0 = **-64** in  
So **robot center at (-26, -64)** in.

---

### Step 6: Calibration offset (optional)

**Equations:** robotX −= CAMERA_OFFSET_X, robotY −= CAMERA_OFFSET_Y.  
**Numerical (both 0):** no change. **Final pose: (-26, -64)** in, heading = odometry (0 rad).

---

## Sanity check (distance 50)

- Tag at **(-70, -64)**.
- Camera→tag = **50 in** along -X ⇒ camera at **(-20, -64)**. ✓ (Step 4)
- Camera 6 in in front of robot (heading 0 = +X) ⇒ robot 6 in behind camera ⇒ robot at **(-26, -64)**. ✓ (Step 5)
- Distance robot→tag = 44 in = 50 − 6. ✓

**Conclusion:** For camera-to-tag distance 50 in, we get **robot at (-26, -64)**. Logic and signs are consistent.
