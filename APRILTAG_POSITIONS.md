# AprilTag positions for relocalization

## FTC field coordinate system (official)

- **Origin:** Center of the field (where the four center tiles meet), on the mat surface.
- **+X:** To the **right** when standing at the **Red Alliance wall** looking at the field.
- **+Y:** **Away from the Red wall** (toward the Blue Alliance / goals).
- **+Z:** Up.
- Field is **12 ft × 12 ft** (144 in × 144 in), so X and Y each run from **-72 in** to **+72 in**.

Source: [FTC Field Coordinate System](https://ftc-docs.firstinspires.org/en/latest/game_specific_resources/field_coordinate_system/field-coordinate-system.html)

## Blue goal (backdrop)

- The **blue goal / backdrop** is on the wall at the **far** end from the Red wall → **Y = +72 inches** (or slightly in from the wall depending on mounting).
- AprilTags on the backdrop are typically placed **left**, **center**, and **right** (three tags per backdrop).
- **Height:** FTC docs say tags are often placed with their reference line **4 inches (101.6 mm) above the playing field surface** (Z ≈ 4 in for the reference line; tag center may be higher depending on tag size).

## Where to get exact positions

- **Field Setup Guide PDF** from FIRST: [Playing Field Resources](https://ftc-docs.firstinspires.org/en/latest/game_specific_resources/playing_field_resources/playing_field_resources.html) → download “Field Setup Guide” from the FIRST website. It has AprilTag placement and measurements for the current game.
- **AprilTag placement:** Tags are aligned with tile seams or element centers; the PDF has the exact instructions and dimensions for each tag.
- **Center Stage:** The SDK’s `AprilTagGameDatabase.getCenterStageTagLibrary()` (or `getCurrentGameTagLibrary()`) provides tag metadata; in SDK 10+, tag **field position** can be in metadata. For custom relocalization we still need X,Y in **inches** in the field frame above.

## Current values in code (verify from Field Setup Guide)

In `CameraRelocalization.java` we use **one tag** for now: **blue goal tag** (ID 20).

- **BLUE_GOAL_TAG_X = -70**, **BLUE_GOAL_TAG_Y = -64** (inches) so blue goal is in the **(-,-) quadrant**, matching `MainDrive` blue goal.
- If your convention has blue goal in (+ , +) (official “+Y toward blue” frame), use positive values (e.g. 60, 72). Tune on Dashboard.

If your Field Setup Guide or game manual gives different coordinates for tag 20 (or your chosen blue goal tag), update **BLUE_GOAL_TAG_X** and **BLUE_GOAL_TAG_Y** in `CameraRelocalization.java` (or on FTC Dashboard, since they are `@Config`).

## Example from another game (DECODE)

FTC docs give this example for the **Red Goal** AprilTag on the DECODE field:  
**(-58.3727, 55.6425, 29.5)** inches (X, Y, Z). So tag positions are game-specific and come from the official field documentation.
