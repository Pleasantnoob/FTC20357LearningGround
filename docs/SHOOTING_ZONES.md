# Shooting Zones and Field Boundaries

The robot uses **odometry position** to detect when it is inside the **close launching triangle** or the **far** distance band, and whether it is inside the field rectangle.

## How it works

- **Field**: 12 ft × 12 ft → X and Y from **-72 to +72 inches** (origin at center).  
  `ShootingZones.isInsideField(x, y)` is true when `|x| ≤ 72` and `|y| ≤ 72`.

- **Close launching zone**: a **triangle** with vertices (field coords, inches):
  - **(-72, 72)** → **(0, 0)** → **(-72, -72)** (top-left corner, center, bottom-left corner).
  `ShootingZones.isInCloseLaunchTriangle(x, y)` is true when the point is inside this triangle (point-in-triangle test using cross products).

- **Far zone**: distance from goal in `[farMinIn, farMaxIn]` (default 48–120 in).  
  `ShootingZones.isInFarShootingZone(robotX, robotY, goalX, goalY)`.

- **Current zone** is computed each loop in MainDrive from **odometry** `pose.position` and `goalX, goalY`, and sent to FTC Dashboard as `shooting_zone` (`"close"`, `"far"`, or `"none"`) and `in_field` (boolean). **Close** = inside the triangle; **far** = in the far distance band; **none** = neither.

## Drawing

The FTC Dashboard field overlay draws:
- **Green triangle**: close launching zone (-72,72) → (0,0) → (-72,-72).
- **Blue circle**: far zone boundary (radius = `farMaxIn`) around the current goal.

## Tuning (FTC Dashboard)

In **ShootingZones**:

- `FIELD_HALF_SIZE` — half the field size in inches (default 72).
- `closeTriX1, closeTriY1` through `closeTriX3, closeTriY3` — triangle vertices (default -72,72; 0,0; -72,-72).
- `farMinIn`, `farMaxIn` — far zone distance range.

## Using zone checks in code

```java
// Odometry position (from pose):
boolean inCloseTriangle = ShootingZones.isInCloseLaunchTriangle(pose.position.x, pose.position.y);
ShootingZones.Zone zone = ShootingZones.getShootingZone(pose.position.x, pose.position.y, goalX, goalY);
boolean inField = ShootingZones.isInsideField(pose.position.x, pose.position.y);

if (zone == ShootingZones.Zone.CLOSE) { /* inside close launching triangle */ }
if (zone == ShootingZones.Zone.FAR)   { /* in far distance band */ }
```
