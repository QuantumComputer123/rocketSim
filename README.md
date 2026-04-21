## Newtonian Rocket Simulator
### Made in C++ using Raylib

### Controls:
  ```
  W : Thrust forwards
  A : Turn Left
  D : Turn Right
  M : Toggle Map Mode
  Z : Reset Camera (if a bit glitchy in map mode, click it a few times)
  Tab : Set Simulation Speed(Warp)
  R : Reset Spaceship
  ```

The green dashed line in front of you is your trajectory. When you are not boosting, it updates less frequently for performance reasons.
If it turns red, it means you are on a collision course with a planet or the sun. It occasionally turns red and turns back green again, I do not know why.

---

### Physics:

Planets are in stable orbits around the sun, and thus I did not make them affected by gravity for performance and to prevent glitches. 
The rocket is simulating gravity from all the planets and the sun at all times.
