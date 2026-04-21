#include <algorithm>
#include <cmath>
#include <vector>

#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"

using std::vector;

#ifndef CYAN
#define CYAN CLITERAL(Color){0, 255, 255, 255}
#endif

const float G = 800.0f;
const float SHIP_THRUST = 25.0f;
const float FUEL_RATE = 8.0f;
const float ROTSPEED = 3.0f;
const float FIXED_DT = 0.02f;
const int MAXTPOINTS = 1200;  // trail
const int SUN_I = 0;
const int EARTH_I = 3;
const int MOON_I = 9;

Camera2D mapCamera = {0};
bool isDragging = false;

struct Planet {
  Vector3 position;
  float mass;
  float rad;
  Color color;
  vector<Vector3> trail;

  float orbitRad;
  float orbitVel;
  int parentIndex;
};

struct Space_ship {
  Vector3 position;
  Vector3 vel;
  float rotation;
  float fuel;
  vector<Vector3> trail;
};

struct OrbitPrediction {
  vector<Vector3> points;
  bool collided;
  int collisionIndex;
  Vector3 apoapsis;
  Vector3 periapsis;
};

Vector3 GetGravity(Vector3 pos, const vector<Planet>& planets) {
  Vector3 totalForce = {0, 0, 0};
  for (const auto& p : planets) {
    Vector3 dir = Vector3Subtract(p.position, pos);
    float distSq = Vector3LengthSqr(dir);
    if (distSq < 1e-8f) continue;
    if (distSq < p.rad * p.rad) continue;

    float forceMag = (G * p.mass) / (distSq + 100.0f);
    Vector3 forceVec = Vector3Scale(Vector3Normalize(dir), forceMag);
    totalForce = Vector3Add(totalForce, forceVec);
  }
  return totalForce;
}

Vector3 circularOrbitVel(Vector3 pos, const Planet& central) {
  Vector3 dir = Vector3Subtract(pos, central.position);
  float r = Vector3Length(dir);
  float speed = sqrtf((G * central.mass) / r);
  Vector3 up = {0, 1, 0};
  Vector3 tangent = Vector3CrossProduct(up, dir);
  tangent = Vector3Normalize(tangent);
  return Vector3Scale(tangent, speed);
}

void updatePlanetPos(vector<Planet>& pts, float time) {
  for (size_t i = 1; i < pts.size(); i++) {
    float angle = time * pts[i].orbitVel;
    Vector3 offset = {cosf(angle) * pts[i].orbitRad, 0,
                      sinf(angle) * pts[i].orbitRad};
    if (pts[i].parentIndex == SUN_I) {
      pts[i].position = offset;
    } else {
      pts[i].position =
          Vector3Add(pts[i].parentIndex >= 0 ? pts[pts[i].parentIndex].position
                                             : Vector3{0, 0, 0},
                     offset);
    }
  }
}

OrbitPrediction predictOrbit(Space_ship ship, int steps, float dt,
                             const vector<Planet>& planets, float currentTime,
                             int stepScale = 1) {
  OrbitPrediction result;
  result.collided = false;
  result.collisionIndex = -1;

  Vector3 p = ship.position;
  Vector3 v = ship.vel;
  auto simPlanets = planets;
  dt *= stepScale;

  float maxD = -1.0f;
  float minD = 1e18f;

  for (int i = 0; i < steps; i++) {
    float futureTime = currentTime + (i * dt);
    updatePlanetPos(simPlanets, futureTime);
    v = Vector3Add(v, Vector3Scale(GetGravity(p, simPlanets), dt));
    p = Vector3Add(p, Vector3Scale(v, dt));
    result.points.push_back(p);

    float d = Vector3Distance(p, simPlanets[SUN_I].position);
    if (d > maxD) {
      maxD = d;
      result.apoapsis = p;
    }
    if (d < minD) {
      minD = d;
      result.periapsis = p;
    }

    for (const auto& planet : simPlanets) {
      if (Vector3Distance(p, planet.position) < planet.rad + 3.0f) {
        result.collided = true;
        result.collisionIndex = i;
        return result;
      }
    }
  }
  return result;
}

void addTrailPoint(vector<Vector3>& trail, Vector3 pos) {
  trail.push_back(pos);
  if (trail.size() > MAXTPOINTS) trail.erase(trail.begin());
}

void resetCamera(Camera3D& cam, Space_ship ship) {
  cam = {0};
  cam.position = Vector3Add(ship.position, {0, 150, 150});
  cam.target = ship.position;
  cam.up = {0.0f, 1.0f, 0.0f};
  cam.fovy = 45.0f;
  cam.projection = CAMERA_PERSPECTIVE;
}

void initMapCamera(const Space_ship& ship) {
  mapCamera.target = {ship.position.x, ship.position.z};
  mapCamera.offset = {GetScreenWidth() / 2.0f, GetScreenHeight() / 2.0f};
  mapCamera.zoom = 0.15f;
  mapCamera.rotation = 0.0f;
}

void orbitRing(Vector3 center, float radius, Color col, bool map,
               float zoom = 1.0f) {
  int segments = 128;
  for (int i = 0; i < segments; i++) {
    float a1 = (float)i / segments * 2.0f * PI;
    float a2 = (float)(i + 1) / segments * 2.0f * PI;
    Vector3 p1 = {center.x + cosf(a1) * radius, 0,
                  center.z + sinf(a1) * radius};
    Vector3 p2 = {center.x + cosf(a2) * radius, 0,
                  center.z + sinf(a2) * radius};
    if (!map)
      DrawLine3D(p1, p2, ColorAlpha(col, 0.2f));
    else
      DrawLineEx({p1.x, p1.z}, {p2.x, p2.z}, 1.5f / zoom,
                 ColorAlpha(col, 0.3f));
  }
}

int main() {
  InitWindow(1280, 720, "Raylib Space Program");

  vector<Planet> planets;
  planets.push_back({{0, 0, 0}, 500000.0f, 800.0f, YELLOW, {}, 0.0f, 0.0f, -1});
  float planetDistances[8] = {5200,  9600,   13333,  20300,
                              69300, 126700, 256000, 400000};
  float planetMasses[8] = {10, 50, 1000, 40, 2000, 1500, 400, 500};
  float planetRadii[8] = {18, 35, 40, 22, 110, 90, 55, 55};
  Color planetColors[8] = {GRAY, BEIGE, BLUE, RED, ORANGE, GOLD, SKYBLUE, BLUE};

  for (int i = 0; i < 8; i++) {
    float omega =
        sqrtf((G * planets[SUN_I].mass) /
              (planetDistances[i] * planetDistances[i] * planetDistances[i]));
    planets.push_back({{planetDistances[i], 0, 0},
                       planetMasses[i],
                       planetRadii[i],
                       planetColors[i],
                       {},
                       planetDistances[i],
                       omega,
                       SUN_I});
  }

  int earthIndex = 3;
  float moonDist = 120.0f;
  float mOmega =
      sqrtf((G * planets[earthIndex].mass) / (moonDist * moonDist * moonDist));
  planets.push_back(
      {{0, 0, 0}, 5.0f, 12.0f, LIGHTGRAY, {}, moonDist, mOmega, earthIndex});
  int moonIndex = 9;

  float simTime = 0.0f;

  auto ResetShip = [&]() -> Space_ship {
    Space_ship s;
    const Planet& earth = planets[earthIndex];
    Vector3 relOffset = {0, 0, earth.rad + 200.0f};
    s.position = Vector3Add(earth.position, relOffset);
    float earthVmag = earth.orbitRad * earth.orbitVel;
    float angle = simTime * earth.orbitVel;
    Vector3 earthVel = {-sinf(angle) * earthVmag, 0, cosf(angle) * earthVmag};
    float shipVmag = sqrtf((G * earth.mass) / Vector3Length(relOffset));
    s.vel = Vector3Add(earthVel, {shipVmag, 0, 0});
    s.rotation = 0.0f;
    s.fuel = 10000.0f;
    s.trail = {};
    return s;
  };

  Space_ship ship = ResetShip();
  initMapCamera(ship);

  int warp = 1;
  Camera3D camera = {0};
  resetCamera(camera, ship);
  OrbitPrediction pre;
  float predictionTimer = 0.0f;
  bool wasThrusting = false;
  bool mapMode = false;

  SetTargetFPS(60);

  while (!WindowShouldClose()) {
    float frameDt = GetFrameTime() * warp;
    simTime += frameDt;
    updatePlanetPos(planets, simTime);
    if (IsKeyDown(KEY_A)) ship.rotation += ROTSPEED * GetFrameTime();
    if (IsKeyDown(KEY_D)) ship.rotation -= ROTSPEED * GetFrameTime();
    if (IsKeyPressed(KEY_M)) mapMode = !mapMode;
    if (IsKeyPressed(KEY_TAB)) warp = (warp >= 64) ? 1 : warp * 2;
    if (IsKeyPressed(KEY_Z)) {
      resetCamera(camera, ship);
      initMapCamera(ship);
    }
    if (IsKeyPressed(KEY_R)) {
      ship = ResetShip();
      resetCamera(camera, ship);
      initMapCamera(ship);
      for (auto& p : planets) p.trail.clear();
      warp = 1;
    }

    Vector3 shipFacing = {sinf(ship.rotation), 0, cosf(ship.rotation)};
    bool isThrusting = (ship.fuel > 0 && IsKeyDown(KEY_W));
    if (isThrusting) {
      ship.vel =
          Vector3Add(ship.vel, Vector3Scale(shipFacing, SHIP_THRUST * frameDt));
      ship.fuel -= FUEL_RATE * frameDt;
    }

    int steps = (int)ceilf(frameDt / FIXED_DT);
    if (warp > 1) steps *= 2;
    float subDt = frameDt / steps;

    for (int i = 0; i < steps; i++) {
      ship.vel = Vector3Add(
          ship.vel, Vector3Scale(GetGravity(ship.position, planets), subDt));
      ship.position = Vector3Add(ship.position, Vector3Scale(ship.vel, subDt));
      if (i % 2 == 0) {
        addTrailPoint(ship.trail, ship.position);
      }
      for (const auto& p : planets) {
        if (Vector3Distance(ship.position, p.position) < p.rad + 3.0f) {
          ship = ResetShip();
          resetCamera(camera, ship);
          initMapCamera(ship);
          for (auto& pl : planets) pl.trail.clear();
          warp = 1;
          break;
        }
      }
    }

    for (auto& p : planets) addTrailPoint(p.trail, p.position);

    predictionTimer -= GetFrameTime();
    if (isThrusting || predictionTimer <= 0.0f ||
        (wasThrusting && !isThrusting)) {
      pre = predictOrbit(ship, 5000, FIXED_DT, planets, simTime, 8);
      predictionTimer = 0.8f;
    }
    wasThrusting = isThrusting;

    if (!mapMode) {
      camera.target = ship.position;
      UpdateCamera(&camera, CAMERA_THIRD_PERSON);
    } else {
      Vector2 mouseWorld = GetScreenToWorld2D(GetMousePosition(), mapCamera);
      float wheel = GetMouseWheelMove();
      if (wheel != 0) {
        mapCamera.zoom =
            Clamp(mapCamera.zoom * (1.0f + wheel * 0.15f), 0.0001f, 5.0f);
        Vector2 mouseNew = GetScreenToWorld2D(GetMousePosition(), mapCamera);
        mapCamera.target.x += (mouseWorld.x - mouseNew.x);
        mapCamera.target.y += (mouseWorld.y - mouseNew.y);
      }
      if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) isDragging = true;
      if (IsMouseButtonReleased(MOUSE_LEFT_BUTTON)) isDragging = false;
      if (isDragging) {
        Vector2 delta = GetMouseDelta();
        mapCamera.target.x -= delta.x / mapCamera.zoom;
        mapCamera.target.y -= delta.y / mapCamera.zoom;
      }
    }

    BeginDrawing();
    ClearBackground(BLACK);

    if (!mapMode) {
      BeginMode3D(camera);
      rlSetMatrixProjection(MatrixPerspective(
          camera.fovy * DEG2RAD, (float)GetScreenWidth() / GetScreenHeight(),
          1.0f, 2000000.0f));
      for (const auto& p : planets) {
        Vector3 center = (p.parentIndex == -1)
                             ? Vector3{0, 0, 0}
                             : planets[p.parentIndex].position;
        orbitRing(center, p.orbitRad, p.color, false);
        for (size_t i = 0; i + 1 < p.trail.size(); i++)
          DrawLine3D(p.trail[i], p.trail[i + 1],
                     ColorAlpha(p.color, (float)i / p.trail.size() * 0.4f));
        DrawSphere(p.position, p.rad, p.color);
      }
      for (size_t i = 0; i + 1 < ship.trail.size(); i++)
        DrawLine3D(ship.trail[i], ship.trail[i + 1],
                   ColorAlpha(SKYBLUE, (float)i / ship.trail.size() * 0.6f));
      DrawCylinderEx(ship.position,
                     Vector3Add(ship.position, Vector3Scale(shipFacing, 6.0f)),
                     2.0f, 0.5f, 8, GOLD);
      Color pCol = pre.collided ? RED : LIME;
      for (size_t i = 0; i + 1 < pre.points.size(); i += 4)
        DrawLine3D(pre.points[i], pre.points[i + 1], pCol);
      EndMode3D();
    } else {
      BeginMode2D(mapCamera);

      for (size_t i = 0; i < planets.size(); i++) {
        if (i > 0) {
          Vector3 center = (planets[i].parentIndex == SUN_I)
                               ? Vector3{0, 0, 0}
                               : planets[planets[i].parentIndex].position;
          orbitRing(center, planets[i].orbitRad, planets[i].color, true,
                    mapCamera.zoom);
        }

        float displayRad;
        if (i == SUN_I) {
          displayRad = planets[i].rad * 15 * mapCamera.zoom;
        } else {
          displayRad =
              (planets[i].rad * mapCamera.zoom) + (2.0f / mapCamera.zoom);
        }

        DrawCircleV({planets[i].position.x, planets[i].position.z}, displayRad,
                    planets[i].color);
      }

      Color predictColor = pre.collided ? RED : LIME;
      for (size_t i = 0; i + 1 < pre.points.size(); i += 2) {
        Vector2 p1 = {pre.points[i].x, pre.points[i].z};
        Vector2 p2 = {pre.points[i + 1].x, pre.points[i + 1].z};
        DrawLineEx(p1, p2, 2.0f / mapCamera.zoom, Fade(predictColor, 0.6f));
      }

      auto DrawM = [&](Vector3 pos, const char* t, Color c) {
        DrawCircleV({pos.x, pos.z}, 6.0f / mapCamera.zoom, c);
        DrawTextEx(GetFontDefault(), t, {pos.x + 10 / mapCamera.zoom, pos.z},
                   20 / mapCamera.zoom, 1, c);
      };
      DrawM(pre.apoapsis, "Ap", CYAN);
      DrawM(pre.periapsis, "Pe", ORANGE);

      if (pre.points.size() > 500) {
        DrawCircleV({pre.points[500].x, pre.points[500].z},
                    4.0f / mapCamera.zoom, Fade(YELLOW, 0.5f));
      }

      Vector2 s2 = {ship.position.x, ship.position.z};
      float sz = 15.0f / mapCamera.zoom;
      Vector2 v1 = {s2.x + sinf(ship.rotation) * sz,
                    s2.y + cosf(ship.rotation) * sz};
      Vector2 v2 = {s2.x + sinf(ship.rotation + PI * 0.8f) * sz,
                    s2.y + cosf(ship.rotation + PI * 0.8f) * sz};
      Vector2 v3 = {s2.x + sinf(ship.rotation - PI * 0.8f) * sz,
                    s2.y + cosf(ship.rotation - PI * 0.8f) * sz};
      DrawTriangle(v1, v2, v3, YELLOW);

      if (isThrusting)
        DrawCircleV(
            Vector2Add(s2, {-sinf(ship.rotation) * 20 / mapCamera.zoom,
                            -cosf(ship.rotation) * 20 / mapCamera.zoom}),
            6 / mapCamera.zoom, ORANGE);

      EndMode2D();
    }

    DrawText(TextFormat("VEL: %.1f | FUEL: %.0f | WARP: %dx",
                        Vector3Length(ship.vel), ship.fuel, warp),
             20, 20, 20, WHITE);
    EndDrawing();
  }
  CloseWindow();
  return 0;
}