#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"
#include <vector>
#include <cmath>

using std::vector;


const float G = 800.0f;
const float SHIP_THRUST = 25.0f;
const float FUEL_RATE = 8.0f;
const float ROTATION_SPEED = 3.0f;
const float FIXED_DT = 0.02f;

struct Planet {
    Vector3 position;
    Vector3 velocity;
    float mass;
    float radius;
    Color color;
};

struct Spacecraft {
    Vector3 position;
    Vector3 velocity;
    float rotation; 
    float fuel;
};

struct OrbitPrediction {
    vector<Vector3> points;
    bool collided;
    int collisionIndex;
};

Vector3 GetGravity(Vector3 pos, const vector<Planet>& planets) {
    Vector3 totalForce = { 0, 0, 0 };
    for (const auto& p : planets) {
        Vector3 dir = Vector3Subtract(p.position, pos);
        float distSq = Vector3LengthSqr(dir);

        if (distSq < 1e-8f) continue;
        if (distSq < p.radius * p.radius) continue;

        float forceMag = (G * p.mass) / distSq;
        Vector3 forceVec = Vector3Scale(Vector3Normalize(dir), forceMag);
        totalForce = Vector3Add(totalForce, forceVec);
    }
    return totalForce;
}

Vector3 GetCircularOrbitVelocity(Vector3 pos, const Planet& central) {
    Vector3 dir = Vector3Subtract(pos, central.position);
    float r = Vector3Length(dir);

    float speed = sqrtf((G * central.mass) / r);

    Vector3 up = {0, 1, 0};
    Vector3 tangent = Vector3CrossProduct(up, dir);
    tangent = Vector3Normalize(tangent);

    return Vector3Scale(tangent, speed);
}

OrbitPrediction PredictOrbit(Spacecraft ship, int steps, float dt, const vector<Planet>& planets, int earthIndex) {
    OrbitPrediction result;
    result.collided = false;
    result.collisionIndex = -1;

    Vector3 p = ship.position;
    Vector3 v = ship.velocity;

    vector<Planet> simPlanets = planets;

    int moonIndex = 9;

    for (int i = 0; i < steps; i++) {

        for (size_t j = 0; j < simPlanets.size(); j++) {
            if (j == 0) continue;

            Vector3 grav;

            if (j == moonIndex) {
                grav = GetGravity(simPlanets[j].position, { simPlanets[earthIndex] });
            } else {
                grav = GetGravity(simPlanets[j].position, { simPlanets[0] });
            }

            simPlanets[j].velocity = Vector3Add(simPlanets[j].velocity, Vector3Scale(grav, dt));
        }

        for (auto& pl : simPlanets) {
            pl.position = Vector3Add(pl.position, Vector3Scale(pl.velocity, dt));
        }

        
        v = Vector3Add(v, Vector3Scale(GetGravity(p, simPlanets), dt));
        p = Vector3Add(p, Vector3Scale(v, dt));

        result.points.push_back(p);

        for (const auto& planet : simPlanets) {
            if (Vector3Distance(p, planet.position) < planet.radius) {
                result.collided = true;
                result.collisionIndex = i;
                return result;
            }
        }
    }

    return result;
}

void resetCamera(Camera3D& cam, Spacecraft ship) {
    cam = { 0 };
    cam.position = ship.position;
    cam.up = (Vector3){ 0.0f, 1.0f, 0.0f };
    cam.fovy = 45.0f;
    cam.projection = CAMERA_PERSPECTIVE;
    cam.target = ship.position;
}

int main() {
    InitWindow(1280, 720, "Raylib Space Program - Stable Orbits");

    vector<Planet> planets;

    planets.push_back({ {0,0,0}, {0,0,0}, 500000.0f, 800.0f, YELLOW });

    float planetDistances[8] = {5200,9600,13333,20300,69300,126700,256000,400000};
    float planetMasses[8]    = {10,50,1000,40,2000,1500,400,500};
    float planetRadii[8]     = {18,35,40,22,110,90,55,55};
    Color planetColors[8]    = {GRAY,BEIGE,BLUE,RED,ORANGE,GOLD,SKYBLUE,BLUE};

    for (int i = 0; i < 8; i++) {
        Vector3 pos = { planetDistances[i], 0, 0 };
        Vector3 vel = GetCircularOrbitVelocity(pos, planets[0]);
        planets.push_back({ pos, vel, planetMasses[i], planetRadii[i], planetColors[i] });
    }

    int earthIndex = 3;

    float moonDist = 120.0f;
    Vector3 earthPos = planets[earthIndex].position;
    Vector3 moonPos = { earthPos.x + moonDist, 0, 0 };
    Vector3 moonRelVel = GetCircularOrbitVelocity(moonPos, planets[earthIndex]);
    Vector3 moonVel = Vector3Add(planets[earthIndex].velocity, moonRelVel);
    planets.push_back({ moonPos, moonVel, 5.0f, 12.0f, LIGHTGRAY });

    auto ResetShip = [&]() -> Spacecraft {
        Spacecraft s;
        const Planet& earth = planets[earthIndex];

        Vector3 relOffset = { 0, 0, earth.radius + 80.0f };
        s.position = Vector3Add(earth.position, relOffset);

        Vector3 relVel = GetCircularOrbitVelocity(s.position, earth);
        s.velocity = Vector3Add(earth.velocity, relVel);

        s.rotation = 0.0f;
        s.fuel = 10000.0f;
        return s;
    };

    Spacecraft ship = ResetShip();
    int timeWarp = 1;

    Camera3D camera = { 0 };
    camera.position = ship.position;
    camera.target = ship.position;
    camera.up = (Vector3){ 0,1,0 };
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    OrbitPrediction prediction;
    float predictionTimer = 0.0f;
    bool wasThrusting = false;

    SetTargetFPS(60);

    while (!WindowShouldClose()) {
        float frameDt = GetFrameTime() * timeWarp;

        if (IsKeyDown(KEY_A)) ship.rotation += ROTATION_SPEED * GetFrameTime();
        if (IsKeyDown(KEY_D)) ship.rotation -= ROTATION_SPEED * GetFrameTime();
        if (IsKeyPressed(KEY_TAB)) timeWarp = (timeWarp >= 64) ? 1 : timeWarp * 2;
        if (IsKeyPressed(KEY_Z)) resetCamera(camera, ship);
        if (IsKeyPressed(KEY_R)) {
            ship = ResetShip();
            resetCamera(camera, ship);
            timeWarp = 1;
        }

        Vector3 shipFacing = { sinf(ship.rotation), 0, cosf(ship.rotation) };
        bool isThrusting = false;

        if (ship.fuel > 0) {
            if (IsKeyDown(KEY_W)) {
                ship.velocity = Vector3Add(ship.velocity, Vector3Scale(shipFacing, SHIP_THRUST * frameDt));
                ship.fuel -= FUEL_RATE * frameDt;
                isThrusting = true;
            }
            if (IsKeyDown(KEY_S)) {
                ship.velocity = Vector3Subtract(ship.velocity, Vector3Scale(shipFacing, SHIP_THRUST * frameDt));
                ship.fuel -= FUEL_RATE * frameDt;
                isThrusting = true;
            }
        }

        int steps = (int)ceilf(frameDt / FIXED_DT);
        float subDt = frameDt / steps;

        int moonIndex = 9;

        for (int i = 0; i < steps; i++) {

            for (size_t j = 0; j < planets.size(); j++) {
                if (j == 0) continue;

                Vector3 grav;

                if (j == moonIndex) {
                    grav = GetGravity(planets[j].position, { planets[earthIndex] });
                } else {
                    grav = GetGravity(planets[j].position, { planets[0] });
                }

                planets[j].velocity = Vector3Add(planets[j].velocity, Vector3Scale(grav, subDt));
            }

            Vector3 shipGrav = GetGravity(ship.position, planets);
            ship.velocity = Vector3Add(ship.velocity, Vector3Scale(shipGrav, subDt));

            for (auto& p : planets) {
                p.position = Vector3Add(p.position, Vector3Scale(p.velocity, subDt));
            }

            ship.position = Vector3Add(ship.position, Vector3Scale(ship.velocity, subDt));
        }

        predictionTimer -= GetFrameTime();

        bool shouldUpdate = false;

        if (isThrusting) shouldUpdate = true;
        else if (predictionTimer <= 0.0f) shouldUpdate = true;

        if (wasThrusting && !isThrusting) shouldUpdate = true;

        if (shouldUpdate) {
            prediction = PredictOrbit(ship, 3000, subDt, planets, earthIndex);
            predictionTimer = 0.1f;
        }

        wasThrusting = isThrusting;

        camera.target = ship.position;
        UpdateCamera(&camera, CAMERA_THIRD_PERSON);

        BeginDrawing();
        ClearBackground(BLACK);
        BeginMode3D(camera);

        float aspect = (float)GetScreenWidth() / (float)GetScreenHeight();
        float nearPlane = 1.0f;
        float farPlane  = 2000000.0f;

        Matrix proj = MatrixPerspective(camera.fovy * DEG2RAD, aspect, nearPlane, farPlane);
        rlSetMatrixProjection(proj);

        for (const auto& p : planets) {
            DrawSphere(p.position, p.radius, p.color);
            DrawSphereWires(p.position, p.radius + 0.5f, 16, 16, ColorAlpha(WHITE, 0.2f));
        }

        Vector3 shipEnd = Vector3Add(ship.position, Vector3Scale(shipFacing, 6.0f));
        DrawCylinderEx(ship.position, shipEnd, 2.0f, 0.5f, 8, GOLD);

        DrawSphere(ship.position, 0.9f, YELLOW);

        DrawSphere(shipEnd, 0.6f, WHITE);

        if (isThrusting) {
            Vector3 flamePos = Vector3Subtract(ship.position, Vector3Scale(shipFacing, 2.5f));
            DrawSphere(flamePos, 1.5f, ORANGE);
        }

        Vector3 velEnd = Vector3Add(ship.position, Vector3Scale(ship.velocity, 2.0f));
        DrawLine3D(ship.position, velEnd, SKYBLUE);
        DrawSphere(velEnd, 0.4f, SKYBLUE);

        if (Vector3Length(ship.velocity) > 0.001f) {

            Vector3 velDir = Vector3Normalize(ship.velocity);
            Vector3 retroDir = Vector3Negate(velDir);

            float markerDistance = 20.0f;

            Vector3 progradePos = Vector3Add(ship.position, Vector3Scale(velDir, markerDistance));
            Vector3 retrogradePos = Vector3Add(ship.position, Vector3Scale(retroDir, markerDistance));

            DrawSphere(progradePos, 1.2f, GREEN);
            DrawLine3D(ship.position, progradePos, ColorAlpha(GREEN, 0.4f));

            DrawSphere(retrogradePos, 1.2f, RED);
            DrawLine3D(ship.position, retrogradePos, ColorAlpha(RED, 0.4f));
        }

        for (size_t i = 0; i + 1 < prediction.points.size(); i++) {
            DrawLine3D(prediction.points[i], prediction.points[i + 1], LIME);
        }

        EndMode3D();

        DrawText(TextFormat("VEL: %.1f", Vector3Length(ship.velocity)), 20, 20, 20, WHITE);
        DrawText(TextFormat("FUEL: %.0f", ship.fuel), 20, 45, 20, GREEN);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}