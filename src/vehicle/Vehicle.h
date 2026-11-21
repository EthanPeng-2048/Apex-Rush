#pragma once

#include <array>
#include <string>

#include "core/Math.h"
#include "physics/IPhysicsWorld.h"
#include "render/IRenderer.h"
#include "vehicle/TireModel.h"

namespace apex {

// Aerodynamic parts that can be damaged. Damage lowers part health in [0,1]
// and modulates the aero coefficients (see AeroCoefficients).
enum class CarPart : int {
    FrontWing = 0,
    RearWing = 1,
    Sidepods = 2,
    Floor = 3,
    Count = 4,
};

enum class WingMode : int {
    ZMode = 0,   // 2026 high-downforce (corners)
    XMode = 1,   // 2026 low-drag (straights)
};

// Complete parameter set for one car. Defaults approximate a 2026-spec F1
// car at simcade level (Real Racing 3 / Monoposto feel).
struct VehicleParams {
    // Chassis
    float mass = 750.f;               // kg
    float chassisHalfX = 0.72f;       // half width [m]
    float chassisHalfY = 0.22f;       // half height [m]
    float chassisHalfZ = 1.85f;       // half length [m]
    float wheelbase = 3.60f;
    float trackWidth = 1.58f;
    float wheelRadius = 0.33f;

    // Suspension (per corner)
    float springK = 38000.f;          // N/m
    float damperC = 2100.f;           // Ns/m
    float suspensionRestLength = 0.16f;
    float suspensionMaxTravel = 0.22f;

    // Tires
    TireModel::Params frontTire;
    TireModel::Params rearTire;

    // Powertrain (rear-wheel drive)
    float maxTorque = 420.f;          // Nm at the crank
    float idleRpm = 3800.f;
    float peakRpm = 10500.f;
    float redlineRpm = 12500.f;
    float finalDrive = 3.0f;
    std::array<float, 8> gearRatios = {3.5f, 2.95f, 2.55f, 2.25f, 2.0f, 1.8f, 1.62f, 1.48f};
    float shiftUpRpm = 11800.f;
    float shiftDownRpm = 7200.f;
    float shiftTime = 0.06f;
    float engineBrakeTorque = 90.f;
    float brakeForceMax = 24000.f;    // N total

    // Aero  (force = 0.5 * rho * coeff * v^2)
    float airDensity = 1.225f;
    float dragArea = 1.02f;           // Cd*A
    float downforceArea = 3.3f;       // Cl*A ≈ 1.3x car weight at 250 km/h
    float frontDownforceShare = 0.44f;
    float centerOfPressureOffset = 0.1f;  // +Z moves aero balance rearward

    // Assists (0 = off, 1 = full)
    float absLevel = 1.f;
    float tcLevel = 1.f;
    float escLevel = 1.f;
    float steerAssist = 1.f;

    // No-damage mode: collisions still bounce, but parts never lose health.
    bool damageEnabled = true;

    // ERS / overtake
    float ersCapacity = 4.0e6f;       // J
    float ersDeployPower = 350000.f;  // W
    float ersRegenPower = 250000.f;   // W under braking
    float ersBaseCharge = 0.55f;

    float overtakeMultiplier = 1.28f;
};

// Live state of one wheel.
struct WheelState {
    bool grounded = false;
    float suspensionLength = 0.f;
    float compression = 0.f;
    float load = 0.f;
    Vec3 contactPoint;
    Vec3 contactNormal;
    Vec3 contactVelocity;
    Vec3 force;
    float slipRatio = 0.f;
    float slipAngleDeg = 0.f;
    float wear = 0.f;
    float temperature = 70.f;
};

// Full per-frame vehicle state exposed to game/HUD/network.
struct VehicleState {
    Transform transform;
    Vec3 velocity;
    Vec3 angularVelocity;
    float speed = 0.f;              // m/s
    float speedKmh = 0.f;
    float forwardSpeed = 0.f;       // signed, + = moving forward
    float rpm = 0.f;
    int gear = 1;                   // 1..N, 0 = reverse
    float throttle = 0.f;
    float brake = 0.f;
    float steerAngleDeg = 0.f;
    std::array<WheelState, 4> wheels;
    std::array<float, 4> partHealth;  // CarPart indexed, 1 = intact
    float ersCharge = 0.f;            // 0..1
    bool ersDeploying = false;
    bool drsActive = false;
    bool drsAvailable = false;
    WingMode wingMode = WingMode::ZMode;
    float frontDownforce = 0.f;
    float rearDownforce = 0.f;
    float dragForce = 0.f;
    float damageTotal = 0.f;
};

struct VehicleInput {
    float throttle = 0.f;
    float brake = 0.f;
    float steer = 0.f;
    bool handbrake = false;
    bool reverse = false;
    bool drsRequest = false;
    bool wingToggle = false;
    bool overtake = false;
};

// Chassis rigid body + 4 raycast-suspension wheels + combined-slip tires.
// All physics goes through IPhysicsWorld so the collision backend is
// replaceable (qu3e today, Jolt later).
class Vehicle {
public:
    Vehicle() = default;

    void Spawn(IPhysicsWorld& world, const Transform& pose, const VehicleParams& params,
               void* userData = nullptr);
    void Despawn();

    // One fixed physics substep. Applies suspension/tire/aero forces to the
    // chassis body; the caller then steps the physics world.
    void Update(const VehicleInput& input, float dt);

    void ProcessContacts(const std::vector<ContactEvent>& contacts, float dt);

    void Reset(const Transform& pose);

    const VehicleState& State() const { return state_; }
    const VehicleParams& Params() const { return params_; }
    BodyHandle Chassis() const { return chassis_; }
    float PartHealth(CarPart part) const {
        return state_.partHealth[static_cast<int>(part)];
    }

    void Render(IRenderer& renderer, const Color& tint) const;

private:
    void UpdatePowertrain(const VehicleInput& input, float dt);
    void UpdateWheels(float dt);
    void UpdateAero(float dt);
    void UpdateAssists(VehicleInput& input);
    void UpdateDamageFromContact(const ContactEvent& event);

    // Aerodynamic coefficients after damage / wing-mode / DRS modulation.
    struct AeroCoefficients {
        float dragArea = 1.f;
        float frontDownforceArea = 1.f;
        float rearDownforceArea = 1.f;
    };
    AeroCoefficients ComputeAero() const;

    IPhysicsWorld* world_ = nullptr;
    BodyHandle chassis_ = kInvalidBody;
    VehicleParams params_;
    VehicleState state_;

    std::array<TireModel, 4> tires_;
    std::array<Vec3, 4> hardpoints_;  // suspension attach points in local space

    // Powertrain
    float engineRpm_ = 0.f;
    int gear_ = 1;
    float shiftTimer_ = 0.f;
    float driveTorque_ = 0.f;

    // Input smoothing
    float steerAngle_ = 0.f;
    float throttleSmooth_ = 0.f;
    float brakeSmooth_ = 0.f;

    // Per-wheel drive/brake force split (N), filled by UpdatePowertrain.
    std::array<float, 4> drivePerWheel_{};
    std::array<float, 4> brakePerWheel_{};

    // Mode state
    bool wingTogglePrev_ = false;
    float drsZoneTimer_ = 0.f;

    // Visual-only wheel rotation (radians), accumulated from road speed.
    float wheelSpinAngle_ = 0.f;
};

}  // namespace apex
