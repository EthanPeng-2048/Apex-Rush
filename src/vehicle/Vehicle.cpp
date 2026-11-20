#include "vehicle/Vehicle.h"

#include <algorithm>
#include <cmath>

namespace apex {

namespace {

// Wheel index convention: 0=FL, 1=FR, 2=RL, 3=RR.
constexpr int kFrontLeft = 0, kFrontRight = 1, kRearLeft = 2, kRearRight = 3;

bool IsFront(int wheel) { return wheel < 2; }

}  // namespace

void Vehicle::Spawn(IPhysicsWorld& world, const Transform& pose, const VehicleParams& params,
                    void* userData) {
    world_ = &world;
    params_ = params;

    BodyDesc desc;
    desc.transform = pose;
    desc.halfExtents = {params.chassisHalfX, params.chassisHalfY, params.chassisHalfZ};
    desc.mass = params.mass;
    desc.friction = 0.2f;
    desc.restitution = 0.05f;
    desc.linearDamping = 0.02f;
    desc.angularDamping = 0.25f;
    desc.allowSleep = false;
    desc.userData = userData;
    chassis_ = world.CreateBody(desc);

    // Hardpoints: front axle at +wheelbase/2, rear at -wheelbase/2 (local +Z forward).
    const float halfWb = params.wheelbase * 0.5f;
    const float halfTrack = params.trackWidth * 0.5f;
    const float hardpointY = params.wheelRadius + params.suspensionRestLength * 0.5f;
    hardpoints_[kFrontLeft] = {-halfTrack, 0.f, halfWb};
    hardpoints_[kFrontRight] = {halfTrack, 0.f, halfWb};
    hardpoints_[kRearLeft] = {-halfTrack, 0.f, -halfWb};
    hardpoints_[kRearRight] = {halfTrack, 0.f, -halfWb};
    (void)hardpointY;

    tires_[kFrontLeft] = TireModel(params.frontTire);
    tires_[kFrontRight] = TireModel(params.frontTire);
    tires_[kRearLeft] = TireModel(params.rearTire);
    tires_[kRearRight] = TireModel(params.rearTire);

    for (int i = 0; i < 4; ++i) {
        state_.wheels[i] = WheelState{};
    }
    for (int i = 0; i < static_cast<int>(CarPart::Count); ++i) {
        state_.partHealth[i] = 1.f;
    }

    engineRpm_ = params.idleRpm;
    gear_ = 1;
    state_.ersCharge = params.ersBaseCharge;
    state_.wingMode = WingMode::ZMode;
    state_.transform = pose;
}

void Vehicle::Despawn() {
    if (world_ && chassis_ != kInvalidBody) {
        world_->DestroyBody(chassis_);
        chassis_ = kInvalidBody;
    }
    world_ = nullptr;
}

void Vehicle::Reset(const Transform& pose) {
    if (!world_ || chassis_ == kInvalidBody) return;
    world_->SetTransform(chassis_, pose);
    world_->SetLinearVelocity(chassis_, Vec3{});
    world_->SetAngularVelocity(chassis_, Vec3{});
    for (int i = 0; i < 4; ++i) {
        state_.wheels[i].wear = 0.f;
        state_.wheels[i].temperature = 70.f;
        tires_[i].SetWear(0.f);
    }
    for (int i = 0; i < static_cast<int>(CarPart::Count); ++i) {
        state_.partHealth[i] = 1.f;
    }
    engineRpm_ = params_.idleRpm;
    gear_ = 1;
    steerAngle_ = 0.f;
    state_.ersCharge = params_.ersBaseCharge;
    state_.wingMode = WingMode::ZMode;
}

Vehicle::AeroCoefficients Vehicle::ComputeAero() const {
    AeroCoefficients aero;
    const float frontH = state_.partHealth[static_cast<int>(CarPart::FrontWing)];
    const float rearH = state_.partHealth[static_cast<int>(CarPart::RearWing)];
    const float sideH = state_.partHealth[static_cast<int>(CarPart::Sidepods)];
    const float floorH = state_.partHealth[static_cast<int>(CarPart::Floor)];

    const float floorGain = 0.55f + 0.45f * floorH;
    const float sideGain = 0.82f + 0.18f * sideH;

    aero.frontDownforceArea = params_.downforceArea * params_.frontDownforceShare *
                              (0.35f + 0.65f * frontH) * floorGain * sideGain;
    aero.rearDownforceArea = params_.downforceArea * (1.f - params_.frontDownforceShare) *
                             (0.30f + 0.70f * rearH) * floorGain * sideGain;

    // Damaged bodywork raises drag (torn surfaces).
    aero.dragArea = params_.dragArea * (1.f + 0.35f * (1.f - sideH) + 0.25f * (1.f - frontH) +
                                        0.30f * (1.f - rearH));

    // 2026 X/Z-mode: X trims both downforce and drag, Z is high downforce.
    if (state_.wingMode == WingMode::XMode) {
        aero.frontDownforceArea *= 0.72f;
        aero.rearDownforceArea *= 0.78f;
        aero.dragArea *= 0.80f;
    }

    // DRS: rear wing flap opens — big drag cut, rear downforce loss.
    if (state_.drsActive) {
        aero.dragArea *= 0.74f;
        aero.rearDownforceArea *= 0.55f;
    }
    return aero;
}

void Vehicle::Update(const VehicleInput& input, float dt) {
    if (!world_ || chassis_ == kInvalidBody) return;

    VehicleInput in = input;
    UpdateAssists(in);

    // Smooth steering (rate limit) for keyboard feel.
    const float steerTarget = in.steer * 21.f * kDegToRad;
    const float steerRate = 4.2f * dt;
    if (steerAngle_ < steerTarget) steerAngle_ = std::min(steerAngle_ + steerRate, steerTarget);
    else steerAngle_ = std::max(steerAngle_ - steerRate, steerTarget);

    throttleSmooth_ += (in.throttle - throttleSmooth_) * std::min(1.f, 8.f * dt);
    brakeSmooth_ += (in.brake - brakeSmooth_) * std::min(1.f, 10.f * dt);

    // Wing mode toggle (2026 active aero).
    if (in.wingToggle && !wingTogglePrev_) {
        state_.wingMode = (state_.wingMode == WingMode::ZMode) ? WingMode::XMode : WingMode::ZMode;
    }
    wingTogglePrev_ = in.wingToggle;

    const Transform transform = world_->GetTransform(chassis_);
    const Vec3 velocity = world_->GetLinearVelocity(chassis_);
    const float speed = Length(velocity);
    const Vec3 forward = TransformDirection(transform, Vec3{0.f, 0.f, 1.f});
    const Vec3 up = TransformDirection(transform, Vec3{0.f, 1.f, 0.f});
    const Vec3 right = TransformDirection(transform, Vec3{1.f, 0.f, 0.f});
    const float forwardSpeed = Dot(velocity, forward);

    // DRS: only when requested, moving forward fast, and pointed reasonably
    // straight (mirrors real activation windows).
    state_.drsAvailable = speed > 55.f && std::fabs(steerAngle_) < 6.f * kDegToRad;
    state_.drsActive = in.drsRequest && state_.drsAvailable;

    // ERS: deploy on overtake button when charge remains; regenerate under braking.
    state_.ersDeploying = false;
    float ersPower = 0.f;
    if (in.overtake && state_.ersCharge > 0.02f && throttleSmooth_ > 0.5f) {
        ersPower = params_.ersDeployPower * params_.overtakeMultiplier;
        state_.ersDeploying = true;
        state_.ersCharge = std::max(0.f, state_.ersCharge - ersPower * dt / params_.ersCapacity);
    } else if (brakeSmooth_ > 0.15f && speed > 8.f) {
        const float regen = params_.ersRegenPower * std::min(1.f, brakeSmooth_ * 1.4f);
        state_.ersCharge = std::min(1.f, state_.ersCharge + regen * dt / params_.ersCapacity);
    }

    UpdatePowertrain(in, dt);

    // --- Aero forces -------------------------------------------------------
    const AeroCoefficients aero = ComputeAero();
    const float v2 = speed * speed;
    const float dynPressure = 0.5f * params_.airDensity * v2;

    const float frontDF = dynPressure * aero.frontDownforceArea;
    const float rearDF = dynPressure * aero.rearDownforceArea;
    state_.frontDownforce = frontDF;
    state_.rearDownforce = rearDF;
    state_.dragForce = dynPressure * aero.dragArea;

    // Downforce acts along -world-up through front/rear axles (split by share).
    const Vec3 worldUp{0.f, 1.f, 0.f};
    const float totalDF = frontDF + rearDF;
    if (totalDF > 1.f) {
        const float frontFrac = frontDF / totalDF;
        const Vec3 frontPoint = transform.position + forward * (params_.wheelbase * 0.5f);
        const Vec3 rearPoint = transform.position - forward * (params_.wheelbase * 0.5f);
        world_->ApplyForce(chassis_, -worldUp * frontDF, frontPoint);
        world_->ApplyForce(chassis_, -worldUp * rearDF, rearPoint);
        (void)frontFrac;
    }

    // Drag opposes velocity through the center of pressure.
    if (speed > 0.2f) {
        const Vec3 cop = transform.position + forward * params_.centerOfPressureOffset;
        world_->ApplyForce(chassis_, Normalize(velocity) * -state_.dragForce, cop);
    }

    UpdateWheels(dt);

    // --- Collect state -----------------------------------------------------
    state_.transform = transform;
    state_.velocity = velocity;
    state_.angularVelocity = world_->GetAngularVelocity(chassis_);
    state_.speed = speed;
    state_.speedKmh = speed * 3.6f;
    state_.forwardSpeed = forwardSpeed;
    state_.rpm = engineRpm_;
    state_.gear = gear_;
    state_.throttle = throttleSmooth_;
    state_.brake = brakeSmooth_;
    state_.steerAngleDeg = steerAngle_ * kRadToDeg;

    float damageSum = 0.f;
    for (int i = 0; i < static_cast<int>(CarPart::Count); ++i) damageSum += state_.partHealth[i];
    state_.damageTotal = 1.f - damageSum / static_cast<float>(CarPart::Count);
}

void Vehicle::UpdatePowertrain(const VehicleInput& input, float dt) {
    const Transform transform = world_->GetTransform(chassis_);
    const Vec3 velocity = world_->GetLinearVelocity(chassis_);
    const Vec3 forward = TransformDirection(transform, Vec3{0.f, 0.f, 1.f});
    const float forwardSpeed = Dot(velocity, forward);

    // Automatic gearbox (simcade): shift on rpm with a short torque cut.
    const float wheelRadius = params_.wheelRadius;
    const float ratio = params_.gearRatios[static_cast<size_t>(std::max(0, gear_ - 1))];
    const float totalRatio = ratio * params_.finalDrive;

    if (shiftTimer_ > 0.f) {
        shiftTimer_ -= dt;
        driveTorque_ = 0.f;
    } else {
        // rpm from wheel speed (clutched to engine).
        const float wheelRps = forwardSpeed / (2.f * kPi * wheelRadius);
        engineRpm_ = std::max(params_.idleRpm,
                              std::fabs(wheelRps) * 60.f * totalRatio);
        engineRpm_ = std::min(engineRpm_, params_.redlineRpm);

        if (engineRpm_ > params_.shiftUpRpm && gear_ < static_cast<int>(params_.gearRatios.size())) {
            ++gear_;
            shiftTimer_ = params_.shiftTime;
        } else if (engineRpm_ < params_.shiftDownRpm && gear_ > 1) {
            --gear_;
            shiftTimer_ = params_.shiftTime * 0.7f;
        }

        // Torque curve: plateau below peak, taper to redline.
        float torqueFactor = 1.f;
        if (engineRpm_ > params_.peakRpm) {
            const float over = (engineRpm_ - params_.peakRpm) /
                               std::max(1.f, params_.redlineRpm - params_.peakRpm);
            torqueFactor = Clampf(1.f - over * 0.45f, 0.4f, 1.f);
        } else if (engineRpm_ < params_.idleRpm + 800.f) {
            torqueFactor = 0.72f + 0.28f * (engineRpm_ - params_.idleRpm) / 800.f;
        }

        float demand = throttleSmooth_;
        // TC cuts torque when rear wheels are slipping.
        driveTorque_ = demand * params_.maxTorque * torqueFactor;
        if (shiftTimer_ > 0.f) driveTorque_ = 0.f;
    }

    // Reverse gear handling.
    if (input.reverse && forwardSpeed < 0.8f) {
        gear_ = 0;
    } else if (gear_ == 0 && !input.reverse) {
        gear_ = 1;
    }

    const float ratioNow =
        gear_ == 0 ? params_.gearRatios[0] : params_.gearRatios[static_cast<size_t>(std::max(0, gear_ - 1))];
    const float totalRatioNow = ratioNow * params_.finalDrive;

    // Drive force at the rear axle.
    float driveForce = 0.f;
    if (gear_ == 0) {
        driveForce = -throttleSmooth_ * params_.maxTorque * totalRatioNow / wheelRadius * 0.55f;
    } else {
        driveForce = driveTorque_ * totalRatioNow / wheelRadius;
    }

    // ERS overtake adds force at the same axle.
    if (state_.ersDeploying) {
        const float v = std::max(8.f, Length(world_->GetLinearVelocity(chassis_)));
        driveForce += params_.ersDeployPower * params_.overtakeMultiplier / v;
    }

    // Engine braking when off throttle.
    float brakeForce = brakeSmooth_ * params_.brakeForceMax;
    if (throttleSmooth_ < 0.05f && gear_ > 0 && std::fabs(Dot(world_->GetLinearVelocity(chassis_),
                                                               TransformDirection(world_->GetTransform(chassis_), Vec3{0, 0, 1}))) > 1.f) {
        brakeForce += params_.engineBrakeTorque * totalRatioNow / wheelRadius * 0.25f;
    }
    if (input.handbrake) brakeForce += params_.brakeForceMax * 0.65f;

    // Split to wheels: drive/brake totals are distributed 50:50 per axle,
    // brake biased 60:40 front like a real single-seater.
    const float drivePerRear = driveForce * 0.5f;
    const float brakeFront = brakeForce * 0.6f * 0.5f;
    const float brakeRear = brakeForce * 0.4f * 0.5f;

    drivePerWheel_[kFrontLeft] = 0.f;
    drivePerWheel_[kFrontRight] = 0.f;
    drivePerWheel_[kRearLeft] = drivePerRear;
    drivePerWheel_[kRearRight] = drivePerRear;
    brakePerWheel_[kFrontLeft] = brakeFront;
    brakePerWheel_[kFrontRight] = brakeFront;
    brakePerWheel_[kRearLeft] = brakeRear;
    brakePerWheel_[kRearRight] = brakeRear;
}

void Vehicle::UpdateWheels(float dt) {
    const Transform transform = world_->GetTransform(chassis_);
    const Vec3 up = TransformDirection(transform, Vec3{0.f, 1.f, 0.f});
    const Vec3 forward = TransformDirection(transform, Vec3{0.f, 0.f, 1.f});
    const Vec3 right = TransformDirection(transform, Vec3{1.f, 0.f, 0.f});

    for (int i = 0; i < 4; ++i) {
        const Vec3 hardpoint = TransformPoint(transform, hardpoints_[i]);

        // Steering: front wheels yaw around the hardpoint.
        Vec3 wheelForward = forward;
        Vec3 wheelRight = right;
        if (IsFront(i)) {
            const Quat steerQ = Quat::FromAxisAngle(up, steerAngle_);
            wheelForward = Rotate(steerQ, forward);
            wheelRight = Rotate(steerQ, right);
        }

        const float maxLen = params_.suspensionRestLength + params_.suspensionMaxTravel;
        const RayHit hit = world_->Raycast(hardpoint, -up, maxLen + params_.wheelRadius, chassis_);

        WheelState& wheel = state_.wheels[i];
        if (hit.hit && Dot(hit.normal, up) > 0.25f) {
            const float distToContact = hit.fraction * (maxLen + params_.wheelRadius);
            const float suspLength = distToContact - params_.wheelRadius;
            const float compression = Clampf(params_.suspensionRestLength - suspLength, -0.02f,
                                             params_.suspensionMaxTravel);
            const float compressionRate =
                (compression - wheel.compression) / std::max(dt, 1e-5f);

            float springForce = params_.springK * std::max(compression, 0.f);
            float damperForce = params_.damperC * compressionRate;
            // Only damp when compressing to avoid jacking.
            if (compressionRate < 0.f) damperForce *= 0.35f;
            float normalForce = std::max(0.f, springForce + damperForce);

            wheel.grounded = true;
            wheel.suspensionLength = suspLength;
            wheel.compression = compression;
            wheel.contactPoint = hit.point;
            wheel.contactNormal = hit.normal;
            wheel.load = normalForce;
            wheel.contactVelocity = world_->GetVelocityAtPoint(chassis_, hit.point);

            // Suspension force along the strut (chassis up).
            world_->ApplyForce(chassis_, up * normalForce, hit.point);

            // Tire forces in the wheel plane.
            const TireModel::Output tire = tires_[i].Compute(
                normalForce, wheel.contactVelocity, wheelForward, wheelRight,
                drivePerWheel_[i], brakePerWheel_[i], dt);
            world_->ApplyForce(chassis_, tire.force, hit.point);
            wheel.force = tire.force;
            wheel.slipRatio = tire.slipRatio;
            wheel.slipAngleDeg = tire.slipAngleDeg;
            wheel.wear = tires_[i].Wear();

            // Tire temperature model: heats with slip work, cools with speed.
            const float slipHeat = std::fabs(tire.slipMagnitude) * normalForce * 1.6e-5f;
            wheel.temperature += slipHeat * dt * 60.f;
            wheel.temperature -= (wheel.temperature - 65.f) * 0.35f * dt;
            wheel.temperature = Clampf(wheel.temperature, 40.f, 160.f);
        } else {
            wheel.grounded = false;
            wheel.suspensionLength = maxLen;
            wheel.compression = 0.f;
            wheel.load = 0.f;
            wheel.force = Vec3{};
        }
    }
}

void Vehicle::UpdateAssists(VehicleInput& input) {
    if (!world_) return;
    const Transform transform = world_->GetTransform(chassis_);
    const Vec3 velocity = world_->GetLinearVelocity(chassis_);
    const Vec3 forward = TransformDirection(transform, Vec3{0.f, 0.f, 1.f});
    const Vec3 right = TransformDirection(transform, Vec3{1.f, 0.f, 0.f});
    const float speed = Length(velocity);
    const float forwardSpeed = Dot(velocity, forward);
    const float lateralSpeed = Dot(velocity, right);

    // TC: cut throttle when rear slip is high (approximated from lateral
    // slide + throttle demand).
    if (params_.tcLevel > 0.f) {
        const float slide = std::fabs(lateralSpeed) / std::max(speed, 4.f);
        const float slipEstimate = std::max(0.f, slide - 0.12f) * 2.2f;
        if (slipEstimate > 0.15f) {
            const float cut = Clampf((slipEstimate - 0.15f) * params_.tcLevel * 1.6f, 0.f, 0.85f);
            input.throttle *= (1.f - cut);
        }
    }

    // ABS: ease off braking when the car is sliding heavily.
    if (params_.absLevel > 0.f) {
        const float slide = std::fabs(lateralSpeed) / std::max(speed, 4.f);
        if (slide > 0.22f) {
            const float cut = Clampf((slide - 0.22f) * params_.absLevel * 1.4f, 0.f, 0.6f);
            input.brake *= (1.f - cut);
        }
    }

    // ESC: counter-steer bias when yaw rate is excessive for the speed.
    if (params_.escLevel > 0.f) {
        const Vec3 angVel = world_->GetAngularVelocity(chassis_);
        const float yawRate = Dot(angVel, Vec3{0.f, 1.f, 0.f});
        const float expectedYaw = forwardSpeed * input.steer * 0.28f;
        const float yawError = yawRate - expectedYaw;
        if (std::fabs(yawError) > 0.55f) {
            const float correction = Clampf(yawError * 0.22f * params_.escLevel, -0.32f, 0.32f);
            input.steer = Clampf(input.steer - correction, -1.f, 1.f);
        }
    }
}

void Vehicle::UpdateAero(float) {
    // Aero coefficients are recomputed in Update() from wing mode, DRS and
    // damage; this hook is reserved for future aero maps (speed-dependent Cl).
}

void Vehicle::ProcessContacts(const std::vector<ContactEvent>& contacts, float) {
    for (const ContactEvent& event : contacts) {
        if (event.bodyA == chassis_ || event.bodyB == chassis_) {
            UpdateDamageFromContact(event);
        }
    }
}

void Vehicle::UpdateDamageFromContact(const ContactEvent& event) {
    if (!world_) return;
    const Transform transform = world_->GetTransform(chassis_);

    // Map the world contact point into chassis-local space and pick the part
    // whose region contains it. Local axes: +X right, +Y up, +Z forward.
    const Vec3 local = RotateInv(transform.rotation, event.worldPoint - transform.position);

    const float impact = Clampf(event.impactSpeed / 28.f, 0.f, 1.f);
    if (impact < 0.06f) return;

    // Region pick: front z > +1.1 => front wing; z < -1.1 => rear wing;
    // |z| <= 1.1 and |x| > 0.55 => sidepods; else floor/floor area.
    CarPart part;
    if (local.z > 1.1f) part = CarPart::FrontWing;
    else if (local.z < -1.15f) part = CarPart::RearWing;
    else if (std::fabs(local.x) > 0.52f) part = CarPart::Sidepods;
    else part = CarPart::Floor;

    const float damage = impact * 0.42f;
    const int index = static_cast<int>(part);
    state_.partHealth[index] = Clampf(state_.partHealth[index] - damage, 0.f, 1.f);

    // Side impacts may also clip the floor edge.
    if (part == CarPart::Sidepods && impact > 0.35f) {
        const int floorIndex = static_cast<int>(CarPart::Floor);
        state_.partHealth[floorIndex] = Clampf(state_.partHealth[floorIndex] - damage * 0.3f, 0.f, 1.f);
    }
}

void Vehicle::Render(IRenderer& renderer, const Color& tint) const {
    if (!world_ || chassis_ == kInvalidBody) return;
    const Transform transform = world_->GetTransform(chassis_);
    const Vec3 forward = TransformDirection(transform, Vec3{0.f, 0.f, 1.f});
    const Vec3 up = TransformDirection(transform, Vec3{0.f, 1.f, 0.f});
    const Vec3 right = TransformDirection(transform, Vec3{1.f, 0.f, 0.f});

    // Chassis block (low-poly, scaled box).
    Transform chassisT = transform;
    renderer.DrawBox({params_.chassisHalfX, params_.chassisHalfY, params_.chassisHalfZ}, chassisT,
                     tint);

    // Front wing
    {
        const float h = state_.partHealth[static_cast<int>(CarPart::FrontWing)];
        Transform t = transform;
        t.position += forward * 2.15f;
        t.position -= up * 0.12f;
        renderer.DrawBox({1.05f, 0.05f, 0.34f}, t, Color{0.85f, 0.12f, 0.12f, h * 0.35f + 0.65f});
    }
    // Rear wing
    {
        const float h = state_.partHealth[static_cast<int>(CarPart::RearWing)];
        Transform t = transform;
        t.position -= forward * 1.95f;
        t.position += up * 0.52f;
        // X-mode tilts visually lower/flatter.
        const float tilt = state_.wingMode == WingMode::XMode ? 0.22f : 0.55f;
        t.position += up * tilt * 0.18f;
        renderer.DrawBox({0.52f, 0.06f, 0.30f}, t, Color{0.85f, 0.12f, 0.12f, h * 0.35f + 0.65f});
    }
    // Sidepods
    {
        const float h = state_.partHealth[static_cast<int>(CarPart::Sidepods)];
        Transform t = transform;
        t.position -= up * 0.02f;
        renderer.DrawBox({params_.chassisHalfX + 0.10f, 0.16f, 1.05f}, t,
                         Color{0.72f, 0.72f, 0.75f, h * 0.3f + 0.7f});
    }

    // Wheels (visual only — physics uses raycast suspension).
    const float maxLen = params_.suspensionRestLength + params_.suspensionMaxTravel;
    for (int i = 0; i < 4; ++i) {
        const Vec3 hardpoint = TransformPoint(transform, hardpoints_[i]);
        const float length = state_.wheels[i].grounded ? state_.wheels[i].suspensionLength : maxLen;
        const Vec3 wheelCenter = hardpoint - up * (length + params_.wheelRadius * 0.15f);

        Transform t = transform;
        t.position = wheelCenter;
        const float wearTint = 1.f - state_.wheels[i].wear * 0.45f;
        renderer.DrawBox({0.16f, params_.wheelRadius * 0.85f, params_.wheelRadius * 0.85f}, t,
                         Color{0.12f, 0.12f, 0.13f, wearTint});

        // Suspension strut
        renderer.DrawLine3D(hardpoint, wheelCenter, Color{0.45f, 0.45f, 0.5f, 1.f});
    }

    // Nose marker so orientation reads at a glance.
    renderer.DrawLine3D(transform.position + up * 0.35f,
                        transform.position + up * 0.35f + forward * 1.4f,
                        Color{1.f, 0.85f, 0.15f, 1.f});
    (void)right;
}

}  // namespace apex
