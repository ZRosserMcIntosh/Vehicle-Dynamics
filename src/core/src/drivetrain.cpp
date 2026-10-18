// OpenRoad Dynamics — engine-independent vehicle physics core.
// SPDX-License-Identifier: MIT
#include "ord/drivetrain.hpp"

#include "ord/linear_solve.hpp"

namespace ord {

namespace {
constexpr std::size_t kEngine = 0;
constexpr std::size_t kGearboxIn = 1;
constexpr std::size_t kWheel0 = 2;
constexpr Real kLockedCapacity = 1e9;
constexpr Real kCompliance = 1e-7;  // (rad/s) per N*m
constexpr std::size_t kSystemCapacity = 2 + 2 * kMaxWheels + 1 + (2 + kMaxWheels + kMaxAxles);
}  // namespace

Drivetrain::Drivetrain(const DrivetrainDef& def, std::size_t wheelCount,
                       const std::array<std::array<int, 2>, kMaxAxles>& axleWheels)
    : def_(def), wheelCount_(wheelCount), axleWheels_(axleWheels) {
    frictionCount_ = 0;
    auto add = [&](FrictionKind k, std::size_t idx) -> FrictionRow& {
        FrictionRow& r = friction_[frictionCount_++];
        r = FrictionRow{};
        r.kind = k;
        r.index = idx;
        return r;
    };
    {
        FrictionRow& c = add(FrictionKind::Clutch, 0);
        c.j[kEngine] = 1;
        c.j[kGearboxIn] = -1;
    }
    for (std::size_t w = 0; w < wheelCount_; ++w) {
        FrictionRow& b = add(FrictionKind::Brake, w);
        b.j[kWheel0 + w] = 1;
    }
    for (std::size_t a = 0; a < def_.drivenAxles.size(); ++a) {
        // Every driven axle gets a locking row (capacity 0 while Open), so
        // the differential can be retuned at run time (setDifferential).
        const auto& da = def_.drivenAxles[a];
        FrictionRow& d = add(FrictionKind::AxleDiff, a);
        d.j[kWheel0 + axleWheels_[da.axleIndex][0]] = 1;
        d.j[kWheel0 + axleWheels_[da.axleIndex][1]] = -1;
    }
    if (def_.drivenAxles.size() == 2 && def_.center != CenterCoupling::Open) {
        FrictionRow& c = add(FrictionKind::CenterDiff, 0);
        for (std::size_t a = 0; a < 2; ++a) {
            const Real s = (a == 0) ? 0.5 : -0.5;
            const auto& wheels = axleWheels_[def_.drivenAxles[a].axleIndex];
            c.j[kWheel0 + wheels[0]] += s;
            c.j[kWheel0 + wheels[1]] += s;
        }
    }
    omega_.fill(0);
    setEngineOmega(rpmToRadS(def_.engine.idleRpm));
}

Real Drivetrain::wotTorque(Real rpm) const {
    const auto& c = def_.engine.torqueCurve;
    if (c.empty()) return 0;
    if (rpm <= c.front().rpm) return c.front().torque;
    if (rpm >= c.back().rpm) return c.back().torque;
    for (std::size_t i = 1; i < c.size(); ++i) {
        if (rpm <= c[i].rpm) {
            const Real t = (rpm - c[i - 1].rpm) / (c[i].rpm - c[i - 1].rpm);
            return lerp(c[i - 1].torque, c[i].torque, t);
        }
    }
    return c.back().torque;
}

Real Drivetrain::engineTorque(Real omega, Real throttle) const {
    const auto& e = def_.engine;
    const Real rpm = radSToRpm(omega);
    // Simple idle governor: opens the throttle as rpm falls below idle.
    const Real idleThrottle = clamp((e.idleRpm - rpm) / 250.0, 0.0, 1.0) * 0.35;
    Real thr = std::max(clamp(throttle, 0.0, 1.0), idleThrottle);
    if (limiterCut_) thr = 0;
    const Real loss = omega > 0 ? e.frictionTorque + e.frictionPerRadS * omega : 0.0;
    return thr * (wotTorque(rpm) + loss) - loss;
}

Real Drivetrain::gearRatio(int gear) const {
    const auto& gb = def_.gearbox;
    if (gear > 0 && gear <= static_cast<int>(gb.forwardRatios.size())) return gb.forwardRatios[gear - 1];
    if (gear < 0) return -gb.reverseRatio;
    return 0;
}

Real Drivetrain::engineOmegaForGear(int gear) const {
    const Real g = gearRatio(gear);
    Real w = 0;
    for (const auto& da : def_.drivenAxles) {
        const auto& wh = axleWheels_[da.axleIndex];
        w += da.torqueShare * g * da.differential.finalDrive * 0.5 * (omega_[kWheel0 + wh[0]] + omega_[kWheel0 + wh[1]]);
    }
    return w;
}

void Drivetrain::step(Real dt, const DrivetrainCommand& cmd, const WheelSolveInput* wheels, WheelSolveOutput* out) {
    const auto& e = def_.engine;
    const std::size_t nShaft = 2 + wheelCount_;
    const std::size_t nSlip = wheelCount_;
    const Real gear = gearRatio(cmd.gear);
    const bool inGear = gear != 0 && !def_.drivenAxles.empty();

    // Rev limiter with hysteresis.
    const Real rpm0 = radSToRpm(omega_[kEngine]);
    if (rpm0 > e.limiterRpm) limiterCut_ = true;
    else if (rpm0 < e.limiterRpm - 150) limiterCut_ = false;

    Real tEngine = engineTorque(omega_[kEngine], cmd.throttle);
    telem_.engineTorque = tEngine;
    if (tEngine > 0 && inGear) tEngine *= def_.gearbox.efficiency;  // lumped mechanical losses

    // Equality row (gear mesh + open differentials):
    //   omega_gbIn - sum_a share_a * gear * final_a * mean(omega_axle_a) = 0
    std::array<Real, 2 + kMaxWheels> jGear{};
    if (inGear) {
        jGear[kGearboxIn] = 1;
        for (const auto& da : def_.drivenAxles) {
            const auto& wh = axleWheels_[da.axleIndex];
            const Real k = da.torqueShare * gear * da.differential.finalDrive * 0.5;
            jGear[kWheel0 + wh[0]] -= k;
            jGear[kWheel0 + wh[1]] -= k;
        }
    }

    // Friction capacities for this step.
    const bool power = tEngine >= 0;
    for (std::size_t f = 0; f < frictionCount_; ++f) {
        FrictionRow& r = friction_[f];
        switch (r.kind) {
            case FrictionKind::Clutch:
                r.capacity = clamp(cmd.clutchEngagement, 0.0, 1.0) * def_.clutch.maxTorque;
                break;
            case FrictionKind::Brake:
                r.capacity = std::max(0.0, wheels[r.index].brakeCapacity);
                break;
            case FrictionKind::AxleDiff: {
                const auto& da = def_.drivenAxles[r.index];
                if (da.differential.type == DifferentialType::Open) {
                    r.capacity = 0;
                } else if (da.differential.type == DifferentialType::Locked) {
                    r.capacity = kLockedCapacity;
                } else {
                    const Real axleIn = std::abs(lastInputTorque_ * gear * da.differential.finalDrive) * da.torqueShare;
                    const Real ramp = power ? da.differential.powerRamp : da.differential.coastRamp;
                    r.capacity = da.differential.preload + ramp * axleIn;
                }
                break;
            }
            case FrictionKind::CenterDiff:
                r.capacity = def_.center == CenterCoupling::Locked
                                 ? kLockedCapacity
                                 : def_.centerPreload + def_.centerRamp * std::abs(lastInputTorque_ * gear);
                break;
        }
        if (r.capacity <= 0) {
            r.locked = false;
            r.slipForce = 0;
            continue;
        }
        Real v0 = 0;
        for (std::size_t i = 0; i < nShaft; ++i) v0 += r.j[i] * omega_[i];
        if (!r.locked) {
            if (std::abs(v0) < 1e-3) r.locked = true;
            else r.slipForce = -r.capacity * sign(v0);
        }
    }

    DenseSystem<kSystemCapacity> sys;
    std::array<int, 2 + kMaxWheels + kMaxAxles> lockedColumn{};
    std::array<Real, 2 + kMaxWheels> omega1{};
    std::array<Real, kMaxWheels> kappa1{};
    Real gearLambda = 0;
    int iterations = 0;
    bool failed = false;

    for (; iterations < 12; ++iterations) {
        std::size_t n = nShaft + nSlip + (inGear ? 1 : 0);
        for (std::size_t f = 0; f < frictionCount_; ++f) {
            lockedColumn[f] = -1;
            if (friction_[f].capacity > 0 && friction_[f].locked) lockedColumn[f] = static_cast<int>(n++);
        }
        sys.reset(n);

        // Shaft rows.
        const Real inertia[2] = {e.inertia, def_.gearbox.inputInertia};
        for (std::size_t i = 0; i < nShaft; ++i) {
            if (i < kWheel0) {
                sys.a(i, i) = inertia[i] / dt;
                sys.b(i) = inertia[i] * omega_[i] / dt;
            } else {
                const WheelSolveInput& w = wheels[i - kWheel0];
                const std::size_t ks = nShaft + (i - kWheel0);
                sys.a(i, i) = w.inertia / dt + w.radius * w.radius * w.slipDamping;
                sys.a(i, ks) = w.radius * w.dFxdKappa;
                sys.b(i) = w.inertia * omega_[i] / dt + w.externalTorque - w.radius * w.fx0 +
                           w.radius * w.dFxdKappa * w.kappa0 + w.radius * w.slipDamping * w.contactVx;
            }
        }
        sys.b(kEngine) += tEngine;

        // Transient longitudinal slip rows:
        //   sigma*(k1 - k0)/dt + |vx|*k1 - r*omega1 = -vx
        for (std::size_t w = 0; w < nSlip; ++w) {
            const WheelSolveInput& in = wheels[w];
            const std::size_t ks = nShaft + w;
            const Real sigma = std::max(in.relaxationLength, 1e-3);
            sys.a(ks, ks) = sigma / dt + std::abs(in.contactVx);
            sys.a(ks, kWheel0 + w) = -in.radius;
            sys.b(ks) = sigma * in.kappa0 / dt - in.contactVx;
        }

        // Equality row.
        if (inGear) {
            const std::size_t row = nShaft + nSlip;
            for (std::size_t i = 0; i < nShaft; ++i) {
                sys.a(row, i) = jGear[i];
                sys.a(i, row) = -jGear[i];
            }
            sys.a(row, row) = -kCompliance;
        }

        // Friction rows: locked -> constraint; slipping -> known generalized force.
        for (std::size_t f = 0; f < frictionCount_; ++f) {
            const FrictionRow& r = friction_[f];
            if (r.capacity <= 0) continue;
            if (lockedColumn[f] >= 0) {
                const std::size_t col = static_cast<std::size_t>(lockedColumn[f]);
                for (std::size_t i = 0; i < nShaft; ++i) {
                    sys.a(col, i) = r.j[i];
                    sys.a(i, col) = -r.j[i];
                }
                // Small compliance keeps redundant locks (e.g. locked diff +
                // both brakes locked) well-posed; error ~ lambda * kCompliance.
                sys.a(col, col) = -kCompliance;
            } else {
                for (std::size_t i = 0; i < nShaft; ++i) sys.b(i) += r.j[i] * r.slipForce;
            }
        }

        if (!sys.solve()) {
            failed = true;
            break;
        }
        for (std::size_t i = 0; i < nShaft; ++i) omega1[i] = sys.x(i);
        for (std::size_t w = 0; w < nSlip; ++w) kappa1[w] = sys.x(nShaft + w);
        gearLambda = inGear ? sys.x(nShaft + nSlip) : 0;

        bool changed = false;
        for (std::size_t f = 0; f < frictionCount_; ++f) {
            FrictionRow& r = friction_[f];
            if (r.capacity <= 0) continue;
            if (lockedColumn[f] >= 0) {
                r.lambda = sys.x(static_cast<std::size_t>(lockedColumn[f]));
                if (std::abs(r.lambda) > r.capacity * (1 + 1e-9)) {
                    r.locked = false;
                    r.slipForce = clamp(r.lambda, -r.capacity, r.capacity);
                    changed = true;
                }
            } else {
                Real v1 = 0;
                for (std::size_t i = 0; i < nShaft; ++i) v1 += r.j[i] * omega1[i];
                r.lambda = r.slipForce;
                // Friction must oppose the resulting relative velocity; if it
                // would reverse it, the element sticks instead.
                if (v1 * r.slipForce > 0) {
                    r.locked = true;
                    changed = true;
                }
            }
        }
        if (!changed) break;
    }

    telem_.activeSolverIterations = iterations + 1;
    telem_.solveFailed = failed;
    if (failed) {
        // Keep the previous state; report zero force. Surfaced in telemetry.
        for (std::size_t w = 0; w < wheelCount_; ++w) {
            out[w].omega = omega_[kWheel0 + w];
            out[w].kappa = wheels[w].kappa0;
            out[w].fx = 0;
            out[w].brakeLocked = false;
        }
        return;
    }

    for (std::size_t i = 0; i < nShaft; ++i) omega_[i] = omega1[i];
    // The engine model has no stall/start cycle yet (roadmap); keep it turning.
    if (omega_[kEngine] < 0) omega_[kEngine] = 0;

    for (std::size_t w = 0; w < wheelCount_; ++w) {
        const WheelSolveInput& in = wheels[w];
        out[w].omega = omega_[kWheel0 + w];
        out[w].kappa = kappa1[w];
        out[w].fx = in.fx0 + in.dFxdKappa * (kappa1[w] - in.kappa0) +
                    in.slipDamping * (in.radius * out[w].omega - in.contactVx);
        out[w].brakeLocked = false;
    }
    for (std::size_t f = 0; f < frictionCount_; ++f) {
        const FrictionRow& r = friction_[f];
        if (r.kind == FrictionKind::Brake && r.capacity > 0 && r.locked) out[r.index].brakeLocked = true;
        if (r.kind == FrictionKind::Clutch) telem_.clutchLocked = r.capacity > 0 && r.locked;
    }

    lastInputTorque_ = gearLambda;
    telem_.driveshaftTorque = gearLambda;
    telem_.engineRpm = radSToRpm(omega_[kEngine]);
    telem_.clutchSlip = omega_[kEngine] - omega_[kGearboxIn];
}

}  // namespace ord
