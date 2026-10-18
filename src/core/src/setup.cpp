// OpenRoad Dynamics — engine-independent vehicle physics core.
// SPDX-License-Identifier: MIT
#include "ord/setup.hpp"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "ord/aero.hpp"

namespace ord {

namespace {

constexpr Real kDeg = kPi / 180.0;
const char* kCompoundNames[kTyreCompounds] = {"stock", "off-road", "all-season", "street", "sport", "semi-slick", "slick", "rain"};

}  // namespace

const char* compoundName(TyreCompound c) {
    const int i = static_cast<int>(c);
    return i >= 0 && i < kTyreCompounds ? kCompoundNames[i] : "stock";
}

void enableCarCamberModel(TireParams& t) {
    // Camber thrust ~1 Fz/rad, best lateral grip leaning ~2 deg into the turn,
    // ~8% lateral grip lost 3 deg away from that, ~3% braking/traction grip
    // lost at 3 deg of camber (a road radial; Pacejka 2012 ch. 4 orders of
    // magnitude, not a measured tyre).
    if (t.camberStiffness <= 0) t.camberStiffness = 0.8;
    if (t.camberGripLoss <= 0) t.camberGripLoss = 30.0;
    if (t.camberLongGripLoss <= 0) t.camberLongGripLoss = 11.0;
    if (t.camberOptimum == 0) t.camberOptimum = 2.0 * kDeg;
}

void applyCompound(TireParams& t, TyreCompound c) {
    if (c == TyreCompound::Stock) return;
    struct C {
        Real mu, stiff, shapeY, wet, aqua, loose, tOpt, tWin, tLoss, rr, relax, camberLoss;
    };
    // Relative to the car's stock road tyre. Representative values: grip
    // ratios follow the usual ladder (road ~1.0 g, semi-slick ~1.2, slick
    // ~1.3 on a dry, warm surface), slicks lose most of their grip in the wet
    // (no tread to clear water: aquaplaning starts far earlier) and on loose
    // ground, knobbly tyres the reverse. Temperature windows: road tyres work
    // cold, slicks need ~80-110 C.
    static const C table[kTyreCompounds] = {
        {1, 1, 0, 1, 1, 1, 60, 45, 0.10, 1, 1, 1},             // stock (unused)
        {0.85, 0.70, 1.25, 0.95, 0.80, 1.30, 45, 50, 0.08, 1.5, 1.3, 0.6},   // off-road (knobbly)
        {0.95, 0.90, 0, 1.05, 0.90, 1.05, 45, 45, 0.08, 1.05, 1.05, 0.8},   // all-season
        {1.00, 1.00, 0, 1.00, 1.00, 1.00, 60, 45, 0.10, 1.0, 1.0, 1.0},     // street (summer)
        {1.08, 1.10, 0, 0.95, 1.10, 0.95, 75, 35, 0.12, 0.95, 0.95, 1.1},   // sport
        {1.18, 1.25, 0, 0.75, 1.60, 0.70, 85, 30, 0.15, 0.9, 0.9, 1.25},    // semi-slick
        {1.30, 1.35, 1.45, 0.45, 4.00, 0.58, 95, 25, 0.20, 0.9, 0.85, 1.35}, // slick
        {1.00, 0.95, 0, 1.15, 0.40, 1.00, 50, 25, 0.18, 1.05, 1.0, 1.0},    // rain (full wet)
    };
    const C& k = table[static_cast<int>(c)];
    t.muX *= k.mu;
    t.muY *= k.mu;
    t.cornerStiffness *= k.stiff;
    t.longStiffnessPerLoad *= k.stiff;
    if (k.shapeY > 0) t.shapeY = k.shapeY;
    t.wetGrip = k.wet;
    t.aquaplaneSensitivity = k.aqua;
    t.looseGrip = k.loose;
    t.optimalTemperature = k.tOpt;
    t.temperatureWindow = k.tWin;
    t.temperatureGripLoss = k.tLoss;
    t.rollingResistance *= k.rr;
    t.relaxationLengthY *= k.relax;
    // One-node tread: ~2 kg of rubber (3 kJ/K), cooled at ~5 + 1 W/K per m/s:
    // warms in a lap or two, settles ~20 K above ambient when cruising.
    t.thermalCapacity = 3000.0;
    t.heatFraction = 0.35;
    t.coolingBase = 5.0;
    t.coolingPerSpeed = 1.0;
    enableCarCamberModel(t);
    t.camberGripLoss *= k.camberLoss;
}

void applyPressure(TireParams& t, Real bar, Real nominal) {
    const Real dp = clamp((bar - nominal) / nominal, -0.5, 0.6);
    t.cornerStiffness *= clamp(1 + 0.4 * dp, 0.6, 1.4);
    t.verticalStiffness *= clamp(1 + 0.8 * dp, 0.5, 1.6);
    t.relaxationLengthY *= clamp(1 - 0.3 * dp, 0.7, 1.3);
    const Real muF = 1 - 0.15 * dp * dp - 0.03 * dp;  // best near nominal, slightly better a touch low
    t.muX *= muF;
    t.muY *= muF;
    t.rollingResistance *= clamp(1 - 0.5 * dp, 0.6, 1.6);
}

VehicleSetup stockSetup(const VehicleDefinition& d) {
    VehicleSetup s;
    auto axle = [&](std::size_t a, AxleSetup& x) {
        if (a >= d.axles.size()) return;
        const WheelDef& w = d.wheels[static_cast<std::size_t>(d.axles[a].leftWheel)];
        x.camberDeg = w.staticCamber / kDeg;
        x.toeDeg = w.staticToe / kDeg;
        x.casterDeg = w.casterAngle / kDeg;
        x.camberGainDegPerCm = w.camberGain / kDeg / 100.0;
        x.springNmm = w.suspension.springRate / 1000.0;
        x.rideHeightMm = 0;
        x.bumpSlow = w.suspension.bumpDamping;
        x.bumpFast = w.suspension.bumpDampingFast;
        x.reboundSlow = w.suspension.reboundDamping;
        x.reboundFast = w.suspension.reboundDampingFast;
        x.arbNmm = d.axles[a].antiRollBarRate / 1000.0;
        x.compound = TyreCompound::Stock;
        x.pressureBar = 2.2;
    };
    axle(0, s.front);
    axle(d.axles.empty() ? 0 : d.axles.size() - 1, s.rear);
    Real front = 0, total = 0, lock = 0;
    for (std::size_t w = 0; w < d.wheels.size(); ++w) {
        total += d.wheels[w].maxBrakeTorque;
        lock = std::max(lock, d.wheels[w].maxSteerAngle);
    }
    if (!d.axles.empty())
        front = d.wheels[static_cast<std::size_t>(d.axles[0].leftWheel)].maxBrakeTorque +
                d.wheels[static_cast<std::size_t>(d.axles[0].rightWheel)].maxBrakeTorque;
    s.brakeBalance = total > 0 ? front / total : 0.6;
    s.brakePressure = 1.0;
    s.steerLockDeg = lock / kDeg;
    if (!d.drivetrain.drivenAxles.empty()) {
        const DifferentialDef& df = d.drivetrain.drivenAxles.back().differential;
        s.diffType = -1;
        s.diffPreload = df.preload;
        s.diffPower = df.powerRamp;
        s.diffCoast = df.coastRamp;
        s.finalDrive = df.finalDrive;
    }
    s.gearCount = static_cast<int>(std::min<std::size_t>(8, d.drivetrain.gearbox.forwardRatios.size()));
    for (int g = 0; g < s.gearCount; ++g) s.gears[static_cast<std::size_t>(g)] = d.drivetrain.gearbox.forwardRatios[static_cast<std::size_t>(g)];
    s.rearWing = d.aero.rearWing.span > 0;
    s.wingAngleDeg = s.rearWing ? d.aero.rearWing.angle / kDeg : 8.0;
    s.altitudeM = 0;
    s.temperatureC = 15;
    return s;
}

VehicleDefinition applySetup(const VehicleDefinition& stock, const VehicleSetup& s) {
    VehicleDefinition d = stock;
    const std::size_t na = d.axles.size();
    for (std::size_t a = 0; a < na; ++a) {
        const AxleSetup& x = a == 0 ? s.front : s.rear;
        d.axles[a].antiRollBarRate = std::max(0.0, x.arbNmm) * 1000.0;
        for (int wi : {d.axles[a].leftWheel, d.axles[a].rightWheel}) {
            WheelDef& w = d.wheels[static_cast<std::size_t>(wi)];
            w.staticCamber = x.camberDeg * kDeg;
            w.staticToe = x.toeDeg * kDeg;
            w.casterAngle = w.maxSteerAngle > 0 ? x.casterDeg * kDeg : 0.0;
            w.camberGain = x.camberGainDegPerCm * kDeg * 100.0;
            w.suspension.springRate = std::max(1.0, x.springNmm) * 1000.0;
            w.suspension.hardpoint.z -= x.rideHeightMm / 1000.0;  // lower hardpoint = wheel further down = car higher
            w.suspension.bumpDamping = std::max(0.0, x.bumpSlow);
            w.suspension.bumpDampingFast = std::max(0.0, x.bumpFast);
            w.suspension.reboundDamping = std::max(0.0, x.reboundSlow);
            w.suspension.reboundDampingFast = std::max(0.0, x.reboundFast);
            applyCompound(w.tire, x.compound);
            applyPressure(w.tire, x.pressureBar);
            enableCarCamberModel(w.tire);
        }
    }
    // Brakes: total x pressure, split front/rear by the balance.
    if (na >= 2) {
        Real total = 0;
        for (const WheelDef& w : stock.wheels) total += w.maxBrakeTorque;
        total *= clamp(s.brakePressure, 0.2, 2.0);
        const Real bal = clamp(s.brakeBalance, 0.2, 0.85);
        std::size_t nf = 0, nr = 0;
        for (std::size_t a = 0; a < na; ++a) (a == 0 ? nf : nr) += 2;
        for (std::size_t a = 0; a < na; ++a)
            for (int wi : {d.axles[a].leftWheel, d.axles[a].rightWheel})
                d.wheels[static_cast<std::size_t>(wi)].maxBrakeTorque =
                    a == 0 ? total * bal / static_cast<Real>(nf) : total * (1 - bal) / static_cast<Real>(nr);
    }
    // Steering lock
    Real lock = 0;
    for (const WheelDef& w : d.wheels) lock = std::max(lock, w.maxSteerAngle);
    if (lock > 0) {
        const Real k = clamp(s.steerLockDeg * kDeg, 0.1, 1.2) / lock;
        for (WheelDef& w : d.wheels) w.maxSteerAngle *= k;
    }
    // Drivetrain
    if (!d.drivetrain.drivenAxles.empty()) {
        for (auto& da : d.drivetrain.drivenAxles) da.differential.finalDrive = clamp(s.finalDrive, 1.5, 8.0);
        DifferentialDef& df = d.drivetrain.drivenAxles.back().differential;
        if (s.diffType >= 0) {
            df.type = s.diffType == 0 ? DifferentialType::Open : s.diffType == 1 ? DifferentialType::Locked : DifferentialType::ClutchLsd;
            df.preload = std::max(0.0, s.diffPreload);
            df.powerRamp = clamp(s.diffPower, 0.0, 1.0);
            df.coastRamp = clamp(s.diffCoast, 0.0, 1.0);
        }
    }
    if (s.gearCount > 0) {
        d.drivetrain.gearbox.forwardRatios.assign(s.gears.begin(), s.gears.begin() + std::min(8, s.gearCount));
        for (Real& r : d.drivetrain.gearbox.forwardRatios) r = clamp(r, 0.3, 6.0);
    }
    // Aero
    d.aero.frontDownforceArea *= std::max(0.0, s.frontAeroScale);
    d.aero.rearDownforceArea *= std::max(0.0, s.rearAeroScale);
    if (s.rearWing) {
        auto& wing = d.aero.rearWing;
        if (wing.span <= 0 && na >= 1) {
            // A 1.4 m x 0.28 m single-element wing over the rear axle, ~0.75 m above the CoM.
            const WheelDef& rw = d.wheels[static_cast<std::size_t>(d.axles[na - 1].leftWheel)];
            wing.span = 1.4;
            wing.chord = 0.28;
            wing.position = {rw.suspension.hardpoint.x - 0.35, 0, d.centerOfMass.z + 0.75};
        }
        wing.angle = clamp(s.wingAngleDeg, -5.0, 30.0) * kDeg;
    } else {
        d.aero.rearWing.span = 0;
    }
    const Real alt = clamp(s.altitudeM, -400.0, 5000.0), temp = clamp(s.temperatureC, -30.0, 50.0);
    d.aero.airDensity = airDensity(alt, temp);
    if (!s.turbocharged) {
        const Real k = enginePowerFactor(alt, temp) / enginePowerFactor(0, 15);
        for (auto& pt : d.drivetrain.engine.torqueCurve) pt.torque *= k;
    }
    d.aero.ambientTemperature = s.temperatureC;
    return d;
}

// ---------------------------------------------------------------- JSON

namespace {

void axleJson(std::string& o, const char* name, const AxleSetup& x) {
    char b[700];
    std::snprintf(b, sizeof b,
                  "  \"%s\": {\"camberDeg\": %.3f, \"toeDeg\": %.3f, \"casterDeg\": %.3f, \"camberGainDegPerCm\": %.3f, "
                  "\"springNmm\": %.2f, \"rideHeightMm\": %.1f, \"bumpSlow\": %.0f, \"bumpFast\": %.0f, \"reboundSlow\": %.0f, "
                  "\"reboundFast\": %.0f, \"arbNmm\": %.2f, \"compound\": \"%s\", \"pressureBar\": %.2f}",
                  name, x.camberDeg, x.toeDeg, x.casterDeg, x.camberGainDegPerCm, x.springNmm, x.rideHeightMm, x.bumpSlow,
                  x.bumpFast, x.reboundSlow, x.reboundFast, x.arbNmm, compoundName(x.compound), x.pressureBar);
    o += b;
}

// Minimal reader for the setup format: finds `"key": value` inside the
// object named by `scope` ("" = top level). Tolerant of whitespace and order.
struct Json {
    const std::string& s;
    // Returns the [begin, end) of the object value of key `name` at top level, or the whole text.
    bool object(const char* name, std::size_t& b, std::size_t& e) const {
        const std::string k = std::string("\"") + name + "\"";
        const std::size_t p = s.find(k);
        if (p == std::string::npos) return false;
        b = s.find('{', p + k.size());
        if (b == std::string::npos) return false;
        int depth = 0;
        for (e = b; e < s.size(); ++e) {
            if (s[e] == '{') ++depth;
            else if (s[e] == '}' && --depth == 0) { ++e; return true; }
        }
        return false;
    }
    bool raw(const char* key, std::size_t b, std::size_t e, std::size_t& v) const {
        const std::string k = std::string("\"") + key + "\"";
        std::size_t p = s.find(k, b);
        while (p != std::string::npos && p < e) {
            std::size_t c = p + k.size();
            while (c < e && std::isspace(static_cast<unsigned char>(s[c]))) ++c;
            if (c < e && s[c] == ':') {
                ++c;
                while (c < e && std::isspace(static_cast<unsigned char>(s[c]))) ++c;
                v = c;
                return true;
            }
            p = s.find(k, p + 1);
        }
        return false;
    }
    void num(const char* key, std::size_t b, std::size_t e, Real& out) const {
        std::size_t v;
        if (raw(key, b, e, v)) {
            char* end = nullptr;
            const double d = std::strtod(s.c_str() + v, &end);
            if (end != s.c_str() + v && std::isfinite(d)) out = d;
        }
    }
    void integer(const char* key, std::size_t b, std::size_t e, int& out) const {
        Real d = out;
        num(key, b, e, d);
        out = static_cast<int>(std::lround(d));
    }
    void boolean(const char* key, std::size_t b, std::size_t e, bool& out) const {
        std::size_t v;
        if (raw(key, b, e, v)) out = s.compare(v, 4, "true") == 0;
    }
    void compound(const char* key, std::size_t b, std::size_t e, TyreCompound& out) const {
        std::size_t v;
        if (!raw(key, b, e, v) || s[v] != '"') return;
        const std::size_t q = s.find('"', v + 1);
        const std::string name = s.substr(v + 1, q - v - 1);
        for (int i = 0; i < kTyreCompounds; ++i)
            if (name == kCompoundNames[i]) out = static_cast<TyreCompound>(i);
    }
};

}  // namespace

std::string setupToJson(const VehicleSetup& s) {
    std::string o = "{\n";
    axleJson(o, "front", s.front);
    o += ",\n";
    axleJson(o, "rear", s.rear);
    char b[900];
    std::string gears;
    for (int g = 0; g < s.gearCount; ++g) {
        char gb[24];
        std::snprintf(gb, sizeof gb, "%s%.3f", g ? ", " : "", s.gears[static_cast<std::size_t>(g)]);
        gears += gb;
    }
    std::snprintf(b, sizeof b,
                  ",\n  \"brakeBalance\": %.3f, \"brakePressure\": %.3f, \"steerLockDeg\": %.2f,\n"
                  "  \"diffType\": %d, \"diffPreload\": %.1f, \"diffPower\": %.3f, \"diffCoast\": %.3f, \"finalDrive\": %.3f,\n"
                  "  \"gears\": [%s],\n"
                  "  \"frontAeroScale\": %.3f, \"rearAeroScale\": %.3f, \"rearWing\": %s, \"wingAngleDeg\": %.2f,\n"
                  "  \"altitudeM\": %.0f, \"temperatureC\": %.1f, \"turbocharged\": %s\n}\n",
                  s.brakeBalance, s.brakePressure, s.steerLockDeg, s.diffType, s.diffPreload, s.diffPower, s.diffCoast,
                  s.finalDrive, gears.c_str(), s.frontAeroScale, s.rearAeroScale, s.rearWing ? "true" : "false",
                  s.wingAngleDeg, s.altitudeM, s.temperatureC, s.turbocharged ? "true" : "false");
    return o + b;
}

std::string setupFromJson(const std::string& text, VehicleSetup& out) {
    if (text.find('{') == std::string::npos) return "not a JSON object";
    {
        // structure: balanced braces and brackets outside strings, strings closed
        int braces = 0, brackets = 0;
        bool inString = false;
        for (std::size_t i = 0; i < text.size(); ++i) {
            const char c = text[i];
            if (inString) {
                if (c == '\\') ++i;
                else if (c == '"') inString = false;
                continue;
            }
            if (c == '"') inString = true;
            else if (c == '{') ++braces;
            else if (c == '}') --braces;
            else if (c == '[') ++brackets;
            else if (c == ']') --brackets;
            if (braces < 0 || brackets < 0) return "unbalanced braces or brackets";
        }
        if (inString) return "unterminated string";
        if (braces != 0 || brackets != 0) return "unbalanced braces or brackets";
    }
    const Json j{text};
    VehicleSetup s = out;
    for (int a = 0; a < 2; ++a) {
        std::size_t b, e;
        if (!j.object(a == 0 ? "front" : "rear", b, e)) continue;
        AxleSetup& x = a == 0 ? s.front : s.rear;
        j.num("camberDeg", b, e, x.camberDeg);
        j.num("toeDeg", b, e, x.toeDeg);
        j.num("casterDeg", b, e, x.casterDeg);
        j.num("camberGainDegPerCm", b, e, x.camberGainDegPerCm);
        j.num("springNmm", b, e, x.springNmm);
        j.num("rideHeightMm", b, e, x.rideHeightMm);
        j.num("bumpSlow", b, e, x.bumpSlow);
        j.num("bumpFast", b, e, x.bumpFast);
        j.num("reboundSlow", b, e, x.reboundSlow);
        j.num("reboundFast", b, e, x.reboundFast);
        j.num("arbNmm", b, e, x.arbNmm);
        j.compound("compound", b, e, x.compound);
        j.num("pressureBar", b, e, x.pressureBar);
    }
    // Top-level keys: search outside the axle objects.
    std::size_t fb = 0, fe = 0, rb = 0, re = 0;
    const bool hf = j.object("front", fb, fe), hr = j.object("rear", rb, re);
    const std::size_t top = std::max(hf ? fe : 0, hr ? re : 0), end = text.size();
    j.num("brakeBalance", top, end, s.brakeBalance);
    j.num("brakePressure", top, end, s.brakePressure);
    j.num("steerLockDeg", top, end, s.steerLockDeg);
    j.integer("diffType", top, end, s.diffType);
    j.num("diffPreload", top, end, s.diffPreload);
    j.num("diffPower", top, end, s.diffPower);
    j.num("diffCoast", top, end, s.diffCoast);
    j.num("finalDrive", top, end, s.finalDrive);
    {
        std::size_t v;
        if (j.raw("gears", top, end, v) && text[v] == '[') {
            int n = 0;
            const char* p = text.c_str() + v + 1;
            while (n < 8) {
                char* q = nullptr;
                const double g = std::strtod(p, &q);
                if (q == p) break;
                s.gears[static_cast<std::size_t>(n++)] = g;
                p = q;
                while (*p == ',' || std::isspace(static_cast<unsigned char>(*p))) ++p;
                if (*p == ']') break;
            }
            s.gearCount = n;
        }
    }
    j.num("frontAeroScale", top, end, s.frontAeroScale);
    j.num("rearAeroScale", top, end, s.rearAeroScale);
    j.boolean("rearWing", top, end, s.rearWing);
    j.num("wingAngleDeg", top, end, s.wingAngleDeg);
    j.num("altitudeM", top, end, s.altitudeM);
    j.num("temperatureC", top, end, s.temperatureC);
    j.boolean("turbocharged", top, end, s.turbocharged);
    out = s;
    return "";
}

}  // namespace ord
