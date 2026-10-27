> Research note, 22 October 2026: open-source code for vehicle damage and deformation, and what an MIT engine may borrow. Licences were read from each repository's own LICENSE/COPYING file on that date; re-check before copying anything. Nothing listed here is part of this repository. The browser demo's dents (crush drawn on the mesh from the crash model) are a first step of path (a) below.

# Vehicle damage and deformation: open-source survey

Target: a small C++20 vehicle engine (rigid body, Pacejka tyres, suspension linkages) compiled to WASM for a three.js demo, MIT licensed. The user's goal is "BeamNG-level destruction" without any BeamNG code or JBeam data.

Survey date: 2026-10-22.

## How this was checked

Licences: I read each LICENSE/COPYING file directly from raw.githubusercontent.com or from a shallow clone, not from the GitHub badge. Where the repo has no licence file, I tried the `main` and `master` branches with the names LICENSE, LICENSE.md, LICENSE.txt, COPYING and license.txt.

Last activity: the date of the newest commit on the default branch, read from a shallow or blobless clone.

Stars: the GitHub API (`gh`) was blocked in this session. Star counts for small repos come from GitHub search results. Counts for the big engines are rough figures from memory and are marked "~(unverified)".

Code: I read the code paths named below in the clone or the raw file. Anything I could not verify is marked as unverified.

---

## 1. Node/beam (mass-spring) soft-body solvers

### 1.1 Rigs of Rods: reference only, cannot borrow
- URL: https://github.com/RigsOfRods/rigs-of-rods
- Licence: **GPL-3.0** (`COPYING` read).
- Language: C++. Last commit 2026-10-20. Stars ~1k+ (unverified).
- What it is: the open-source ancestor of BeamNG's approach. It is a complete node/beam vehicle solver.
- Code worth reading (not copying):
  - `source/main/physics/ActorForcesEuler.cpp`: `Actor::CalcBeams()` (around line 1210) holds the plastic yield logic: `L += deform`, `plastic_coef`, `strength -= deform*k`, and the break test.
  - `CalcWheels()` and `CalcShocks()` are in the same file.
  - Node-to-triangle collision is in `source/main/physics/collision/` (`DynamicCollisions.cpp`, `PointColDetector.cpp`).
  - `SimData.h` holds the beam and node structs.
- Verdict: **reference only**. GPL-3 would force the MIT engine into GPL. You can learn the published idea (spring-damper beam, rest length drifts past a yield force, beam breaks above strength) and re-implement it from papers or the permissive repos below. Do not port RoR's code line by line.
- Related: https://github.com/htdguide/ror-godot puts the RoR solver in a Godot GDExtension. It is **GPL-3.0** (read) and also reference only.

### 1.2 kami-vehicle (kotoba-lang): adapt algorithm (code is Clojure)
- URL: https://github.com/kotoba-lang/kami-vehicle
- Licence: **Apache-2.0** (`LICENSE` read).
- Language: CLJC (Clojure dialect, `.cljk` files). About 2.1k lines. Last commit 2026-09-24. 0 stars.
- What it is: a port of a deleted Rust crate. It is a BeamNG-style soft-body car with:
  - node and beam types (normal, bounded, hydro, pressured, support);
  - an XPBD loop at 2000 Hz substeps, plus an implicit-Euler + CG integrator;
  - shape-matching projection for the rigid chassis;
  - wheel tyre rings with a Pacejka tyre model;
  - a powertrain and a JBeam-subset loader;
  - a parametric sedan of about 86 nodes and 220 beams.
- Worth borrowing:
  - `src/vehicle/beam.cljk` `update-plastic`: yield at `deform-limit` strain, effective rest length follows, capped by `max-plastic-strain`, snaps at `break-limit`. Defaults are 0.10, 0.45 and 0.40.
  - `src/vehicle/models/sedan.cljk`: a *parametric* layout of floor, belt and roof rings plus subframe and wheel hubs. It is generated from wheelbase and track numbers, not copied from BeamNG data. The docstring says it "matches Pessima granularity", which is a description, not data reuse.
  - `rigid_chassis.cljk`: shape-matching chassis.
  - `implicit.cljk`: CG solver.
- Integration effort: medium. It has to be rewritten in C++, but it is small and has tests (56 tests).
- Caveats: very new, zero stars, auto-ported. Treat it as a design sketch and validate it yourself. Apache-2.0 requires keeping the NOTICE and attribution if you copy code.
- Verdict: **adapt the algorithm**. This is the closest permissive blueprint of a whole BeamNG-like vehicle architecture.

### 1.3 CrashVector (ArrowSK): adapt the algorithm, MPL file-level copyleft
- URL: https://github.com/ArrowSK/crashvector
- Licence: **MPL-2.0** (`LICENSE` read; per-file MPL headers). This is not in the permissive list:
  - files you copy must stay MPL and keep their source available;
  - your own files can stay MIT;
  - re-implementing the algorithm has no restriction.
- Language: GDScript (Godot 4.4). About 38k lines. Last commit 2026-10-20. 0 stars.
- Architecture: this is the most relevant one for us. A `RigidBody3D` owns the vehicle's world motion. A separate local **structural graph** (nodes, beams, bending constraints, longitudinal guards) owns permanent deformation. The visual shell is rebuilt from the graph's node positions.
- Worth borrowing:
  - `src/structural/structural_beam.gd`:
    - progressive force-strain curve: linear, then a post-yield stiffness ratio, then hardening;
    - `_apply_plastic_flow()`: rate-limited rest-length drift with plastic-energy accounting;
    - break strain.
  - `structural_bending_constraint.gd`.
  - `structural_longitudinal_guard.gd`: stops crush going through the cabin.
  - `structural_model.gd`: substepping plus an energy ledger (elastic, plastic, fracture, damping). This ledger is useful for tuning.
  - `src/vehicles/deformable_body_shell.gd`: procedural body mesh rebuilt from structural "stations".
  - `src/calibration/*`: matching crush depth against reference crash numbers.
- Integration effort: medium. It has to be ported to C++. The hybrid design maps cleanly onto our rigid-body engine.
- Verdict: **adapt the algorithm** (do not copy files unless you accept MPL on them).

### 1.4 Reformers (kbence): algorithm reference only, provenance risk
- URL: https://github.com/kbence/reformers
- Licence: the repo's own code is **MIT** (read). However, it is a "literal 16.16 fixed-point port" of the 1996 DOS game -DEFORMERS-, reverse-engineered: the comments cite binary addresses such as `0x4fe3: yield`. The game assets belong to the original author.
- Language: Rust. About 10k lines. Has a WASM web build. Last commit 2026-10-21. 1 star.
- Worth looking at: `src/sim.rs` `Sim::springs()` (around line 781). It is a tiny position-based spring: yield moves the rest length by e>>4, and the spring breaks below a threshold. `collide_cars()` and `damage()` are also there.
- Verdict: **reference only, for the idea**. The MIT grant over a decompiled port is legally murky, and the method is trivial to re-implement anyway.

### 1.5 TOTALLED (Dumb-Tony): reference only, no licence
- URL: https://github.com/Dumb-Tony/TOTALLED
- Licence: **no LICENSE file**. It is a Unity 6 C# node-and-beam derby with a WebGL build. Last commit 2026-09-30. 0 stars.
- It is useful as proof that a node/beam car can run in a browser. Its docs folder holds benchmarks and destruction before/after JSON.
- Verdict: **reference only**.

### 1.6 JellyCar (DrakonPL), JBeam2D (vertviger), TR-MZ softbody approximation: reference only
- JellyCar (https://github.com/DrakonPL/JellyCar): C++, 2D pressure soft bodies, last commit 2020, about 41 stars, no LICENSE.
- JBeam2D (https://github.com/vertviger/JBeam2D): C++, no LICENSE.
- TR-MZ (https://github.com/TR-MZ/Realtime-Car-Softbody-Approximation): only a README and videos, no code.
- Verdict: **reference only, low value**.

### 1.7 Ten Minute Physics (Matthias Müller): borrow
- URL: https://github.com/matthias-research/pages (folder `tenMinutePhysics/`)
- Licence: the repo has no top-level LICENSE, but **each demo file carries an MIT header**. I read the headers in `10-softBodies.html` and `12-softBodySkinning.html`; the `01`–`08` files also mention it.
- Language: JavaScript with three.js, in single HTML files. Last activity: recent. Stars ~2k+ (unverified).
- Worth borrowing:
  - `10-softBodies.html`: XPBD tetrahedral soft body with edge and volume constraints, about 200 lines.
  - `12-softBodySkinning.html`: embeds a high-res visual mesh in a coarse tet mesh using barycentric skinning. This is exactly the "drive the three.js car mesh from a coarse physics cage" technique.
  - `11-hashing.html`: spatial hash for node collisions.
  - `22-rigidBodies.html` and `25-joints.html`: XPBD rigid bodies and joints.
  - The `09-xpbd.pdf` notes.
- Integration effort: low. Porting to C++ is direct, and the skinning stays in JS/three.js.
- Verdict: **borrow** (keep the MIT notice).

### 1.8 PositionBasedDynamics (Jan Bender): borrow selectively
- URL: https://github.com/InteractiveComputerGraphics/PositionBasedDynamics
- Licence: **MIT** (read). Language: C++ (needs Eigen3). Last commit 2026-09-01. Stars ~2k (unverified).
- Worth borrowing, in `PositionBasedDynamics/PositionBasedDynamics.{h,cpp}` and `XPBD.{h,cpp}`:
  - `solve_DistanceConstraint` (the beam);
  - `solve_VolumeConstraint`;
  - `solve_FEMTetraConstraint` and `solve_StrainTetraConstraint` (anisotropic stiffness for sheet-metal-like behaviour);
  - `init/solve_ShapeMatchingConstraint`;
  - `solve_DihedralConstraint`, `solve_IsometricBendingConstraint` (panel bending);
  - `PositionBasedElasticRods.*` (Cosserat rods for frame rails).
  - XPBD versions are in `XPBD.cpp`.
- Integration effort: low to medium. The functions are static and self-contained. Eigen compiles to WASM fine, or you can swap in your own math types. Plasticity is not included: add rest-length or rest-shape updates yourself.
- Verdict: **borrow** the constraint kernels.

### 1.9 NVIDIA FleX: reference only
- URL: https://github.com/NVIDIAGameWorks/FleX
- Licence: "Nvidia Source Code License (1-Way Commercial)" (read). It is not OSI-permissive and is GPU/CUDA/DX only, so it is unusable in WASM.
- Verdict: **reference only**.

---

## 2. Mesh crumple from an impact point (visual only)

### 2.1 Meshes-Deformation-Script-Unity (TheMrFeliX): reference only, no licence
- URL: https://github.com/TheMrFeliX/Meshes-Deformation-Script-Unity
- Licence: **no LICENSE**. Language: C#, one file `MeshesDeformation.cs`. Last commit 2020-08-04. About 48 stars.
- It shows the canonical game approach: on contact, every vertex within `deformationRadius` of the local contact point moves along the local contact velocity, scaled by `(radius - d) * impactDamage`. The displacement from the original vertex is clamped to `maxDeformation`. Then the mesh collider is updated, and a lerp restores the vertices for repair.
- This is a well-known folk algorithm and is trivial to re-implement.
- Verdict: **reference only**. Re-implement the idea; it is about 30 lines.

### 2.2 Real-Time Plastic Deformation of Car Bodyworks (JamesTheButler): reference only, no licence
- URLs:
  - https://github.com/JamesTheButler/MasterThesisPrototype (Unity C#, last commit 2020-08-31)
  - https://github.com/JamesTheButler/PlasticDeformationDLL (C++)
- Licence: **no LICENSE found** in either repo (checked `master` and `main`).
- Method: a master's thesis. The car surface mesh is tetrahedralised with TetWild. A CPU position-based tet simulation is collided against colliders, and the deformation is mapped back to the surface mesh in under 10 ms per frame.
- This is the same idea as Müller's soft-body skinning (1.7). Use that MIT implementation instead.
- Verdict: **reference only** (useful as a validation that the approach works on a full car mesh).

### 2.3 three.js ConvexObjectBreaker plus Ammo break example: borrow
- URLs:
  - https://github.com/mrdoob/three.js `examples/jsm/misc/ConvexObjectBreaker.js`
  - the `physics_ammo_break.html` example
- Licence: **MIT** (three.js LICENSE read). Ammo.js is **zlib** (read).
- What it gives: `subdivideByImpact(object, point, normal, ...)` and `cutByPlane`. They split convex meshes into debris. Good for glass, light lenses and bumper chunks on the JS side.
- Integration effort: low. It runs in the browser and needs no C++.
- Verdict: **borrow** for cosmetic shattering and debris.

### 2.4 Edy's Vehicle Physics damage and Unity Asset Store tools: reference only
- Impact Deformable, Arcade Damage System and the old Edy's Vehicle Physics wiki (`CarDamage`) all describe the same vertex crumple, plus deforming the collider so later impacts stay consistent.
- They are paid or proprietary.
- Verdict: **reference only**.

---

## 3. Crash and crush models, detachable parts, damage coupled to mechanics

- **Kamal lumped-mass model** (SAE 1970, "Analysis and Simulation of Vehicle to Barrier Impact") and the piecewise lumped-parameter variants:
  - MDPI *Designs* 2018, 2(4):43: https://www.mdpi.com/2411-9660/2/4/43
  - Zubir et al., *Automotive Experiences* 2023: https://journal.unimma.ac.id/AutomotiveExperiences/article/view/9289
  - I found **no open-source code** for these. They are papers, so implement them yourself. A Kamal model is about 5 masses with nonlinear springs and a few dozen lines.
  - Use a force-crush curve with loading and unloading hysteresis per crush zone (front, rear, sides, roof) to drive both the impulse and a "crush depth" per zone.
- **CRASH3 / EES**: the US NHTSA crush-energy method (A and B stiffness coefficients per zone).
  - https://github.com/brunopneves/crash3vision is Python research code. Its licence was **not checked**.
  - Use it as a source of formulas only. The CRASH3 coefficients (energy = f(crush depth, width)) give a physical crush-energy-to-depth mapping for path (a).
- **Detachable parts and breakable joints**: every major permissive engine lets you break a constraint when its impulse passes a threshold. The pattern is identical everywhere: read the impulse, compare it with a threshold, disable the constraint. Doors, bumpers and wheels then become free rigid bodies. See section 4.
- **Damage coupled to mechanics**:
  - I found **no permissive, reusable implementation**.
  - Rigs of Rods gets coupling for free because wheels and suspension are nodes in the same beam graph, so a bent beam is bent geometry. This is GPL, so it is an idea only.
  - CrashVector (MPL) tracks per-component and per-role beam counts (`component_beam_count`, `role_beam_count`). That bookkeeping is a usable pattern for "radiator zone crushed past X → overheating".
  - In our engine, the coupling is our own design: map zone crush depth or per-node offsets to these targets:
    - suspension hardpoint offsets (camber and toe change through our existing linkage solver);
    - steering rack offset;
    - wheel detach (break joint);
    - radiator and engine health;
    - aero drag.

---

## 4. Fracture and breakable constraints in permissive engines

| Engine | Licence (read) | Last commit | What is directly usable | Notes |
|---|---|---|---|---|
| **Jolt Physics** (https://github.com/jrouwe/JoltPhysics) | MIT | 2026-10-18 | `Constraint::SetEnabled()`. The header comment says it is meant for breakable constraints: check `GetTotalLambdaPosition` and disable. XPBD soft bodies (`Jolt/Physics/SoftBody/SoftBodySharedSettings.h`) have edges, dihedral bends, tet volumes, skinned constraints, LRA and Cosserat rods. | I found no built-in plasticity or tearing. Constraints live in a *shared*, conceptually immutable settings object, so plastic rest-length drift needs per-vehicle settings plus custom mutation (unverified whether supported at runtime). |
| **JoltPhysics.js** (https://github.com/jrouwe/JoltPhysics.js) | MIT | 2026-08-23 | Jolt compiled to WASM, with `Examples/soft/*.html` and `vehicle_*.html`. | Shows Jolt-in-WASM with three.js works. You would compile Jolt into our own WASM rather than use this. |
| **Bullet3** (https://github.com/bulletphysics/bullet3) | zlib (except `Extras/` and `examples/ThirdPartyLibs`) | 2026-10-09 | `btTypedConstraint::setBreakingImpulseThreshold()`. `btSoftBody::cutLink()`. `examples/FractureDemo/btFractureBody.*` and `btFractureDynamicsWorld.*` (compound bodies that split on impulse). `examples/VoronoiFracture/`. | `btSoftBody` has no plasticity. Fracture-body code is small and portable (zlib). |
| **Box2D v3** (https://github.com/erincatto/box2d) | MIT | 2026-10-20 | Joint events with `forceThreshold` and `torqueThreshold` (`b2JointEvent`, `b2World_GetJointEvents`) for breakable joints. Soft-step (TGS-soft) solver design. | 2D only. Use it as a reference for the soft-constraint solver and the break-event API design. |
| **Rapier** (https://github.com/dimforge/rapier) | Apache-2.0 | 2026-10-10 | `ImpulseJoint.impulses` is public, so you can implement break-on-threshold yourself. | I found no built-in break threshold. Rust, so it is not directly linkable to our C++. |
| **PhysX 5** (https://github.com/NVIDIA-Omniverse/PhysX) | BSD-3-Clause | 2026-10-15 | `PxJoint::setBreakForce(force, torque)`. | Deformable volumes and FEM are GPU/CUDA only, so there is no WASM path. A WASM build exists (https://github.com/fabmax/physx-js-webidl, MIT) but it is heavy for our use. |

Verdict for section 4: there is nothing to drop in. Copy the *pattern*: per-joint impulse accumulator, threshold, disable, then spawn a free body. Bullet's `btFractureBody` (zlib) is the one small, liftable piece of fracture code.

---

## Ranked shortlist

1. **Ten Minute Physics, Müller** (MIT per file). **Borrow.**
   - Gives the XPBD soft body (`10-softBodies.html`) and, more importantly, embedded-mesh skinning (`12-softBodySkinning.html`). That lets a coarse physics cage drive the detailed three.js car mesh.
   - This is the key visual piece for both paths (a) and (c).
2. **PositionBasedDynamics, Bender** (MIT, C++). **Borrow** the constraint kernels: distance, volume, FEM and strain tet, shape matching, bending, rods.
   - Add plasticity yourself by updating the rest length or rest shape.
3. **CrashVector** (MPL-2.0, GDScript). **Adapt the algorithm.**
   - Best architectural match: the rigid body keeps world motion, a local structural graph holds plastic deformation, and the body shell is rebuilt from the graph.
   - Also has a progressive yield curve, plastic flow with an energy ledger, and calibration tooling.
4. **kami-vehicle** (Apache-2.0, Clojure). **Adapt the algorithm.**
   - Compact whole-vehicle node/beam blueprint: plastic and break beams, shape-matching chassis, tyre rings, parametric sedan cage of about 86 nodes and 220 beams. Young and unproven.
5. **Bullet `btFractureBody` and the breakable-constraint pattern** (zlib), plus three.js `ConvexObjectBreaker` (MIT).
   - **Borrow** for detachable parts and debris and glass.
   - Rigs of Rods `Actor::CalcBeams()` (GPL-3) remains the best *reading* reference for tuned beam yield and break behaviour. Do not copy it.

## Recommended architecture path

Build this in three stages. Each stage reuses the one before it.

**(a) Visual crumple driven by an impact model.** About 1–2 weeks.
- Keep the C++ rigid body as is.
- On each contact impulse above a threshold, the engine emits `(local point, normal, impulse, zone)`.
- A per-zone force-crush curve (Kamal or CRASH3-style, with hysteresis) turns the absorbed energy into crush depth.
- In JS, displace the vertices of a *coarse cage* of about 100–300 points within a radius. Clamp each against a max depth and against "hard points": cabin and wheel wells.
- Skin the detailed three.js mesh to that cage with Müller's barycentric embedding, then recompute normals.
- Use three.js ConvexObjectBreaker for glass and lights.
- Cost: low, and no stability risk. It does look procedural: there is no buckling or folding.

**(b) Mechanical damage coupling.** About 1 week, on top of (a).
- The cage offsets, or zone crush depths, feed our existing suspension linkage solver as hardpoint offsets. This gives toe and camber change, a pulling car and a bent wheel.
- Add these effects:
  - steering offset;
  - per-joint impulse thresholds that detach wheels, doors and bumpers into free rigid bodies (the Jolt, Bullet or PhysX breakable pattern);
  - "radiator zone crushed → coolant leak → engine temperature";
  - drag increase.
- No library does this for us. CrashVector's component and role bookkeeping is a good template.
- This is where most of the gameplay value of "destruction" comes from.

**(c) A real node/beam soft-body chassis.** About 1–3 months.
- Model the body as about 100–400 nodes. Use XPBD distance constraints with plastic rest-length drift and break strain: CrashVector's `structural_beam.gd` curve or kami's `update-plastic`, built on Bender's or Müller's kernels.
- Add volume and bending constraints for panels.
- Run it as a *local deformation layer* coupled to the existing rigid body, as CrashVector does. This avoids replacing our tyre and suspension model; kami-vehicle's shape-matching chassis is an alternative. Only later, if ever, make suspension mounts real nodes the way BeamNG and RoR do.
- It needs node-vs-world and node-vs-triangle collision (spatial hash, Müller `11-hashing`) and substepping at 1–2 kHz. This is affordable in WASM for 1–4 cars of about 300 nodes, with SIMD and no allocations.
- Content authoring of the cage per car is the long pole. kami's parametric sedan builder shows how to generate it procedurally instead of hand-authoring JBeam-like data.

**Recommendation:** ship (a) and (b) first. They deliver about 80% of the visible and gameplay effect for about 20% of the cost. Design (a)'s coarse cage and skinning so that (c) can later replace the procedural crumple with simulated XPBD nodes on the same cage, with no change to the rendering side.

## Sources
- https://en.wikipedia.org/wiki/Rigs_of_Rods
- https://documentation.beamng.com/beamng_tech/architecture/
- https://forum.godotengine.org/t/how-would-i-make-a-car-in-godot-deform-similar-to-a-real-car/58203
- https://forum.godotengine.org/t/car-crash-realistic-softbody-development-in-godot/117924
- https://discussions.unity.com/t/now-with-source-mesh-deformation-on-impact/414788
- https://assetstore.unity.com/packages/tools/physics/impact-deformable-4799
- https://www.mdpi.com/2411-9660/2/4/43
- https://journal.unimma.ac.id/AutomotiveExperiences/article/view/9289
- The repos listed above. Licence files were read directly.
