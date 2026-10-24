# Gyro Production Review and Change Map

**Review date:** 2026-10-24  
**Baseline:** Cemu v2.6, commit a6fb0a4  
**Scope:** The gyro feature patch, its settings/UI path, and adjacent motion consumers. The modified dependencies/vcpkg submodule is pre-existing and outside this review. Source changes described below were applied on 2026-10-24; no build or tests were run.

## Production-readiness decision

The patch is **not yet verified as final production code**. This implementation pass addresses calibration rollback, settings coverage/bounds, Eden-compatible DSU rate scaling, duplicate preview redraws, unrelated edits, and opt-in test integration. A build, test run, and controller/runtime comparison are still required before calling it production-ready.

The strongest code-level explanation for an apparent “spring back” is bias learning while rotation is occurring. The gyro output subtracts the learned bias from every later sample. If the estimator learned part of an upward rotation as bias, the residual after the user stops moving can have the opposite sign. This is a plausible mechanism, not a confirmed diagnosis of the runtime symptom.

## Implementation status

Applied in the working tree:

- Calibration uses a candidate bias during an explicit run and commits it only after all 300 samples complete. Cancel and invalid-data failure discard the candidate. There is deliberately no movement/stillness rejection: a successful calibration that samples rotation can still absorb some rotation into bias.
- DSU gyro conversion now uses Eden's local `1/312` scale and the shared conversion helper tests its pitch/roll/yaw signs and acceleration mapping. This is source-level parity; no live comparison with the user's DSU sender was run.
- Sensitivity and threshold are finite/range-checked in `ControllerBase::set_settings()`. WPAD now passes its controller settings to `getVPADGyroChange()`, and calibration controls are disabled for providers without calibration support.
- Loaded DSU bias is applied only when all three values are finite.
- The gyro preview skips a redraw when Euler values are unchanged. The dialog still polls at 33 ms; no audio-performance conclusion is drawn from this change.
- The unrelated DSU message validation, joystick redraw optimization, and whitespace-only settings edits have been removed from the feature diff.
- `CEMU_BUILD_GYRO_TESTS` adds an opt-in CTest executable with active checks for rollback, moving calibration acceptance, rest-threshold behavior, threshold/sensitivity output, and DSU mapping.

Not verified yet:

- No build or test command was run, per the current instruction. The CMake target and source edits therefore remain uncompiled and unexecuted.
- Runtime tests still need DSU source/unit confirmation, axis direction checks, slow intentional movement, calibration during motion/cancel, and threshold/sensitivity checks in Cemu and Eden.
- The changed `MahonySensorFusion` path is shared with SDL motion. The new tests exercise the common estimator but do not cover SDL provider behavior or provider persistence/shared-index behavior.
- The 33 ms preview polling rate and the audio crackling/static symptom have not been measured together in this pass.
## Current end-to-end path

1. DSUControllerProvider::reader_thread() receives a DSU data packet and calls integrate_motion() while holding the pad's m_mutex[index].
2. integrate_motion() derives deltaTime, converts DSU gyro values with Eden's local 1/312 scale into radians/second, maps pitch/roll/yaw and acceleration axes, and passes the sample and threshold to WiiUMotionHandler.
3. WiiUMotionHandler stores the current gyro sample, runs Mahony fusion, and returns a MotionSample containing bias-corrected gyro rates plus fused orientation/quaternion.
4. EmulatedController::get_motion_data(sample, sensitivity, threshold) picks the first controller with motion enabled and returns its sample/settings.
5. VPADController::update_motion() writes the scaled and thresholded rate into VPADStatus::gyroChange, and writes fused orientation/attitude fields separately.
6. The controller settings dialog persists sensitivity/threshold values through InputManager; calibration bias is persisted in the DSU controller XML node.

The distinction in step 5 matters: **gyro rate output and fused orientation are separate outputs**. Accelerometer feedback in Mahony can level the 3D preview and affects fused orientation fields. It does not directly rewrite the stored raw gyro rate used by getVPADGyroChange(). In the inspected DSU path, fusion/sample/calibration access uses the per-pad mutex, and the threshold is atomic; no obvious unguarded access was found in that path.

## Findings

### P1 — Calibration can learn movement as bias; cancel keeps a partial estimate

GyroCalibration::update() has two estimators:

- Passive rest tracking updates bias with an EMA coefficient of 0.0001 when residual gyro magnitude is below 0.05 turns/second and acceleration magnitude is between 0.90g and 1.10g.
- Explicit calibration updates bias with an EMA coefficient of 0.01 on every sample while calibrating, without checking whether the controller is moving. It finishes after 300 samples.

The explicit-calibration rule follows Eden's broad behavior of not rejecting movement. However, it means a moving controller's angular rate can be learned as bias. Subsequent output subtracts that bias and can briefly point opposite the movement. The previously supplied log showed roughly 1,100 motion samples/second; if that is the effective integration rate during calibration, 300 samples is about 0.27 seconds. Confirm that rate on-device before relying on the estimate.

cancel() and fail() clear progress/state but leave the partially updated bias in place. Closing the dialog calls cancel_gyro_calibration(), so canceling an in-progress calibration does not undo bias changes already made.

References: src/input/motion/GyroCalibration.h:19-31, 60-90; src/input/motion/Mahony.h:73-78; src/gui/input/settings/DefaultControllerSettings.cpp:286; src/input/api/DSU/DSUControllerProvider.cpp:525-532.

**Required decision/change:** Keep the user's requirement of no “remain still” rejection. Define calibration result semantics to match Eden, but make cancel/failure lifecycle explicit and ensure a canceled attempt cannot silently leave a partial estimate. Do not claim calibration is motion-proof: without motion rejection or an independent reference, a moving sample is mathematically indistinguishable from sensor bias. Verify completion duration against actual DSU timestamp cadence.

### P1 — Passive rest tracking can absorb slow intentional rotation

The passive tracker accepts residual motion up to 0.05 turns/second, approximately 18°/second, when acceleration magnitude is near 1g. The supplied stationary log had gyro magnitude around 15°/second, which is inside this rest threshold. Slow intentional rotation can also fall inside it and be incorporated into the estimated bias. When the rotation stops, subtracting the newly learned bias can look like a return movement.

Eden uses the same nominal IsAtRestRelaxed = 0.05 turns/second and 0.0001 EMA update, so this is a meaningful Eden-compatible behavior, not proof that Cemu's value is wrong. The new Cemu implementation still needs testing at realistic slow rates; the current tests only check stationary drift and much faster gameplay motion.

References: src/input/motion/GyroCalibration.h:69-75, 95.

**Required decision/change:** Preserve Eden's thresholds if Eden parity is the target. Add coverage around rates below, at, and above 18°/second and document the resulting behavior. Do not silently lower the threshold or add a stillness rejection as a “fix”; either would change the requested behavior and diverge from Eden.

### P1 — Sensitivity is exposed more broadly than it is applied

The settings dialog displays a sensitivity slider for every controller where has_motion() is true. The new settings-aware motion getter and VPADController::update_motion() apply it to the VPAD gyro-rate path. WPADController still calls the old no-argument get_motion_data() and then getVPADGyroChange() with default sensitivity, so its slider value is not used. Calibration methods on the base ControllerBase are no-ops; only DSU overrides them, but the dialog currently enables the calibration button for any motion-capable controller.

References: src/gui/input/settings/DefaultControllerSettings.cpp:192-216; src/input/emulated/EmulatedController.cpp:126-144; src/input/emulated/VPADController.cpp:242-254; src/input/emulated/WPADController.cpp:261-285; src/input/api/Controller.h:130-134.

**Required change:** Either restrict these controls to DSU-backed motion (and disable calibration for APIs without an implementation), or thread the settings through every consumer the UI claims to support. Make the supported API set explicit and test each supported route.

### P1 — Sensitivity and practical threshold use a different DSU rate scale than Eden

Cemu currently assumes DSU gyro values are degrees/second and converts them to radians/second (0.0174533), then radians/second → turns/second in MotionSample. That makes 100% output effectively raw value / 360. Eden's local Cemuhook UDP parser instead scales BetterJoy gyro values by 1/312 into its motion units. With the same BetterJoy DSU payload, Cemu's 100% baseline is about 86.7% of Eden's (roughly 13.3% lower). The threshold cutoff is affected by the same scale mismatch: Cemu's default .007 threshold corresponds to 2.52 numeric DSU units under its degree/second assumption; Eden's 1/312 scale corresponds to 2.184 units for the same threshold. The axis/sign mapping matches Eden; the scale does not.

References: src/input/api/DSU/DSUControllerProvider.cpp:525-531; src/input/motion/MotionSample.h:144-157; Eden src/input_common/drivers/udp_client.cpp:250-261.

**Required decision/change:** If 100% and the threshold are intended to match Eden, establish and document the DSU source unit, apply the chosen baseline scale once before fusion and output, then apply the user multiplier. Verify with the same DSU sender and known rotations before changing the factor. Add tests that demonstrate 100% parity and linear behavior at 500%; do not treat the current presence of a slider as proof that its base calibration matches Eden.

### P1 — Added calibration tests are not wired into the project test runner

The patch adds tests/test_gyro_calibration.cpp, but no CMake/CTest target or add_test() registration is included. The test uses assert(), which is commonly disabled in Release configurations by NDEBUG; a Release run could print “passed” while checking none of the assertions. It also does not test threshold conversion, sensitivity scaling, moving calibration, cancel/failure rollback, or DSU axis mapping. Its “active gameplay” sample uses 0.25 rad/s and 1.3g, which bypasses the passive rest classifier and misses slow-motion contamination.

References: tests/test_gyro_calibration.cpp; no test-target change exists in the current patch.

**Required change:** Integrate the tests with the repository's supported test mechanism, keep checks active in the configuration used to run them, and add tests for the listed contract boundaries. Avoid treating the current standalone test source as production verification.

### P2 — Preview polling/redraw work increased

The dialog timer changed from 100 ms (10 Hz) to 33 ms (about 30 Hz). Each timer tick supplies a new orientation to wxGyroDraw::SetOrientation(), which unconditionally queues Refresh(false), even when Euler values did not change. The joystick drawings now skip unchanged values, but the gyro preview does not.

This is a confirmed increase in preview polling/redraw frequency while the dialog is open. It is **not proof** of the audio crackling cause.

References: src/gui/input/settings/DefaultControllerSettings.cpp:273-274, 321-329; src/gui/components/wxGyroDraw.cpp:44-50.

**Required change:** Keep the preview responsive while avoiding unchanged redraws; choose a refresh cadence based on measured visual need and verify audio/input behavior with the dialog open and closed. Do not infer an audio root cause from this code change alone.

### P2 — Fusion and preview return-to-level behavior must be distinguished from gyro-rate behavior

Mahony fusion uses accelerometer gravity feedback to correct orientation. wxGyroDraw renders Euler angles from that fused orientation, so the box can move back toward a gravity-aligned pose. That feedback existed in the v2.6 baseline; the DSU acceleration coordinate transform was changed in this patch to match Eden's UDP mapping (x, -z, y).

The patch's DSU mapping is consistent with Eden's local udp_client.cpp: gyro pitch → X, roll → Y, negative yaw → Z; acceleration → X, negative Z, Y. This check supports the transform, but runtime behavior still needs validation with known single-axis rotations.

References: src/input/motion/Mahony.h:84-90; src/gui/components/wxGyroDraw.cpp:44-49; src/input/api/DSU/DSUControllerProvider.cpp:521-531; Eden src/input_common/drivers/udp_client.cpp:253-261.

**Required change:** Test the preview and actual game gyro independently. Record gyroChange and fused orientation separately during a controlled pitch/roll movement. Do not attribute a preview leveling effect to the VPAD rate path without that distinction.

### P2 — The patch contains unrelated protocol and UI edits

src/input/api/DSU/DSUMessages.cpp now adds message-type, protocol-version, and exact-size checks to ValidateCRC32(). This is unrelated to gyro calibration/settings and can change which DSU messages are accepted. src/gui/components/wxInputDraw.cpp changes shared joystick redraw behavior, and src/gui/input/InputSettings2.cpp has whitespace-only edits.

References: those files in the current staged diff.

**Required change:** Remove unrelated protocol validation and whitespace changes from the gyro production patch. Keep the shared joystick redraw optimization only if it is separately justified and reviewed.

### P2 — Fusion changes are shared with SDL motion

WiiUMotionHandler and MahonySensorFusion are also used by the SDL motion provider. Replacing the old bias estimator in Mahony.h with GyroCalibration therefore changes passive drift behavior for SDL as well, even though the new calibration/settings UI is primarily DSU-oriented.

Reference: src/input/api/SDL/SDLControllerProvider.cpp:232; src/input/motion/Mahony.h; src/input/motion/GyroCalibration.h.

**Required change:** Decide whether the fusion/bias changes are intended for every WiiUMotionHandler consumer. If DSU-only behavior is intended, isolate the provider-specific behavior; if shared behavior is intended, include SDL in the regression matrix.

### P3 — Bias persistence and shared DSU state need a documented contract

Threshold and calibration state live in DSUControllerProvider per DSU pad index. Multiple controller configurations that refer to the same provider/index share that state; each settings write can update the same threshold. This is the shared-state behavior already accepted for threshold settings. Bias is also stored per provider/index at runtime and serialized into the DSU controller XML node.

References: src/input/api/DSU/DSUControllerProvider.h:122-126; src/input/api/DSU/DSUController.cpp:31-41, 52-67, 90-99.

**Required change:** Preserve the accepted shared-threshold model, document it, and test load/save behavior when multiple configurations refer to one DSU pad. Validate loaded bias values for finiteness before applying them.

## Threshold and sensitivity contract

| Setting | UI range | Stored unit | Applied at | Intended effect |
|---|---:|---|---|---|
| Sensitivity | 100–500% | multiplier 1.0–5.0 | VPAD gyro-rate output | Scales emitted gyroChange; does not change bias estimation or fusion rate |
| Gyro threshold | 0–100% | 0–0.1 turns/second; default 0.007 | Mahony integration and VPAD gyro-rate output | Zeros gyro below a magnitude cutoff; 0% disables the cutoff |

The unit conversion is internally consistent in the current implementation: Cemu currently treats DSU gyro fields as degrees/second and converts them to radians/second; the threshold helper converts turns/second to radians/second for comparison; MotionSample converts radians/second to VPAD turns/second; sensitivity multiplies the output afterward.

Important implications:

- Threshold is a magnitude check across all three gyro axes, not a separate per-axis deadzone.
- The threshold is applied to fused orientation integration and separately to output rate. These are distinct data paths, not the same value being applied twice to one sample.
- Sensitivity is intended to change game rate output only. It does not alter threshold behavior, the bias estimator, or the 3D preview orientation.
- The threshold slider's 100% value is 0.1 turns/second (36°/second), while its default is 0.007 turns/second (about 2.52°/second). These endpoints and labels need explicit UI verification.
- DSUController::set_settings() clamps threshold but does not centrally clamp sensitivity. The GUI and XML load clamp sensitivity; API-level invariants should be enforced in one place.

References: src/input/motion/GyroThreshold.h; src/input/motion/MotionSample.h:144-162; src/input/api/DSU/DSUControllerProvider.cpp:521-532; src/gui/input/settings/DefaultControllerSettings.cpp:207-232, 392-411; src/input/InputManager.cpp:174-190, 519-530.

## Ordered change map

1. **Freeze the intended scope and behavior**
   - Keep DSU axis conversion matched to Eden.
   - Keep threshold sharing per DSU provider/index as previously accepted.
   - Keep the no-stillness-rejection requirement.
   - Decide whether the UI supports DSU only or all motion APIs, and whether the shared fusion/bias behavior is intended for SDL.

2. **Make calibration lifecycle safe and explicit**
   - Define start, success, cancel, and invalid-data behavior.
   - Ensure canceled/failed attempts do not leave an undocumented partial bias.
   - Keep successful calibration semantics aligned with Eden and explain the consequence of movement during the sampling window.
   - Confirm the actual sample cadence and calibration duration from DSU timestamps.

3. **Lock down threshold/sensitivity contracts**
   - Enforce finite/range checks at the settings API boundary, not only UI/XML boundaries.
   - Confirm all displayed controls affect every API where they are shown; disable unsupported calibration controls.
   - Keep the threshold unit and vector-magnitude semantics consistent through configuration, provider, fusion, and VPAD output.

4. **Remove unrelated changes and bound preview work**
   - Remove DSU message validation edits and whitespace-only changes from the gyro patch.
   - Avoid redraws when preview orientation is unchanged.
   - Select a settings-dialog polling cadence based on observed responsiveness and audio behavior.

5. **Integrate meaningful tests**
   - Add a real CMake/CTest target using checks active in the chosen build configuration.
   - Cover unit conversion, threshold boundaries (including 0%), sensitivity endpoints and linear scaling, DSU axis mapping, bias persistence, shared DSU index behavior, cancel/fail during calibration, slow movement near the rest threshold, and the distinction between rate and fused orientation.
   - Include SDL and WPAD in coverage if those routes remain supported.

6. **Validate at runtime before calling the patch final**
   - Compare Cemu and Eden using the same DSU source, controller, movement, and settings.
   - Verify fixed-axis pitch/roll/yaw directions and signs.
   - Verify slow intentional motion below and above the 18°/s rest-tracking boundary.
   - Verify calibration while stationary, while moving, and when canceled.
   - Verify threshold at 0%, default, and 100%, and sensitivity at 100% and 500%.
   - Verify preview behavior separately from in-game gyro aiming, and repeat audio checks with the settings dialog open and closed.

## Files in the intended gyro scope

- src/input/motion/GyroCalibration.h
- src/input/motion/GyroThreshold.h
- src/input/motion/Mahony.h
- src/input/motion/MotionHandler.h
- src/input/motion/MotionSample.h
- src/input/api/Controller.h
- src/input/api/DSU/DSUController.cpp and .h
- src/input/api/DSU/DSUControllerProvider.cpp and .h
- src/input/emulated/EmulatedController.cpp and .h
- src/input/emulated/VPADController.cpp
- src/input/InputManager.cpp
- src/gui/CMakeLists.txt
- src/gui/components/wxGyroDraw.cpp and .h
- src/gui/components/wxGyroPanel.cpp and .h
- src/gui/input/settings/DefaultControllerSettings.cpp and .h
- tests/test_gyro_calibration.cpp and tests/mock_includes/wx/math.h

src/input/api/DSU/DSUMessages.cpp/.h, whitespace-only InputSettings2.cpp edits, and the existing modified dependencies/vcpkg submodule are not part of the intended gyro feature change set.






