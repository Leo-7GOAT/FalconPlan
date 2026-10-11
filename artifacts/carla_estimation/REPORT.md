# CARLA estimated-feedback acceptance

The final strictly sequential matrix covers **12/12 runs**, six profiles and two seeds, each with 400 closed-loop ticks at 50 ms plus the terminal physics sample. **10/12 pass all feedback assertions; 0/12 pass the complete feedback plus physical envelope. The software stage remains PARTIAL.** Engine/client are genuine CARLA 0.9.15; this is not HIL or physical vehicle evidence.

The estimator initializes position from real GNSS callbacks and heading from real IMU compass while an explicit parking-brake hold guarantees the stationary speed prior. Its initial velocity variance is 0.0001 (m/s)^2. It never seeds pose from actor truth. Predictions consume each accepted IMU measurement once and use sensor timestamps, not controller call count. Duplicate/out-of-order, future, nonfinite and impossible startup derivative packets are rejected. Forward speed itself is not clipped after GNSS corrections to hide unsupported reverse; Safety's existing reverse rejection remains unchanged.

Stanley, PID, Safety, and the low-level throttle acceleration correction use the estimated state or IMU acceleration. A sealed vehicle proxy raises immediately if the control path requests actor transform, velocity or acceleration; all 12 runs recorded zero forbidden accesses. Before sealing, the adapter derives static axle geometry from CARLA wheel metadata and its coordinate frame; that configuration is not an initial localization estimate. OpenDRIVE route/georeference are static configuration. Ego actor pose/velocity are read only after issuing the command for offline evaluation and physical threshold measurements.

Noise uses real CARLA blueprint attributes: GNSS latitude/longitude standard deviation 1e-6 degrees, forward acceleration standard deviation 0.05 m/s2 and yaw-rate standard deviation 0.005 rad/s, seeds 7/19. CARLA compass remains ideal; this does not certify magnetometer noise, bias identification, grade/gravity compensation or hardware calibration. The [official CARLA 0.9.15 sensor reference](https://carla.readthedocs.io/en/0.9.15/ref_sensors/) defines callback timestamps, noise attributes and compass north (world negative Y).

Short dropout discards real callbacks over two 100 ms windows; each run dropped eight packets and remained active. Disorder runs rejected 40 delayed/duplicate packets and 26 future-stamped injections per run, while continuing to consume valid callbacks. One-second simultaneous GNSS/IMU dropout discarded 40 packets per run, triggered the unchanged 200 ms freshness watchdog and latched braking; both ended at physical speed 0 m/s. However whole-window speed RMSE was 1.422/1.440 m/s, and one position RMSE was 0.811 m, so these two runs **fail estimation accuracy**. Stale intervals remain included in all reported accuracy distributions.

The physical gates remain acceleration [-6, +3] m/s2 and absolute jerk <=6 m/s3. Every closed-loop physical velocity difference and the final tick is measured without clipping, filtering, emergency exclusions or downsampling. All runs fail at least one physical gate. Peak measured positive acceleration is 4.335 m/s2, minimum acceleration -26.877 m/s2 and absolute jerk 514.206 m/s3. Software command limits and successful emergency stopping do not certify the physical envelopes.

| Profile | Seed | Feedback | Position RMSE m | Speed RMSE m/s | Tracking RMSE m | Physical a min/max m/s2 | Max absolute jerk m/s3 |
|---|---:|---|---:|---:|---:|---:|---:|
| ideal_straight | 7 | PASS_REAL_SIM | 0.001028 | 0.018424 | 0.000277 | -1.973 / 3.581 | 89.621 |
| ideal_straight | 19 | PASS_REAL_SIM | 0.001028 | 0.018424 | 0.000277 | -1.973 / 3.581 | 89.621 |
| ideal_curve | 7 | PASS_REAL_SIM | 0.001631 | 0.016052 | 0.031065 | -1.951 / 3.538 | 80.745 |
| ideal_curve | 19 | PASS_REAL_SIM | 0.001631 | 0.016052 | 0.031065 | -1.951 / 3.538 | 80.745 |
| noisy_curve | 7 | PASS_REAL_SIM | 0.070008 | 0.049182 | 0.044403 | -1.950 / 3.641 | 80.753 |
| noisy_curve | 19 | PASS_REAL_SIM | 0.067889 | 0.048012 | 0.039793 | -1.951 / 3.521 | 80.821 |
| short_dropout | 7 | PASS_REAL_SIM | 0.081435 | 0.043035 | 0.019388 | -1.950 / 3.529 | 100.190 |
| short_dropout | 19 | PASS_REAL_SIM | 0.082801 | 0.056109 | 0.018259 | -2.413 / 3.678 | 96.061 |
| timestamp_disorder | 7 | PASS_REAL_SIM | 0.069032 | 0.042497 | 0.018805 | -1.950 / 3.529 | 80.712 |
| timestamp_disorder | 19 | PASS_REAL_SIM | 0.068131 | 0.048282 | 0.017721 | -1.975 / 4.335 | 116.865 |
| long_dropout | 7 | FAIL | 0.810586 | 1.422440 | 0.019662 | -26.352 / 3.529 | 425.042 |
| long_dropout | 19 | FAIL | 0.442525 | 1.440009 | 0.016148 | -26.877 / 3.678 | 514.206 |

## Reproduction and evidence

From the repository in WSL, with the matching running CARLA server and exclusive tick ownership:

```bash
python3 -m demos.carla_estimation_regression --steps 400 --seeds 7,19 --output artifacts/carla_estimation
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q tests/test_carla_sensor_contract.py tests/test_carla_contract.py tests/test_state_estimator.py
python3 -m demos.carla_regression --steps 80 --scenarios straight --seeds 7 --output artifacts/carla_estimation_legacy_final_smoke
```

The matrix returns exit code 1 intentionally because full acceptance is incomplete. `regression.json` and each `summary.json` retain the denominator and assertion booleans; `state.jsonl`, `sensor_callbacks.jsonl` and `startup.jsonl` preserve measured control, raw callbacks and initialization. `source_manifest.json` records frozen source hashes and commands. Eleven focused tests pass, including GT read traps, sensor-only initialization, once-per-measurement integration, timestamp rejection, dropout freshness and noisy stationary initialization. Test doubles are unit coverage, not simulator evidence.

The final legacy smoke retains its existing explicitly labelled actor-GT control and original benchmark gates. Motion and sensor diagnostics pass; overall status remains PARTIAL because physical jerk exceeds the original limit. No old benchmark artifacts were overwritten.

`cleanup.json` confirms no new actors remain and original asynchronous settings were restored. After the separately completed legacy smoke, `cleanup_after_legacy.json` independently confirms no vehicle/sensor actors and asynchronous mode with no fixed delta. No demo client remains active; server cleanup belongs to root.

Earlier outputs are retained transparently: `carla_estimation_before_stationary_prior` is a valid failed implementation (noisy initial velocity uncertainty caused two false reverse stops), corrected by a known-stationary covariance prior. `carla_estimation_invalid_tick_overlap` and `carla_estimation_legacy_smoke` are INVALID histories from an operator error with briefly concurrent tick clients and contribute no acceptance successes. The final matrix and final legacy smoke were executed strictly sequentially.
