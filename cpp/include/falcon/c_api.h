#pragma once
#include <stddef.h>
#ifdef __cplusplus
#define FALCON_NOEXCEPT noexcept
extern "C" {
#else
#define FALCON_NOEXCEPT
#endif
/* Return 0 on success, -1 for required null pointer, -2 for invalid data.
 * Caller owns contiguous arrays and guarantees their documented lengths.
 * No exception crosses the boundary. Outputs are unchanged on invalid data. */
int falcon_stanley(double heading, double cte, double speed, double gain,
                  double softening, double limit, double* output) FALCON_NOEXCEPT;
int falcon_bicycle(const double* state4, const double* command2,
                  double dt, double* output4) FALCON_NOEXCEPT;
/* samples4: [x_m, y_m, lateral_jerk_m_s3, longitudinal_jerk_m_s3].
 * offsets: candidate_count+1 indices, first 0, last sample_count.
 * metadata3: [target_d_m, target_speed_m_s, duration_s] per candidate.
 * obstacles3: [x_m, y_m, radius_m]; may be null only for zero obstacles.
 * weights5: lateral offset, speed error, lateral jerk, longitudinal jerk, duration.
 * costs6: unweighted five components followed by weighted total, per candidate.
 * collisions: 0 free, 1 blocked. best_index: first minimum free candidate,
 * or SIZE_MAX if all blocked (valid evaluation, no solution).
 * Counts capped at 1M samples / 10k candidates / 10k obstacles. */
int falcon_rank_candidates(const double* samples4, size_t sample_count,
    const size_t* offsets, size_t candidate_count, const double* metadata3,
    const double* obstacles3, size_t obstacle_count, const double* weights5,
    double desired_d, double desired_speed, double vehicle_radius, double margin,
    double* costs6, int* collisions, size_t* best_index) FALCON_NOEXCEPT;
#ifdef __cplusplus
}
#endif
#undef FALCON_NOEXCEPT
