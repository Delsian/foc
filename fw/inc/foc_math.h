/*
 * Copyright (c) 2025 FOC Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef FOC_MATH_H
#define FOC_MATH_H

#include <zephyr/kernel.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_2PI
#define M_2PI (2.0f * M_PI)
#endif

#ifndef M_SQRT3
#define M_SQRT3 1.732050807568877f
#endif

#ifndef M_SQRT3_2
#define M_SQRT3_2 0.866025403784439f
#endif

/**
 * @brief DQ voltage/current structure (rotating reference frame)
 */
struct dq_frame {
	float d;  /* Direct axis component */
	float q;  /* Quadrature axis component */
};

/**
 * @brief Alpha-Beta voltage/current structure (stationary reference frame)
 */
struct ab_frame {
	float alpha;  /* Alpha axis component */
	float beta;   /* Beta axis component */
};

/**
 * @brief Three-phase voltage/current structure
 */
struct abc_frame {
	float a;  /* Phase A */
	float b;  /* Phase B */
	float c;  /* Phase C */
};

/**
 * @brief Normalize angle to [0, 2π] range
 *
 * @param angle Angle in radians
 * @return Normalized angle in radians
 */
float foc_normalize_angle(float angle);

/**
 * @brief Calculate electrical angle from mechanical angle
 *
 * @param mechanical_angle Mechanical shaft angle in radians
 * @param pole_pairs Number of motor pole pairs
 * @return Electrical angle in radians
 */
float foc_electrical_angle(float mechanical_angle, int pole_pairs);

/**
 * @brief Fast sine approximation
 *
 * @param angle Angle in radians
 * @return Sine of angle
 */
float foc_sin(float angle);

/**
 * @brief Fast cosine approximation
 *
 * @param angle Angle in radians
 * @return Cosine of angle
 */
float foc_cos(float angle);

/**
 * @brief Fast sine and cosine calculation
 *
 * @param angle Angle in radians
 * @param sin_out Pointer to store sine value
 * @param cos_out Pointer to store cosine value
 */
void foc_sincos(float angle, float *sin_out, float *cos_out);

/**
 * @brief Clarke Transform: Convert three-phase to alpha-beta frame
 *
 * Transforms three-phase quantities (a, b, c) to two-phase stationary
 * reference frame (alpha, beta).
 *
 * @param abc Three-phase input
 * @param ab Alpha-beta output
 */
void foc_clarke_transform(const struct abc_frame *abc, struct ab_frame *ab);

/**
 * @brief Inverse Clarke Transform: Convert alpha-beta to three-phase frame
 *
 * @param ab Alpha-beta input
 * @param abc Three-phase output
 */
void foc_inverse_clarke_transform(const struct ab_frame *ab, struct abc_frame *abc);

/**
 * @brief Park Transform: Convert alpha-beta to dq rotating frame
 *
 * Transforms stationary frame (alpha, beta) to rotating reference frame
 * (d, q) aligned with rotor flux.
 *
 * @param ab Alpha-beta input
 * @param angle_el Electrical angle in radians
 * @param dq DQ output
 */
void foc_park_transform(const struct ab_frame *ab, float angle_el, struct dq_frame *dq);

/**
 * @brief Inverse Park Transform: Convert dq to alpha-beta frame
 *
 * @param dq DQ input
 * @param angle_el Electrical angle in radians
 * @param ab Alpha-beta output
 */
void foc_inverse_park_transform(const struct dq_frame *dq, float angle_el, struct ab_frame *ab);

/**
 * @brief Space Vector Modulation: Set three-phase voltages
 *
 * This is the core FOC algorithm that converts Uq, Ud voltages in the
 * rotating frame to three-phase PWM voltages.
 *
 * @param uq Quadrature voltage (torque producing)
 * @param ud Direct voltage (flux producing)
 * @param angle_el Electrical angle in radians
 * @param voltage_limit Maximum voltage limit
 * @param abc Output three-phase voltages
 */
void foc_svm(float uq, float ud, float angle_el, float voltage_limit,
             struct abc_frame *abc);

/**
 * @brief Constrain value to range
 *
 * @param value Input value
 * @param min Minimum value
 * @param max Maximum value
 * @return Constrained value
 */
static inline float foc_constrain(float value, float min, float max)
{
	if (value < min) return min;
	if (value > max) return max;
	return value;
}

/**
 * @brief Calculate magnitude of vector
 *
 * @param x X component
 * @param y Y component
 * @return Magnitude sqrt(x^2 + y^2)
 */
static inline float foc_magnitude(float x, float y)
{
	return sqrtf(x * x + y * y);
}

#endif /* FOC_MATH_H */
