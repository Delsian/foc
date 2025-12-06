/*
 * Copyright (c) 2025 FOC Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "foc_math.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(foc_math, LOG_LEVEL_DBG);

/* Sine lookup table for fast approximation (65 elements for 0 to π/2) */
static const float sine_table[65] = {
	0.00000000f, 0.02454123f, 0.04906767f, 0.07356456f, 0.09801714f, 0.12241068f,
	0.14673047f, 0.17096189f, 0.19509032f, 0.21910124f, 0.24298018f, 0.26671276f,
	0.29028468f, 0.31368174f, 0.33688985f, 0.35989504f, 0.38268343f, 0.40524131f,
	0.42755509f, 0.44961133f, 0.47139674f, 0.49289819f, 0.51410274f, 0.53499762f,
	0.55557023f, 0.57580819f, 0.59569930f, 0.61523159f, 0.63439328f, 0.65317284f,
	0.67155895f, 0.68954054f, 0.70710678f, 0.72424708f, 0.74095113f, 0.75720885f,
	0.77301045f, 0.78834643f, 0.80320753f, 0.81758481f, 0.83146961f, 0.84485357f,
	0.85772861f, 0.87008699f, 0.88192126f, 0.89322430f, 0.90398929f, 0.91420976f,
	0.92387953f, 0.93299280f, 0.94154407f, 0.94952818f, 0.95694034f, 0.96377607f,
	0.97003125f, 0.97570213f, 0.98078528f, 0.98527764f, 0.98917651f, 0.99247953f,
	0.99518473f, 0.99729046f, 0.99879546f, 0.99969882f, 1.00000000f
};

float foc_normalize_angle(float angle)
{
	angle = fmodf(angle, M_2PI);
	if (angle < 0) {
		angle += M_2PI;
	}
	return angle;
}

float foc_electrical_angle(float mechanical_angle, int pole_pairs)
{
	return foc_normalize_angle((float)pole_pairs * mechanical_angle);
}

float foc_sin(float angle)
{
	/* Normalize angle to [0, 2π] */
	angle = foc_normalize_angle(angle);

	/* Determine quadrant and adjust angle to [0, π/2] */
	int quadrant = (int)(angle / (M_PI / 2.0f));
	float angle_mod = fmodf(angle, M_PI / 2.0f);

	/* Convert to table index (0-64) */
	float index_f = angle_mod * (64.0f / (M_PI / 2.0f));
	int index = (int)index_f;
	float frac = index_f - (float)index;

	/* Linear interpolation between two table values */
	float val1 = sine_table[index];
	float val2 = sine_table[index + 1];
	float result = val1 + (val2 - val1) * frac;

	/* Adjust sign based on quadrant */
	switch (quadrant) {
	case 0: /* [0, π/2] */
		return result;
	case 1: /* [π/2, π] */
		return result;
	case 2: /* [π, 3π/2] */
		return -result;
	case 3: /* [3π/2, 2π] */
		return -result;
	default:
		return 0.0f;
	}
}

float foc_cos(float angle)
{
	/* cos(x) = sin(x + π/2) */
	return foc_sin(angle + M_PI / 2.0f);
}

void foc_sincos(float angle, float *sin_out, float *cos_out)
{
	if (!sin_out || !cos_out) {
		return;
	}

	/* Use math.h functions for accuracy - can be optimized later if needed */
	*sin_out = sinf(angle);
	*cos_out = cosf(angle);
}

void foc_clarke_transform(const struct abc_frame *abc, struct ab_frame *ab)
{
	if (!abc || !ab) {
		return;
	}

	/* Clarke transform (power-invariant form):
	 * alpha = a
	 * beta = (a + 2*b) / sqrt(3)
	 *
	 * For balanced three-phase: a + b + c = 0, so:
	 * beta = (2*b + a) / sqrt(3) = -(c - b) * sqrt(3) / 3 * 2
	 */
	ab->alpha = abc->a;
	ab->beta = (abc->a + 2.0f * abc->b) / M_SQRT3;
}

void foc_inverse_clarke_transform(const struct ab_frame *ab, struct abc_frame *abc)
{
	if (!ab || !abc) {
		return;
	}

	/* Inverse Clarke transform:
	 * a = alpha
	 * b = -0.5 * alpha + sqrt(3)/2 * beta
	 * c = -0.5 * alpha - sqrt(3)/2 * beta
	 */
	abc->a = ab->alpha;
	abc->b = -0.5f * ab->alpha + M_SQRT3_2 * ab->beta;
	abc->c = -0.5f * ab->alpha - M_SQRT3_2 * ab->beta;
}

void foc_park_transform(const struct ab_frame *ab, float angle_el, struct dq_frame *dq)
{
	float sin_a, cos_a;

	if (!ab || !dq) {
		return;
	}

	foc_sincos(angle_el, &sin_a, &cos_a);

	/* Park transform:
	 * d =  alpha * cos(θ) + beta * sin(θ)
	 * q = -alpha * sin(θ) + beta * cos(θ)
	 */
	dq->d = ab->alpha * cos_a + ab->beta * sin_a;
	dq->q = -ab->alpha * sin_a + ab->beta * cos_a;
}

void foc_inverse_park_transform(const struct dq_frame *dq, float angle_el, struct ab_frame *ab)
{
	float sin_a, cos_a;

	if (!dq || !ab) {
		return;
	}

	foc_sincos(angle_el, &sin_a, &cos_a);

	/* Inverse Park transform:
	 * alpha = d * cos(θ) - q * sin(θ)
	 * beta  = d * sin(θ) + q * cos(θ)
	 */
	ab->alpha = dq->d * cos_a - dq->q * sin_a;
	ab->beta = dq->d * sin_a + dq->q * cos_a;
}

void foc_svm(float uq, float ud, float angle_el, float voltage_limit,
             struct abc_frame *abc)
{
	struct dq_frame dq;
	struct ab_frame ab;
	float magnitude;

	if (!abc) {
		return;
	}

	/* Limit voltage magnitude */
	magnitude = foc_magnitude(ud, uq);
	if (magnitude > voltage_limit) {
		float scale = voltage_limit / magnitude;
		uq *= scale;
		ud *= scale;
	}

	/* Apply inverse Park transform */
	dq.d = ud;
	dq.q = uq;
	foc_inverse_park_transform(&dq, angle_el, &ab);

	/* Apply inverse Clarke transform to get three-phase voltages */
	foc_inverse_clarke_transform(&ab, abc);

	/* Apply voltage centering for Space Vector Modulation
	 * This improves efficiency by centering the voltages
	 */
	float mid = (abc->a + abc->b + abc->c) / 3.0f;
	abc->a -= mid;
	abc->b -= mid;
	abc->c -= mid;
}
