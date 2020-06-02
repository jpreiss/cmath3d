#include <assert.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>

#include "math3d.h"

//
// helper functions and special asserts
// (use regular assert() when no special assert applies)
//

float randu(float a, float b) {
	double s = rand() / ((double)RAND_MAX);
	double x = (double)a + ((double)b - (double)a) * s;
	return (float)x;
}

// APPROXIMATELY unit normal random number
float randn() {
	// central limit theorem at work
	return randu(-1.0f, 1.0f) + randu(-1.0f, 1.0f) + randu(-1.0f, 1.0f);
}

// returns a random vector uniformly sampled from the unit cube
struct vec randcube() {
	return mkvec(randu(-1.0f, 1.0f), randu(-1.0f, 1.0f), randu(-1.0f, 1.0f));
}

// returns a random vector sampled from a Gaussian with expected L2 norm of 1.
// Note that this is *not* an identity covariance matrix.
struct vec randnvec() {
	struct vec v = mkvec(randn(), randn(), randn());
	return vscl(1.0f / sqrtf(3.0f), v);
}

// returns a random vector approximately uniformly sampled from the unit sphere
struct vec randsphere() {
	struct vec v = mkvec(randn(), randn(), randn());
	return vnormalize(v);
}

// returns a random quaternion not necessarily uniformly sampled
struct quat randquat() {
	struct quat q = mkquat(
		randu(-1.0f, 1.0f), randu(-1.0f, 1.0f), randu(-1.0f, 1.0f), randu(-1.0f, 1.0f));
	return qnormalize(q);
}

void printvec(struct vec v) {
	printf("%f, %f, %f", (double)v.x, (double)v.y, (double)v.z);
}

#define ASSERT_VEQ_EPSILON(a, b, epsilon) \
do { \
	if (!veqepsilon(a, b, epsilon)) { \
		printf("\t" #a " = "); printvec(a); printf("\n"); \
		printf("\t" #b " = "); printvec(b); printf("\n"); \
		assert(veqepsilon(a, b, epsilon)); \
	} \
} while(0) \

// Generates a random polytope with n rows.
// Caller must guarantee that the arrays pointed to by p.A and p.b
// are large enough to hold n rows.
//
// The rows of A will be normalized to have L2 norm of 1.
// The polytope interior will be non-empty.
// The polytope may or may not contain the origin.
// The polytope may or may not be bounded, but will be with high probability
//   as n increases.
// The returned interior point will be, on average, around 1 unit away
//   from any polytope face.
//
// Returns a point in the polytope interior.
struct vec randpolytope(float *A, float *b, int n) {
	// If we generate random Ax <= b with b always positive, we ensure that
	// x = 0 is always a solution; hence the polytope is non-empty. However,
	// we also want to test nonempty polytopes that don't contain x = 0.
	// Therefore, we use A(x - shift) <= b with b positive, which is
	// equivalent to Ax <= b + Ashift.
	struct vec shift = vscl(randu(0.0f, 4.0f), randsphere());

	for (int i = 0; i < n; ++i) {
		struct vec a = randsphere();
		vstoref(a, A + 3 * i);
		b[i] = randu(0.01f, 2.0f) + vdot(a, shift);
	}

	return shift;
}


//
// tests - when adding new tests, make sure to add to test_fns array
//

void test_vec_basic()
{
	struct vec v = mkvec(1.0f, 2.0f, 3.0f);
	assert(vindex(v, 0) == 1.0f);
	assert(vindex(v, 1) == 2.0f);
	assert(vindex(v, 2) == 3.0f);

	printf("%s passed\n", __func__);
}

void test_mat_axisangle()
{
	srand(0); // deterministic
	int const N = 10000;

	for (int i = 0; i < N; ++i) {
		// random rotation axis and point to rotate
		struct vec const axis = randsphere();
		struct vec const point = randsphere();

		// rotation angle s.t. we will go a full circle
		int const divisions = randu(0.0, 100.0);
		float const theta = 2.0f * M_PI_F / divisions;

		// rotate the point in a full circle
		struct vec pointrot = point;
		struct mat33 const R = maxisangle(axis, theta);
		for (int j = 0; j < divisions; ++j) {
			pointrot = mvmul(R, pointrot);
		}

		// should be back where we started
		ASSERT_VEQ_EPSILON(point, pointrot, 0.00001f); // must be fairly loose due to 32 bit trig, etc.
	}
	printf("%s passed\n", __func__);
}

void test_quat_conversions()
{
	srand(0); // deterministic
	int const N = 10000;

	// rpy->quat->rpy
	for (int i = 0; i < N; ++i) {
		float yaw   = randu(-0.98f*M_PI_F, 0.98f*M_PI_F); // quat2rpy never outputs outside [-pi, pi]
		float pitch = randu(-0.48f*M_PI_F, 0.48f*M_PI_F); // avoid singularity
		float roll  = randu(-0.98f*M_PI_F, 0.98f*M_PI_F);
		struct vec rpy0 = mkvec(roll, pitch, yaw);
		struct vec rpy1 = quat2rpy(rpy2quat(rpy0));
		ASSERT_VEQ_EPSILON(rpy0, rpy1, 0.00001f); // must be fairly loose due to 32 bit trig, etc.
	}

	// quat->matrix->quat
	for (int i = 0; i < N; ++i) {
		struct quat const q = randquat();
		struct mat33 const m = quat2rotmat(q);
		struct quat const qq = mat2quat(m);
		float const angle = qanglebetween(q, qq);
		// TODO: seems like a lot of precision loss -- can we improve?
		assert(fabsf(angle) < radians(0.1f));
	}

	// quat->axis/angle->quat
	for (int i = 0; i < N; ++i) {
		struct quat const q = randquat();
		struct vec qaxis = quat2axis(q);
		float qangle = quat2angle(q);
		struct quat qq = qaxisangle(qaxis, qangle);
		float const angle = qanglebetween(q, qq);
		// TODO: seems like a lot of precision loss -- can we improve?
		assert(fabsf(angle) < radians(0.1f));
	}

	// axis/angle->quat->axis/angle
	for (int i = 0; i < N; ++i) {
		struct vec axis = randcube();
		float angle = randn();
		if (fabsf(angle) < 1e-3f) {
			// conversion is not stable for small angles.
			continue;
		}
		struct quat q = qaxisangle(axis, angle);
		struct vec qaxis = quat2axis(q);
		float qangle = quat2angle(q);
		float anorm = vmag(axis);
		float qanorm = vmag(qaxis);
		float dot = vdot(vdiv(axis, anorm), vdiv(qaxis, qanorm));
		assert(fabsf(dot) >= (1.0f - 1e-6f));
		if (dot < 0) {
			qangle *= -1.0f;
		}
		assert(fabsf(qangle - angle) < 1e-4f);
	}

	printf("%s passed\n", __func__);
}

void test_qvectovec()
{
	srand(0); // deterministic
	int const N = 10000;
	struct quat const qzero = mkquat(0.0f, 0.0f, 0.0f, 0.0f);

	for (int i = 0; i < N; ++i) {
		struct vec a = randcube(), b = randcube();
		// do not try to normalize tiny vectors.
		if (vmag2(a) < 1e-8f || vmag2(b) < 1e-8f) continue;
		a = vnormalize(a);
		b = vnormalize(b);
		// degenerate case - test explicitly, not accidentally.
		if (vdot(a, b) < -0.99f) continue;
		// should return zero vector for degenerate case.
		assert(qeq(qvectovec(a, vneg(a)), qzero));

		// non-degenerate case.
		struct quat const q = qvectovec(a, b);
		struct vec const qa = qvrot(q, a);
		ASSERT_VEQ_EPSILON(qa, b, 0.00001f); 

		struct vec cross = vcross(a, b);
		struct vec const qcross = qvrot(q, cross);
		ASSERT_VEQ_EPSILON(qcross, cross, 0.00001f); 
	}
	printf("%s passed\n", __func__);
}

void test_qslerp()
{
	srand(0); // deterministic

	int const N = 10000;

	for (int i = 0; i < N; ++i) {
		// two random quaternions
		struct quat a = randquat();
		struct quat b = randquat();

		// construct quaternion dq such that b = (dq)^steps * a
		int steps = 1 + rand() % 5;
		float t = 1.0 / steps;
		struct quat q = qslerp(a, b, t);
		struct quat dq = qqmul(q, qinv(a));

		// verify
		struct quat b2 = a;
		for (int s = 0; s < steps; ++s) {
			b2 = qqmul(dq, b2);
		}
		float angle = qanglebetween(b, b2);

		// TODO: seems like a lot of precision loss -- can we improve?
		assert(angle <= radians(0.1f));
	}
	printf("%s passed\n", __func__);
}

void test_quat_lowprecision()
{
	srand(20); // deterministic

	int const N = 10000;

	for (int i = 0; i < N; ++i) {
		struct vec rpy = vscl(1e-2f, randcube());
		struct quat exact = rpy2quat(rpy);
		struct quat approx = rpy2quat_small(rpy);
		assert(qanglebetween(exact, approx) < 1e-3f);
	}

	for (int i = 0; i < N; ++i) {
		struct vec gyro = randsphere();
		struct quat init = randquat();
		int steps = 100;
		float t = randu(0.1, 2.0);
		float dt = t / steps;
		struct quat q = init;
		for (int j = 0; j < steps; ++j) {
			q = quat_gyro_update(q, gyro, dt);
		}
		struct quat target = qqmul(qaxisangle(gyro, t), init);
		assert(qanglebetween(q, target) < 1e-6f);
	}

	printf("%s passed\n", __func__);
}

void test_polytope_projection()
{
	srand(1); // deterministic

	// For each polytope, generate a few points, project them,
	// and check that the projections are closer than other points.
	int const N_POLYTOPES = 100;
	int const N_PROJECTIONS = 10;
	int const N_OTHERS = 100;
	float const TOLERANCE = 1e-6;
	float const MAXITERS = 1000;

	int const MAX_FACES = 30;
	float *A = malloc(sizeof(float) * MAX_FACES * 3);
	float *b = malloc(sizeof(float) * MAX_FACES);
	float *work = malloc(sizeof(float) * MAX_FACES * 3);


	for (int trial = 0; trial < N_POLYTOPES; ++trial) {

		// Random number of polytope faces.
		int n = rand() % MAX_FACES + 1;

		struct vec interior = randpolytope(A, b, n);
		struct polytope p = {
			.A = A,
			.b = b,
			.n = n
		};
		assert(vinpolytope(p, interior, 1e-10));

		for (int point = 0; point < N_PROJECTIONS; ++point) {

			struct vec x = randsphere();
			struct vec xp = vprojectpolytope(p, x, work, TOLERANCE, MAXITERS);

			// Feasibility check: projection is inside polytope.
			// The tolerance is looser than the vprojectpolytope tolerance
			// because that tolerance doesn't actually guarantee a rigid bound
			// on constraint violations.
			assert(vinpolytope(p, xp, 10 * TOLERANCE));

			// Optimality check: projection is closer than other random points
			// to query point. Very large N_OTHERS would be more thorough...
			for (int other = 0; other < N_OTHERS; ++other) {
				struct vec other = vadd(randnvec(), interior);
				if (vinpolytope(p, other, 0.0f)) {
					assert(vdist2(xp, x) <= vdist2(other, x));
				}
			}
		}
	}

	free(A);
	free(b);
	free(work);

	printf("%s passed\n", __func__);
}

void test_polytope_ray()
{
	srand(1); // deterministic

	// 1. Basic checks using an infinite rectangular "tube".
	{
		float A[4][3] = {
			{ 1.0f,  0.0f,  0.0f},
			{ 0.0f,  1.0f,  0.0f},
			{-1.0f,  0.0f,  0.0f},
			{ 0.0f, -1.0f,  0.0f},
		};
		float b[4] = {
			1.0,
			2.0,
			3.0,
			4.0,
		};
		struct polytope p = {
			.A = A[0],
			.b = b,
			.n = 4,
		};

		int row = -1;

		for (int i = 0; i < 4; ++i) {
			// Test the face.
			struct vec dir = vloadf(&A[i][0]);
			float s = rayintersectpolytope(p, vzero(), dir, &row);
			assert(s == b[i]); // No floating-point error expected.
			assert(row == i);

			// Test the corner.
			int j = (i + 1) % 4;
			struct vec next_dir = vloadf(&A[j][0]);
			struct vec corner_us = vadd(
				vscl(b[i] + 1e-6f, dir),
				vscl(b[j], next_dir)
			);
			rayintersectpolytope(p, vzero(), corner_us, &row);
			assert(row == i);

			struct vec corner_next = vadd(
				vscl(b[i], dir),
				vscl(b[j] + 1e-6f, next_dir)
			);
			rayintersectpolytope(p, vzero(), corner_next, &row);
			assert(row == j);
		}

		// Test the open ends of the tube.
		float s = rayintersectpolytope(p, vzero(), mkvec(0, 0, 1), &row);
		assert(isinf(s));

		s = rayintersectpolytope(p, vzero(), mkvec(0, 0, -1), &row);
		assert(isinf(s));

		// Test that we can find very far away interesections.
		s = rayintersectpolytope(p, vzero(), mkvec(0, 1e-9, -1), &row);
		assert(!isinf(s));
	}

	// 2. Test that we handle loose-everywhere constraints.
	{
		float A[2][3];
		float b[2];
		struct vec v = randsphere();

		vstoref(v, A[0]);
		b[0] = 1.0;

		vstoref(v, A[1]);
		b[1] = 1.1;

		struct polytope p = { .A = A[0], .b = b, .n = 2 };
		int row;
		float s = rayintersectpolytope(p, vscl(0.5, v), v, &row);
		assert(fcloseulps(s, 0.5, 10));
	}

	// 3. Test the stated behavior for empty polytopes.
	{
		float A[2][3];
		float b[2];
		struct vec v = randsphere();

		vstoref(v, A[0]);
		b[0] = -1.0;

		vstoref(vneg(v), A[1]);
		b[1] = -1.0;

		struct polytope p = { .A = A[0], .b = b, .n = 2 };
		int row;
		float s = rayintersectpolytope(p, vscl(0.5, v), v, &row);
		assert(s < 0.0f);
	}

	// 4. Test random polytopes.

	printf("%s passed\n", __func__);
}


// micro test framework
typedef void (*voidvoid_fn)(void);
voidvoid_fn test_fns[] = {
	test_vec_basic,
	test_mat_axisangle,
	test_quat_conversions,
	test_qvectovec,
	test_qslerp,
	test_quat_lowprecision,
	test_polytope_projection,
	test_polytope_ray,
};

static int i_test = -1;
static int recursion_level = 0;
static int exit_code = 0;
static int const n_tests = sizeof(test_fns) / sizeof(test_fns[0]);

void sighandler(int sig)
{
	++i_test;
	if (i_test > recursion_level) {
		exit_code = 1;
	}
	if (i_test < n_tests) {
		(*test_fns[i_test])();
		++recursion_level;
		sighandler(sig);
	}
	else {
		if (exit_code == 0) {
			puts("All tests passed.");
		}
		exit(exit_code);
	}
}

int main()
{
	signal(SIGABRT, sighandler);
	sighandler(SIGABRT);
}
