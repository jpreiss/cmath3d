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

void test_quat_rpy_conversions()
{
	srand(0); // deterministic
	int const N = 10000;
	for (int i = 0; i < N; ++i) {
		float yaw   = randu(-0.98f*M_PI_F, 0.98f*M_PI_F); // quat2rpy never outputs outside [-pi, pi]
		float pitch = randu(-0.48f*M_PI_F, 0.48f*M_PI_F); // avoid singularity
		float roll  = randu(-0.98f*M_PI_F, 0.98f*M_PI_F);
		struct vec rpy0 = mkvec(roll, pitch, yaw);
		struct vec rpy1 = quat2rpy(rpy2quat(rpy0));
		ASSERT_VEQ_EPSILON(rpy0, rpy1, 0.00001f); // must be fairly loose due to 32 bit trig, etc.
	}
	printf("%s passed\n", __func__);
}

void test_quat_mat_conversions()
{
	srand(0); // deterministic
	int const N = 10000;
	for (int i = 0; i < N; ++i) {
		struct quat const q = randquat();
		struct mat33 const m = quat2rotmat(q);
		struct quat const qq = mat2quat(m);
		float const angle = qanglebetween(q, qq);
		assert(fabs(angle) < radians(1e-4));
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
		if (vmag2(a) < 1e-8 || vmag2(b) < 1e-8) continue;
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

// micro test framework
typedef void (*voidvoid_fn)(void);
voidvoid_fn test_fns[] = {
	test_vec_basic,
	test_mat_axisangle,
	test_quat_rpy_conversions,
	test_quat_mat_conversions,
	test_qvectovec,
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
