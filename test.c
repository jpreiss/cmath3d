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
	return vscl(1.0 / sqrtf(3.0), v);
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

// Generates n linear inequalities Ax <= b representing a convex polytope.
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
struct vec randpolytope(float A[], float b[], int n) {
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

void test_polytope_projection()
{
	srand(1); // deterministic

	// For each polytope, generate a few points, project them,
	// and check that the projections are closer than other points.
	int const N_POLYTOPES = 100;
	int const N_PROJECTIONS = 10;
	int const N_OTHERS = 100;
	float const TOLERANCE = 1e-6;
	float const MAXITERS = 500;

	int const MAX_FACES = 30;
	float *A = malloc(sizeof(float) * MAX_FACES * 3);
	float *b = malloc(sizeof(float) * MAX_FACES);
	float *work = malloc(sizeof(float) * MAX_FACES * 3);


	for (int trial = 0; trial < N_POLYTOPES; ++trial) {

		// Random number of polytope faces.
		int n = rand() % MAX_FACES + 1;

		struct vec interior = randpolytope(A, b, n);
		assert(vinpolytope(interior, A, b, n, 1e-10));

		for (int point = 0; point < N_PROJECTIONS; ++point) {

			struct vec x = randsphere();
			struct vec xp = vprojectpolytope(x, A, b, work, n, TOLERANCE, MAXITERS);

			// Feasibility check: projection is inside polytope.
			// The tolerance is looser than the vprojectpolytope tolerance
			// because that tolerance doesn't actually guarantee a rigid bound
			// on constraint violations.
			assert(vinpolytope(xp, A, b, n, 10 * TOLERANCE));

			// Optimality check: projection is closer than other random points
			// to query point. Very large N_OTHERS would be more thorough...
			for (int other = 0; other < N_OTHERS; ++other) {
				struct vec other = vadd(randnvec(), interior);
				if (vinpolytope(other, A, b, n, 0.0f)) {
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


// micro test framework
typedef void (*voidvoid_fn)(void);
voidvoid_fn test_fns[] = {
	test_vec_basic,
	test_mat_axisangle,
	test_quat_rpy_conversions,
	test_quat_mat_conversions,
	test_qvectovec,
	test_polytope_projection,
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
