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
}

// micro test framework
typedef void (*voidvoid_fn)(void);
voidvoid_fn test_fns[] = {
	test_quat_rpy_conversions,
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
