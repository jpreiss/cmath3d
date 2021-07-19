CFLAGS = -std=c99 -Werror -Wall -Wpedantic -Wdouble-promotion -Wno-error=unused-function
TESTFLAGS = -DCMATH3D_ASSERTS -g

test_math3d: test.c math3d.h
	$(CC) $(CFLAGS) $(TESTFLAGS) -o test_math3d test.c -lm

coverage:
	$(CC) $(CFLAGS) $(TESTFLAGS) -DCMATH3D_COVERAGE -fprofile-arcs -ftest-coverage -o cov_math3d test.c -lm -lgcov
	./cov_math3d
	gcov test.c math3d.h

test: ./test_math3d
	./test_math3d

clean:
	rm test_math3d *.gcov *.gcda *.gcno
