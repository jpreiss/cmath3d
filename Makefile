test_math3d: test.c math3d.h
	$(CC) -std=c99 -Wall -Wpedantic -Wdouble-promotion -g -DCMATH3D_ASSERTS -o test_math3d test.c -lm

test: ./test_math3d
	./test_math3d

clean:
	rm test_math3d
