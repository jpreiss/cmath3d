# cmath3d
3d math library for C. Vectors, 3x3 matrices, quaternions.

Unlike many other C libraries of this type, `cmath3d` passes arguments
and returns results by value instead of by pointer and pointer-to-output.
This choice has several motivations:

- avoids the need to name non-meaningful intermediate results for the sake of taking their address
- enables nested expressions
- reduces bugs by allowing more variables to be delcared `const`
- gives the optimizing compiler complete knowledge about function semantics,
  theoretically enabling better optimizations 
  (see [Chandler Carruth's talk](https://www.youtube.com/watch?v=eR34r7HOU14)

Although Carruth's talk implies that using these functions with inline, 
header-only definitions -- thus hiding no code from the compiler 
in external object files -- will enable as good or better optimizations
than a pass-by-pointer version, in practice this library seems to consume
more stack memory than it should.
I am currently researching ways to fix this problem, since I believe
a SSA compiler with the ability to destructure structs should produce
optimal code from this library.
