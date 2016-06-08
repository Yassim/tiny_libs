//
// This demo uses the C interface to vectorial. 
//  from https://github.com/scoopr/vectorial
//

#include "../vendor/vectorial/config.h"
#include "../vendor/vectorial/simd4f.h"

#define TCSG_HAVE_F3 1
#define tcsg_f3 simd4f 

tcsg_f3 tcsg_f3_lerp(const tcsg_f3 i_0, const tcsg_f3 i_1, float i_0to1)
{
    const float _1to0 = 1.0f - i_0to1;
    return simd4f_add(
        simd4f_mul(i_0, simd4f_splat(_1to0)),
        simd4f_mul(i_1, simd4f_splat(i_0to1))
        );
}
tcsg_f3 tcsg_f3_cross(const tcsg_f3 i_a, const tcsg_f3 i_b)
{
    return simd4f_cross3(i_a, i_b);
}
float   tcsg_f3_dot(const tcsg_f3 i_a, const tcsg_f3 i_b)
{
    return simd4f_dot3_scalar(i_a, i_b);
}
tcsg_f3 tcsg_f3_normalise(const tcsg_f3 i_a)
{
    return simd4f_normalize3(i_a);
}
tcsg_f3 tcsg_f3_invert(const tcsg_f3 i_a)
{
    return simd4f_sub(simd4f_splat(0.f), i_a);
}
tcsg_f3 tcsg_f3_sub(const tcsg_f3 i_a, const tcsg_f3 i_b)
{
    return simd4f_sub(i_a, i_b);
}
tcsg_f3 tcsg_f3_add(const tcsg_f3 i_a, const tcsg_f3 i_b)
{
    return simd4f_add(i_a, i_b);
}
tcsg_f3 tcsg_f3_scale(const tcsg_f3 i_a, float i_f)
{
    return simd4f_mul(i_a, simd4f_splat(i_f));
}
tcsg_f3 tcsg_f3_new(float i_x, float i_y, float i_z)
{
    return simd4f_create(i_x, i_y, i_z, 0.0f);
}
float   tcsg_f3_x(const tcsg_f3 i_a)
{
    return simd4f_get_x(i_a);
}
float   tcsg_f3_y(const tcsg_f3 i_a)
{
    return simd4f_get_y(i_a);
}
float   tcsg_f3_z(const tcsg_f3 i_a)
{
    return simd4f_get_z(i_a);
}

//
// this section might be windows only currently.
//
#include <mmintrin.h>

// Defines
// NOTE: when using smid stuff, its a good idea to align heap memory to 16 bytes
#define TCSG_HAVE_MEMORY 1
#define tcsg_malloc(N)      _aligned_malloc((N), 16)
#define tcsg_realloc(P, N)  _aligned_realloc((P), (N), 16)
#define tcsg_free(P)        _aligned_free((P))

#define TINY_CSG_IMPLEMENTATION
#define TINY_CSG_GLDEMO

#include "../Public/tiny_csg.h"

