#ifndef TINY_MATH
#define TINY_MATH

#include <math.h> // for trig

#ifndef tm_inline
#define tm_inline    __inline
#endif
// #ifndef tm_imp
// #define tm_imp(...)    { __VA_ARGS__ }
// #endif

// Types
typedef struct { float X, Y; }          float2;
typedef struct { float X, Y, Z; }       float3;
typedef struct { float X, Y, Z, W; }    float4;
typedef struct { float m[3 * 3]; }      matrix3;
typedef struct { float m[4 * 4]; }      matrix4;
typedef struct { float3 N; float D; }   plane3;
typedef struct { float x, y, z, w; }    quat4f; //TODO

typedef enum {
    kPlane3_CoPlane,
    kPlane3_Front,
    kPlane3_Back,
    kPlane3_Span
} plane3_res;

#define kPlane3_Eppsilon    0.00001f
#define kMath_Pi            3.14159265358979323846f

// API
//  Float2
tm_inline float2  float2_new(float x, float y);
tm_inline float2  float2_newv(const float* v);
tm_inline float2  float2_newf(float v);
tm_inline float2  float2_add(float2 a, float2 b);
tm_inline float2  float2_sub(float2 a, float2 b);
tm_inline float2  float2_mul(float2 a, float2 b);
tm_inline float2  float2_mulf(float2 a, float b);
tm_inline float2  float2_div(float2 a, float2 b);
tm_inline float   float2_hsum(float2 a);
tm_inline float   float2_lengthsq(float2 a);
tm_inline float   float2_length(float2 a);
tm_inline float2  float2_normalize(float2 a);
tm_inline float2  float2_90dcw(float2 a);
tm_inline float2  float2_90dccw(float2 a);
tm_inline int     float2_eql(float2 a, float2 b, float t);
tm_inline int     float2_lt(float2 a, float2 b, float t);
tm_inline int     float2_intercet(float2 a1, float2 a2, float2 b1, float2 b2, float2* o_i);

//  Float3
tm_inline float3  float3_new(float x, float y, float z);
tm_inline float3  float3_newv(const float* v);
tm_inline float3  float3_newf(float v);
tm_inline float3  float3_add(float3 a, float3 b);
tm_inline float3  float3_sub(float3 a, float3 b);
tm_inline float3  float3_mul(float3 a, float3 b);
tm_inline float3  float3_mulf(float3 a, float b);
tm_inline float3  float3_div(float3 a, float3 b);
tm_inline float   float3_hsum(float3 a);
tm_inline float   float3_lengthsq(float3 a);
tm_inline float   float3_length(float3 a);
tm_inline float3  float3_normalize(float3 a);
tm_inline float3  float3_normal(float3 a, float3 b, float3 c);
tm_inline float3  float3_cross(float3 a, float3 b);
tm_inline float   float3_dot(float3 a, float3 b);
//  Float4
tm_inline float4  float4_new(float x, float y, float z, float w);
tm_inline float4  float4_newv(const float* v);
tm_inline float4  float4_newf(float v);
tm_inline float4  float4_add(float4 a, float4 b);
tm_inline float4  float4_sub(float4 a, float4 b);
tm_inline float4  float4_mul(float4 a, float4 b);
tm_inline float4  float4_mulf(float4 a, float b);
tm_inline float4  float4_div(float4 a, float4 b);
tm_inline float   float4_hsum(float4 a);
tm_inline float   float4_lengthsq(float4 a);
tm_inline float   float4_length(float4 a);
tm_inline float4  float4_normalize(float4 a);
//  Matrix3
tm_inline matrix3 matrix3_identity();
tm_inline matrix3 matrix3_rotx(float x);
tm_inline matrix3 matrix3_roty(float y);
tm_inline matrix3 matrix3_rotz(float z);
tm_inline matrix3 matrix3_scale(float3 a);
tm_inline matrix3 matrix3_transpose(matrix3 i);
tm_inline matrix3 matrix3_add(matrix3 a, matrix3 b);
tm_inline matrix3 matrix3_mul(matrix3 a, matrix3 b);
tm_inline matrix3 matrix3_mulf(matrix3 a, float b);
tm_inline float3  matrix3_mulf3(matrix3 a, float3 b);
tm_inline void    matrix3_mulf3v(matrix3 a, int c, const float3* s, float3* d);
//  Matrix4
tm_inline matrix4 matrix4_identity();
tm_inline matrix4 matrix4_rotx(float x);
tm_inline matrix4 matrix4_roty(float y);
tm_inline matrix4 matrix4_rotz(float z);
tm_inline matrix4 matrix4_scale(float3 a);
tm_inline matrix4 matrix4_translate(float3 a);
tm_inline matrix4 matrix4_transpose(matrix4 i);
tm_inline matrix3 matrix4_rotation(matrix4 i);
tm_inline matrix4 matrix4_add(matrix4 a, matrix4 b);
tm_inline matrix4 matrix4_mul(matrix4 a, matrix4 b);
tm_inline matrix4 matrix4_mulf(matrix4 a, float b);
tm_inline float3  matrix4_mulf3(matrix4 a, float3 b);
tm_inline void    matrix4_mulf3v(matrix4 a, int c, const float3* s, float3* d);
tm_inline matrix4 matrix4_lookAt(float3 p, float3 l, float3 v);
tm_inline matrix4 matrix4_lookAtf(float px, float py, float pz,
                                  float lx, float ly, float lz,
                                  float vx, float vy, float vz);
tm_inline matrix4 matrix4_perspective(float fovyInDegrees, float aspectRatio, float znear, float zfar);
tm_inline matrix4 matrix4_frustum(float left, float right, float bottom, float top, float znear, float zfar);

//  Plane3
tm_inline plane3     plane3_new(float3 a, float3 b, float3 c);
tm_inline plane3_res plane3_test(plane3 p, float3 a);
tm_inline plane3_res plane3_testv(plane3 p, int c, const float3* v, plane3_res* o);
//  Quat4f
tm_inline quat4f quat4f_identity();
tm_inline quat4f quat4f_axisangle(float3 axis, float ang);
tm_inline quat4f quat4f_mul(quat4f a, quat4f b);
tm_inline quat4f quat4f_conjugate(quat4f i);
tm_inline float3 quat4f_mulf3(quat4f a, float3 b);


// Implimentation
///
//  Float2
///
tm_inline float2  float2_new(float x, float y)
{
    const float2 o = { x, y };
    return o;
}

tm_inline float2  float2_newv(const float* v)
{
    const float2 o = { v[0], v[1] };
    return o;
}

tm_inline float2  float2_newf(float v)
{
    const float2 o = { v, v };
    return o;
}

tm_inline float2  float2_add(float2 a, float2 b)
{
    const float2 o = { a.X + b.X, a.Y + b.Y };
    return o;
}

tm_inline float2  float2_sub(float2 a, float2 b)
{
    const float2 o = { a.X - b.X, a.Y - b.Y };
    return o;
}

tm_inline float2  float2_mul(float2 a, float2 b)
{
    const float2 o = { a.X * b.X, a.Y * b.Y };
    return o;
}

tm_inline float2  float2_mulf(float2 a, float b)
{
    const float2 o = { a.X * b, a.Y * b };
    return o;
}

tm_inline float2  float2_div(float2 a, float2 b)
{
    const float2 o = { a.X / b.X, a.Y / b.Y };
    return o;
}

tm_inline float   float2_hsum(float2 a)
{
    return a.X + a.Y;
}

tm_inline float2 float2_lerp(float2 a, float2 b, float t)
{
    return float2_add(a, float2_mulf(float2_sub(b, a), t));
}

tm_inline float   float2_lengthsq(float2 a)
{
    return float2_hsum(float2_mul(a, a));
}

tm_inline float   float2_length(float2 a)
{
    return sqrtf(float2_lengthsq(a));
}

tm_inline float2  float2_normalize(float2 a)
{
    return float2_mulf(a, 1.0f / float2_length(a));
}

tm_inline float2  float2_90dcw(float2 a)
{
    const float2 o = { a.Y, -a.X };
    return o;
}

tm_inline float2  float2_90dccw(float2 a)
{
    const float2 o = { -a.Y, a.X };
    return o;
}

tm_inline int     float2_eql(float2 a, float2 b, float t)
{
    return fabs(a.X - b.X) <= t
        && fabs(a.Y - b.Y) <= t;
}

tm_inline int     float2_lt(float2 a, float2 b, float t)
{
    return a.X < b.X
        || a.Y < b.Y && fabs(a.X - b.X) <= t;
}

tm_inline int     float2_intercet(float2 a1, float2 a2, float2 b1, float2 b2, float2* o_i)
{
    const float ta1 = b1.Y - a1.Y;
    const float tb1 = a1.X - b1.X;
    const float ta2 = b2.Y - a2.Y;
    const float tb2 = a2.X - b2.X;
    const float det = ta1 * tb2 - ta2 * tb1;
    if (det != 0.0f) {
        const float tc1 = ta1 * a1.X + tb1 * a1.Y;
        const float tc2 = ta2 * a2.X + tb2 * a2.Y;
        const float x = (tb2 * tc1 - tb1 * tc2) / det;
        const float y = (ta1 * tc2 - ta2 * tc1) / det;

#define lmin(A, B) (A < B ? A : B)
#define lmax(A, B) (A < B ? A : B)

        if((lmin(a1.X, b1.X) </*=*/ x && x </*=*/ lmax(a1.X, b1.X))
        && (lmin(a1.Y, b1.Y) </*=*/ y && y </*=*/ lmax(a1.Y, b1.Y))
        && (lmin(a2.X, b2.X) </*=*/ x && x </*=*/ lmax(a2.X, b2.X))
        && (lmin(a2.Y, b2.Y) </*=*/ y && y </*=*/ lmax(a2.Y, b2.Y)))
        {
            *o_i = float2_new(x, y);
            return 1;
        }
    }

#undef lmin
#undef lmax

    return 0;
}


///
// Float3
///
tm_inline float3  float3_new(float x, float y, float z) 
{
    const float3 o = { x, y, z };
    return o;
}

tm_inline float3  float3_newv(const float* v)
{
    const float3 o = { v[0], v[1], v[2] };
    return o;
}

tm_inline float3  float3_newf(float v) 
{
    const float3 o = { v, v, v };
    return o;
}

tm_inline float3  float3_add(float3 a, float3 b) 
{
    const float3 o = { a.X + b.X, a.Y + b.Y, a.Z + b.Z };
    return o;
}

tm_inline float3  float3_sub(float3 a, float3 b) 
{
    const float3 o = { a.X - b.X, a.Y - b.Y, a.Z - b.Z };
    return o;
}

tm_inline float3  float3_mul(float3 a, float3 b) 
{
    const float3 o = { a.X * b.X, a.Y * b.Y, a.Z * b.Z };
    return o;
}

tm_inline float3  float3_mulf(float3 a, float b) 
{
    const float3 o = { a.X * b, a.Y * b, a.Z * b };
    return o;
}

tm_inline float3  float3_div(float3 a, float3 b) 
{
    const float3 o = { a.X / b.X, a.Y / b.Y, a.Z / b.Z };
    return o;
}

tm_inline float  float3_hsum(float3 a) 
{
    return a.X + a.Y + a.Z;
}

tm_inline float3  float3_cross(float3 a, float3 b) 
{
    const float3 o = { 
        a.Y * b.Z - a.Z * b.Y, 
        a.Z * b.X - a.X * b.Z, 
        a.X * b.Y - a.Y * b.X, 
    };
    return o;
}

tm_inline float  float3_dot(float3 a, float3 b) 
{
    return float3_hsum(float3_mul(a, b));
}

tm_inline float3 float3_lerp(float3 a, float3 b, float t)
{
    return float3_add(a, float3_mulf(float3_sub(b, a), t));
}

tm_inline float  float3_lengthsq(float3 a) 
{
    return float3_dot(a, a);
}

tm_inline float  float3_length(float3 a) 
{
    return sqrtf(float3_lengthsq(a));
}

tm_inline float3  float3_normalize(float3 a) 
{
    return float3_mulf(a, 1.0f/ float3_length(a));
}

tm_inline float3  float3_normal(float3 a, float3 b, float3 c) 
{
    return plane3_new(a, b, c).N;
}
///
//  Float4
///
tm_inline float4  float4_new(float x, float y, float z, float w)
{
    const float4 o = { x, y, z, w };
    return o;
}

tm_inline float4  float4_newv(const float* v)
{
    const float4 o = { v[0], v[1], v[2], v[3] };
    return o;
}

tm_inline float4  float4_newf(float v)
{
    const float4 o = { v, v, v, v };
    return o;
}

tm_inline float4  float4_add(float4 a, float4 b)
{
    const float4 o = { a.X + b.X, a.Y + b.Y, a.Z + b.Z, a.W + b.W };
    return o;
}

tm_inline float4  float4_sub(float4 a, float4 b)
{
    const float4 o = { a.X - b.X, a.Y - b.Y, a.Z - b.Z, a.W - b.W };
    return o;
}

tm_inline float4  float4_mul(float4 a, float4 b)
{
    const float4 o = { a.X * b.X, a.Y * b.Y, a.Z * b.Z, a.W * b.W };
    return o;
}

tm_inline float4  float4_mulf(float4 a, float b)
{
    const float4 o = { a.X * b, a.Y * b, a.Z * b, a.W * b };
    return o;
}

tm_inline float4  float4_div(float4 a, float4 b)
{
    const float4 o = { a.X / b.X, a.Y / b.Y, a.Z / b.Z, a.W / b.W };
    return o;
}

tm_inline float   float4_hsum(float4 a)
{
    return a.X + a.Y + a.Z + a.W;
}

tm_inline float4 float4_lerp(float4 a, float4 b, float t)
{
    return float4_add(a, float4_mulf(float4_sub(b, a), t));
}

tm_inline float   float4_lengthsq(float4 a)
{
    return float4_hsum(float4_mul(a, a));
}

tm_inline float   float4_length(float4 a)
{
    return sqrtf(float4_lengthsq(a));
}

tm_inline float4  float4_normalize(float4 a)
{
    return float4_mulf(a, 1.0f / float4_length(a));
}

///
//  Matrix3
///
tm_inline matrix3 matrix3_identity()
{
    const matrix3 o = { {
        1.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 1.0f,
    } };
    return o;
}

tm_inline matrix3 matrix3_rotx(float x)
{
    const float s = sinf(x);
    const float c = cosf(x);
    const matrix3 o = { {
            1.0f, 0.0f, 0.0f,
            0.0f,    c,    s,
            0.0f,   -s,    c,
        } };
    return o;
}

tm_inline matrix3 matrix3_roty(float y)
{
    const float s = sinf(y);
    const float c = cosf(y);
    const matrix3 o = { {
               c, 0.0f,   -s,
            0.0f, 1.0f, 0.0f,
               s, 0.0f,    c,
        } };
    return o;
}

tm_inline matrix3 matrix3_rotz(float z)
{
    const float s = sinf(z);
    const float c = cosf(z);
    const matrix3 o = { {
               c,    s, 0.0f,
              -s,    c, 0.0f,
            0.0f, 0.0f, 1.0f,
        } };
    return o;
}

tm_inline matrix3 matrix3_scale(float3 a)
{
    const matrix3 o = { {
            a.X, 0.0f, 0.0f,
            0.0f, a.Y, 0.0f,
            0.0f, 0.0f, a.Z,
        } };
    return o;
}


tm_inline matrix3 matrix3_transpose(matrix3 i)
{
#define M(X, Y)    i.m[X + Y * 3]
    const matrix3 o = { {
            M(0,0), M(0,1), M(0,2),
            M(1,0), M(1,1), M(1,2),
            M(2,0), M(2,1), M(2,2),
        } };
    return o;
#undef M
}

tm_inline matrix3 matrix3_add(matrix3 a, matrix3 b)
{
#define M(X, Y)    a.m[X + Y * 3] + b.m[X + Y * 3]
    const matrix3 o = { {
            M(0,0), M(0,1), M(0,2),
            M(1,0), M(1,1), M(1,2),
            M(2,0), M(2,1), M(2,2),
        } };
    return o;
#undef M
}

tm_inline matrix3 matrix3_mul(matrix3 a, matrix3 b)
{
    const matrix3 t = matrix3_transpose(b);
#define M(X, Y) float3_hsum(float3_mul(float3_newv(a.m + Y*3), float3_newv(t.m + X*3)))
    const matrix3 o = { {
            M(0,0), M(1,0), M(2,0),
            M(0,1), M(1,1), M(2,1),
            M(0,2), M(1,2), M(2,2),
        } };
    return o;
#undef M
}

tm_inline matrix3 matrix3_mulf(matrix3 a, float b)
{
#define M(X, Y)    a.m[X + Y * 3] * b
    const matrix3 o = { {
            M(0,0), M(0,1), M(0,2),
            M(1,0), M(1,1), M(1,2),
            M(2,0), M(2,1), M(2,2),
        } };
    return o;
#undef M
}

tm_inline float3  matrix3_mulf3(matrix3 a, float3 b)
{
    const float3 o = {
        a.m[0x0] * b.X + a.m[0x3] * b.Y + a.m[0x6] * b.Z,
        a.m[0x1] * b.X + a.m[0x4] * b.Y + a.m[0x7] * b.Z,
        a.m[0x2] * b.X + a.m[0x5] * b.Y + a.m[0x8] * b.Z,
    };
    return o;
}

tm_inline void    matrix3_mulf3v(matrix3 a, int c, const float3* s, float3* d)
{
    const matrix3 t = matrix3_transpose(a);
    const float3 x = float3_newv(t.m + 0);
    const float3 y = float3_newv(t.m + 3);
    const float3 z = float3_newv(t.m + 6);
    do {
        *d = float3_new(float3_hsum(float3_mul(x, *s)),
                        float3_hsum(float3_mul(y, *s)),
                        float3_hsum(float3_mul(z, *s))
            );
        d++;
        s++;
    } while (--c);
}

///
//  Matrix4
///
tm_inline matrix4 matrix4_identity()
{
    const matrix4 o = { {
            1.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f,
        } };
    return o;
}

tm_inline matrix4 matrix4_rotx(float x)
{
    const float s = sinf(x);
    const float c = cosf(x);
    const matrix4 o = { {
            1.0f, 0.0f, 0.0f, 0.0f,
            0.0f,    c,    s, 0.0f,
            0.0f,   -s,    c, 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f,
        } };
    return o;
}

tm_inline matrix4 matrix4_roty(float y)
{
    const float s = sinf(y);
    const float c = cosf(y);
    const matrix4 o = { {
               c, 0.0f,   -s, 0.0f,
            0.0f, 1.0f, 0.0f, 0.0f,
               s, 0.0f,    c, 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f,
        } };
    return o;
}

tm_inline matrix4 matrix4_rotz(float z)
{
    const float s = sinf(z);
    const float c = cosf(z);
    const matrix4 o = { {
               c,    s, 0.0f, 0.0f,
              -s,    c, 0.0f, 0.0f,
            0.0f, 0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f,
        } };
    return o;
}

tm_inline matrix4 matrix4_scale(float3 a)
{
    const matrix4 o = { {
            1.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f,
        } };
    return o;
}

tm_inline matrix4 matrix4_translate(float3 a)
{
    const matrix4 o = { {
            1.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 1.0f, 0.0f,
             a.X,  a.Y,  a.Z, 1.0f,
        } };
    return o;
}

tm_inline matrix4 matrix4_transpose(matrix4 i)
{
#define M(X, Y)    i.m[X + Y * 4]
    const matrix4 o = { {
            M(0,0), M(0,1), M(0,2), M(0,3),
            M(1,0), M(1,1), M(1,2), M(1,3),
            M(2,0), M(2,1), M(2,2), M(2,3),
            M(3,0), M(3,1), M(3,2), M(3,3),
        } };
    return o;
#undef M
}

tm_inline matrix3 matrix4_rotation(matrix4 i)
{
#define M(X, Y)    i.m[X + Y * 4]
    const matrix3 o = { {
            M(0,0), M(1,0), M(2,0),
            M(0,1), M(1,1), M(2,1),
            M(0,2), M(1,2), M(2,2),
        } };
    return o;
#undef M
}

tm_inline matrix4 matrix4_add(matrix4 a, matrix4 b)
{
#define M(X, Y)    a.m[X + Y * 4] + b.m[X + Y * 4]
    const matrix4 o = { {
            M(0,0), M(1,0), M(2,0), M(3,0),
            M(0,1), M(1,1), M(2,1), M(3,1),
            M(0,2), M(1,2), M(2,2), M(3,2),
            M(0,3), M(1,3), M(2,3), M(3,3),
        } };
    return o;
#undef M
}

tm_inline matrix4 matrix4_mul(matrix4 a, matrix4 b)
{
    const matrix4 t = matrix4_transpose(b);
#define M(X, Y) float4_hsum(float4_mul(float4_newv(a.m + Y*4), float4_newv(t.m + X*4)))
    const matrix4 o = { {
        M(0,0), M(1,0), M(2,0), M(3,0),
        M(0,1), M(1,1), M(2,1), M(3,1),
        M(0,2), M(1,2), M(2,2), M(3,2),
        M(0,3), M(1,3), M(2,3), M(3,3),
    } };
    return o;
#undef M
}

tm_inline matrix4 matrix4_mulf(matrix4 a, float b)
{
    const matrix4 o = { {
        a.m[0x0] * b, a.m[0x1] * b, a.m[0x2] * b, a.m[0x3] * b,
        a.m[0x4] * b, a.m[0x5] * b, a.m[0x6] * b, a.m[0x7] * b,
        a.m[0x8] * b, a.m[0x9] * b, a.m[0xA] * b, a.m[0xB] * b,
        a.m[0xC] * b, a.m[0xD] * b, a.m[0xE] * b, a.m[0xF] * b,
    } };
    return o;
}

tm_inline float3 matrix4_mulf3(matrix4 a, float3 b)
{
    const float3 o = {
        a.m[0x0] * b.X + a.m[0x4] * b.Y + a.m[0x8] * b.Z + a.m[0xC],
        a.m[0x1] * b.X + a.m[0x5] * b.Y + a.m[0x9] * b.Z + a.m[0xD],
        a.m[0x2] * b.X + a.m[0x6] * b.Y + a.m[0xA] * b.Z + a.m[0xE],
    };
    return o;
}

tm_inline void    matrix4_mulf3v(matrix4 a, int c, const float3* s, float3* d)
{
    const matrix4 t = matrix4_transpose(a);
    const float3 x = float3_newv(t.m + 0x0);
    const float3 y = float3_newv(t.m + 0x4);
    const float3 z = float3_newv(t.m + 0x8);
    const float3 w = float3_newv(a.m + 0xC); //NOTE: not transposed
    do {
        *d = float3_add(w, 
                float3_new(float3_hsum(float3_mul(x, *s)),
                            float3_hsum(float3_mul(y, *s)),
                            float3_hsum(float3_mul(z, *s))
                )
             );
        d++;
        s++;
    } while (--c);
}


tm_inline matrix4 matrix4_lookAt(float3 p, float3 t, float3 v)
{
    const float3 a = float3_normalize(float3_sub(p, t));
    const float3 b = float3_normalize(float3_sub(v, a));
    const float3 c = float3_cross(a, b);

    const matrix4 o = { {
             a.X,  a.Y,  a.Z, 0.0f,
             b.X,  b.Y,  b.Z, 0.0f,
             c.X,  c.Y,  c.Z, 0.0f,
            -p.X, -p.Y, -p.Z, 1.0f
    } };

    return o;
}

tm_inline matrix4 matrix4_lookAtf(float px, float py, float pz,
                                  float tx, float ty, float tz,
                                  float vx, float vy, float vz)
{
    return matrix4_lookAt(float3_new(px, py, pz), float3_new(tx, ty, tz), float3_new(vx, vy, vz));
}

tm_inline matrix4 matrix4_perspective(float fovyInDegrees, float aspectRatio, float znear, float zfar)
{
    float ymax, xmax;
    ymax = znear * tanf(fovyInDegrees * kMath_Pi / 360.0f);
    if (aspectRatio < 1.0)
    {
        ymax = znear * tanf(fovyInDegrees * kMath_Pi / 360.0f);
        xmax = ymax * aspectRatio;
    }
    else
    {
        xmax = znear * tanf(fovyInDegrees * kMath_Pi / 360.0f);
        ymax = xmax / aspectRatio;
    }

    return matrix4_frustum(-xmax, xmax, -ymax, ymax, znear, zfar);
}

tm_inline matrix4 matrix4_frustum(float left, float right, float bottom, float top, float znear, float zfar)
{
    const float near2 = 2.0f * znear;
    const float width = right - left;
    const float height = top - bottom;
    const float depth = zfar - znear;

    const float m_00 = near2 / width;            // 2*near/(right-left)
    const float m_11 = near2 / height;           // 2*near/(top - bottom)
    const float m_20 = (right + left) / width;   // A = r+l / r-l
    const float m_21 = (top + bottom) / height;  // B = t+b / t-b
    const float m_22 = (-zfar - znear) / depth;  // C = -(f+n) / f-n
    const float m_32 = (-near2 * zfar) / depth;  // D = -2fn / f-n

    const matrix4 o = { {
        m_00, 0.0f, 0.0f, 0.0f,
        0.0f, m_11, 0.0f, 0.0f,
        m_20, m_21, m_22,-1.0f,
        0.0f, 0.0f, m_32, 0.0f,
    } };
    return o;
}

///
// Plane3
///
tm_inline plane3 plane3_new(float3 a, float3 b, float3 c)
{
    const float3 tmp = float3_cross(float3_sub(a, b), float3_sub(a, c));
    const float D = float3_length(tmp);
    const plane3 o = { float3_mulf(tmp, 1.0f / D), D };
    return o;
}

tm_inline plane3_res plane3_test(plane3 p, float3 a)
{
    const float t = float3_dot(p.N, a) - p.D;
    return (t < -kPlane3_Eppsilon) ? kPlane3_Back : ((t > kPlane3_Eppsilon) ? kPlane3_Front : kPlane3_CoPlane);
}

tm_inline plane3_res plane3_testv(plane3 p, int c, const float3* v, plane3_res* o)
{
    int agg = kPlane3_CoPlane;

    do {
        agg |= *o = plane3_test(p, *v);
        o++;
        v++;
    } while (--c);

    return (plane3_res)agg;
}

tm_inline plane3_res plane3_testb(plane3 p, int c, const float3* v)
{
    int agg = kPlane3_CoPlane;

    do {
        agg |= plane3_test(p, *v);
        v++;
    } while (--c);

    return (plane3_res)agg;
}

///
//  Quat4f
///
tm_inline quat4f quat4f_identity()
{
    const quat4f o = { 0.0f, 0.0f, 0.0f, 1.0f };
    return o;
}

tm_inline quat4f quat4f_axisangle(float3 axis, float ang)
{
    const float id = 1.0f / float3_length(axis);
    const float ids = id * sinf(ang);
    const float c = cosf(ang);
    const quat4f o = {
        ids * axis.X,
        ids * axis.Y,
        ids * axis.Z,
        c
    };
    return o;
}

tm_inline quat4f quat4f_mul(quat4f a, quat4f b)
{
    const quat4f o = {
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y + a.y * b.w + a.z * b.x - a.x * b.z,
        a.w * b.z + a.z * b.w + a.x * b.y - a.y * b.x,
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z, 
    };
    return o;
}

tm_inline quat4f quat4f_conjugate(quat4f i)
{
    const quat4f o = { -i.x, -i.y, -i.z, i.w };
    return o;
}

tm_inline float3 quat4f_mulf3(quat4f a, float3 b)
{
    const quat4f qb = { b.X, b.Y, b.Z, 0.0f };
    const quat4f ia = quat4f_conjugate(a);
    const quat4f qo = quat4f_mul(quat4f_mul(qb, ia), a);
    const float3 o = { qo.x, qo.y, qo.z };
    return o;
}


#endif // !TINY_MATH
