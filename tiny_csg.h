#define TINY_CSG_IMPLEMENTATION
#define TINY_CSG_GLDEMO


#ifndef TINY_CSG_HEADER
#define TINY_CSG_HEADER
/*
A Tiny CSG (constructive solid geometry). (v0.1 pre alpha)

This is inspired from:
https://github.com/evanw/csg.js
and
https://github.com/nothings/stb -- lib in a header

Main differences between this and csg.js
1) This is in C (so I do more memory management)
2) The BSP node do not hold a reference to its co-planer polygons.
3) The BSP nodes are in an array, using indcies for pointers.
4) The the test array in the plane_split allocated once (and prefereably on the stack)
5) There is only 1 BSP clip function, which handel both inside and outside.

NOTES:
* Poloygons are treated as immutable and are reference counted between vectors.


TODO:
Make Vector user overridable.
Make Vertex user overridable.
Make Memory functions user overrideable.
Make polygone ref counting atomic if need be.
Add AABB to polygone and polygon_vector?
Add poly mergeing

*/

typedef union {
    void*           p;

    int             i;
    unsigned int    u;
} tcsg_user_data;

// Vector 3
// TODO: make overrideable
typedef struct {
    float   x, y, z;
} tcsg_f3;

tcsg_f3 tcsg_f3_lerp(const tcsg_f3* i_0, const tcsg_f3* i_1, float i_0to1);
tcsg_f3 tcsg_f3_cross(const tcsg_f3* i_a, const tcsg_f3* i_b);
float   tcsg_f3_dot(const tcsg_f3* i_a, const tcsg_f3* i_b);
tcsg_f3 tcsg_f3_normalise(const tcsg_f3* i_a);
tcsg_f3 tcsg_f3_invert(const tcsg_f3* i_a);
tcsg_f3 tcsg_f3_sub(const tcsg_f3* i_a, const tcsg_f3* i_b);
tcsg_f3 tcsg_f3_scale(const tcsg_f3* i_a, float i_f);
tcsg_f3 tcsg_f3_new(float i_x, float i_y, float i_z);

// Vertex
// TODO: make overrideable
typedef struct {
    tcsg_f3     position;
    tcsg_f3     normal;
} tcsg_vert;

tcsg_vert   tcsg_vert_lerp(const tcsg_vert* i_0, const tcsg_vert* i_1, float i_0to1);
tcsg_f3     tcsg_vert_position(const tcsg_vert* i_a);
tcsg_f3     tcsg_vert_normal(const tcsg_vert* i_a);

// Plane 
typedef struct {
    tcsg_f3     normal;
    float       d;
} tcsg_plane;

typedef enum {
    tcsg_k_co_plane = 0,
    tcsg_k_front = 1,
    tcsg_k_back = 2,
    tcsg_k_spanning = 3
} tcsg_plane_test;

#define tcsg_k_epsilon  0.00001f

// Polygone
typedef struct {
    int             ref_count;
    int             count;
    tcsg_user_data  user;
    tcsg_plane      plane;
    tcsg_vert       verts[1];
} tcsg_polygon, *tcsg_polygon_ptr;

tcsg_polygon* tcsg_polygon_new(tcsg_user_data i_ud, int i_count, const tcsg_vert* i_pnts);
tcsg_polygon* tcsg_polygon_new3(tcsg_user_data i_ud, const tcsg_vert* i_a, const tcsg_vert* i_b, const tcsg_vert* i_c);
tcsg_polygon* tcsg_polygon_new4(tcsg_user_data i_ud, const tcsg_vert* i_a, const tcsg_vert* i_b, const tcsg_vert* i_c, const tcsg_vert* i_d);
void tcsg_polygon_incref(tcsg_polygon* i_a);
void tcsg_polygon_decref(tcsg_polygon* i_a);
void tcsg_polygon_invert(tcsg_polygon* i_a);

typedef struct {
    int             max_vert_count;
    tcsg_polygon**  polys;
} tcsg_polygon_vector;

void tcsg_polygon_vector_pushback(tcsg_polygon_vector* i_self, tcsg_polygon* i_value);
void tcsg_polygon_vector_concat(tcsg_polygon_vector* i_self, tcsg_polygon_vector* i_rhs);
void tcsg_polygon_vector_free(tcsg_polygon_vector* i_self);

// Main CSG operations
tcsg_polygon_vector tcsg_union(tcsg_polygon_vector* i_a, tcsg_polygon_vector* i_b);
tcsg_polygon_vector tcsg_intersect(tcsg_polygon_vector* i_a, tcsg_polygon_vector* i_b);
tcsg_polygon_vector tcsg_subtract(tcsg_polygon_vector* i_a, tcsg_polygon_vector* i_b);
tcsg_polygon_vector tcsg_merge(tcsg_polygon_vector* i_a);
void tcsg_split(const tcsg_plane* i_p, tcsg_polygon_vector* i_list, tcsg_polygon_vector* o_front, tcsg_polygon_vector* o_back);

enum {
    tcsg_k_py,
    tcsg_k_ny,
    tcsg_k_px,
    tcsg_k_nx,
    tcsg_k_pz,
    tcsg_k_nz,
    tcsg_k_div_max
};
void tcsg_divide(tcsg_polygon_vector* i_list, tcsg_polygon_vector* o_dirs[tcsg_k_div_max]);

void tcsg_from_tris(tcsg_vert* i_verts, unsigned short* i_index, int i_tri_count, tcsg_user_data i_ud, tcsg_polygon_vector* o_v);
tcsg_polygon_vector tcsg_cube(tcsg_user_data i_ud, const tcsg_f3* i_cntr, const tcsg_f3* i_rad);


typedef struct {
    tcsg_user_data  m;
    int             offset;
    int             count;
} tcsg_draw;

typedef struct {
    tcsg_vert*      verts;
    unsigned short* indcies;
    tcsg_draw*      draws;
} tcsg_model;

tcsg_model tcsg_new_model(const tcsg_polygon_vector* i_list);
void tcsg_model_free(tcsg_model* i_self);

#ifdef TINY_CSG_IMPLEMENTATION
#include <stdlib.h>
#include <math.h>
#include <memory.h>
#include <malloc.h>

// Defines

#define tcsg_malloc(N)      malloc((N))
#define tcsg_realloc(P, N)  realloc((P), (N))
#define tcsg_free(P)        free((P))
#define tcsg__max(A, B)     (((A) > (B)) ? (A) : (B))


// Utils
//  A variation on the STB strechy buffer
//   Original from ...
// Changes:
//  * Defined a struct for the head (or control block)
//  * Renamed functions
//  * added beign and end to be more stl(ish)
typedef struct {
    int count;
    int capacity;
} tscg__vector_head;

static void tcsg__sbgrowf(void **arr, int increment, int itemsize);

#define tcsg__sb_head(P)            (((tscg__vector_head*)(P))-1)

#define tcsg__sb_free(a)            ((a) ? tcsg_free(tcsg__sb_head(a)),0 : 0)
#define tcsg__sb_pushback(a,v)      (stb__sbmaybegrow(a,1), (a)[stb__sbn(a)++] = (v))
#define tcsg__sb_count(a)           ((a) ? stb__sbn(a) : 0)
#define tcsg__sb_last(a)            ((a)[stb__sbn(a)-1])
#define tcsg__sb_reserve(a,n)       (stb__sbmaybegrow(a, n - tcsg__sb_count(a)))
#define tcsg__sb_reserve_extra(a,n) (stb__sbmaybegrow(a, n))

#define tcsg__sb_begin(a)           ((a))
#define tcsg__sb_end(a)             ((a) + tcsg__sb_count(a))
#define tcsg__sb_foreach(t, i, a)   for (t *i = tcsg__sb_begin(a), *i##e = tcsg__sb_end(a); i != i##e; ++i)

#define stb__sbm(a)                 (tcsg__sb_head(a)->capacity)
#define stb__sbn(a)                 (tcsg__sb_head(a)->count)

#define stb__sbneedgrow(a,n)        ((a)==0 || stb__sbn(a)+(n) >= stb__sbm(a))
#define stb__sbmaybegrow(a,n)       (stb__sbneedgrow(a,(n)) ? stb__sbgrow(a,n) : 0)
#define stb__sbgrow(a,n)            (tcsg__sbgrowf((void**)&(a), (n), sizeof(*(a))))

static void tcsg__sbgrowf(void **i_arr, int increment, int itemsize)
{
    void* arr = *i_arr;
    int dbl_cur = arr ? 2 * stb__sbm(arr) : 0;
    int min_needed = tcsg__sb_count(arr) + increment;
    int m = dbl_cur > min_needed ? dbl_cur : min_needed;
    tscg__vector_head* p = (tscg__vector_head*)tcsg_realloc(arr ? tcsg__sb_head(arr) : 0, itemsize * m + sizeof(tscg__vector_head));
    if (p) {
        if (!arr)
            p->count = 0;
        p->capacity = m;
        *i_arr = (p + 1);
        //return p + 1;
    }
    else {
#ifdef STRETCHY_BUFFER_OUT_OF_MEMORY
        STRETCHY_BUFFER_OUT_OF_MEMORY;
#endif
        *i_arr = 0;
        //return (void *)(sizeof(tscg__vector_head)); // try to force a NULL pointer exception later
    }
}




tcsg_f3 tcsg_f3_lerp(const tcsg_f3* i_0, const tcsg_f3* i_1, float i_0to1)
{
    tcsg_f3 o;
    o.x = i_0->x + (i_1->x - i_0->x) * i_0to1;
    o.y = i_0->y + (i_1->y - i_0->y) * i_0to1;
    o.z = i_0->z + (i_1->z - i_0->z) * i_0to1;
    return o;
}

tcsg_f3 tcsg_f3_cross(const tcsg_f3* i_a, const tcsg_f3* i_b)
{
    tcsg_f3 o;
    o.x = i_a->y * i_b->z - i_a->z * i_b->y;
    o.y = i_a->z * i_b->x - i_a->x * i_b->z;
    o.z = i_a->x * i_b->y - i_a->y * i_b->x;
    return o;
}

float   tcsg_f3_dot(const tcsg_f3* i_a, const tcsg_f3* i_b)
{
    return i_a->x * i_b->x + i_a->y * i_b->y + i_a->z * i_b->z;
}

tcsg_f3 tcsg_f3_normalise(const tcsg_f3* i_a)
{
    const float il = 1.0f / sqrtf(tcsg_f3_dot(i_a, i_a));
    tcsg_f3 o;
    o.x = i_a->x * il;
    o.y = i_a->y * il;
    o.z = i_a->z * il;
    return o;
}

tcsg_f3 tcsg_f3_invert(const tcsg_f3* i_a)
{
    tcsg_f3 o;
    o.x = -i_a->x;
    o.y = -i_a->y;
    o.z = -i_a->z;
    return o;
}

tcsg_f3 tcsg_f3_sub(const tcsg_f3* i_a, const tcsg_f3* i_b)
{
    tcsg_f3 o;
    o.x = i_a->x - i_b->x;
    o.y = i_a->y - i_b->y;
    o.z = i_a->z - i_b->z;
    return o;
}

tcsg_f3 tcsg_f3_scale(const tcsg_f3* i_a, float i_f)
{
    tcsg_f3 o;
    o.x = i_a->x * i_f;
    o.y = i_a->y * i_f;
    o.z = i_a->z * i_f;
    return o;
}

tcsg_f3 tcsg_f3_new(float i_x, float i_y, float i_z)
{
    tcsg_f3 o;
    o.x = i_x;
    o.y = i_y;
    o.z = i_z;
    return o;

}



tcsg_vert   tcsg_vert_lerp(const tcsg_vert* i_0, const tcsg_vert* i_1, float i_0to1)
{
    tcsg_vert o;
    o.position = tcsg_f3_lerp(&i_0->position, &i_1->position, i_0to1);
    o.normal = tcsg_f3_lerp(&i_0->normal, &i_1->normal, i_0to1);
    return o;
}

tcsg_f3     tcsg_vert_position(const tcsg_vert* i_a)
{
    return i_a->position;
}

tcsg_f3     tcsg_vert_normal(const tcsg_vert* i_a)
{
    return i_a->normal;
}


tcsg_plane tcsg_plane_new(const tcsg_vert* i_a, const tcsg_vert* i_b, const tcsg_vert* i_c)
{
    tcsg_plane o;
    const tcsg_f3 pa = tcsg_vert_position(i_a);
    const tcsg_f3 pb = tcsg_vert_position(i_b);
    const tcsg_f3 pc = tcsg_vert_position(i_c);
    const tcsg_f3 pb_a = tcsg_f3_sub(&pb, &pa);
    const tcsg_f3 pc_a = tcsg_f3_sub(&pc, &pa);
    const tcsg_f3 pC_baca = tcsg_f3_cross(&pb_a, &pc_a);
    const tcsg_f3 n = tcsg_f3_normalise(&pC_baca);
    o.normal = n;
    o.d = tcsg_f3_dot(&pa, &n);
    return o;
}

void tcsg_plane_invert(tcsg_plane* io_self)
{
    io_self->normal = tcsg_f3_invert(&io_self->normal);
    io_self->d = -io_self->d;
}




tcsg_polygon* tcsg_polygon_new(tcsg_user_data i_ud, int i_count, const tcsg_vert* i_pnts)
{
    tcsg_polygon* o = (tcsg_polygon*)tcsg_malloc(sizeof(tcsg_polygon) + sizeof(tcsg_vert) * (i_count - 1));
    o->ref_count = 0;
    o->count = i_count;
    if (i_pnts) {
        o->plane = tcsg_plane_new(i_pnts, i_pnts + 1, i_pnts + 2);
        o->user = i_ud;
        memcpy(o->verts, i_pnts, sizeof(tcsg_vert) * i_count);
    }
    return o;
}

tcsg_polygon* tcsg_polygon_new3(tcsg_user_data i_ud, const tcsg_vert* i_a, const tcsg_vert* i_b, const tcsg_vert* i_c)
{
    tcsg_polygon* o = (tcsg_polygon*)tcsg_malloc(sizeof(tcsg_polygon) + sizeof(tcsg_vert) * 2);
    o->ref_count = 3;
    o->plane = tcsg_plane_new(i_a, i_b, i_c);
    o->user = i_ud;
    o->verts[0] = *i_a;
    o->verts[1] = *i_b;
    o->verts[2] = *i_c;
    return o;
}

tcsg_polygon* tcsg_polygon_new4(tcsg_user_data i_ud, const tcsg_vert* i_a, const tcsg_vert* i_b, const tcsg_vert* i_c, const tcsg_vert* i_d)
{
    tcsg_polygon* o = (tcsg_polygon*)tcsg_malloc(sizeof(tcsg_polygon) + sizeof(tcsg_vert) * 3);
    o->ref_count = 4;
    o->plane = tcsg_plane_new(i_a, i_b, i_c);
    o->user = i_ud;
    o->verts[0] = *i_a;
    o->verts[1] = *i_b;
    o->verts[2] = *i_c;
    o->verts[3] = *i_d;
    return o;
}

void tcsg_polygon_incref(tcsg_polygon* i_a)
{
    i_a->ref_count++;
}
void tcsg_polygon_decref(tcsg_polygon* i_a)
{
    if (0 == --i_a->ref_count) {
        tcsg_free(i_a);
    }
}

void tcsg_polygon_invert(tcsg_polygon* i_a)
{
    tcsg_vert* s = i_a->verts;
    tcsg_vert* e = i_a->verts + i_a->count - 1;
    for (; s < e; ++s, --e) {
        tcsg_vert t = *s;
        *s = *e;
        *e = t;

        s->normal = tcsg_f3_invert(&s->normal);
        e->normal = tcsg_f3_invert(&e->normal);
    }
    tcsg_plane_invert(&i_a->plane);
}

#define tcsg_alloca(N)  _malloca((N))
#define tcsg_freea(P)   _freea((P))

void tcsg__split(const tcsg_plane* i_p, tcsg_polygon** i_begin, tcsg_polygon** i_end, int i_max_vert_count, tcsg_polygon_vector* o_front, tcsg_polygon_vector* o_co_front, tcsg_polygon_vector* o_co_back, tcsg_polygon_vector* o_back)
{
    tcsg_plane_test* tests = (tcsg_plane_test*)tcsg_alloca(i_max_vert_count * sizeof(tcsg_plane_test));

    // foreach polygons
    for (tcsg_polygon** pi = i_begin; pi != i_end; ++pi) {
        int all_tests = 0;

        // foreach vert in the poly
        {
            tcsg_plane_test* ti = tests;
            for (const tcsg_vert* vi = (*pi)->verts, *ve = vi + (*pi)->count; vi != ve; ++vi, ++ti) {
                const tcsg_f3 vp = tcsg_vert_position(vi);
                const float t = tcsg_f3_dot(&i_p->normal, &vp) - i_p->d;
                all_tests |= (*ti = (t < -tcsg_k_epsilon) ? tcsg_k_back : ((t > tcsg_k_epsilon) ? tcsg_k_front : tcsg_k_co_plane));
            }
        }

        switch ((tcsg_plane_test)all_tests)
        {
        case tcsg_k_co_plane: {
            tcsg_polygon_vector* o = ((o_co_front == o_co_back) || (tcsg_f3_dot(&i_p->normal, &(*pi)->plane.normal) > 0.0f)) ? o_co_front : o_co_back;
            tcsg_polygon_vector_pushback(o, *pi);
        } break;
        case tcsg_k_front: tcsg_polygon_vector_pushback(o_front, *pi); break;
        case tcsg_k_back:  tcsg_polygon_vector_pushback(o_back, *pi); break;
        case tcsg_k_spanning: {
            tcsg_polygon* nf = tcsg_polygon_new((*pi)->user, (*pi)->count + 1, NULL);
            tcsg_polygon* nb = tcsg_polygon_new((*pi)->user, (*pi)->count + 1, NULL);
            tcsg_vert* vf = nf->verts;
            tcsg_vert* vb = nb->verts;
            const tcsg_plane_test* ti = tests;

            for (int i = 0; i < (*pi)->count; ++i) {
                int j = (i + 1) % (*pi)->count;

                if (ti[i] != tcsg_k_back) {
                    *vf++ = (*pi)->verts[i];
                }
                if (ti[i] != tcsg_k_front) {
                    *vb++ = (*pi)->verts[i];
                }
                if ((ti[i] | ti[j]) == tcsg_k_spanning) {
                    const tcsg_f3 vpi = tcsg_vert_position((*pi)->verts + i);
                    const tcsg_f3 vpj = tcsg_vert_position((*pi)->verts + j);
                    const tcsg_f3 vpj_vpi = tcsg_f3_sub(&vpj, &vpi);
                    const float dn_vpi = tcsg_f3_dot(&i_p->normal, &vpi);
                    const float dn_vpj_vpi = tcsg_f3_dot(&i_p->normal, &vpj_vpi);
                    const float t = (i_p->d - dn_vpi) / dn_vpj_vpi;
                    const tcsg_vert nv = tcsg_vert_lerp((*pi)->verts + i, (*pi)->verts + j, t);
                    *vf++ = nv;
                    *vb++ = nv;
                }
            }

            nf->count = vf - nf->verts;
            nf->plane = (*pi)->plane;
            nf->user = (*pi)->user;

            nb->count = vb - nb->verts;
            nb->plane = (*pi)->plane;
            nb->user = (*pi)->user;

            tcsg_polygon_vector_pushback(o_front, nf);
            tcsg_polygon_vector_pushback(o_back, nb);
        } break;
        }
    }

    tcsg_freea(tests);
}

void tcsg_polygon_vector_pushback(tcsg_polygon_vector* i_self, tcsg_polygon* i_value)
{
    tcsg_polygon_incref(i_value);
    i_self->max_vert_count = tcsg__max(i_self->max_vert_count, i_value->count);
    tcsg__sb_pushback(i_self->polys, i_value);
}

void tcsg_polygon_vector_remove(tcsg_polygon_vector* i_self, tcsg_polygon* i_value)
{
    for (tcsg_polygon** pi = tcsg__sb_begin(i_self->polys), **pe = tcsg__sb_end(i_self->polys); pi != pe; ++pi) {
        if (*pi == i_value) {
            tcsg_polygon_decref(i_value);
            *pi = *(pe - 1);
            stb__sbn(i_self->polys) -= 1;
        }
    }
}

// Adds rhs to the left
void tcsg_polygon_vector_concat(tcsg_polygon_vector* i_lhs, tcsg_polygon_vector* i_rhs)
{
    i_lhs->max_vert_count = tcsg__max(i_lhs->max_vert_count, i_rhs->max_vert_count);
    tcsg__sb_reserve(i_lhs->polys, tcsg__sb_count(i_lhs->polys) + tcsg__sb_count(i_rhs->polys));
    for (tcsg_polygon** pi = tcsg__sb_begin(i_rhs->polys), **pe = tcsg__sb_end(i_rhs->polys); pi != pe; ++pi) {
        tcsg_polygon_incref(*pi);
        tcsg__sb_pushback(i_lhs->polys, *pi);
    }
}

void tcsg_polygon_vector_invert(tcsg_polygon_vector* i_list)
{
    for (tcsg_polygon** pi = tcsg__sb_begin(i_list->polys), **pe = tcsg__sb_end(i_list->polys); pi != pe; ++pi) {
        tcsg_polygon_invert(*pi);
    }
}

void tcsg_polygon_vector_free(tcsg_polygon_vector* i_list)
{
    for (tcsg_polygon** pi = tcsg__sb_begin(i_list->polys), **pe = tcsg__sb_end(i_list->polys); pi != pe; ++pi) {
        tcsg_polygon_decref(*pi);
    }
    tcsg__sb_free(i_list->polys);
}

typedef struct {
    tcsg_plane  p;
    int         front;
    int         back;
} tcsg__bsp_node;

typedef struct {
    tcsg__bsp_node* nodes;
} tcsg__bsp;

int tcsg__bsp_pushback(tcsg__bsp* i_self, const tcsg_plane* i_p)
{
    int i = tcsg__sb_count(i_self->nodes);
    tcsg__bsp_node n = { 0 };
    n.p = (*i_p);
    tcsg__sb_pushback(i_self->nodes, n);

    return i;
}

int tcsg__build_bsp_r(tcsg__bsp* i_self, tcsg_polygon_vector* i_list)
{
    const tcsg_plane p = i_list->polys[0]->plane;
    const int n = tcsg__bsp_pushback(i_self, &p);
    int fn = 0, bn = 0;

    if (tcsg__sb_count(i_list->polys) > 1) {
        tcsg_polygon_vector f = { 0 };
        tcsg_polygon_vector b = { 0 };
        tcsg_polygon_vector c = { 0 };

        tcsg__split(&p, tcsg__sb_begin(i_list->polys)+1, tcsg__sb_end(i_list->polys), i_list->max_vert_count, &f, &c, &c, &b);

        if (tcsg__sb_count(f.polys)) {
            fn = tcsg__build_bsp_r(i_self, &f);
        }
        if (tcsg__sb_count(b.polys)) {
            bn = tcsg__build_bsp_r(i_self, &b);
        }

        tcsg_polygon_vector_free(&f);
        tcsg_polygon_vector_free(&b);
        tcsg_polygon_vector_free(&c);
    }

    // populate the node we created for here
    // but only after the tree has been allocated.
    {
        tcsg__bsp_node* node = i_self->nodes + n;
        node->front = fn;
        node->back = bn;
    }

    return n;
}

tcsg__bsp tcsg__build_bsp(tcsg_polygon_vector* i_list)
{
    tcsg__bsp o = { 0 };
    tcsg__sb_reserve(o.nodes, tcsg__sb_count(i_list->polys));
    tcsg__build_bsp_r(&o, i_list);
    return o;
}

void tcsg__bsp_free(tcsg__bsp* i_self)
{
    tcsg_free(i_self->nodes);
}

typedef enum {
    tcsg__k_outside,
    tcsg__k_inside,
    tcsg__k_force_clip_method_int = 0x7fffffff
} tcsg__clip_method;

tcsg_polygon_vector tcsg__clip(tcsg__clip_method i_inside, tcsg__bsp* i_bsp, int i_node, tcsg_polygon_vector* i_list)
{
    tcsg_polygon_vector o = { 0 };

    if (tcsg__sb_count(i_list->polys)) {
        tcsg_polygon_vector f = { 0 };
        tcsg_polygon_vector b = { 0 };
        tcsg__bsp_node* node = i_bsp->nodes + i_node;

        tcsg__split(&node->p, tcsg__sb_begin(i_list->polys), tcsg__sb_end(i_list->polys), i_list->max_vert_count, &f, &f, &b, &b);

        if (node->front) {
            tcsg_polygon_vector t = tcsg__clip(i_inside, i_bsp, node->front, &f);
            tcsg_polygon_vector_concat(&o, &t);
            tcsg_polygon_vector_free(&t);
        }
        else if (!i_inside) {
            tcsg_polygon_vector_concat(&o, &f);
        }

        if (node->back) {
            tcsg_polygon_vector t = tcsg__clip(i_inside, i_bsp, node->back, &b);
            tcsg_polygon_vector_concat(&o, &t);
            tcsg_polygon_vector_free(&t);
        }
        else if (i_inside) {
            tcsg_polygon_vector_concat(&o, &b);
        }

        tcsg_polygon_vector_free(&f);
        tcsg_polygon_vector_free(&b);
    }

    return o;
}

tcsg_polygon_vector tcsg_union(tcsg_polygon_vector* i_a, tcsg_polygon_vector* i_b)
{
    tcsg__bsp ba = tcsg__build_bsp(i_a);
    tcsg__bsp bb = tcsg__build_bsp(i_b);;
    tcsg_polygon_vector aob = tcsg__clip(tcsg__k_outside, &bb, 0, i_a);
    tcsg_polygon_vector boa = tcsg__clip(tcsg__k_outside, &ba, 0, i_b);

    tcsg_polygon_vector_concat(&aob, &boa);
    tcsg_polygon_vector_free(&boa);

    return aob;
}

tcsg_polygon_vector tcsg_intersect(tcsg_polygon_vector* i_a, tcsg_polygon_vector* i_b)
{
    tcsg__bsp ba = tcsg__build_bsp(i_a);
    tcsg__bsp bb = tcsg__build_bsp(i_b);;
    tcsg_polygon_vector aib = tcsg__clip(tcsg__k_inside, &bb, 0, i_a);
    tcsg_polygon_vector bia = tcsg__clip(tcsg__k_inside, &ba, 0, i_b);

    tcsg_polygon_vector_concat(&aib, &bia);
    tcsg_polygon_vector_free(&bia);

    return aib;
}

tcsg_polygon_vector tcsg_subtract(tcsg_polygon_vector* i_a, tcsg_polygon_vector* i_b)
{
    tcsg__bsp ba = tcsg__build_bsp(i_a);
    tcsg__bsp bb = tcsg__build_bsp(i_b);
    tcsg_polygon_vector aob = tcsg__clip(tcsg__k_outside, &bb, 0, i_a);
    tcsg_polygon_vector bia = tcsg__clip(tcsg__k_inside, &ba, 0, i_b);

    tcsg_polygon_vector_invert(&bia);
    tcsg_polygon_vector_concat(&aob, &bia);
    tcsg_polygon_vector_free(&bia);

    return aob;
}

void tcsg_split(const tcsg_plane* i_p, tcsg_polygon_vector* i_list, tcsg_polygon_vector* o_front, tcsg_polygon_vector* o_back)
{
    tcsg_polygon_vector f = { 0 };
    tcsg_polygon_vector b = { 0 };

    if (!o_front) {
        o_front = &f;
    }
    if (!o_back) {
        o_back = &b;
    }

    tcsg__split(i_p, tcsg__sb_begin(i_list->polys), tcsg__sb_end(i_list->polys), i_list->max_vert_count, o_front, o_front, o_back, o_back);

    if (o_front == &f) {
        tcsg_polygon_vector_free(&f);
    }
    if (o_back == &b) {
        tcsg_polygon_vector_free(&b);
    }
}

void tcsg_divide(tcsg_polygon_vector* i_list, tcsg_polygon_vector* o_dirs[tcsg_k_div_max])
{
    const tcsg_f3 data[tcsg_k_div_max] = {
        { 0, 1, 0 },
        { 0, -1, 0 },
        { 1, 0, 0 },
        { -1, 0, 0 },
        { 0, 0, 1 },
        { 0, 0, -1 },
    };
    const float k_45deg = 0.4f;

    for (tcsg_polygon** pi = tcsg__sb_begin(i_list->polys), **pe = tcsg__sb_end(i_list->polys); pi != pe; ++pi) {
        for (int i = 0; i < tcsg_k_div_max; ++i) {
            const float dp = tcsg_f3_dot(data + i, &(*pi)->plane.normal);
            if (k_45deg < dp) {
                if (o_dirs[i]) {
                    tcsg_polygon_vector_pushback(o_dirs[i], *pi);
                    break;
                }
            }
        }
    }
}

#include <stdlib.h>

int cmpfloats(const float* i_a, const float * i_b, size_t i_n)
{
    do {
        const float d = *i_a++ - *i_b++;
        if (fabsf(d) > tcsg_k_epsilon)
            return d < 0 ? -1 : 1;

    } while (--i_n);

    return 0;
}

int tcsg__plane_sort_cmp(const void* i_a, const void* i_b)
{
    const float* a = (const float*)i_a;
    const float* b = (const float*)i_b;
    return cmpfloats(a, b, 4);
}

int tcsg__poly_sort_cmp(const void* i_a, const void* i_b)
{
    const tcsg_polygon* a = *(const tcsg_polygon**)i_a;
    const tcsg_polygon* b = *(const tcsg_polygon**)i_b;
    return tcsg__plane_sort_cmp(&a->plane, &b->plane);
}

typedef struct {
    tcsg_vert a, b;
} tcsg__edge;

typedef struct {
    tcsg_plane p;
    tcsg_polygon** list;
} tcsg__plane_key;

typedef struct {
    tcsg__edge e;
    int        ei;
    tcsg_polygon* poly;
} tcsg__edge_key;

tcsg_polygon_vector tcsg_merge(tcsg_polygon_vector* i_list)
{
    tcsg_polygon_vector o = { 0 };

    /*
    ---------
    -   - B -
    - A -----
    -   - C -
    ---------

    should, merge B and C => D
    then merge A and D => E

    ---------
    -   -   -
    - A - D -
    -   -   -
    ---------

    ---------
    -       -
    -   E   -
    -       -
    ---------

    */

    tcsg__plane_key * groups = 0;
    
    // group by plane.. if they dont share a plane, the cant be merged. material can be added her as well.
    for (tcsg_polygon** pi = tcsg__sb_begin(i_list->polys), **pe = tcsg__sb_end(i_list->polys); pi != pe; ++pi) {
        tcsg__plane_key e;
        e.p = (*pi)->plane;
        e.list = 0;

        for (tcsg__plane_key* ei = tcsg__sb_begin(groups), *ee = tcsg__sb_end(groups); ei != ee; ++ei) {
            if (0 == tcsg__plane_sort_cmp(&ei->p, &e)
            && (ei->list[0]->user.p == (*pi)->user.p)) {
                tcsg__sb_pushback(ei->list, *pi);
                goto next_poly;
            }
        }
        tcsg__sb_pushback(e.list, *pi);
        tcsg__sb_pushback(groups, e);

        next_poly:{}
    }

//    printf("groups %d\n", tcsg__sb_count(groups));

    for (tcsg__plane_key* ei = tcsg__sb_begin(groups), *ee = tcsg__sb_end(groups); ei != ee; ++ei) {
        if (tcsg__sb_count(ei->list) == 1) {
            // only 1 poly, on this plane.. push it to the result.
            tcsg_polygon_vector_pushback(&o, ei->list[0]);
        } else {
            tcsg__edge_key* edges = 0;
            // make all edges clockwise.
            for (tcsg_polygon** pi = tcsg__sb_begin(ei->list), **pe = tcsg__sb_end(ei->list); pi != pe; ++pi) {
                tcsg_polygon* p = *pi;
                for (int i = 0; i < p->count; ++i) {
                    tcsg__edge_key e;
                    e.e.a = p->verts[i];
                    e.e.b = p->verts[(i + 1) % p->count];
                    e.ei = i;
                    e.poly = p;
                    tcsg__sb_pushback(edges, e);
                }
            }

            // search anti clockwise
            tcsg_polygon** merged = 0;
            for (tcsg_polygon** pi = tcsg__sb_begin(ei->list), **pe = tcsg__sb_end(ei->list); pi != pe; ++pi) {
                tcsg_polygon* p = *pi;

                for (tcsg_polygon** mi = tcsg__sb_begin(merged), **me = tcsg__sb_end(merged); mi != me; ++mi) {
                    if (p == *mi) {
                        p = 0;
                        break;
                    }
                }

                if (!p)
                    continue;

                test_p:
                for (int i = 0; i < p->count; ++i) {
                    tcsg__edge_key* edge = 0;
                    tcsg__edge_key e;
                    e.e.a = p->verts[(i + 1) % p->count];
                    e.e.b = p->verts[i];


                    // find an edge
                    for (tcsg__edge_key* ei = tcsg__sb_begin(edges), *ee = tcsg__sb_end(edges); ei != ee; ++ei) {
                        if (ei->poly && 0 == memcmp(&ei->e, &e.e, sizeof(e.e))) {
                            edge = ei;
                            break;
                        }
                    }

                    if (!edge)
                        continue;

                    {
                        // this is an edge that can be collapsed.. maybe.
                        tcsg_polygon* A = edge->poly;
                        tcsg_polygon* B = p;

                        // calc the vectors eading to and from this edge.
                        // if they continue each other, then we can merge.
                        const tcsg_f3 a1 = tcsg_vert_position(A->verts + ((edge->ei + A->count - 1) % A->count)); // + count so I dont have to deal with negitive numbers
                        const tcsg_f3 a2 = tcsg_vert_position(A->verts + edge->ei);
                        const tcsg_f3 a3 = tcsg_vert_position(A->verts + ((edge->ei + 1) % A->count));
                        const tcsg_f3 a4 = tcsg_vert_position(A->verts + ((edge->ei + 2) % A->count));
                        const tcsg_f3 a1_2 = tcsg_f3_sub(&a2, &a1);
                        const tcsg_f3 a3_4 = tcsg_f3_sub(&a4, &a3);

                        const tcsg_f3 b4 = tcsg_vert_position(B->verts + ((i + B->count - 1) % B->count)); // + count so I dont have to deal with negitive numbers
                        const tcsg_f3 b3 = tcsg_vert_position(B->verts + i); // a2
                        const tcsg_f3 b2 = tcsg_vert_position(B->verts + ((i + 1) % B->count)); // a3
                        const tcsg_f3 b1 = tcsg_vert_position(B->verts + ((i + 2) % B->count));
                        const tcsg_f3 b1_2 = tcsg_f3_sub(&b2, &b1);
                        const tcsg_f3 b3_4 = tcsg_f3_sub(&b4, &b3);

                        const float d1 = 1.f + tcsg_f3_dot(&a1_2, &b1_2);
                        const float d2 = 1.f + tcsg_f3_dot(&a3_4, &b3_4);

                        if ((d1 > -tcsg_k_epsilon) && (d1 < tcsg_k_epsilon)
                        && (d2 > -tcsg_k_epsilon) && (d2 < tcsg_k_epsilon)) {
                            // merge A and B
                            tcsg_polygon* C = tcsg_polygon_new(A->user, A->count + B->count - 4, NULL);
                            int vi = 0;

                            int as = ((edge->ei + 2) % A->count);
                            int bs = ((i + 2) % B->count);

                            C->plane = A->plane;
                            C->user = A->user;

                            /*
                            -----
                            - A -
                            0 ----- ei
                            - B -
                            -----
                            */

                            for (int j = 0; j < A->count - 2; ++j) {
                                C->verts[vi++] = A->verts[(as + j) % A->count];
                            }
                            for (int j = 0; j < B->count - 2; ++j) {
                                C->verts[vi++] = B->verts[(bs + j) % B->count];
                            }

                            for (tcsg__edge_key* ei = tcsg__sb_begin(edges), *ee = tcsg__sb_end(edges); ei != ee; ++ei)
                            {
                                if (ei->poly == A) ei->poly = NULL;
                                if (ei->poly == B) ei->poly = NULL;
                            }

                            tcsg__sb_pushback(merged, A);
                            tcsg__sb_pushback(merged, B);

                            tcsg_polygon_vector_remove(&o, A);
                            tcsg_polygon_vector_remove(&o, B);

                            p = C;
                            // TODO: should add the edges of C
                            goto test_p;
                        }
                    }

                }
                tcsg_polygon_vector_pushback(&o, p);
            }

            tcsg__sb_free(merged); merged = 0;
        }
    }

    // free memory
    for (tcsg__plane_key* ei = tcsg__sb_begin(groups), *ee = tcsg__sb_end(groups); ei != ee; ++ei) {
        tcsg__sb_free(ei->list);
    }
    tcsg__sb_free(groups);

    return o;
}


void tcsg_from_tris(tcsg_vert* i_verts, unsigned short* i_index, int i_tri_count, tcsg_user_data i_ud, tcsg_polygon_vector* o_v)
{
    tcsg__sb_reserve_extra(o_v->polys, i_tri_count);
    o_v->max_vert_count = tcsg__max(o_v->max_vert_count, 3);
    for (int i = 0; i < i_tri_count; ++i, i_index += 3) {
        tcsg_polygon_vector_pushback(o_v, tcsg_polygon_new3(i_ud, i_verts + i_index[0], i_verts + i_index[1], i_verts + i_index[2]));
    }
}

tcsg_polygon_vector tcsg_cube(tcsg_user_data i_ud, const tcsg_f3* i_cntr, const tcsg_f3* i_rad)
{
    tcsg_polygon_vector o = { 0 };
    tcsg__sb_reserve(o.polys, 6);

    struct {
        int flags[4];
        tcsg_f3 normal;
    } const data[6] = {
        {{0, 4, 6, 2}, {-1,  0,  0}},
        {{1, 3, 7, 5}, { 1,  0,  0}},
        {{0, 1, 5, 4}, { 0, -1,  0}},
        {{2, 6, 7, 3}, { 0,  1,  0}},
        {{0, 2, 3, 1}, { 0,  0, -1}},
        {{4, 5, 7, 6}, { 0,  0,  1}}
    };

    const tcsg_f3 zero = { 0 };
    const tcsg_f3 one = { 1.f, 1.f, 1.f };

    if (!i_cntr) i_cntr = &zero;
    if (!i_rad) i_rad = &one;

    for (int x = 0; x < 6; x++)
    {
        const int* v = data[x].flags;
        const tcsg_f3 n = data[x].normal;

        tcsg_vert vx[4];
        for (int i = 0; i < 4; i++)
        {
            tcsg_f3 p = tcsg_f3_new(
                i_cntr->x + (i_rad->x * (2 * (((v[i] & 1) > 0) ? 1 : 0) - 1)),
                i_cntr->y + (i_rad->y * (2 * (((v[i] & 2) > 0) ? 1 : 0) - 1)),
                i_cntr->z + (i_rad->z * (2 * (((v[i] & 4) > 0) ? 1 : 0) - 1))
            );
            vx[i].position = p;
            vx[i].normal = n;
        }
        tcsg_polygon_vector_pushback(&o, tcsg_polygon_new(i_ud, 4, vx));
    }
    return o;
}

unsigned short tcsg__add_unqiue_vert(tcsg_model* io_m, const tcsg_vert* i_v)
{
    for (tcsg_vert* i = tcsg__sb_begin(io_m->verts), *e = tcsg__sb_end(io_m->verts); i != e; ++i) {
        if (0 == memcmp(i, i_v, sizeof(*i_v))) {
            return i - tcsg__sb_begin(io_m->verts);
        }
    }

    {
        unsigned short o = tcsg__sb_count(io_m->verts);
        tcsg__sb_pushback(io_m->verts, *i_v);
        return o;
    }
}

int tcsg__poly_sort_user_cmp(const void* i_a, const void* i_b)
{
    const tcsg_polygon* a = *(const tcsg_polygon**)i_a;
    const tcsg_polygon* b = *(const tcsg_polygon**)i_b;
    return (a->user.p < b->user.p) - (b->user.p < a->user.p);
}

tcsg_model tcsg_new_model(const tcsg_polygon_vector* i_list)
{
    tcsg_model o = { 0 };
    tcsg_draw d = { 0 };

    qsort((void*)i_list->polys, tcsg__sb_count(i_list->polys), sizeof(tcsg_polygon*), tcsg__poly_sort_user_cmp);
    d.m = i_list->polys[0]->user;
    for (tcsg_polygon ** i = tcsg__sb_begin(i_list->polys), ** e = tcsg__sb_end(i_list->polys); i != e; ++i) {

        if (d.m.p != (*i)->user.p) {
            int c = tcsg__sb_count(o.indcies);
            d.count = c - d.offset;
            tcsg__sb_pushback(o.draws, d);
            d.offset = c;
            d.m = (*i)->user;
        }

        {
            tcsg_polygon * p = *i;
            unsigned short a = tcsg__add_unqiue_vert(&o, p->verts + 0);
            unsigned short l = tcsg__add_unqiue_vert(&o, p->verts + 1);

            for (tcsg_vert* v = p->verts + 2, *ve = p->verts + p->count; v != ve; ++v) {
                unsigned short j = tcsg__add_unqiue_vert(&o, v);

                tcsg__sb_reserve_extra(o.indcies, 3);
                tcsg__sb_pushback(o.indcies, a);
                tcsg__sb_pushback(o.indcies, l);
                tcsg__sb_pushback(o.indcies, j);
                l = j;
            }
        }
    }
    d.count = tcsg__sb_count(o.indcies) - d.offset;
    tcsg__sb_pushback(o.draws, d);

    return o;
}

void tcsg_model_free(tcsg_model* i_self)
{
    tcsg__sb_free(i_self->verts);
    tcsg__sb_free(i_self->indcies);
    tcsg__sb_free(i_self->draws);
}

#endif//TINY_CSG_IMPLEMENTATION

#ifdef TINY_CSG_TEST

int main(int i_argc, char** i_argv)
{
    const tcsg_f3 one = { 1.f, 1.f, 1.f };
    tcsg_user_data m = { 0 };
    tcsg_polygon_vector a = tcsg_cube(m, NULL, NULL);
    tcsg_polygon_vector b = tcsg_cube(m, &one, NULL);

    tcsg_polygon_vector aUb = tcsg_union(&a, &b);
    tcsg_polygon_vector aIb = tcsg_intersect(&a, &b);
    tcsg_polygon_vector aSb = tcsg_subtract(&a, &b);

    {
        tcsg_polygon_vector _aUb = { 0 };
        tcsg_polygon_vector _aIb = { 0 };
        tcsg_polygon_vector _aSb = { 0 };

        tcsg_polygon_vector* t[tcsg_k_div_max] = { 0 };
        t[0] = &_aUb;
        tcsg_divide(&aUb, t);

        t[0] = &_aIb;
        tcsg_divide(&aIb, t);

        t[0] = &_aSb;
        tcsg_divide(&aSb, t);

        tcsg_polygon_vector_free(&_aUb);
        tcsg_polygon_vector_free(&_aIb);
        tcsg_polygon_vector_free(&_aSb);
    }

    {
        tcsg_polygon_vector aUb_m = tcsg_merge(&aUb);
        tcsg_polygon_vector_free(&aUb_m);
    }

    tcsg_polygon_vector_free(&a);
    tcsg_polygon_vector_free(&b);
    tcsg_polygon_vector_free(&aUb);
    tcsg_polygon_vector_free(&aIb);
    tcsg_polygon_vector_free(&aSb);

    return 0;
}

#endif//TINY_CSG_TEST

#ifdef TINY_CSG_GLDEMO

int g_wire_frame = 0;
int g_model = 0;

#if defined(_WIN32) || defined(__WIN32__) || defined(__WINDOWS__)
    #define WIN32_LEAN_AND_MEAN
    #define WIN32_EXTRA_LEAN
    #include <windows.h>
    #include <mmsystem.h>
    #include <GL/gl.h>
    #include <GL/glu.h>

#pragma comment(lib, "opengl32.lib")
#pragma comment(lib, "glu32.lib")

typedef struct
{
    //---------------
    HINSTANCE   hInstance;
    HDC         hDC;
    HGLRC       hRC;
    HWND        hWnd;
    //---------------
    int         full;
    //---------------
    char        wndclass[4];	// window class and title :)
    //---------------
} tcsg__wininfo;

static const PIXELFORMATDESCRIPTOR pfd =
{
    sizeof(PIXELFORMATDESCRIPTOR),
    1,
    PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER,
    PFD_TYPE_RGBA,
    32,
    0, 0, 0, 0, 0, 0, 8, 0,
    0, 0, 0, 0, 0,  // accum
    32,             // zbuffer
    0,              // stencil!
    0,              // aux
    PFD_MAIN_PLANE,
    0, 0, 0, 0
};

static tcsg__wininfo wininfo = { 0, 0, 0, 0, 0,
{ 'i', 'q', '_', 0 }
};

#define XRES 800
#define YRES 600

static LRESULT CALLBACK WndProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
    // salvapantallas
    if (uMsg == WM_SYSCOMMAND && (wParam == SC_SCREENSAVE || wParam == SC_MONITORPOWER))
        return(0);

    // boton x o pulsacion de escape
    if (uMsg == WM_CLOSE || uMsg == WM_DESTROY || (uMsg == WM_KEYDOWN && wParam == VK_ESCAPE))
    {
        PostQuitMessage(0);
        return(0);
    }

    if (uMsg == WM_CHAR)
    {
        switch (wParam) {
            default: break;
            case VK_ESCAPE:
                PostQuitMessage(0);
                return(0);
            case 'W':
            case 'w':
                g_wire_frame = !g_wire_frame;
                break;
            case '1': case '2': case '3': case '4': case '5':
                g_model = wParam - '1';
                break;
        }
        
    }

    return(DefWindowProc(hWnd, uMsg, wParam, lParam));
}

int tscg_platform_init()
{
    unsigned int	PixelFormat;
    DWORD			dwExStyle, dwStyle;
    DEVMODE			dmScreenSettings;
    RECT			rec;

    WNDCLASS		wc;

    wininfo.hInstance = GetModuleHandle(0);

    ZeroMemory(&wc, sizeof(WNDCLASS));
    wc.style = CS_OWNDC | CS_HREDRAW | CS_VREDRAW;
    wc.lpfnWndProc = WndProc;
    wc.hInstance = wininfo.hInstance;
    wc.lpszClassName = wininfo.wndclass;

    if (!RegisterClass(&wc))
        return(0);

    if (wininfo.full)
    {
        dmScreenSettings.dmSize = sizeof(DEVMODE);
        dmScreenSettings.dmFields = DM_BITSPERPEL | DM_PELSWIDTH | DM_PELSHEIGHT;
        dmScreenSettings.dmBitsPerPel = 32;
        dmScreenSettings.dmPelsWidth = XRES;
        dmScreenSettings.dmPelsHeight = YRES;
        if (ChangeDisplaySettings(&dmScreenSettings, CDS_FULLSCREEN) != DISP_CHANGE_SUCCESSFUL)
            return(0);
        dwExStyle = WS_EX_APPWINDOW;
        dwStyle = WS_VISIBLE | WS_POPUP;// | WS_CLIPSIBLINGS | WS_CLIPCHILDREN;
        ShowCursor(0);
    }
    else
    {
        dwExStyle = WS_EX_APPWINDOW | WS_EX_WINDOWEDGE;
        dwStyle = WS_VISIBLE | WS_CAPTION | WS_CLIPSIBLINGS | WS_CLIPCHILDREN | WS_SYSMENU;
    }

    rec.left = 0;
    rec.top = 0;
    rec.right = XRES;
    rec.bottom = YRES;
    AdjustWindowRect(&rec, dwStyle, 0);

    wininfo.hWnd = CreateWindowEx(dwExStyle, wc.lpszClassName, "avada kedabra!", dwStyle,
        (GetSystemMetrics(SM_CXSCREEN) - rec.right + rec.left) >> 1,
        (GetSystemMetrics(SM_CYSCREEN) - rec.bottom + rec.top) >> 1,
        rec.right - rec.left, rec.bottom - rec.top, 0, 0, wininfo.hInstance, 0);
    if (!wininfo.hWnd)
        return(0);

    if (!(wininfo.hDC = GetDC(wininfo.hWnd)))
        return(0);

    if (!(PixelFormat = ChoosePixelFormat(wininfo.hDC, &pfd)))
        return(0);

    if (!SetPixelFormat(wininfo.hDC, PixelFormat, &pfd))
        return(0);

    if (!(wininfo.hRC = wglCreateContext(wininfo.hDC)))
        return(0);

    if (!wglMakeCurrent(wininfo.hDC, wininfo.hRC))
        return(0);

    return(1);
}
int tscg_platform_tick()
{
    int done = 0;
    MSG         msg;

    while (PeekMessage(&msg, 0, 0, 0, PM_REMOVE))
    {
        if (msg.message == WM_QUIT) done = 1;
        TranslateMessage(&msg);
        DispatchMessage(&msg);
    }

    SwapBuffers(wininfo.hDC);

    return !done;
}
void tscg_platform_shutdown()
{
    if (wininfo.hRC)
    {
        wglMakeCurrent(0, 0);
        wglDeleteContext(wininfo.hRC);
    }

    if (wininfo.hDC) ReleaseDC(wininfo.hWnd, wininfo.hDC);
    if (wininfo.hWnd) DestroyWindow(wininfo.hWnd);

    UnregisterClass(wininfo.wndclass, wininfo.hInstance);

    if (wininfo.full)
    {
        ChangeDisplaySettings(0, 0);
        ShowCursor(1);
    }
}

#endif

tcsg_polygon_vector colourize(tcsg_polygon_vector i_n, int i_offset)
{
    tcsg__sb_foreach(tcsg_polygon_ptr, i, i_n.polys) {
        (*i)->user.i = i_offset++;
    }
    return i_n;
}

int main(int i_argc, char** i_argv)
{
    const tcsg_f3 one = { 1.f, 1.f, 1.f };
    tcsg_user_data m = { 0 };
    tcsg_polygon_vector a = colourize(tcsg_cube(m, NULL, NULL), 0);
    tcsg_polygon_vector b = colourize(tcsg_cube(m, &one, NULL), 6);

    tcsg_polygon_vector aUb = tcsg_union(&a, &b);
    tcsg_polygon_vector aIb = tcsg_intersect(&a, &b);
    tcsg_polygon_vector aSb = tcsg_subtract(&a, &b);
    tcsg_polygon_vector aSbM = tcsg_merge(&aSb);

    tcsg_model am = tcsg_new_model(&a);
    tcsg_model aUbm = tcsg_new_model(&aUb);
    tcsg_model aIbm = tcsg_new_model(&aIb);
    tcsg_model aSbm = tcsg_new_model(&aSb);
    tcsg_model aSbMm = tcsg_new_model(&aSbM);

    tcsg_model* models[] = {
        &am,
        &aUbm,
        &aIbm,
        &aSbm,
        &aSbMm,
    };

    tcsg_polygon_vector_free(&a);
    tcsg_polygon_vector_free(&b);
    tcsg_polygon_vector_free(&aUb);
    tcsg_polygon_vector_free(&aIb);
    tcsg_polygon_vector_free(&aSb);

    tscg_platform_init();
    while (tscg_platform_tick())
    {

#define fzn  0.005f
#define fzf  1000.0f

        static const float projectionmatrix[16] = {
            1.0f, 0.00f, 0.0f, 0.0f,
            0.0f, 1.25f, 0.0f, 0.0f,
            0.0f, 0.00f, -(fzf + fzn) / (fzf - fzn), -1.0f,
            0.0f, 0.00f, -2.0f*fzf*fzn / (fzf - fzn), 0.0f };


        static float ftime = 0.0f;// 0.001f*(float)itime;
        ftime += 0.01f;

        // animate
        float pos[3] = { 3.0f*cosf(ftime*1.0f),
            3.0f*cosf(ftime*0.6f),
            3.0f*sinf(ftime*1.0f) };
        float tar[3] = { 0.0f, 0.0f, 0.0f };

        // render
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_CULL_FACE);
        glDisable(GL_BLEND);
        glEnable(GL_LIGHTING);
        glEnable(GL_LIGHT0);
        glEnable(GL_NORMALIZE);
        glEnable(GL_COLOR_MATERIAL);

        glClearColor(0.3f, 0.3f, 0.3f, 0.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glMatrixMode(GL_PROJECTION);
        glLoadMatrixf(projectionmatrix);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        gluLookAt(pos[0], pos[1], pos[2], tar[0], tar[1], tar[2], 0.0f, 1.0f, 0.0f);

        if (g_wire_frame) {
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        } else {
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        }

        // draw cube
        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_NORMAL_ARRAY);
        {
            static const unsigned int colours[] = {
                0xFFFFFF, //White	
                0xC0C0C0, //Silver	
                0x808080, //Gray	
                0x000000, //Black	
                0xFF0000, //Red		
                0x800000, //Maroon	
                0xFFFF00, //Yellow	
                0x808000, //Olive	
                0x00FF00, //Lime	
                0x008000, //Green	
                0x00FFFF, //Aqua	
                0x008080, //Teal	
                0x0000FF, //Blue	
                0x000080, //Navy	
                0xFF00FF, //Fuchsia	
                0x800080, //Purple	
            };

            tcsg_model* m = models[g_model];
            glVertexPointer(3, GL_FLOAT, sizeof(tcsg_vert), &m->verts->position);
            glNormalPointer(GL_FLOAT, sizeof(tcsg_vert), &m->verts->normal);
            tcsg__sb_foreach(tcsg_draw, d, m->draws) {
                unsigned int c = colours[d->m.i];
                const GLubyte cv[] = { (GLubyte)((c >> 16) & 0xff), (GLubyte)((c >> 8) & 0xff), (GLubyte)(c & 0xff) };
                glColor3ubv(cv);
                glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
                glDrawElements(GL_TRIANGLES, d->count, GL_UNSIGNED_SHORT, m->indcies + d->offset);
            }
        }
        glDisableClientState(GL_NORMAL_ARRAY);
        glDisableClientState(GL_VERTEX_ARRAY);
    }
    tscg_platform_shutdown();

    for (int m = 0; m < sizeof(models) / sizeof(models[0]); ++m) {
        tcsg_model_free(models[m]);
    }

    return 0;
}

#endif//TINY_CSG_GLDEMO

#endif//TINY_CSG_HEADER
