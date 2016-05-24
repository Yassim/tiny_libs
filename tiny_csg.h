#define TINY_CSG_IMPLEMENTATION
#define TINY_CSG_TEST

#ifndef TINY_CSG_HEADER
#define TINY_CSG_HEADER
/*
A Tiny CSG (constructive solid geometry). (v0.1 pre alpha)

This is inspired from:
https://github.com/evanw/csg.js
and
https://github.com/nothings/stb -- lib in a header

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
tcsg_f3 tcsg_f3_sub(const tcsg_f3* i_a, const tcsg_f3* i_b);
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
} tcsg_polygon;

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

void tcsg_from_tris(tcsg_vert* i_verts, unsigned short* i_index, int i_tri_count, tcsg_user_data i_ud, tcsg_polygon_vector* o_v);
tcsg_polygon_vector tcsg_cube(tcsg_user_data i_ud, const tcsg_f3* i_cntr, const tcsg_f3* i_rad);


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

static void * stb__sbgrowf(void *arr, int increment, int itemsize);

#define tcsg__sb_head(P)            (((tscg__vector_head*)(P))-1)

#define tcsg__sb_free(a)            ((a) ? tcsg_free(tcsg__sb_head(a)),0 : 0)
#define tcsg__sb_pushback(a,v)      (stb__sbmaybegrow(a,1), (a)[stb__sbn(a)++] = (v))
#define tcsg__sb_count(a)           ((a) ? stb__sbn(a) : 0)
#define tcsg__sb_last(a)            ((a)[stb__sbn(a)-1])
#define tcsg__sb_reserve(a,n)       (stb__sbmaybegrow(a, n - tcsg__sb_count(a)))
#define tcsg__sb_reserve_extra(a,n) (stb__sbmaybegrow(a, n))

#define tcsg__sb_begin(a)           ((a))
#define tcsg__sb_end(a)             ((a) + tcsg__sb_count(a))

#define stb__sbm(a)                 (tcsg__sb_head(a)->capacity)
#define stb__sbn(a)                 (tcsg__sb_head(a)->count)

#define stb__sbneedgrow(a,n)        ((a)==0 || stb__sbn(a)+(n) >= stb__sbm(a))
#define stb__sbmaybegrow(a,n)       (stb__sbneedgrow(a,(n)) ? stb__sbgrow(a,n) : 0)
#define stb__sbgrow(a,n)            ((a) = stb__sbgrowf((a), (n), sizeof(*(a))))

static void * stb__sbgrowf(void *arr, int increment, int itemsize)
{
    int dbl_cur = arr ? 2 * stb__sbm(arr) : 0;
    int min_needed = tcsg__sb_count(arr) + increment;
    int m = dbl_cur > min_needed ? dbl_cur : min_needed;
    tscg__vector_head* p = (tscg__vector_head*)tcsg_realloc(arr ? tcsg__sb_head(arr) : 0, itemsize * m + sizeof(tscg__vector_head));
    if (p) {
        if (!arr)
            p->count = 0;
        p->capacity = m;
        return p + 1;
    }
    else {
#ifdef STRETCHY_BUFFER_OUT_OF_MEMORY
        STRETCHY_BUFFER_OUT_OF_MEMORY;
#endif
        return (void *)(sizeof(tscg__vector_head)); // try to force a NULL pointer exception later
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

tcsg_f3 tcsg_f3_sub(const tcsg_f3* i_a, const tcsg_f3* i_b)
{
    tcsg_f3 o;
    o.x = i_a->x - i_b->x;
    o.y = i_a->y - i_b->y;
    o.z = i_a->z - i_b->z;
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
    float* v = (float*)io_self;
    for (int i = 0; i < 4; ++i) {
        v[i] = -v[i];
    }
}

tcsg_polygon* tcsg_polygon_new(tcsg_user_data i_ud, int i_count, const tcsg_vert* i_pnts)
{
    tcsg_polygon* o = (tcsg_polygon*)tcsg_malloc(sizeof(tcsg_polygon) + sizeof(tcsg_vert) * (i_count - 1));
    o->ref_count = 0;
    if (i_pnts) {
        o->count = i_count;
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
    }
    tcsg_plane_invert(&i_a->plane);
}

#define tcsg_alloca(N)  _malloca((N))
#define tcsg_freea(P)   _freea((P))

//void tcsg__split(const tcsg_plane* i_p, const tcsg_polygon_vector* i_list, tcsg_polygon_vector* o_front, tcsg_polygon_vector* o_co_front, tcsg_polygon_vector* o_co_back, tcsg_polygon_vector* o_back)
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
    tcsg__build_bsp_r(&o, i_list);
    return o;
}

void tcsg__bsp_free(tcsg__bsp* i_self)
{
    tcsg_free(i_self->nodes);
}

tcsg_polygon_vector tcsg__clip(int i_inside, tcsg__bsp* i_bsp, int i_node, tcsg_polygon_vector* i_list)
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
    tcsg_polygon_vector aob = tcsg__clip(0, &bb, 0, i_a);
    tcsg_polygon_vector boa = tcsg__clip(0, &ba, 0, i_b);

    tcsg_polygon_vector_concat(&aob, &boa);
    tcsg_polygon_vector_free(&boa);

    return aob;
}

tcsg_polygon_vector tcsg_intersect(tcsg_polygon_vector* i_a, tcsg_polygon_vector* i_b)
{
    tcsg__bsp ba = tcsg__build_bsp(i_a);
    tcsg__bsp bb = tcsg__build_bsp(i_b);;
    tcsg_polygon_vector aib = tcsg__clip(1, &bb, 0, i_a);
    tcsg_polygon_vector bia = tcsg__clip(1, &ba, 0, i_b);

    tcsg_polygon_vector_concat(&aib, &bia);
    tcsg_polygon_vector_free(&bia);

    return aib;
}

tcsg_polygon_vector tcsg_subtract(tcsg_polygon_vector* i_a, tcsg_polygon_vector* i_b)
{
    tcsg__bsp ba = tcsg__build_bsp(i_a);
    tcsg__bsp bb = tcsg__build_bsp(i_b);
    tcsg_polygon_vector aob = tcsg__clip(0, &bb, 0, i_a);
    tcsg_polygon_vector bia = tcsg__clip(1, &ba, 0, i_b);

    tcsg_polygon_vector_invert(&bia);
    tcsg_polygon_vector_concat(&aob, &bia);
    tcsg_polygon_vector_free(&bia);

    return aob;
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
        {{0, 4, 6, 2}, {-1, 0, 0}},
        {{1, 3, 7, 5}, {1, 0, 0}},
        {{0, 1, 5, 4}, {0, -1, 0}},
        {{2, 6, 7, 3}, {0, 1, 0}},
        {{0, 2, 3, 1}, {0, 0, -1}},
        {{4, 5, 7, 6}, {0, 0, 1}}
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

    tcsg_polygon_vector_free(&a);
    tcsg_polygon_vector_free(&b);
    tcsg_polygon_vector_free(&aUb);
    tcsg_polygon_vector_free(&aIb);
    tcsg_polygon_vector_free(&aSb);

    return 0;
}

#endif//TINY_CSG_TEST

#endif//TINY_CSG_HEADER
