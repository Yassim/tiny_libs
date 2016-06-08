// Variant of https://github.com/nothings/stb/blob/master/stretchy_buffer.h
//  Really only good for POD types..
//
#define tv_free(a)              ((a) ? tv__free(tv__sbraw(a)),0 : 0)
#define tv_pushback(a,v)        (tv__sbmaybegrow(a,1), (a)[tv__sbn(a)++] = (v))

//#define tv_insert(a,i,v)    { tv__sbmaybegrow(a,1); for(int j = tv__sbn(a); j >= i; --j) { (a)[j] = (a)[j - 1]; }; tv__sbn(a)++; } ((a)[i] = (v))
//#define tv_remove(a,i)      { for(int j = i; j < tv__sbn(a)-1; ++j) {     (a)[j] = (a)[j + 1];  }; tv__sbn(a)--; }; ((a)+i)
#define tv_insert_i(a,i,v)        (tv__sbmaybegrow(a,1), memmove((a)+i+1, (a)+i, (tv__sbn(a) - i + 1) * sizeof(*(a))), tv__sbn(a)++, ((a)[i] = (v)))
#define tv_remove_i(a,i)         (memmove((a)+i, (a)+i+1, (tv__sbn(a) - i + 1) * sizeof(*(a))), tv__sbn(a)--, ((a)+i))

#define tv__max(a, b)           ((a) > (b) ? (a) : (b))
#define tv_move(a,d,s)          memmove((d), (s), sizeof(*(a)) * (tv_end(a) - tv__max(s, d)))
#define tv_insert(a,i,v)        (tv__sbmaybegrow(a,1), tv_move((i)+1, i, a), tv__sbn(a)++, ((a)[i] = (v)))
#define tv_remove(a,i)          (tv_move(i, (i)+1, a), tv__sbn(a)--, (i)-1)

#define tv_count(a)             ((a) ? tv__sbn(a) : 0)
#define tv_resize(a,n)          (tv__sbmaybegrow(a,n), tv__sbn(a)+=(n), &(a)[tv__sbn(a) - (n)])
#define tv_reserve(a,n)         (tv__sbmaybegrow(a,n))
#define tv_head(a)              ((a)[0])
#define tv_tail(a)              ((a)[tv_count(a)-1])

#define tv_begin(a)             ((a))
#define tv_end(a)               ((a) + tv_count(a))

#define tv_foreach(t,i,a)       for (t* i = tv_begin(a); i != tv_end(a); ++i)
#define tv_foreach_i(i,a)       for (int i = 0; i < tv_count(a); ++i)
#define tv_foreach_r(t,i,a)     if (tv_count(a)) for(t* i = tv_end(a)-1; i >= tv_begin(a); --i)
#define tv_foreach_ri(i,a)      if (tv_count(a)) for(int i = tv_count(a) - 1; i >= 0; --i)
#define tv_foreach_f(t,i,a)     for (t* i = tv_begin(a), *e = tv_end(a); i != e; ++i)
#define tv_foreach_fr(t,i,a)    for (t* i = tv_end(a)-1, *e = tv_begin(a) - 1; i != e; --i)

#define tv_revese(t,a)          if (tv_count(a)) for(t *s = tv_begin(a), *e = tv_end(a) - 1; s < e; ++s, --e) { t tmp = *s; *s = *e; *e = tmp; }


typedef struct {
    int count;
    int capacity;
    int pad[2]; // easier to controll padding
} tv__head;


#define tv__sbraw(a)            ((tv__head *) (a) - 1)
#define tv__sbm(a)              tv__sbraw(a)->capacity
#define tv__sbn(a)              tv__sbraw(a)->count

#define tv__sbneedgrow(a,n)     ((a)==0 || tv__sbn(a)+(n) >= tv__sbm(a))
#define tv__sbmaybegrow(a,n)    (tv__sbneedgrow(a,(n)) ? tv__sbgrow(a,n) : 0)
#define tv__sbgrow(a,n)         ((*(void**)&(a)) = tv__sbgrowf((a), (n), sizeof(*(a))))

#if !defined(tv__realloc)
#include <stdlib.h>
#define tv__realloc(P, N) realloc((P), (N))
#define tv__free(P)       free((P))
#endif

static void * tv__sbgrowf(void *arr, int increment, int itemsize)
{
    int dbl_cur = arr ? 2 * tv__sbm(arr) : 0;
    int min_needed = tv_count(arr) + increment;
    int m = dbl_cur > min_needed ? dbl_cur : min_needed;
    tv__head *op = arr ? tv__sbraw(arr) : 0;
    tv__head *np = (tv__head *)tv__realloc(op, itemsize * m + sizeof(tv__head));

    // explode now if out of memory
    if (!arr)
        np->count = 0;
    np->capacity = m;

    return np + 1;
}


#define tv_imp_find(t)                          \
 static int tv_find_##t(t* i_a, t i_v) {        \
         tv_foreach(t, i, i_a) {                \
             if (*i == i_v) {                   \
                     return (int)(i - i_a);     \
             }                                  \
     }                                          \
     return tv_count(i_a);                      \
}


#define tv_indexof(t, a, v) tv_find_##t(a, v)
#define tv_find(t, a, v) ((a) + tv_find_##t(a, v))

 // TODO, binary or interpolate search
#define tsv_imp_find(t)                         \
 static int tsv_find_##t(t* i_a, t i_v) {       \
         tv_foreach(t, i, i_a) {                \
             if (*i == i_v) {                   \
                     return (int)(i - i_a);     \
             }                                  \
             if (*i > i_v) {                    \
                     return -(int)(i - i_a);    \
             }                                  \
     }                                          \
     return -tv_count(i_a);                     \
}

#define tsv_indexof(t, a, v) tsv_find_##t(a, v)
#define tsv_find(t, a, v) ((a) + tsv_find_##t(a, v))

//#include <string.h>
//#define tv_insert(a,i,v)   (tv__sbmaybegrow(a,1) /*, for(int j = tv__sbn(a); j >= i; --j) { (a)[j] = (a)[j - 1]; }*/ , tv__sbn(a)++, (a)[i] = (v))
//#define tv_insert(a,i,v)   (tv__sbmaybegrow(a,1), memmove((a)+i+1, (a)+i, sizeof(*(a)) * tv__sbn(a) - i), tv__sbn(a)++, (a)[i] = (v))

#include <stdio.h>
#include <string.h>

int main(int i_argc, char** i_argv)
{
    int * v = 0;

    tv_pushback(v, 1);
    tv_pushback(v, 2);
    tv_pushback(v, 3);

    tv_insert_i(v, 1, 42);
    tv_insert_i(v, 2, 98);
    tv_remove_i(v, 2);

    tv_foreach(int, i, v)
        printf("%d\n", *i);

    tv_revese(int, v);

    printf("---\n");
    tv_foreach(int, i, v)
        printf("%d\n", *i);

    tv_foreach(int, i, v)
        if (*i % 2)
            i = tv_remove(v, i);

    printf("---\n");
    tv_foreach(int, i, v)
        printf("%d\n", *i);

    tv_free(v);
    return 0;
}
