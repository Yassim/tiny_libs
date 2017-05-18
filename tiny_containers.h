#include <assert.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>

//
// Containers in C
//
// [X] Vector
// [X] Map
// [X] Set
// [X] Bit Set
// [ ] Unordered Map
// [ ] Unordered Set
//

//
// Overrideable memory management
//
#ifndef rt_free
#define rt_free(P)			free(P)
#endif
#ifndef rt_realloc
#define rt_realloc(P, N)	realloc(P, N)
#endif

//
// Common API
// (tc prefix)
#define tc_count(a)             ((a) ? tc__n(a) : 0)
#define tc_capacity(a)          ((a) ? tc__m(a) : 0)
#define tc_reserve(a,n)         (tc__maybegrow(a,n))

//
// Vector
// (tv prefix)
#define tv_free(a)              ((a) ? rt_free(tv__raw(a)),0 : 0)
#define tv_pushback(a,v)        (tv__maybegrow(a,1), (a)[tv__n(a)++] = (v))
#define tv_insert(a,i,v)        (tv__maybegrow(a,1), tc__move((i)+1, i, a), tv__n(a)++, ((a)[i] = (v)))
#define tv_remove(a,i)          (tc__move(i, (i)+1, a), tv__n(a)--, (i)-1)

#define tv_count(a)             ((a) ? tv__n(a) : 0)
#define tv_resize(a,n)          (tv__maybegrow(a,n), tv__n(a)+=(n), &(a)[tv__n(a) - (n)])
#define tv_reserve(a,n)         (tv__maybegrow(a,n))
#define tv_head(a)              ((a)[0])
#define tv_tail(a)              ((a)[tc_count(a)-1])

#define tv_begin(a)             ((a))
#define tv_end(a)               ((a) + tc_count(a))


//
// Map
// (tm prefix)
#define tm_init(a, c, kop)		(tm_reserve(a, c), tc__raw(a)->op = kop)
#define tm_free(a)              ((a) ? rt_free(tc__raw(a)),0 : 0)
#define tm_reserve(a,n)         (tc__maybegrow(a,n))

#define tm_index(a,k)			(tm__find(a, &a->Key, sizeof(*(a)), k))
#define tm_count(a)             ((a) ? tc__n(a) : 0)

#define tm_find(a, k)			(tm_index(a,k) >= 0 ? (void*)&((a)[tc__lfr(a)].Value) : NULL)
#define tm_add(a,k,v)			(tm_index(a,k) >= 0 ? ((a)[tc__lfr(a)].Value = (v)) : (tc__spacei(a,tc__lfri(a)), (a)[tc__lfri(a)].Key = (k), (a)[tc__lfri(a)].Value = (v)))
#define tm_get(a,k)				(tm_index(a,k) >= 0 ? ((a)[tc__lfr(a)].Value) : (tc__inserti(a,tc__lfri(a)), (a)[tc__lfri(a)].Key = (k), (a)[tc__lfri(a)].Value))
#define tm_at(a,k)				((a)[tm_index(a,k)].Value)
#define tm_del(a,k)				(tm_index(a,k) >= 0 ? tc__removei(a, tm__lfr(a)) : (a))

#define tm_begin(a)             ((a))
#define tm_end(a)               ((a) + tm_count(a))


//
// Set
// (ts prefix)
#define ts_init(a, c, kop)		(tm_reserve(a, c), tm__raw(a)->op = kop)
#define ts_free(a)              ((a) ? rt_free(tm__raw(a)),0 : 0)

#define ts_count(a)             ((a) ? tv__n(a) : 0)
#define ts_index(a,k)			(tm__find(a, &a->Key, sizeof(*(a)), k))
#define ts_contains(a,k)		(tm__find(a, &a->Key, sizeof(*(a)), k) > 0)
#define ts_add(a,k)				(tm_index(a,k) > 0 ? ((k)) : (tc__inserti(a,tc__lfri(a)), (a)[tc__lfri(a)].Key = (k)))
#define ts_del(a,k)				(tm_index(a,k) > 0 ? tc__removei(a, tm__lfr(a)) : (a))

#define ts_begin(a)             ((a))
#define ts_end(a)               ((a) + ts_count(a))

//
// Bit set
// (tbs prefix)
typedef unsigned int* tbs_set;
enum { tbs_bitcount = 32 };

#define tbs_init(a, c)			(tv__maybegrow(a,(c)/tbs_bitcount), tv__n(a)+=((c)/tbs_bitcount))
#define tbs_free(a)				((a) ? rt_free(tm__raw(a)),0 : 0)
#define tbs_count(a)            ((a) ? (tv__n(a) * tbs_bitcount) : 0)
#define tbs_get(a, i)			(0 != ((a)[i/tbs_bitcount] & (1 << (i % tbs_bitcount))))
#define tbs_set(a, i, v)		((v) ? tbs_set1(a, i) : tbs_set0(a, i))
#define tbs_set1(a, i)			((a)[i/tbs_bitcount] |= (1 << (i % tbs_bitcount)))
#define tbs_set0(a, i)			((a)[i/tbs_bitcount] ^= (1 << (i % tbs_bitcount)))

//
// Details
//  Private(ish)
//
#define tc__max(a, b)          ((a) > (b) ? (a) : (b))
#define tc__move(a,d,s)        memmove((d), (s), sizeof(*(a)) * (tv_end(a) - tc__max(s, d)))

#define tc__spacei(a,i)        (tc__maybegrow(a,1), memmove((a)+i+1, (a)+i, (tc__n(a) - i + 1) * sizeof(*(a))), tc__n(a)++)
#define tc__inserti(a,i,v)     (tc__maybegrow(a,1), memmove((a)+i+1, (a)+i, (tc__n(a) - i + 1) * sizeof(*(a))), tc__n(a)++, ((a)[i] = (v)))
#define tc__removei(a,i)       (memmove((a)+i, (a)+i+1, (tc__n(a) - i + 1) * sizeof(*(a))), tc__n(a)--, ((a)+i))


#define tc__raw(a)             ((tc__head *) (a) - 1)
#define tc__m(a)               tc__raw(a)->capacity
#define tc__n(a)               tc__raw(a)->count
#define tc__lfr(a)             tc__raw(a)->last_found
#define tc__lfri(a)            ((-tc__raw(a)->last_found) - 1)

#define tc__needgrow(a,n)     ((a)==0 || tc__n(a)+(n) >= tc__m(a))
#define tc__maybegrow(a,n)    (tc__needgrow(a,(n)) ? tc__grow(a,n) : 0)
#define tc__grow(a,n)         ((*(void**)&(a)) = tc__growf((a), (n), sizeof(*(a))))


typedef enum {
	tccmd_find,
	tccmd_hash
} tc__cmd;

typedef int tc__int;

typedef tc__int(*tc__sysop)(tc__cmd, void*, int, int, va_list);

typedef struct {
	tc__sysop	op;
	tc__int		count;
	tc__int		capacity;
	tc__int		last_found;
} tc__head;

static int np2(int x)
{
	x -= 1;
	x |= x >> 1;
	x |= x >> 2;
	x |= x >> 4;
	x |= x >> 8;
	x |= x >> 16;
	return x + 1;
}

static void * tc__growf(void *arr, int increment, int itemsize)
{
	int dbl_cur = arr ? 2 * tc__m(arr) : 0;
	int min_needed = np2(tc_count(arr) + increment);
	int m = dbl_cur > min_needed ? dbl_cur : min_needed;
	tc__head *op = arr ? tc__raw(arr) : 0;
	tc__head *np = (tc__head *)rt_realloc(op, itemsize * m + sizeof(tc__head));

	if (!op)
		np->count = 0;

	// explode now if out of memory
	np->capacity = m;

	return np + 1;
}


static int tm__find(void* m, void* k, int itemsize, ...)
{
	int o;
	va_list v;
	tc__head *op = m ? tc__raw(m) : 0;
	va_start(v, itemsize);
	o = op->op(tccmd_find, k, op->count, itemsize, v);
	op->last_found = o;
	va_end(v);
	return o;
}


int op_int(int op, void * m, int count, int stride, va_list args)
{
	switch (op) {
	case tccmd_find: {
		int needle = va_arg(args, int);
		int i;
		for (i = 0; i < count; i++) {
			if (needle == (*(int*)m)) {
				return i;
			}
			m = (char*)m + stride;
		}
		return -(count + 1);
	}
					 break;
	default: assert(!"UnKnown Op"); return 0;
	}
}

int op_sorted_int(int op, void * m, int count, int stride, va_list args)
{
	switch (op) {
	case tccmd_find: {
		int needle = va_arg(args, int);
		int i;
		for (i = 0; i < count; i++) {
			if (needle == (*(int*)m)) {
				return i;
			}
			else
				if (needle < (*(int*)m)) {
					return -(i + 1);
				}
			m = (char*)m + stride;
		}
		return -(count + 1);
	}
					 break;
	default: assert(!"UnKnown Op"); return 0;
	}
}


int op_charStar(int op, void * m, int count, int stride, va_list args)
{
	switch (op) {
	case tccmd_find: {
		const char* needle = va_arg(args, const char*);
		int i;
		for (i = 0; i < count; i++) {
			if (0 == strcmp(needle, (*(const char**)m))) {
				return i;
			}
			m = (char*)m + stride;
		}
		return -(count + 1);
	}
					 break;
	default: assert(!"UnKnown Op"); return 0;
	}
}

int op_sorted_charStar(int op, void * m, int count, int stride, va_list args)
{
	switch (op) {
	case tccmd_find: {
		const char* needle = va_arg(args, const char*);
		int i;
		for (i = 0; i < count; i++) {
			int r = strcmp(needle, (*(const char**)m));
			if (0 == r) {
				return i;
			}
			else if (r < 0) {
				return -(i + 1);
			}
			m = (char*)m + stride;
		}
		return -(count + 1);
	}
					 break;
	default: assert(!"UnKnown Op"); return 0;
	}
}
