#ifndef TINY_CONTAINERS
#define TINY_CONTAINERS

#include <assert.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>

//
// (Tiny) Containers in C
//
// [X] Vector
// [X] Set
// [X] Map
// [X] Bit Set
// [ ] Unordered Map
// [ ] Unordered Set
// [ ] Owning containers?
//
// The designe of these containers is _HEAVLY_ inspired from the 'streachy buffers' from
// https://github.com/nothings/stb/blob/master/stretchy_buffer.h
// and is mainly good for POD types (so also be carful with dynamic memory).
// 
// All but Vector require an 'init' function to be called.
//  This will set the inital capacity and behaviour of the contaner.
//
// All but the Unordered (Map|Set) allow for simple ittoration from p to p+count-1.
//
// All conatiners are 1 allocation of memory, that may be realloced in expansion.
//  This means unless you know that the conatiner will not expand, then you cannot
//  hold pointers to its contents safly.
//  
// Vector is fine to start as a NULL pointer, 
// 
// Set can be sorted or un sorted, depending on the op function to the init.
// 
// Map requires the type used to have a Key and Value fields.
//	
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
#define tc_free(a)              ((a) ? rt_free(tc__raw(a)),0 : 0)
#define tc_count(a)             ((a) ? tc__n(a) : 0)
#define tc_capacity(a)          ((a) ? tc__m(a) : 0)
#define tc_reserve(a,n)         (tc__maybegrow(a,n))

//
// Vector
// (tv prefix)
#define tv_free(a)               tc_free(a)      
#define tv_count(a)              tc_count(a)     
#define tv_pushback(a,v)        (tc__maybegrow(a,1), (a)[tc__n(a)++] = (v))
#define tv_insert(a,i,v)        (tc__maybegrow(a,1), tc__move((i)+1, i, a), tc__n(a)++, ((a)[i] = (v)))
#define tv_remove(a,i)          (tc__move(i, (i)+1, a), tc__n(a)--, (i)-1)

#define tv_resize(a,n)          (tc__maybegrow(a,n), tc__n(a)+=(n), &(a)[tc__n(a) - (n)])
#define tv_reserve(a,n)          tc_reserve(a,n) 

#define tv_head(a)              ((a)[0])
#define tv_tail(a)              ((a)[tv_count(a)-1])

#define tv_begin(a)             ((a))
#define tv_end(a)               ((a) + tv_count(a))


//
// Set
// (ts prefix)
#define ts_init(a, c, kop)		(tm_reserve(a, c), tc__raw(a)->op = kop)
#define ts_free(a)               tc_free(a)     
#define ts_count(a)              tc_count(a)    
#define ts_index(a,k)			(tc__find(a, a, sizeof(*(a)), k))
#define ts_contains(a,k)		(tc__find(a, a, sizeof(*(a)), k) > 0)
#define ts_add(a,k)				(ts_index(a,k) > 0 ? ((a)[tc__lfr(a)]) : (tc__inserti(a,tc__lfri(a), (k)), (a)[tc__lfri(a)]))
#define ts_del(a,k)				(ts_index(a,k) > 0 ? tc__removei(a, tc__lfr(a)) : (a))

#define ts_capacity(a)           tc_capacity(a) 
#define ts_reserve(a,n)          tc_reserve(a,n)

#define ts_head(a)              ((a)[0])
#define ts_tail(a)              ((a)[ts_count(a)-1])

#define ts_begin(a)             ((a))
#define ts_end(a)               ((a) + ts_count(a))

//
// Map
// (tm prefix)
#define tm_init(a, c, kop)		(tm_reserve(a, c), tc__raw(a)->op = kop)
#define tm_free(a)               tc_free(a)     
#define tm_count(a)              tc_count(a)    
#define tm_reserve(a,n)          tc_reserve(a,n)

#define tm_index(a,k)			(tc__find(a, &a->Key, sizeof(*(a)), k))

#define tm_find(a, k)			(tm_index(a,k) >= 0 ? (void*)&((a)[tc__lfr(a)].Value) : NULL)
#define tm_add(a,k,v)			(tm_index(a,k) >= 0 ? ((a)[tc__lfr(a)].Value = (v)) : (tc__spacei(a,tc__lfri(a)), (a)[tc__lfri(a)].Key = (k), (a)[tc__lfri(a)].Value = (v)))
#define tm_get(a,k)				(tm_index(a,k) >= 0 ? ((a)[tc__lfr(a)].Value) : (tc__inserti(a,tc__lfri(a)), (a)[tc__lfri(a)].Key = (k), (a)[tc__lfri(a)].Value))
#define tm_at(a,k)				((a)[tm_index(a,k)].Value)
#define tm_del(a,k)				(tm_index(a,k) >= 0 ? tc__removei(a, tc__lfr(a)) : (a))

#define tm_begin(a)             ((a))
#define tm_end(a)               ((a) + tm_count(a))




//
// Bit set
// (tbs prefix)
typedef unsigned int* tbs_set;
enum { tbs_bitcount = 32 };


#define tbs_init(a, c)			(tc__maybegrow(a,(c)/tbs_bitcount), tc__n(a)+=((c)/tbs_bitcount))
#define tbs_free(a)             tc_free(a)     
#define tbs_count(a)            (tc_count(a) * tbs_bitcount)    
#define tbs_capacity(a)         (tc_capacity(a) * tbs_bitcount) 
#define tbs_reserve(a,n)        (tc_reserve(a,(n+tbs_bitcount-1)/tbs_bitcount))
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
	tccmd_hash,

	tccmd_duplicate,
	tccmd_destroydup,
} tc__cmd;

typedef int tc__int;

typedef tc__int (*tc__sysop)(tc__cmd, void*, int, int, va_list);

typedef struct {
	tc__sysop	op;
	tc__int		count;
	tc__int		capacity;
	tc__int		last_found;
} tc__head;

//
// Util Funtions
// All marked static so they may be in multiple translation units.
//
static int np2(int x)
{
	//x = ((x + 31) & 31);
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


static int tc__find(void* m, void* k, int itemsize, ...)
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

//
// Map and Set 'op' functions
//  These are like SysOp functions in that they do a few different things
//  depending on arguments passed.
//
int op_int(tc__cmd op, void * m, int count, int stride, va_list args)
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

int op_sorted_int(tc__cmd op, void * m, int count, int stride, va_list args)
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


int op_charStar(tc__cmd op, void * m, int count, int stride, va_list args)
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

int op_sorted_charStar(tc__cmd op, void * m, int count, int stride, va_list args)
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

#endif // !TINY_CONTAINERS
