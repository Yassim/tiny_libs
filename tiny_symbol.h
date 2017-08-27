#include <stdlib.h>
#include <stdint.h>
#include <string.h>

// TINY Symbols.
//  Symbols are a dynamic table of unique strings, with optional values.
//  Symbols are unique inside of a single table.. (Case sensitive)
//  which means they can be compared quickly by just compairring the address.
//  Symbols are nul terminated, and the address is to the first byte of the ascii string
//   so they work fine as arguments to fopen, printf and anything else that wants a string.
//  Symbols also have a Value asscoiated to them. (inital value 0)
//  Symbols are allocated in pages, to reduce fragmentation and allocation time.
//  Symbols do not require ref counting, they live from creation to the destruction of the table.
//  Symbols do not leak (appart from the heap abstraction), no global vars used.
//  SymbolTables can also be ittorated with a foreach using a callback, which can also be exitied
//    if the callback returns a non zero result.

#ifndef TINY_SYMBOL__INTERFACE
#define TINY_SYMBOL__INTERFACE

typedef const char* symbol;

typedef struct {
    void* (*alloc)(size_t);
    struct symbol_page* page;
    struct symbol_head* tbl[127];
} symbol_table;

typedef union {
    void* P;
    const char* S;
    int   I;
    float F;
} symbol_value;

symbol       symbol_new(symbol_table*, const char*);
#define      symbol_value(S) (*symbol_valueRef_((S)))
symbol_value* symbol_valueRef_(symbol);

void        symboltable_foreach(symbol_table*, int(*)(symbol, void*), void*);
symbol_table* symboltable_new(void*(*)(size_t));
void         symboltable_free(symbol_table*, void(*)(void*));

#endif//TINY_SYMBOL__INTERFACE

/*
Example:
void main(void)
{
    symbol_table t = { 0 }; // intial ize table to 0.
                            // optinally, set alloc to your malloc replacment
    symbol foo = symbol_new(&t, "foo");
    symbol bar = symbol_new(&t, "bar");
    symbol foo_dup = symbol_new(&t, "foo"); // foo_dup == foo
    symbol_value fv = symbol_value(foo); //read value

    symbol_value(bar).I = 42; // value is also writable

    symboltable_free(&t, NULL); // frees all symbols. 
                             // optionally, pass in your own free replacment
}
*/


////////////////////////////////////////////////////////////////////////////
// IMP
//// Inline

#ifndef TINY_SYMBOL__INLINES
#define TINY_SYMBOL__INLINES

struct symbol_head {
    struct symbol_head* next;
    symbol_value V;
    const char   S[1];
};

static symbol_value* symbol_valueRef_(symbol s)
{
    return &(((struct symbol_head*)s) - 1)->V;
}

#endif//TINY_SYMBOL__INLINES

#ifdef TINY_SYMBOL__IMPLIMENTATION
////////////////////////////////////////////////////////////////////////////
// IMP
//// Private

struct symbol_page {
    struct symbol_page* next;
    uint8_t*            end;
};

#define pad(X, A)   (((X)+((A)-1)) &~((A)-1))

static size_t page_free(struct symbol_page* p)
{
    uint8_t* s = (void*)(p + 1);
    return p->end - s;
}

static void* page_alloc(struct symbol_page* p, size_t n)
{
    return p->end -= n;
}

static uint32_t djb2_hash(const char *s, size_t l)
{
    // from http://www.cse.yorku.ca/~oz/hash.html
    // using the xor varient
    uint32_t o = 5381;
    for (const char* e = s + l; s != e; s++) {
        o = o * 33 ^ (uint32_t)*s;
    }
    return o;
}

symbol       symbol_new(symbol_table* t, const char* s)
{
    const size_t l = strlen(s);
    const uint32_t h = djb2_hash(s, l);
    const uint32_t bi = h % (sizeof(t->tbl) / sizeof(t->tbl[0]));

    // find
    for (struct symbol_head* i = t->tbl[bi]; i; i = i->next) {
        if (0 == strcmp(i->S, s)) {
            return i->S;
        }
    }

    // alloc 
    {
        size_t tl = pad(l + sizeof(struct symbol_head), sizeof(void*));
        struct symbol_page* p = t->page;

        for (; p; p = p->next) {
            if (tl <= page_free(p)) {
                break;
            }
        }

        // new page
        if (!p)
        {
            size_t pl = pad(tl + sizeof(struct symbol_page), 2048);
            p = (t->alloc ? t->alloc : malloc)(pl);
            p->next = t->page;
            t->page = p;
            p->end = ((uint8_t*)p) + pl;
        }

        // make new symbol
        {
            struct symbol_head* h = page_alloc(p, tl);
            memcpy(h->S, s, l + 1);
            h->V = (symbol_value) { 0 };
            h->next = t->tbl[bi];
            t->tbl[bi] = h;
            return h->S;
        }
    }
}

void        symboltable_foreach(symbol_table* t, int (*op)(symbol, void*), void* o)
{
    for (int i = 0; i < sizeof(t->tbl) / sizeof(t->tbl[0]); i++) {
        for (struct symbol_head* j = t->tbl[i]; j; j = j->next) {
            if (op(j->S, o)) 
                return;
        }
    }
}

symbol_table* symboltable_new(void*(*a)(size_t))
{
    symbol_table* o = NULL;
    if (!a) a = malloc;
    o = memset(a(sizeof(*o)), 0, sizeof(*o));
    o->alloc = a;
    return o;
}

void         symboltable_free(symbol_table* t, void(*f)(void*))
{
    struct symbol_page* np, *p = t->page;

    if (!f) f = free;

    for (; p; p = np) {
        np = p->next;
        f(p);
    }

    memset(t, 0, sizeof(*t));
}

#endif//TINY_SYMBOL__IMPLIMENTATION

#ifdef TINY_SYMBOL_TESTING
#include <assert.h>

void test(void)
{
    symbol_table t = { 0 };
    symbol foo = symbol_new(&t, "foo");
    symbol bar = symbol_new(&t, "bar");
    symbol foo_dup = symbol_new(&t, "foo");
    symbol_value fv = symbol_value(foo);

    symbol_value(bar).I = 42;

    assert(foo == foo_dup);
    assert(foo != bar);
    assert(fv.P == NULL);
    assert(42 == symbol_value(bar).I);

    symboltable_free(&t, NULL);
}

void main(void)
{
    test();
}
#endif//TINY_SYMBOL_TESTING
