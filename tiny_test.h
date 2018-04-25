//
// TINY TEST Framework
// tiny_test uses a linker trick to collect all tests collected accross
// all translation units.
// This means you can sprinkle test through your code, and they will
// magicly turn up in the test app.
//
// TODO: 
//  [ ] test filtering
//    [ ] by name
//    [ ] by fixture
//    [ ] by file
//  [ ] test fixtures (IDEA, using the test struct to partition, and run setup and teardown)
// [ ] gcc complie
// [ ] clang compile
// 
#include <math.h>

typedef void(*TTEmitF)(void*, const char*, ...);

typedef struct {
    TTEmitF emitf;
    void*   emito;
} TTestCtx;

typedef int(__cdecl *TTestOp)(TTestCtx* tt_ctx);

typedef __declspec(align(32)) struct TTest_ {
    const char* Name;
    TTestOp     Op;
    const char* File;
    void* notUsed;
} TTest;

#define tiny_test(FIXTURE, NAME)                \
int __cdecl TTest_ ## FIXTURE ## _ ## NAME ## _op (TTestCtx* tt_ctx);   \
TEST_DATA_TYPES                                 \
const TTest TTest_ ## FIXTURE ## _ ## NAME = {  \
    #FIXTURE "." #NAME,                         \
    TTest_ ## FIXTURE ## _ ## NAME ## _op,      \
    __FILE__ ,                                  \
    0                                           \
};                                              \
int __cdecl TTest_ ## FIXTURE ## _ ## NAME ## _op (TTestCtx* tt_ctx)   



#define tt_logf(FMT, ...)   do { tt_ctx->emitf(tt_ctx->emito, FMT , ## __VA_ARGS__); } while(0)

#define tt_pass(MSG, ...)    do { tt_logf("PASS: " MSG , ## __VA_ARGS__); return 1; } while(0)
#define tt_fail(MSG, ...)    do { tt_logf("FAIL: " MSG , ## __VA_ARGS__); return 0; } while(0)

#define tt_assert(EXP, MSG, ...)    do { if (!(EXP)) { tt_logf("ASSERT: (%s) " MSG, #EXP , ## __VA_ARGS__);  return 0; } } while(0)

#define tt_assert_i(VALUE, TST, EXPECTED, MSG, ...)    \
    tt_assert((VALUE) TST (EXPECTED), "(%d " # TST " %d) " MSG, (VALUE), (EXPECTED), ## __VA_ARGS__)

#define tt_assert_eqi(VALUE, EXPECTED, MSG, ...)    tt_assert_i(VALUE, ==, EXPECTED, MSG , ## __VA_ARGS__ )
#define tt_assert_neqi(VALUE, EXPECTED, MSG, ...)   tt_assert_i(VALUE, !=, EXPECTED, MSG , ## __VA_ARGS__ )
#define tt_assert_lti(VALUE, EXPECTED, MSG, ...)    tt_assert_i(VALUE, < , EXPECTED, MSG , ## __VA_ARGS__ )
#define tt_assert_lteqi(VALUE, EXPECTED, MSG, ...)  tt_assert_i(VALUE, <=, EXPECTED, MSG , ## __VA_ARGS__ )
#define tt_assert_gteqi(VALUE, EXPECTED, MSG, ...)  tt_assert_i(VALUE, >=, EXPECTED, MSG , ## __VA_ARGS__ )
#define tt_assert_gti(VALUE, EXPECTED, MSG, ...)    tt_assert_i(VALUE, > , EXPECTED, MSG , ## __VA_ARGS__ )

#define tt_assert_f(VALUE, TST, EXPECTED, MSG, ...)    \
    tt_assert((VALUE) TST (EXPECTED), "(%f " # TST " %f) " MSG, (VALUE), (EXPECTED), ## __VA_ARGS__)

#define tt_assert_eqtf(VALUE, EXPECTED, TOL, MSG, ...) \
     tt_assert( fabs((VALUE) - (EXPECTED)) < TOL, "(%f ~== %f) " MSG, (VALUE), (EXPECTED), ## __VA_ARGS__)
#define tt_assert_neqtf(VALUE, EXPECTED, TOL, MSG, ...)\
   tt_assert( fabs((VALUE) - (EXPECTED)) >= TOL, "(%f ~!= %f) " MSG, (VALUE), (EXPECTED), ## __VA_ARGS__)

#define tt_assert_eqf(VALUE, EXPECTED, MSG, ...)    tt_assert_eqtf(VALUE, EXPECTED, 0.00001f, MSG, ## __VA_ARGS__  )
#define tt_assert_neqf(VALUE, EXPECTED, MSG, ...)   tt_assert_neqtf(VALUE, EXPECTED, 0.00001f, MSG , ## __VA_ARGS__ )
#define tt_assert_ltf(VALUE, EXPECTED, MSG, ...)    tt_assert_f(VALUE, < , EXPECTED, MSG , ## __VA_ARGS__ )
#define tt_assert_lteqf(VALUE, EXPECTED, MSG, ...)  tt_assert_f(VALUE, <=, EXPECTED, MSG , ## __VA_ARGS__ )
#define tt_assert_gteqf(VALUE, EXPECTED, MSG, ...)  tt_assert_f(VALUE, >=, EXPECTED, MSG , ## __VA_ARGS__ )
#define tt_assert_gtf(VALUE, EXPECTED, MSG, ...)    tt_assert_f(VALUE, > , EXPECTED, MSG , ## __VA_ARGS__ )



// ++ WINDOWS

// from https://msdn.microsoft.com/en-us/library/7977wcck.aspx
// // The order here is important.  
// // Section names must be 8 characters or less.  
// // The sections with the same name before the $  
// // are merged into one section. The order that  
// // they are merged is determined by sorting  
// // the characters after the $.  
// // InitSegStart and InitSegEnd are used to set  
// // boundaries so we can find the real functions  
// // that we need to call for initialization.  

// tell the linker to make my section
#pragma section("ttests$a", read)
#pragma section("ttests$b", read)
#pragma section("ttests$c", read)

// sprinkle this infront of Types
#define TEST_DATA_TYPES __declspec(allocate("ttests$b"))

// -- WINDOWS





#ifdef TINY_TEST__IMPLIMENTATION
#include <assert.h>
#include <stdint.h>
#include <stdio.h>

// ++ WINDOWS
__declspec(allocate("ttests$a")) const uint8_t ttest_mark_start[1] = { 0 };
__declspec(allocate("ttests$c")) const uint8_t ttest_mark_end[1] = { 0 };

// Test Begin for windows searches from mark_start
// because in devlopment, the linker added an extra 4 bytes. to the 256 byte page.
// so, instead, I walk from the start (the page is zeroed), and look for the first
// non zero byte. I then align that back to the start of my struct;
//
// Pages are also not uniformly 256 bytes. release build for x86, seems to compress down.
// Debug x86, and both x64 builds do the 256 (+4)
const TTest * TTest_Begin() {
    const uint8_t* i = ttest_mark_start;
    while (0 == *i) i++;
    return (const TTest *)( (~(__alignof(TTest)-1)) & (intptr_t)i);
}

const TTest * TTest_End() {
    return (const TTest *)(ttest_mark_end);
}
// -- WINDOWS


int tt_runtests(int argc, char** argv)
{
    const TTest * i = TTest_Begin();
    const TTest * e = TTest_End();
    int res = 0;

    TTestCtx ctx = { (TTEmitF)fprintf, stdout };
    struct { int tried, passed, faild; } cnts = { 0 };


    while (i < e) {
        if (i->Name) {
            printf("Running %s -- ", i->Name);
            cnts.tried++;
            int res = i->Op(&ctx);
            if (res) {
                cnts.passed++;
                printf(" pass");
            }
            else {
                cnts.faild++;
                printf(" fail");
            }
            printf("\n");
        }
        //i++;
        // NOTE: for some reason, the alignment is not carrying through to 
        // the pointer type. Hence the next line for the step.
        i = (const void*)(__alignof(TTest) +(intptr_t)i); 
    }

    printf("Tests Run %d\n"
        "   Passed %d\n"
        "    Faild %d\n", cnts.tried, cnts.passed, cnts.faild);

    return res;
}

#endif//TINY_TEST__IMPLIMENTATION
