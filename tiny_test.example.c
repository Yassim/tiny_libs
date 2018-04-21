#include "tiny_test.h"


tiny_test(First, First) {
    return 1;
}

tiny_test(First, Pass) {
    tt_assert(1, "Always pass");
    tt_pass("A+");
}

tiny_test(First, Fail) {
    tt_assert(!"bang", "Always Fail");
    tt_fail("A+");
}

tiny_test(First, IntEq)
{
    int exp = 5;
    tt_assert_neqi(exp, 10, "");
    tt_assert_eqf(2.0f, (float)sqrt(4.0), "");
    tt_pass("A+");
}
