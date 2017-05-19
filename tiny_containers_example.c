#include "tiny_containers.h"

void test_vector_int()
{
	int* v = NULL;
	tv_pushback(v, 1);
	tv_pushback(v, 3);
	tv_pushback(v, 2);


	{
		int* i;
		for (i = tv_begin(v); i != tv_end(v); ++i)
		{
			*i += 10;
		}
	}

	tv_free(v);
}

void test_set_int()
{
	int* v = NULL;
	ts_init(v, 0, op_int); // unsorted set

	ts_add(v, 1);
	ts_add(v, 3);
	ts_add(v, 2);

	ts_free(v);
}

void test_map_intint()
{
	struct {
		int Key;
		int Value;
	} * v = NULL;
	tm_init(v, 0, op_int);

	tm_add(v, 1, 10);
	tm_add(v, 3, 30);
	tm_add(v, 2, 20);

	tm_free(v);
}

void main()
{
	test_vector_int();
	test_set_int();
	test_map_intint();
}
