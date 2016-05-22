#ifndef TINY_TASK_SCHEDULER_HEADER
#define TINY_TASK_SCHEDULER_HEADER

#include <stdlib.h> // for size_t

typedef struct {
    //                      Defaults to:
    int worker_count;       //  processor count
    int worker_stack_size;  //  8k
    int worker_queue_max;   //  128 tasks
} tts_config;

typedef enum {
    tts_false,
    tts_true,

    tts_force_signed_int = 0x7fffffff
} tts_bool;

typedef union {
    void*           p;

    int             i;
    unsigned int    u;
    float           f;

    short           s[2];
    unsigned short  us[2];
    char            c[4];
    unsigned char   uc[4];
} tts_user_data;

typedef volatile int tts_busy_flag;

typedef void(*tts_op)(tts_user_data i_args[5]);
typedef tts_bool(*tts_split)(tts_user_data i_new[5], tts_user_data i_orig[5]);

typedef struct {
    tts_busy_flag*  flag;
    tts_op          op;
    tts_split       split;
    tts_user_data   args[5];
} tts_task; // NOTE: 8 pointers in size

#define tts_is_done(flag)  (0 == (flag))

void tts_start(const tts_config* i_cfg);
void tts_may_yeild(tts_bool i_value);

void tts_enqueue(const tts_task* i_task);
void tts_wait_until(tts_busy_flag* i_flag);

void tts_foreach(void* i_start, void* i_end, size_t i_step, void(*i_op)(void*));
void tts_foreach_async(void* i_start, void* i_end, size_t i_step, void(*i_op)(void*), tts_busy_flag* i_flag);
void tts_foreach_user(void* i_start, void* i_end, size_t i_step, void(*i_op)(tts_user_data, void*), tts_user_data i_ud);
void tts_foreach_user_async(void* i_start, void* i_end, size_t i_step, void(*i_op)(tts_user_data, void*), tts_user_data i_ud, tts_busy_flag* i_flag);

void tts_stop();

#endif//TINY_TASK_SCHEDULER_HEADER
                        ////implementation
#ifdef TINY_TASK_SCHEDULER_IMPLEMENTATION
////////////////////////////////////
//

#if defined(unix) || defined(__unix__) || defined(__unix) || defined(__APPLE__)
#define TTS_SYSV    1
#include <pthreads.h>
// TODO
#endif

#if defined(_WIN32) || defined(__WIN32__) || defined(__WINDOWS__)
#define TTS_WINDOWS     1
#define WIN32_LEAN_AND_MEAN
#define WIN32_EXTRA_LEAN
#include <windows.h>
typedef DWORD tts_tls_key;
typedef HANDLE tts_sema;

#define tts__new_key()                  TlsAlloc()
#define tts__get_tls(i_key)             TlsGetValue((i_key))
#define tts__set_tls(i_key, i_value)    TlsSetValue((i_key), (i_value))

#define tts__inc_flag(i_flag, i_value)  InterlockedAdd((i_flag), (i_value))

#define tts__cas32(T, C, V)             InterlockedCompareExchange((T), (V), (C))

#define tts__pause()                    _mm_pause()

#define tts__new_sema()                 CreateSemaphore(NULL, 0, 255, NULL)
#define tts__wait_sema(i_sema)          WaitForSingleObject((i_sema), INFINITE)
#define tts__signal_sema(i_sema, i_n)   ReleaseSemaphore((i_sema), (i_n), NULL)

#ifndef tts__malloc
// memory is overridable
#define tts__malloc(N)                  malloc((N))
#define tts__free(P)                    free((P))
#endif//tts__malloc

static int tts__get_cpu_count()
{
    SYSTEM_INFO si;
    GetSystemInfo(&si);
    return (int)si.dwNumberOfProcessors;
}

static tts_user_data tts__get_current_worker()
{
    tts_user_data o; o.p = GetCurrentThread();
    return o;
}

static void tts__start_work(void* i_self);

static DWORD WINAPI tts__thread_start(void* p)
{
    tts__start_work(p);

    return 0;
}

static tts_user_data tts__start_worker(size_t i_stackSize, void* i_worker)
{
    tts_user_data o; o.p = CreateThread(NULL, i_stackSize, tts__thread_start, i_worker, 0, NULL);
    return o;
}

static void tts__destroy_worker(tts_user_data i_h)
{
    WaitForSingleObject(i_h.p, INFINITE);
    CloseHandle(i_h.p);
}
#endif

typedef volatile int tts_spin_lock;
typedef unsigned char tts_byte;

typedef struct {
    tts_spin_lock   lock;
    int             capacity;
    int             count;
    tts_task        tasks[1];
} tts_tasks;

typedef struct tts_worker {
    tts_tasks*      tasks;
    tts_user_data   thread;
} tts_worker;

typedef struct {
    tts_worker*     workers;
    tts_tls_key     key;
    int             worker_count;

    tts_sema        wake_sleepers;
    tts_sema        wait_sleepers;

    volatile tts_bool running;
    volatile tts_bool pause;
} tts_system;

static tts_system* tts_g = NULL;


static tts_bool tts__try_lock(tts_spin_lock* i_lock)
{
    return tts__cas32(i_lock, 0, 1) == 0;
}

static void tts__lock(tts_spin_lock* i_lock)
{
    while (tts__try_lock(i_lock)) { tts__pause(); }
}

static void tts__unlock(tts_spin_lock* i_lock)
{
    tts__cas32(i_lock, 1, 0);
}

static tts_worker* tts__worker()
{
    return (tts_worker*)tts__get_tls(tts_g->key);
}

static void tts__push(tts_tasks* i_tasks, const tts_task* i_task)
{
    tts__lock(&i_tasks->lock);
    i_tasks->tasks[i_tasks->count++] = *i_task;
    tts__unlock(&i_tasks->lock);
}

static tts_bool tts__pop(tts_tasks* i_tasks, tts_task* o_task)
{
    tts_bool o = tts_false;
    tts__lock(&i_tasks->lock);
    if (i_tasks->count) {
        *o_task = i_tasks->tasks[--i_tasks->count];
        o = tts_true;
    }
    tts__unlock(&i_tasks->lock);
    return o;
}

static tts_bool tts__stealing(tts_tasks* i_lhs, tts_tasks* i_rhs)
{
    tts_bool o = tts_false;
    tts__lock(&i_lhs->lock);
    tts__lock(&i_rhs->lock);
    switch (i_rhs->count)
    {
    case 0: break;
    case 1: {
        tts_task t = i_rhs->tasks[0];
        if (t.split) {
            if (t.split(t.args, i_rhs->tasks[0].args)) {
                i_lhs->tasks[i_lhs->count++] = t;
                if (t.flag) {
                    tts__inc_flag(t.flag, +1);
                }
                o = tts_true;
            }
        }
    } break;
    default: {
        int grab = (i_rhs->count + 1) / 2;

        for (int i = 0; i < grab; ++i) {
            i_lhs->tasks[i_lhs->count++] = i_rhs->tasks[--i_rhs->count];
        }

        o = tts_true;
    }
    }
    tts__unlock(&i_rhs->lock);
    tts__unlock(&i_lhs->lock);
    return o;
}

static tts_bool tts__steal(tts_worker* i_self)
{
    tts_worker* b = tts_g->workers;
    tts_worker* e = tts_g->workers + tts_g->worker_count;

    for (tts_worker* i = i_self + 1; i != e; ++i) {
        if (tts__stealing(i_self->tasks, i->tasks)) {
            return tts_true;
        }
    }

    for (tts_worker* i = b; i != i_self; ++i) {
        if (tts__stealing(i_self->tasks, i->tasks)) {
            return tts_true;
        }
    }

    return tts_false;
}

static void tts__do_work(tts_worker* i_self, tts_busy_flag* i_flag)
{
    do {
        tts_task t;

        while (tts__pop(i_self->tasks, &t)) {
            t.op(t.args);
            if (t.flag) {
                tts__inc_flag(t.flag, -1);
            }
            if (i_flag && tts_is_done(*i_flag)) {
                return;
            }
        }

    } while (tts__steal(i_self));
}

static void tts__signal_sleeping()
{
    tts__signal_sema(tts_g->wait_sleepers, 1);
}

static void tts__yeild_to_wake()
{
    tts__wait_sema(tts_g->wake_sleepers);
}

static void tts__wake_all()
{
    tts__signal_sema(tts_g->wake_sleepers, tts_g->worker_count - 1);
}

static void tts__start_work(void* i_self)
{
    tts_worker* self = (tts_worker*)i_self;
    tts__set_tls(tts_g->key, self);

    while (tts_g->running) {
        if (tts_g->pause) {
            tts__signal_sleeping();
            tts__yeild_to_wake();
        }

        tts__do_work(self, NULL);
    }
}

void tts__enqueue(tts_worker* i_self, const tts_task* i_task)
{
    if (i_task->flag) {
        tts__inc_flag(i_task->flag, +1);
    }
    tts__push(i_self->tasks, i_task);
}

void tts_start(const tts_config* i_cfg)
{
#define config(NAME)    (i_cfg && i_cfg->NAME > 0) ? i_cfg->NAME

    static tts_system g;

    int stack_size = config(worker_stack_size) : 8 << 10;
    int queue_max = config(worker_queue_max) : 128;
    tts_g = &g;

    g.running = tts_true;
    g.wait_sleepers = tts__new_sema();
    g.wake_sleepers = tts__new_sema();
    g.key = tts__new_key();
    g.worker_count = config(worker_count) : tts__get_cpu_count();

    g.workers = tts__malloc(sizeof(tts_worker) * g.worker_count);

    {
        tts_tasks* t = g.workers[0].tasks = tts__malloc(sizeof(tts_tasks) + sizeof(tts_task) * queue_max - 1);
        t->capacity = queue_max;
        t->count = 0;
        t->lock = 0;
        g.workers[0].thread = tts__get_current_worker();
        tts__set_tls(g.key, g.workers + 0);
    }

    for (int i = 1; i < g.worker_count; ++i) {
        tts_tasks* t = g.workers[i].tasks = tts__malloc(sizeof(tts_tasks) + sizeof(tts_task) * 64);
        t->capacity = queue_max;
        t->count = 0;
        t->lock = 0;
        g.workers[i].thread = tts__start_worker(stack_size, g.workers + i);
    }
}

void tts_enqueue(const tts_task* i_task)
{
    tts__enqueue(tts__worker(), i_task);
}

void tts_wait_until(tts_busy_flag* i_flag)
{
    tts__do_work(tts__worker(), i_flag);
}

void tts_foreach(void* i_start, void* i_end, size_t i_step, void(*i_op)(void*))
{
    tts_busy_flag f = 0;
    tts_foreach_async(i_start, i_end, i_step, i_op, &f);
    tts_wait_until(&f);
}

void tts_foreach_async(void* i_start, void* i_end, size_t i_step, void(*i_op)(void*), tts_busy_flag* i_flag)
{
    extern void tts__foreach_op(tts_user_data i_args[5]);
    extern tts_bool tts__foreach_split(tts_user_data i_new[5], tts_user_data i_orig[5]);

    tts_task t = { i_flag, tts__foreach_op, tts__foreach_split, { i_start, i_end, (void*)i_step, i_op, NULL } };
    tts__enqueue(tts__worker(), &t);
}

void tts_foreach_user(void* i_start, void* i_end, size_t i_step, void(*i_op)(tts_user_data, void*), tts_user_data i_ud)
{
    tts_busy_flag f = 0;
    tts_foreach_user_async(i_start, i_end, i_step, i_op, i_ud, &f);
    tts_wait_until(&f);
}

void tts_foreach_user_async(void* i_start, void* i_end, size_t i_step, void(*i_op)(tts_user_data, void*), tts_user_data i_ud, tts_busy_flag* i_flag)
{
    extern void tts__foreach_op_user(tts_user_data i_args[5]);
    extern tts_bool tts__foreach_split(tts_user_data i_new[5], tts_user_data i_orig[5]);

    tts_task t = { i_flag, tts__foreach_op_user, tts__foreach_split, { i_start, i_end, (void*)i_step, i_op, i_ud.p } };
    tts__enqueue(tts__worker(), &t);
}

void tts__foreach_op(tts_user_data i_args[5])
{
    tts_byte * i = (tts_byte*)i_args[0].p;
    tts_byte * e = (tts_byte*)i_args[1].p;
    size_t s = (size_t)i_args[2].p;
    void(*op)(void*) = (void(*)(void*))i_args[3].p;

    for (; i != e; i += s) {
        op(i);
    }
}

void tts__foreach_op_user(tts_user_data i_args[5])
{
    tts_byte * i = (tts_byte*)i_args[0].p;
    tts_byte * e = (tts_byte*)i_args[1].p;
    size_t s = (size_t)i_args[2].p;
    void(*op)(tts_user_data, void*) = (void(*)(tts_user_data, void*))i_args[3].p;
    tts_user_data ud = i_args[4];

    for (; i != e; i += s) {
        op(ud, i);
    }
}

tts_bool tts__foreach_split(tts_user_data i_new[5], tts_user_data i_orig[5])
{
    tts_byte * i = (tts_byte*)i_orig[0].p;
    tts_byte * e = (tts_byte*)i_orig[1].p;
    size_t s = (size_t)i_orig[2].p;
    size_t c = (e - i) / s;
    size_t d = c / 2;
    tts_byte * m = i + d * s;

    i_new[1].p = m;
    i_orig[0].p = m;

    return tts_true;
}

void tts_stop()
{
    tts_g->running = tts_false;
    tts_g->pause = tts_false;
    tts__wake_all();

    tts__free(tts_g->workers[0].tasks);
    for (int i = 1; i < tts_g->worker_count; ++i) {
        tts__destroy_worker(tts_g->workers[i].thread);
        tts__free(tts_g->workers[i].tasks);
    }
    tts__free(tts_g->workers);

    tts_g = NULL;
}

#endif//TINY_TASK_SCHEDULER_IMPLEMENTATION

#ifdef TINY_TASK_SCHEDULER_TEST
////////////////////////////////////
//
int main(int i_argc, char** i_argv)
{
    tts_start(NULL);


    tts_stop();
    return 0;
}

#endif//TINY_TASK_SCHEDULER_TEST
