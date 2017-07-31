#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>

enum {
    kTinyString_InlineLength = 15
};

typedef union {
    struct {
        unsigned char Len;
        char          Str[kTinyString_InlineLength];
    } small;
    struct {
        unsigned char Flag;
        int           Len;
        char*         Str;
    } large;
    unsigned char  SmallLenOrFlag;
} string;

static size_t string_length(string a)
{
    return (a.SmallLenOrFlag < kTinyString_InlineLength) ? (size_t)a.small.Len : (size_t)a.large.Len;
}

//TODO : string literal macro for compile time init (want to chack it agains compilers)

#define string_cstr(S) (((S).SmallLenOrFlag < kTinyString_InlineLength) ? (const char*)(S).small.Str : (const char*)(S).large.Str)
#define string_str(S)  (((S).SmallLenOrFlag < kTinyString_InlineLength) ? ( char*)(S).small.Str : ( char*)(S).large.Str)


static string string_alloc(size_t l)
{
    string o;
    if (l < kTinyString_InlineLength) {
        o.small.Len = (unsigned char)l;
    } else {
        o.SmallLenOrFlag = 0xff;
        o.large.Len = l;
        o.large.Str = malloc(l + 1);
    }

    return o;
}

static string string_newn(const char* str, size_t l)
{
    string o = string_alloc(l);
    char* s = string_str(o);
    memcpy(s, str, l);
    s[l] = 0;
    return o;
}


static string string_new(const char* str)
{
    return string_newn(str, strlen(str));
}

static string string_newf(const char* fmt, ...)
{
    va_list v;
    size_t l;
    // NOTE: find the way to get a v?printf to tell me the end length.
    // vsprintf under windows asserts on NULL buffer
    char b[2048];
    va_start(v, fmt);
    l = vsprintf(b, fmt, v);
    va_end(v);
    return string_newn(b, l);
}

static void string_free(string str)
{
    if (str.SmallLenOrFlag >= kTinyString_InlineLength) {
        free(str.large.Str);
    }
}

static string string_cat(string a, string b)
{
    string o;
    char* dst;
    size_t tl = string_length(a) + string_length(b);
    o = string_alloc(tl);
    dst = string_str(o);
    memcpy(dst, string_cstr(a), string_length(a));
    memcpy(dst + string_length(a), string_cstr(b), string_length(b) + 1); //copy its null
    return o;
}

static int string_cmp(string a, string b)
{
    return strcmp(string_cstr(a), string_cstr(b));
}

static string string_fromFile(string filename)
{
    string o = { 0 };
    FILE* fp = fopen(string_cstr(filename), "rb");
    if (fp) {
        size_t l;
        fseek(fp, 0, SEEK_END);
        l = ftell(fp);
        fseek(fp, 0, SEEK_SET);
        o = string_alloc(l);
        fread(string_str(o), 1, l, fp);
        string_str(o)[l] = 0;
        fclose(fp);
    }
    return o;
}

static size_t string_toFile(string filename, string contents)
{
    FILE* fp = fopen(string_cstr(filename), "wb");
    if (fp) {
        size_t o = fwrite(string_cstr(contents), 1, string_length(contents), fp);
        fflush(fp);
        fclose(fp);
        return o;
    }
    return 0;
}
