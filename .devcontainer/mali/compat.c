// libmali-compat.so: the glibc >= 2.38 entry points that NXP's libmali (built against the i.MX BSP's
// glibc 2.43) calls but Ubuntu 22.04's glibc 2.35 does not have. patch_libmali.py adds this library as a
// DT_NEEDED of the container's copy of libmali.so. The C23 variants differ from the old ones only in
// accepting a "0b" prefix / %b, which libmali does not rely on for anything the GPU path needs.
#define _GNU_SOURCE
#include <inttypes.h>
#include <locale.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

long __isoc23_strtol(const char *s, char **e, int b) { return strtol(s, e, b); }
unsigned long __isoc23_strtoul(const char *s, char **e, int b) { return strtoul(s, e, b); }
long long __isoc23_strtoll(const char *s, char **e, int b) { return strtoll(s, e, b); }
unsigned long long __isoc23_strtoull(const char *s, char **e, int b) { return strtoull(s, e, b); }
long __isoc23_strtol_l(const char *s, char **e, int b, locale_t l) { return strtol_l(s, e, b, l); }
unsigned long __isoc23_strtoul_l(const char *s, char **e, int b, locale_t l) { return strtoul_l(s, e, b, l); }
long long __isoc23_strtoll_l(const char *s, char **e, int b, locale_t l) { return strtoll_l(s, e, b, l); }
unsigned long long __isoc23_strtoull_l(const char *s, char **e, int b, locale_t l) { return strtoull_l(s, e, b, l); }
intmax_t __isoc23_strtoimax(const char *s, char **e, int b) { return strtoimax(s, e, b); }
uintmax_t __isoc23_strtoumax(const char *s, char **e, int b) { return strtoumax(s, e, b); }

int __isoc23_vsscanf(const char *s, const char *f, va_list ap) { return vsscanf(s, f, ap); }
int __isoc23_vfscanf(FILE *fp, const char *f, va_list ap) { return vfscanf(fp, f, ap); }
int __isoc23_vscanf(const char *f, va_list ap) { return vscanf(f, ap); }
int __isoc23_sscanf(const char *s, const char *f, ...) {
    va_list ap;
    va_start(ap, f);
    int r = vsscanf(s, f, ap);
    va_end(ap);
    return r;
}
int __isoc23_fscanf(FILE *fp, const char *f, ...) {
    va_list ap;
    va_start(ap, f);
    int r = vfscanf(fp, f, ap);
    va_end(ap);
    return r;
}
int __isoc23_scanf(const char *f, ...) {
    va_list ap;
    va_start(ap, f);
    int r = vscanf(f, ap);
    va_end(ap);
    return r;
}
