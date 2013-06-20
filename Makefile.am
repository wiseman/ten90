lib_LTLIBRARIES = libten90.la

libten90_la_SOURCES = libten90.c
include_HEADERS = ten90.h

bin_PROGRAMS = dump1090

noinst_PROGRAMS = test_libten90

dump1090_SOURCES = dump1090.c anet.c
dump1090_CFLAGS = $(libten90_la_CFLAGS)
dump1090_LDADD = $(libten90_la_LIBADD) libten90.la

test_libten90_SOURCES = test_libten90.c
test_libten90_CFLAGS = $(libten90_la_CFLAGS)
test_libten90_LDADD = $(libten90_la_LIBADD) libten90.la

TESTS = test_libten90
