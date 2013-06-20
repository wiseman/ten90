lib_LTLIBRARIES = libten90.la

libten90_la_SOURCES = libten90.c
include_HEADERS = ten90.h

noinst_PROGRAMS = test_libten90

test_libten90_SOURCES = $(libten90_la_SOURCES) test_libten90.c
test_libten90_CFLAGS = $(libten90_la_CFLAGS)
test_libten90_LDADD = $(libten90_la_LIBADD)

TESTS = test_libten90
