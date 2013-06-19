ACLOCAL_AMFLAGS = -I m4

lib_LTLIBRARIES = libdump1090.la

libdump1090_la_SOURCES = libdump1090.c
include_HEADERS = dump1090.h

noinst_PROGRAMS = test_libdump1090

test_libdump1090_SOURCES = $(libdump1090_la_SOURCES) test_libdump1090.c
test_libdump1090_CFLAGS = $(libdump1090_la_CFLAGS)
test_libdump1090_LDADD = $(libdump1090_la_LIBADD)

TESTS = test_audio_manager
