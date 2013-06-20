check_PROGRAMS = test_libten90

test_libten90_CPPFLAGS = $(libten90_la_CFLAGS) \
	                 -I$(top_srcdir)/gtest/include \
                         -I$(top_builddir)/gtest/include
test_libten90_LDADD = $(libten90_la_LIBADD) $(top_builddir)/libten90.la \
                      $(top_builddir)/gtest/lib/libgtest.la \
                      $(top_builddir)/gtest/lib/libgtest_main.la

test_libten90_SOURCES = test_libten90.c

TESTS = test_libten90
