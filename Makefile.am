SUBDIRS = . tests

# Always include gtest in distributions.
DIST_SUBDIRS = $(subdirs) gtest


# Build gtest before we build protobuf tests.  We don't add gtest to SUBDIRS
# because then "make check" would also build and run all of gtest's own tests,
# which takes a lot of time and is generally not useful to us.  Also, we don't
# want "make install" to recurse into gtest since we don't want to overwrite
# the installed version of gtest if there is one.
check-local:
	@echo "Making lib/libgtest.a lib/libgtest_main.a in gtest"
	@cd gtest && $(MAKE) $(AM_MAKEFLAGS) lib/libgtest.la lib/libgtest_main.la

# We would like to clean gtest when "make clean" is invoked.  But we have to
# be careful because clean-local is also invoked during "make distclean", but
# "make distclean" already recurses into gtest because it's listed among the
# DIST_SUBDIRS.  distclean will delete gtest/Makefile, so if we then try to
# cd to the directory again and "make clean" it will fail.  So, check that the
# Makefile exists before recursing.
clean-local:
	@if test -e gtest/Makefile; then \
	  echo "Making clean in gtest"; \
	  cd gtest && $(MAKE) $(AM_MAKEFLAGS) clean; \
	fi


lib_LTLIBRARIES = libten90.la

libten90_la_SOURCES = libten90.c
include_HEADERS = ten90.h

bin_PROGRAMS = dump1090

#noinst_PROGRAMS = test_libten90

dump1090_SOURCES = dump1090.c anet.c
dump1090_CFLAGS = $(libten90_la_CFLAGS)
dump1090_LDADD = $(libten90_la_LIBADD) libten90.la
