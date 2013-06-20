#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.66])
AC_INIT([libdump1090], [0.0.0], [jjwiseman@gmail.com])

AM_INIT_AUTOMAKE([dist-bzip2])
LT_INIT

AC_PROG_MAKE_SET
AC_PROG_CC
AC_PROG_INSTALL
AC_PROG_LIBTOOL

PKG_CHECK_MODULES(LIBRTLSDR, librtlsdr)
LIBS="$LIBS $LIBRTLSDR_LIBS"
CFLAGS="$CFLAGS $LIBRTLSDR_CFLAGS"

dnl Check for required libraries.
dnl libpthread
AC_CHECK_LIB(pthread, pthread_create, [LIBS="$LIBS -lpthread"])
dnl libmath.
AC_CHECK_LIB(m, atan2, [LIBS="$LIBS -lm"])

# Checks for header files.
AC_HEADER_STDC
AC_CHECK_HEADERS([fcntl.h stdint.h stdlib.h string.h sys/time.h sys/timeb.h unistd.h])

# pc variables
AC_SUBST(dump1090_PC_LIBS,["$LIBS"])
AC_SUBST(dump1090_PC_CFLAGS,["$CFLAGS"])

AC_MSG_CHECKING(whether compiler understands -Wall)
old_CFLAGS="$CFLAGS"
CFLAGS="$CFLAGS -Wall -Wextra -Wno-unused -Wsign-compare"
AC_TRY_COMPILE([],[],
  AC_MSG_RESULT(yes),
  AC_MSG_RESULT(no)
  CFLAGS="$old_CFLAGS")

AC_CONFIG_MACRO_DIR([m4])

AC_CONFIG_SRCDIR([libdump1090.c])

AC_CONFIG_HEADERS([config.h])
AC_CONFIG_FILES([Makefile])
AC_OUTPUT
