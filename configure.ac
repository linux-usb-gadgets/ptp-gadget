AC_PREREQ(2.59)

esyscmd([echo "define(1.1, $(dir=$(basename $(pwd)); if test \"${dir%%-trunk}\" != \"$dir\"; then echo 99999999-9; else prefix=${dir%%-[0-9]*-[0-9]}; echo ${dir##$prefix-}; fi))"])

AC_INIT([ptp], 1.1, [linux-usb@vger.kernel.org])
AC_CONFIG_SRCDIR([ptp.c])
AC_CANONICAL_BUILD
AC_CANONICAL_HOST
AC_SYS_LARGEFILE

AM_MAINTAINER_MODE

CFLAGS="${CFLAGS} -W -Wall"

#
# libtool library versioning stuff
#
# Library code modified:            REVISION++
# Interfaces changed/added/removed: CURRENT++   REVISION=0
# Interfaces added:                 AGE++
# Interfaces removed:               AGE=0
LT_CURRENT=0
LT_REVISION=0
LT_AGE=0
AC_SUBST(LT_CURRENT)
AC_SUBST(LT_REVISION)
AC_SUBST(LT_AGE)


#
# Checks for programs.
#
AC_PROG_CC
AC_PROG_LIBTOOL

AM_INIT_AUTOMAKE([foreign no-exeext])


##
# glib
##
REQUIRES_GLIB="glib >= 2.24"
AC_SUBST(REQUIRES_GLIB)
PKG_CHECK_MODULES([GLIB], [glib-2.0 > 2.24])

#
# Debugging
#
AC_MSG_CHECKING([whether to enable debugging])
AC_ARG_ENABLE(debug,
    AS_HELP_STRING([--enable-debug], [enable debugging @<:@default=yes@:>@]),
	[case "$enableval" in
	y | yes) CONFIG_DEBUG=yes ;;
        *) CONFIG_DEBUG=no ;;
    esac],
    [CONFIG_DEBUG=yes])
AC_MSG_RESULT([${CONFIG_DEBUG}])
if test "${CONFIG_DEBUG}" = "yes"; then
    CFLAGS="${CFLAGS} -Werror -Wsign-compare -Wfloat-equal -Wformat-security -g -O1"
    AC_DEFINE(DEBUG, 1, [debugging])
else
    CFLAGS="${CFLAGS} -O2"
fi


AC_CONFIG_FILES([
	Makefile
])
AC_OUTPUT
