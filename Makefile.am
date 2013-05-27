bin_PROGRAMS = \
	ptp

AM_CPPFLAGS = \
	-I$(top_srcdir)/include \
	-I$(top_builddir)/include \
	$(GLIB_CFLAGS) \
	-pthread

EXTRA_DIST = \
	autogen.sh

MAINTAINERCLEANFILES = \
	configure \
	aclocal.m4 \
	Makefile.in

#
# binary
#
ptp_SOURCES = \
	ptp.c

ptp_LDADD = \
	-lpthread \
	$(GLIB_LIBS)
