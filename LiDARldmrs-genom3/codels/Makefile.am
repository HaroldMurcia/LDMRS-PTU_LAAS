lib_LTLIBRARIES = libLiDARldmrs_codels.la

libLiDARldmrs_codels_la_SOURCES=	LiDARldmrs_c_types.h
libLiDARldmrs_codels_la_SOURCES+=	LiDARldmrs_codels.c
libLiDARldmrs_codels_la_SOURCES+=	LiDARldmrs_scan_pub_codels.c

libLiDARldmrs_codels_la_CPPFLAGS=	$(requires_CFLAGS)
libLiDARldmrs_codels_la_LDFLAGS=	$(requires_LIBS)
libLiDARldmrs_codels_la_LDFLAGS+=	-release $(PACKAGE_VERSION)


# idl  mappings
BUILT_SOURCES=	LiDARldmrs_c_types.h
CLEANFILES=	${BUILT_SOURCES}
LiDARldmrs_c_types.h: ${top_srcdir}/LiDARldmrs.gen
	${GENOM3}  mappings \
	  -MD -MF .deps/$@.d -MT $@ --signature -l c $< >$@

-include .deps/LiDARldmrs_c_types.h.d
