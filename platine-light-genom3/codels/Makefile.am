lib_LTLIBRARIES = libplatine_light_codels.la

libplatine_light_codels_la_SOURCES=	platine_light_c_types.h
libplatine_light_codels_la_SOURCES+=	platine_light_codels.c
libplatine_light_codels_la_SOURCES+=	platine_light_pos_pub_codels.c
libplatine_light_codels_la_SOURCES+=	platine_light_cmd_codels.c
libplatine_light_codels_la_SOURCES+=    platine_driver.c 

libplatine_light_codels_la_CPPFLAGS=	$(requires_CFLAGS)
libplatine_light_codels_la_LDFLAGS=	$(requires_LIBS)
libplatine_light_codels_la_LDFLAGS+=	-release $(PACKAGE_VERSION)


# idl  mappings
BUILT_SOURCES=	platine_light_c_types.h
CLEANFILES=	${BUILT_SOURCES}
platine_light_c_types.h: ${top_srcdir}/platine-light.gen
	${genom3}  mappings \
	  -MD -MF .deps/$@.d -MT $@ --signature -l c $< >$@

-include .deps/platine_light_c_types.h.d
