dnl Autoconf file for building platine_light codels library.

AC_PREREQ(2.59)

AC_INIT([platine_light-genom3],[0.1],[openrobots@laas.fr])
AC_CONFIG_MACRO_DIR([autoconf])
AC_CONFIG_AUX_DIR([autoconf])
AC_CONFIG_HEADERS([autoconf/acplatine_light.h])
AM_INIT_AUTOMAKE([foreign no-define])

dnl Compilers
dnl
LT_INIT
AC_PROG_CC
AC_PROG_CXX


dnl External packages
PKG_CHECK_MODULES(requires, [
  openrobots-idl >= 1.0
  genom3 >= 2.99.20
])

_PKG_CONFIG(genom3, [variable=GENOM_TOOL], [genom3 >= 2.99.20])
AC_SUBST(genom3, [$pkg_cv_genom3])


dnl --with-templates option
AG_OPT_TEMPLATES([$genom3 ],
    [$srcdir/platine-light.gen])


dnl Output
AC_CONFIG_FILES([
	platine_light-genom3.pc
	platine_light-genom3-uninstalled.pc
	Makefile
	codels/Makefile
])
AC_OUTPUT
AG_OUTPUT_TEMPLATES
