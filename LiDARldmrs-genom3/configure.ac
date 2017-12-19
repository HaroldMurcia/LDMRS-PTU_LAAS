dnl Autoconf file for building LiDARldmrs codels library.

AC_PREREQ(2.59)

AC_INIT([LiDARldmrs-genom3],[0.1],[openrobots@laas.fr])
AC_CONFIG_MACRO_DIR([autoconf])
AC_CONFIG_AUX_DIR([autoconf])
AC_CONFIG_HEADERS([autoconf/acLiDARldmrs.h])
AM_INIT_AUTOMAKE([foreign no-define])

dnl Compilers
dnl
LT_INIT
AC_PROG_CC


dnl External packages
PKG_CHECK_MODULES(requires, [
  genom3 >= 2.99.20
])

AC_PATH_PROG(GENOM3, [genom3], AC_MSG_ERROR([genom3 tool not found], 2))


dnl --with-templates option
AG_OPT_TEMPLATES([$GENOM3 ],
    [$srcdir/LiDARldmrs.gen])


dnl Output
AC_CONFIG_FILES([
	LiDARldmrs-genom3.pc
	LiDARldmrs-genom3-uninstalled.pc
	Makefile
	codels/Makefile
])
AC_OUTPUT
AG_OUTPUT_TEMPLATES
