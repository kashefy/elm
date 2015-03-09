# ----------------------------------------------------------------------------
# CMake Macro for setting up build for profiling
#
# Profiling implies optimizing the build
# From https://gcc.gnu.org/onlinedocs/gcc/Optimize-Options.html:
#   -O3 turns on all optimizations specified by -O2
#   and also turns on the
#   -finline-functions, -funswitch-loops, -fpredictive-commoning, -fgcse-after-reload, -ftree-loop-vectorize, -ftree-loop-distribute-patterns, -ftree-slp-vectorize, -fvect-cost-model, -ftree-partial-pre and -fipa-cp-clone
#   options.
# TODO: applicable to compilers other than GNU?
# ----------------------------------------------------------------------------
MACRO(SETUP_PROFILING)
	set(CMAKE_CXX_FLAGS_PROFILE "-O3 -pg")
	set(CMAKE_C_FLAGS_PROFILE   "-O3 -pg")
	set(CMAKE_EXE_LINKER_FLAGS_PROFILE      "-pg")
	set(CMAKE_MODULE_LINKER_FLAGS_PROFILE   "-pg")
ENDMACRO(SETUP_PROFILING)
