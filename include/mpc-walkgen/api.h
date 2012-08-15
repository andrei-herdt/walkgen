#pragma once
#ifndef MPC_WALKGEN_API_H
#define MPC_WALKGEN_API_H

////////////////////////////////////////////////////////////////////////////////
///
///\file	api.h
///\brief	Definition of public API
///\author	Barthélémy Sébastien
///\author Andrei herdt
////////////////////////////////////////////////////////////////////////////////


#if defined _WIN32 || defined __CYGWIN__
#  define EXPORT_API __declspec(dllexport)
#  if defined _WINDLL
#    define IMPORT_API __declspec(dllimport)
#  else
#    define IMPORT_API
#  endif
#elif __GNUC__ >= 4
#  define EXPORT_API __attribute__ ((visibility("default")))
#  define IMPORT_API __attribute__ ((visibility("default")))
#else
#  define EXPORT_API
#  define IMPORT_API
#endif

// mpc_walkgen_EXPORTS is defined by the build system, only when building the
// library
#ifdef mpc_walkgen_EXPORTS
# define MPC_WALKGEN_API EXPORT_API
#else
# define MPC_WALKGEN_API IMPORT_API
#endif

#endif  // MPC_WALKGEN_API_H
