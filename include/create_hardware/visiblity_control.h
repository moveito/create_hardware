#pragma once

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define CREATE_HARDWARE_EXPORT __attribute__((dllexport))
#define CREATE_HARDWARE_IMPORT __attribute__((dllimport))
#else
#define CREATE_HARDWARE_EXPORT __declspec(dllexport)
#define CREATE_HARDWARE_IMPORT __declspec(dllimport)
#endif
#ifdef CREATE_HARDWARE_BUILDING_DLL
#define CREATE_HARDWARE_PUBLIC CREATE_HARDWARE_EXPORT
#else
#define CREATE_HARDWARE_PUBLIC CREATE_HARDWARE_IMPORT
#endif
#define CREATE_HARDWARE_PUBLIC_TYPE CREATE_HARDWARE_PUBLIC
#define CREATE_HARDWARE_LOCAL
#else
#define CREATE_HARDWARE_EXPORT __attribute__((visibility("default")))
#define CREATE_HARDWARE_IMPORT
#if __GNUC__ >= 4
#define CREATE_HARDWARE_PUBLIC __attribute__((visibility("default")))
#define CREATE_HARDWARE_LOCAL __attribute__((visibility("hidden")))
#else
#define CREATE_HARDWARE_PUBLIC
#define CREATE_HARDWARE_LOCAL
#endif
#define CREATE_HARDWARE_PUBLIC_TYPE
#endif
