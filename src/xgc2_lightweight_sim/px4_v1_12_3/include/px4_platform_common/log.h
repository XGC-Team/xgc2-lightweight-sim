#pragma once
// Standalone core never logs through PX4 platform services.
// The upstream defines.h includes this header; none of the extracted control
// functions call a logging macro. No runtime service is supplied here.

#ifndef __EXPORT
#define __EXPORT __attribute__((visibility("default")))
#endif
