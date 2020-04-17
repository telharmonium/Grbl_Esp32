// This file is where you choose the machine type, by including
// one or more machine definition files as described below.

#ifndef _machine_h
#define _machine_h

#ifndef MACHINE_FILENAME

// !!! For initial testing, start with test_drive.h which disables
// all I/O pins
#include "Machines/4axis_external_driver.h"

// !!! For actual use, change the line above to select a board
// from Machines/, for example:
// #include "Machines/3axis_v4.h"

// Some configurations use two files, the first establishing a base
// configuration and the second providing additional customization,
// for example:
// #include "Machines/3axis_v4.h"
// #include "Machines/add_esc_spindle.h"

// === OEM Single File Configuration Option
// OEMs that wish to publish source code that is configured for a
// specific machine may put all of their configuration definitions
// directly in this file, without including any other file above.

#else

// By using the external environment to define MACHINE_FILENAME,
// a configuration can be chosen without editing this file.
// That is useful for automated testing scripts.
//
// For example, when using the platformio compilation environment
// under Linux, you could issue the following command line:
//   PLATFORMIO_BUILD_FLAGS=-DMACHINE_FILENAME=3axis_v4.h platformio run
//
// Under Windows, using PowerShell, the command would be:
//   $env:PLATFORMIO_BUILD_FLAGS='-DMACHINE_FILENAME=3axis_v4.h'; platformio run
//
// When using the Arduino IDE, there is no easy way to pass variables
// to the compiler, so this feature is not useful for Arduino.
//
// MACHINE_FILENAME must not include the Machines/ path prefix; it is
// supplied automatically.

// MACHINE_PATHNAME_QUOTED constructs a path that is suitable for #include
#define MACHINE_PATHNAME_QUOTED(name) <Machines/name>

#include MACHINE_PATHNAME_QUOTED(MACHINE_FILENAME)

// You can choose two-file configurations by also defining MACHINE_FILENAME2,
// for example:
//   $env:PLATFORMIO_BUILD_FLAGS='-DMACHINE_FILENAME=3axis_v4.h -DMACHINE_FILENAME2=add_esc_spindle.h'; platformio run

#ifdef MACHINE_FILENAME2
#include MACHINE_PATHNAME_QUOTED(MACHINE_FILENAME2)
#endif

#endif // MACHINE_FILENAME

#endif // _machine_h
