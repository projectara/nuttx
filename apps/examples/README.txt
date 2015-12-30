examples
^^^^^^^^

  Selecting examples:

    The examples directory contains several sample applications that
    can be linked with NuttX.  The specific example is selected in the
    configs/<board-name>/defconfig file via the CONFIG_EXAMPLES_xyz
    setting where xyz is the name of the example. For example,

      CONFIG_EXAMPLES_OSTEST=y

    Selects the examples/ostest example.

  Built-In functions

    Some of the examples may be built as "built-in" functions that
    can be executed at run time (rather than as NuttX "main" programs).
    These "built-in" examples can be also be executed from the NuttShell
    (NSH) command line.  In order to configure these built-in  NSH
    functions, you have to set up the following:

    - CONFIG_NSH_BUILTIN_APPS - Enable support for external registered,
      "named" applications that can be executed from the NSH
      command line (see apps/README.txt for more information).

examples/cpuhog
^^^^^^^^^^^^^^^

  Attempts to keep the system busy by passing data through a pipe in loop
  back mode.  This may be useful if you are trying run down other problems
  that you think might only occur when the system is very busy.

examples/cxxtest
^^^^^^^^^^^^^^^^

  This is a test of the C++ standard library.  At present a port of the uClibc++
  C++ library is available.  Due to licensing issues, the uClibc++ C++ library
  is not included in the NuttX source tree by default, but must be installed
  (see misc/uClibc++/README.txt for installation).

  The uClibc++ test includes simple test of:

    - iostreams,
    - STL,
    - RTTI, and
    - Exceptions

  Example Configuration Options
  -----------------------------
    CONFIG_EXAMPLES_CXXTEST=y - Eanbles the example
    CONFIG_EXAMPLES_CXXTEST_CXXINITIALIZE=y - By default, if CONFIG_HAVE_CXX
      and CONFIG_HAVE_CXXINITIALIZE are defined, then this example
      will call the NuttX function to initialize static C++ constructors.
      This option may be disabled, however, if that static initialization
      was performed elsewhere.

  Other Required Configuration Settings
  -------------------------------------
  Other NuttX setting that are required include:

    CONFIG_HAVE_CXX=y
    CONFIG_HAVE_CXXINITIALIZE=y
    CONFIG_UCLIBCXX=y

  Additional uClibc++ settings may be required in your build environment.

examples/elf
^^^^^^^^^^^^

  This example builds a small ELF loader test case.  This includes several
  test programs under examples/elf tests.  These tests are build using
  the relocatable ELF format and installed in a ROMFS file system.  At run time,
  each program in the ROMFS file system is executed.  Requires CONFIG_ELF.
  Other configuration options:

    CONFIG_EXAMPLES_ELF_DEVMINOR - The minor device number of the ROMFS block
      driver. For example, the N in /dev/ramN. Used for registering the RAM
      block driver that will hold the ROMFS file system containing the ELF
      executables to be tested.  Default: 0

    CONFIG_EXAMPLES_ELF_DEVPATH - The path to the ROMFS block driver device.  This
      must match EXAMPLES_ELF_DEVMINOR. Used for registering the RAM block driver
      that will hold the ROMFS file system containing the ELF executables to be
      tested.  Default: "/dev/ram0"

  NOTES:

  1. CFLAGS should be provided in CELFFLAGS.  RAM and FLASH memory regions
     may require long allcs.  For ARM, this might be:

       CELFFLAGS = $(CFLAGS) -mlong-calls

     Similarly for C++ flags which must be provided in CXXELFFLAGS.

  2. Your top-level nuttx/Make.defs file must also include an approproate definition,
     LDELFFLAGS, to generate a relocatable ELF object.  With GNU LD, this should
     include '-r' and '-e main' (or _main on some platforms).

       LDELFFLAGS = -r -e main

     If you use GCC to link, you make also need to include '-nostdlib' or
     '-nostartfiles' and '-nodefaultlibs'.

  3. This example also requires genromfs.  genromfs can be build as part of the
     nuttx toolchain.  Or can built from the genromfs sources that can be found
     at misc/tools/genromfs-0.5.2.tar.gz.  In any event, the PATH variable must
     include the path to the genromfs executable.

  4. ELF size:  The ELF files in this example are, be default, quite large
     because they include a lot of "build garbage".  You can greatly reduce the
     size of the ELF binaries are using the 'objcopy --strip-unneeded' command to
     remove un-necessary information from the ELF files.

  5. Simulator.  You cannot use this example with the the NuttX simulator on
     Cygwin.  That is because the Cygwin GCC does not generate ELF file but
     rather some Windows-native binary format.

     If you really want to do this, you can create a NuttX x86 buildroot toolchain
     and use that be build the ELF executables for the ROMFS file system.

  6. Linker scripts.  You might also want to use a linker scripts to combine
     sections better.  An example linker script is at nuttx/binfmt/libelf/gnu-elf.ld.
     That example might have to be tuned for your particular linker output to
     position additional sections correctly.  The GNU LD LDELFFLAGS then might
     be:

       LDELFFLAGS = -r -e main -T$(TOPDIR)/binfmt/libelf/gnu-elf.ld

examples/hello
^^^^^^^^^^^^^^

  This is the mandatory, "Hello, World!!" example.  It is little more
  than examples/null with a single printf statement.  Really useful only
  for bringing up new NuttX architectures.

  * CONFIG_NSH_BUILTIN_APPS
    Build the "Hello, World" example as an NSH built-in application.

examples/modbus
^^^^^^^^^^^^^^^

  This is a port of the FreeModbus Linux demo.  It derives from the
  demos/LINUX directory of the FreeModBus version 1.5.0 (June 6, 2010)
  that can be downloaded in its entirety from http://developer.berlios.de/project/showfiles.php?group_id=6120.

    CONFIG_EXAMPLES_MODBUS_PORT, Default 0 (for /dev/ttyS0)
    CONFIG_EXAMPLES_MODBUS_BAUD, Default B38400
    CONFIG_EXAMPLES_MODBUS_PARITY, Default MB_PAR_EVEN

    CONFIG_EXAMPLES_MODBUS_REG_INPUT_START, Default 1000
    CONFIG_EXAMPLES_MODBUS_REG_INPUT_NREGS, Default 4
    CONFIG_EXAMPLES_MODBUS_REG_HOLDING_START, Default 2000
    CONFIG_EXAMPLES_MODBUS_REG_HOLDING_NREGS, Default 130

  The FreeModBus library resides at apps/modbus.  See apps/modbus/README.txt
  for additional configuration information.

examples/nsh
^^^^^^^^^^^^

  Basic Configuration
  -------------------
  This directory provides an example of how to configure and use
  the NuttShell (NSH) application.  NSH is a simple shell
  application.  NSH is described in its own README located at
  apps/nshlib/README.txt.  This function is enabled with:

    CONFIG_EXAMPLES_NSH=y

  Applications using this example will need to provide an defconfig
  file in the configuration directory with instruction to build
  the NSH library like:

    CONFIG_NSH_LIBRARY=y

  Other Configuration Requirements
  --------------------------------
  NOTE:  If the NSH serial console is used, then following is also
  required to build the readline() library:

    CONFIG_SYSTEM_READLINE=y

  And if networking is included:

    CONFIG_NETUTILS_NETLIB=y
    CONFIG_NETUTILS_DHCPC=y
    CONFIG_NETUTILS_DNSCLIENT=y
    CONFIG_NETUTILS_TFTPC=y
    CONFIG_NETUTILS_WEBCLIENT=y

  If the Telnet console is enabled, then the defconfig file should
  also include:

    CONFIG_NETUTILS_TELNETD=y

  Also if the Telnet console is enabled, make sure that you have the
  following set in the NuttX configuration file or else the performance
  will be very bad (because there will be only one character per TCP
  transfer):

    CONFIG_STDIO_BUFFER_SIZE - Some value >= 64
    CONFIG_STDIO_LINEBUFFER=y

  C++ Support
  -----------
  If CONFIG_HAVE_CXX=y and CONFIG_HAVE_CXXINITIALIZE=y, then this NSH
  example can be configured to initialize C++ constructors when it
  is started.  NSH does not use C++ and, by default, assumes that
  constructors are initialized elsewhere.  However, you can force
  NSH to initialize constructors by setting:

    CONFIG_EXAMPLES_NSH_CXXINITIALIZE=y

examples/nxflat
^^^^^^^^^^^^^^^

  This example builds a small NXFLAT test case.  This includes several
  test programs under examples/nxflat tests.  These tests are build using
  the NXFLAT format and installed in a ROMFS file system.  At run time,
  each program in the ROMFS file system is executed.  Requires CONFIG_NXFLAT.
