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

examples/flash_test
^^^^^^^^^^^^^^^^^^^

  This example performs a SMART flash block device test.  This test performs
  a sector allocate, read, write, free and garbage collection test on a SMART
  MTD block device.

    * CONFIG_EXAMPLES_FLASH_TEST=y - Enables the FLASH Test

  Dependencies:

    * CONFIG_MTD_SMART=y - SMART block driver support
    * CONFIG_NSH_BUILTIN_APPS=y - This example can only be built as an NSH
      command
    * CONFIG_BUILD_PROTECTED=n and CONFIG_BUILD_KERNEL=n- This test uses
      internal OS interfaces and so is not available in the NUTTX kernel
      builds

examples/hello
^^^^^^^^^^^^^^

  This is the mandatory, "Hello, World!!" example.  It is little more
  than examples/null with a single printf statement.  Really useful only
  for bringing up new NuttX architectures.

  * CONFIG_NSH_BUILTIN_APPS
    Build the "Hello, World" example as an NSH built-in application.

examples/helloxx
^^^^^^^^^^^^^^^^

  This is C++ version of the "Hello, World!!" example.  It is intended
  only to verify that the C++ compiler is functional, that basic C++
  library suupport is available, and that class are instantiated
  correctly.

  NuttX configuration prerequisites:

    CONFIG_HAVE_CXX -- Enable C++ Support

  Optional NuttX configuration settings:

    CONFIG_HAVE_CXXINITIALIZE -- Enable support for static constructors
      (may not be available on all platforms).

  NuttX configuration settings specific to this examp;le:

    CONFIG_NSH_BUILTIN_APPS -- Build the helloxx example as a
      "built-in"  that can be executed from the NSH command line.
    CONFIG_EXAMPLES_HELLOXX_NOSTACKCONST - Set if the system does not
      support construction of objects on the stack.
    CONFIG_EXAMPLES_HELLOXX_CXXINITIALIZE - By default, if CONFIG_HAVE_CXX
      and CONFIG_HAVE_CXXINITIALIZE are defined, then this example
      will call the NuttX function to initialize static C++ constructors.
      This option may be disabled, however, if that static initialization
      was performed elsewhere.

  Also needed:

    CONFIG_HAVE_CXX=y

  And you may have to tinker with the following to get libxx to compile
  properly:

    CONFIG_CXX_NEWLONG=y or =n

  The argument of the 'new' operators should take a type of size_t.  But size_t
  has an unknown underlying.  In the nuttx sys/types.h header file, size_t
  is typed as uint32_t (which is determined by architecture-specific logic).
  But the C++ compiler may believe that size_t is of a different type resulting
  in compilation errors in the operator.  Using the underlying integer type
  Instead of size_t seems to resolve the compilation issues.

examples/igmp
^^^^^^^^^^^^^

  This is a trivial test of the NuttX IGMP capability.  It present it
  does not do much of value -- Much more is needed in order to verify
  the IGMP features!

  * CONFIG_EXAMPLES_IGMP_NOMAC
      Set if the hardware has no MAC address; one will be assigned
  * CONFIG_EXAMPLES_IGMP_IPADDR
      Target board IP address
  * CONFIG_EXAMPLES_IGMP_DRIPADDR
      Default router address
  * CONFIG_EXAMPLES_IGMP_NETMASK
      Network mask
  * CONFIG_EXAMPLES_IGMP_GRPADDR
      Multicast group address
  * CONFIG_EXAMPLES_NETLIB
      The networking library is needed

examples/i2c
^^^^^^^^^^^^

  A mindlessly simple test of an I2C driver.  It reads an write garbage data to the
  I2C transmitter and/or received as fast possible.

  This test depends on these specific I2S/AUDIO/NSH configurations settings (your
  specific I2S settings might require additional settings).

    CONFIG_I2S - Enabled I2S support
    CONFIG_AUDIO - Enabled audio support
    CONFIG_AUDIO_DEVICES - Enable audio device support
    CONFIG_AUDIO_I2SCHAR = Enabled support for the I2S character device
    CONFIG_NSH_BUILTIN_APPS - Build the I2S test as an NSH built-in function.
      Default: Built as a standalone problem

  Specific configuration options for this example include:

    CONFIG_EXAMPLES_I2SCHAR - Enables the I2C test
    CONFIG_EXAMPLES_I2SCHAR_DEVPATH - The default path to the ADC device.
      Default: /dev/i2schar0
    CONFIG_EXAMPLES_I2SCHAR_TX - This should be set if the I2S device supports
      a transmitter.
    CONFIG_EXAMPLES_I2SCHAR_TXBUFFERS - This is the default number of audio
      buffers to send before the TX transfers terminate.  When both TX and
      RX transfers terminate, the task exits (and, if an NSH builtin, the
      i2schar command returns).  This number can be changed from the NSH
      command line.
    CONFIG_EXAMPLES_I2SCHAR_TXSTACKSIZE - This is the stack size to use when
      starting the transmitter thread.  Default 1536.
    CONFIG_EXAMPLES_I2SCHAR_RX - This should be set if the I2S device supports
      a transmitter.
    CONFIG_EXAMPLES_I2SCHAR_RXBUFFERS - This is the default number of audio
      buffers to receive before the RX transfers terminate.  When both TX and
      RX transfers terminate, the task exits (and, if an NSH builtin, the
      i2schar command returns).  This number can be changed from the NSH
      command line.
    CONFIG_EXAMPLES_I2SCHAR_RXSTACKSIZE - This is the stack size to use when
      starting the receiver thread.  Default 1536.
    CONFIG_EXAMPLES_I2SCHAR_BUFSIZE - The size of the data payload in one
      audio buffer.  Applies to both TX and RX audio buffers.
    CONFIG_EXAMPLES_I2SCHAR_DEVINIT - Define if architecture-specific I2S
      device initialize is available.  If defined, the the platform specific
      code must provide a function i2schar_devinit() that will be called
      each time that this test executes.  Not available in the kernel build
      mode.

examples/json
^^^^^^^^^^^^^

  This example exercises the cJSON implementation at apps/netutils/json.
  This example contains logic taken from the cJSON project:

    http://sourceforge.net/projects/cjson/

  The example corresponds to SVN revision r42 (with lots of changes for
  NuttX coding standards).  As of r42, the SVN repository was last updated
  on 2011-10-10 so I presume that the code is stable and there is no risk
  of maintaining duplicate logic in the NuttX repository.

examples/keypadtest
^^^^^^^^^^^^^^^^^^^

  This is a generic keypad test example.  It is similar to the USB hidkbd
  example, but makes no assumptions about the underlying keyboard interface.
  It uses the interfaces of include/nuttx/input/keypad.h.

  CONFIG_EXAMPLES_KEYPADTEST - Selects the keypadtest example (only need
    if the mconf/Kconfig tool is used.

  CONFIG_EXAMPLES_KEYPAD_DEVNAME - The name of the keypad device that will
    be opened in order to perform the keypad test.  Default: "/dev/keypad"

examples/mm
^^^^^^^^^^^

  This is a simple test of the memory manager.

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

examples/mount
^^^^^^^^^^^^^^

  This contains a simple test of filesystem mountpoints.

  * CONFIG_EXAMPLES_MOUNT_DEVNAME
      The name of the user-provided block device to mount.
      If CONFIG_EXAMPLES_MOUNT_DEVNAME is not provided, then
      a RAM disk will be configured.

  * CONFIG_EXAMPLES_MOUNT_NSECTORS
      The number of "sectors" in the RAM disk used when
      CONFIG_EXAMPLES_MOUNT_DEVNAME is not defined.

  * CONFIG_EXAMPLES_MOUNT_SECTORSIZE
      The size of each sectors in the RAM disk used when
      CONFIG_EXAMPLES_MOUNT_DEVNAME is not defined.

  * CONFIG_EXAMPLES_MOUNT_RAMDEVNO
      The RAM device minor number used to mount the RAM disk used
      when CONFIG_EXAMPLES_MOUNT_DEVNAME is not defined.  The
      default is zero (meaning that "/dev/ram0" will be used).

examples/mtdpart
^^^^^^^^^^^^^^^^

  This examples provides a simple test of MTD partition logic.

  * CONFIG_EXAMPLES_MTDPART - Enables the MTD partition test example
  * CONFIG_EXAMPLES_MTDPART_ARCHINIT - The default is to use the RAM MTD
    device at drivers/mtd/rammtd.c. But an architecture-specific MTD driver
    can be used instead by defining CONFIG_EXAMPLES_MTDPART_ARCHINIT.  In
    this case, the initialization logic will call mtdpart_archinitialize()
    to obtain the MTD driver instance.
  * CONFIG_EXAMPLES_MTDPART_NPARTITIONS - This setting provides the number
    of partitions to test.  The test will divide the reported size of the
    MTD device into equal-sized sub-regions for each test partition. Default:
    3

  When CONFIG_EXAMPLES_MTDPART_ARCHINIT is not defined, this test will use
  the RAM MTD device at drivers/mtd/rammtd.c to simulate FLASH. The size of
  the allocated RAM drive will be: CONFIG_EXMPLES_RAMMTD_ERASESIZE *
  CONFIG_EXAMPLES_MTDPART_NEBLOCKS

  * CONFIG_EXAMPLES_MTDPART_ERASESIZE - This value gives the size of one
    erase block in the MTD RAM device. This must exactly match the default
    configuration in drivers/mtd/rammtd.c!
  * CONFIG_EXAMPLES_MTDPART_NEBLOCKS - This value gives the nubmer of erase
    blocks in MTD RAM device.

examples/mtdrwb
^^^^^^^^^^^^^^^^

  This examples provides a simple test of MTD Read-Ahead/Write buffering
  logic.

  * CONFIG_EXAMPLES_MTDRWB - Enables the MTD R/W buffering test example
  * CONFIG_EXAMPLES_MTDRWB_ARCHINIT - The default is to use the RAM MTD
    device at drivers/mtd/rammtd.c. But an architecture-specific MTD driver
    can be used instead by defining CONFIG_EXAMPLES_MTDRWB_ARCHINIT.  In
    this case, the initialization logic will call mtdrwb_archinitialize()
    to obtain the MTD driver instance.

  When CONFIG_EXAMPLES_MTDRWB_ARCHINIT is not defined, this test will use
  the RAM MTD device at drivers/mtd/rammtd.c to simulate FLASH. The size of
  the allocated RAM drive will be: CONFIG_EXMPLES_RAMMTD_ERASESIZE *
  CONFIG_EXAMPLES_MTDRWB_NEBLOCKS

  * CONFIG_EXAMPLES_MTDRWB_ERASESIZE - This value gives the size of one
    erase block in the MTD RAM device. This must exactly match the default
    configuration in drivers/mtd/rammtd.c!
  * CONFIG_EXAMPLES_MTDRWB_NEBLOCKS - This value gives the nubmer of erase
    blocks in MTD RAM device.

examples/nrf24l01_term
^^^^^^^^^^^^^^^^^^^^^^

  These is a simple test of NRF24L01-based wireless connectivity.  Enabled\
  with:

    CONFIG_EXAMPLES_NRF24L01TERM

  Options:

    CONFIG_NSH_BUILTIN_APPS - Built as an NSH built-in applications.

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
