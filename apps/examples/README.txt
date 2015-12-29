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

examples/nxffs
^^^^^^^^^^^^^^

  This is a test of the NuttX NXFFS FLASH file system.  This is an NXFFS
  stress test and beats on the file system very hard.  It should only
  be used in a simulation environment!  Putting this NXFFS test on real
  hardware will most likely destroy your FLASH.  You have been warned.

examples/nxflat
^^^^^^^^^^^^^^^

  This example builds a small NXFLAT test case.  This includes several
  test programs under examples/nxflat tests.  These tests are build using
  the NXFLAT format and installed in a ROMFS file system.  At run time,
  each program in the ROMFS file system is executed.  Requires CONFIG_NXFLAT.

examples/null
^^^^^^^^^^^^^

  This is the do nothing application.  It is only used for bringing
  up new NuttX architectures in the most minimal of environments.

examples/ostest
^^^^^^^^^^^^^^^

  This is the NuttX 'qualification' suite.  It attempts to exercise
  a broad set of OS functionality.  Its coverage is not very extensive
  as of this writing, but it is used to qualify each NuttX release.

  The behavior of the ostest can be modified with the following
  settings in the configs/<board-name>/defconfig file:

  * CONFIG_NSH_BUILTIN_APPS
      Build the OS test example as an NSH built-in application.
  * CONFIG_EXAMPLES_OSTEST_LOOPS
      Used to control the number of executions of the test.  If
      undefined, the test executes one time.  If defined to be
      zero, the test runs forever.
  * CONFIG_EXAMPLES_OSTEST_STACKSIZE
      Used to create the ostest task.  Default is 8192.
  * CONFIG_EXAMPLES_OSTEST_NBARRIER_THREADS
      Specifies the number of threads to create in the barrier
      test.  The default is 8 but a smaller number may be needed on
      systems without sufficient memory to start so many threads.
  * CONFIG_EXAMPLES_OSTEST_RR_RANGE
      During round-robin scheduling test two threads are created. Each of the threads
      searches for prime numbers in the configurable range, doing that configurable
      number of times.
      This value specifies the end of search range and together with number of runs
      allows to configure the length of this test - it should last at least a few
      tens of seconds. Allowed values [1; 32767], default 10000
  * CONFIG_EXAMPLES_OSTEST_RR_RUNS
      During round-robin scheduling test two threads are created. Each of the threads
      searches for prime numbers in the configurable range, doing that configurable
      number of times.

examples/pashello
^^^^^^^^^^^^^^^^^

  This is "Hello, World" implemented via the Pascal P-Code interpreter. In
  order to use this example, you must first download and install the
  NuttX pascal module.  After unpacking the pascal module, you can find
  installation instructions in pascal/nuttx/README.txt.

  The correct install location for the NuttX examples and build files is
  apps/interpreters.

examples/pipe
^^^^^^^^^^^^^

  A test of the mkfifo() and pipe() APIs.

 * CONFIG_EXAMPLES_PIPE_STACKSIZE
     Sets the size of the stack to use when creating the child tasks.
     The default size is 1024.

examples/poll
^^^^^^^^^^^^^

  A test of the poll() and select() APIs using FIFOs and, if available,
  stdin, and a TCP/IP socket.  In order to build this test, you must the
  following selected in your NuttX configuration file:

  CONFIG_NFILE_DESCRIPTORS          - Defined to be greater than 0
  CONFIG_DISABLE_POLL               - NOT defined

  In order to use the TCP/IP select test, you have also the following
  additional things selected in your NuttX configuration file:

  CONFIG_NET                        - Defined for general network support
  CONFIG_NET_TCP                    - Defined for TCP/IP support
  CONFIG_NSOCKET_DESCRIPTORS        - Defined to be greater than 0
  CONFIG_NET_TCP_READAHEAD          - Defined
  CONFIG_NET_NTCP_READAHEAD_BUFFERS - Defined to be greater than zero

  CONFIG_EXAMPLES_POLL_NOMAC         - (May be defined to use software assigned MAC)
  CONFIG_EXAMPLES_POLL_IPADDR        - Target IP address
  CONFIG_EXAMPLES_POLL_DRIPADDR      - Default router IP addess
  CONFIG_EXAMPLES_POLL_NETMASK       - Network mask

  In order to for select to work with incoming connections, you
  must also select:

  CONFIG_NET_TCPBACKLOG             - Incoming connections pend in a backlog until accept() is called.

  In additional to the target device-side example, there is also
  a host-side application in this directory.  It can be compiled under
  Linux or Cygwin as follows:

    cd examples/poll
    make -f Makefile.host TOPDIR=<nuttx-directory> TARGETIP=<target-ip>

  Where <target-ip> is the IP address of your target board.

  This will generate a small program called 'host'.  Usage:

  1. Build the examples/poll target program with TCP/IP poll support
     and start the target.

  3. Then start the host application:

       ./host

  The host and target will exchange are variety of small messages. Each
  message sent from the host should cause the select to return in target.
  The target example should read the small message and send it back to
  the host.  The host should then receive the echo'ed message.

  If networking is enabled, applications using this example will need to
  provide the following definition in the defconfig file to enable the
  networking library:

    CONFIG_NETUTILS_NETLIB=y

examples/posix_spawn
^^^^^^^^^^^^^^^^^^^^

  This is a simple test of the posix_spawn() API. The example derives from
  examples/elf.  As a result, these tests are built using the relocatable
  ELF format installed in a ROMFS file system.  At run time, the test program
  in the ROMFS file system is spawned using posix_spawn().

  Requires:

    CONFIG_BINFMT_DISABLE=n           - Don't disable the binary loader
    CONFIG_ELF=y                      - Enable ELF binary loader
    CONFIG_LIBC_EXECFUNCS=y           - Enable support for posix_spawn
    CONFIG_EXECFUNCS_SYMTAB="exports" - The name of the symbol table
                                        created by the test.
    CONFIG_EXECFUNCS_NSYMBOLS=10      - Value does not matter, it will be
                                        corrected at runtime.
    CONFIG_POSIX_SPAWN_STACKSIZE=768  - This default setting.

  Test-specific configuration options:

    CONFIG_EXAMPLES_POSIXSPAWN_DEVMINOR - The minor device number of the ROMFS
      block. driver.  For example, the N in /dev/ramN. Used for registering the
      RAM block driver that will hold the ROMFS file system containing the ELF
      executables to be tested.  Default: 0

    CONFIG_EXAMPLES_POSIXSPAWN_DEVPATH - The path to the ROMFS block driver
      device.  This must match EXAMPLES_POSIXSPAWN_DEVMINOR. Used for
      registering the RAM block driver that will hold the ROMFS file system
      containing the ELF executables to be tested.  Default: "/dev/ram0"

  NOTES:

  1. CFLAGS should be provided in CELFFLAGS.  RAM and FLASH memory regions
     may require long allcs.  For ARM, this might be:

       CELFFLAGS = $(CFLAGS) -mlong-calls

     Similarly for C++ flags which must be provided in CXXELFFLAGS.

  2. Your top-level nuttx/Make.defs file must also include an approproate
     definition, LDELFFLAGS, to generate a relocatable ELF object.  With GNU
     LD, this should include '-r' and '-e main' (or _main on some platforms).

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

examples/pwm
^^^^^^^^^^^^

  A test of a PWM device driver. It simply enables a pulsed output for
  a specified frequency and duty for a specified period of time.  This
  example can ONLY be built as an NSH built-in function.

  This test depends on these specific PWM/NSH configurations settings (your
  specific PWM settings might require additional settings).

    CONFIG_PWM - Enables PWM support.
    CONFIG_PWM_PULSECOUNT - Enables PWM pulse count support (if the hardware
      supports it).
    CONFIG_NSH_BUILTIN_APPS - Build the PWM test as an NSH built-in function.
      Default: Not built!  The example can only be used as an NSH built-in
      application

  Specific configuration options for this example include:

    CONFIG_EXAMPLES_PWM_DEVPATH - The path to the default PWM device. Default: /dev/pwm0
    CONFIG_EXAMPLES_PWM_FREQUENCY - The initial PWM frequency.  Default: 100 Hz
    CONFIG_EXAMPLES_PWM_DUTYPCT - The initial PWM duty as a percentage.  Default: 50%
    CONFIG_EXAMPLES_PWM_DURATION - The initial PWM pulse train duration in seconds.
       Used only if the current pulse count is zero (pulse count is only supported
       if CONFIG_PWM_PULSECOUNT is defined). Default: 5 seconds
    CONFIG_EXAMPLES_PWM_PULSECOUNT - The initial PWM pulse count.  This option is
       only available if CONFIG_PWM_PULSECOUNT is non-zero. Default: 0 (i.e., use
       the duration, not the count).

examples/qencoder
^^^^^^^^^^^^^^^^^

  This example is a simple test of a Quadrature Encoder driver.  It simply reads
  positional data from the encoder and prints it.,

  This test depends on these specific QE/NSH configurations settings (your
  specific PWM settings might require additional settings).

    CONFIG_QENCODER - Enables quadrature encoder support (upper-half driver).
    CONFIG_NSH_BUILTIN_APPS - Build the QE test as an NSH built-in function.
      Default: Built as a standalone progrem.

  Additional configuration options will mostly likely be required for the board-
  specific lower-half driver.  See the README.txt file in your board configuration
  directory.

  Specific configuration options for this example include:

    CONFIG_EXAMPLES_QENCODER_DEVPATH - The path to the QE device. Default:
      /dev/qe0
    CONFIG_EXAMPLES_QENCODER_NSAMPLES - If CONFIG_NSH_BUILTIN_APPS
      is defined, then the number of samples is provided on the command line
      and this value is ignored.  Otherwise, this number of samples is
      collected and the program terminates.  Default:  Samples are collected
      indefinitely.
    CONFIG_EXAMPLES_QENCODER_DELAY - This value provides the delay (in
      milliseonds) between each sample.  If CONFIG_NSH_BUILTIN_APPS
      is defined, then this value is the default delay if no other delay is
      provided on the command line.  Default:  100 milliseconds

examples/random
^^^^^^^^^^^^^^^

  This is a very simply test of /dev/random.  It simple collects random
  numbers and displays them on the console.

  Prerequistes:

    CONFIG_DEV_RANDOM - Support for /dev/random must be enabled in order
      to select this example.

  Configuration:

    CONFIG_EXAMPLES_RANDOM - Enables the /dev/random test
    CONFIG_EXAMPLES_MAXSAMPLES - This is the size of the /dev/random I/O
      buffer in units of 32-bit samples.  Careful!  This buffer is allocated
      on the stack as needed! Default 64.
    CONFIG_EXAMPLES_NSAMPLES; - When you execute the rand command, a number
      of samples ranging from 1 to EXAMPLES_MAXSAMPLES may be specified.  If
      no argument is specified, this is the default number of samples that\
      will be collected and displayed.  Default 8

examples/relays
^^^^^^^^^^^^^^^

  Requires CONFIG_ARCH_RELAYS.
  Contributed by Darcy Gong.

  NOTE: This test exercises internal relay driver interfaces.  As such, it
  relies on internal OS interfaces that are not normally available to a
  user-space program.  As a result, this example cannot be used if a
  NuttX is built as a protected, supervisor kernel (CONFIG_BUILD_PROTECTED
  or CONFIG_BUILD_KERNEL).

examples/rgmp
^^^^^^^^^^^^^

  RGMP stands for RTOS and GPOS on Multi-Processor.  RGMP is a project for
  running GPOS and RTOS simultaneously on multi-processor platforms. You can
  port your favorite RTOS to RGMP together with an unmodified Linux to form a
  hybrid operating system. This makes your application able to use both RTOS
  and GPOS features.

  See http://rgmp.sourceforge.net/wiki/index.php/Main_Page for further

  At present, the RGMP example folder contains only an empty rgmp_main.c file.

examples/romfs
^^^^^^^^^^^^^^

  This example exercises the romfs filesystem.  Configuration options
  include:

  * CONFIG_EXAMPLES_ROMFS_RAMDEVNO
      The minor device number to use for the ROM disk.  The default is
      1 (meaning /dev/ram1)

  * CONFIG_EXAMPLES_ROMFS_SECTORSIZE
      The ROM disk sector size to use.  Default is 64.

  * CONFIG_EXAMPLES_ROMFS_MOUNTPOINT
      The location to mount the ROM disk.  Deafault: "/usr/local/share"

examples/sendmail
^^^^^^^^^^^^^^^^^

  This examples exercises the uIP SMTP logic by sending a test message
  to a selected recipient.  This test can also be built to execute on
  the Cygwin/Linux host environment:

    cd examples/sendmail
    make -f Makefile.host TOPDIR=<nuttx-directory>

 Settings unique to this example include:

    CONFIG_EXAMPLES_SENDMAIL_NOMAC     - May be defined to use software assigned MAC (optional)
    CONFIG_EXAMPLES_SENDMAIL_IPADDR    - Target IP address (required)
    CONFIG_EXAMPLES_SENDMAIL_DRIPADDR  - Default router IP addess (required)
    CONFIG_EXAMPLES_SENDMAILT_NETMASK  - Network mask (required)
    CONFIG_EXAMPLES_SENDMAIL_RECIPIENT - The recipient of the email (required)
    CONFIG_EXAMPLES_SENDMAIL_SENDER    - Optional. Default: "nuttx-testing@example.com"
    CONFIG_EXAMPLES_SENDMAIL_SUBJECT   - Optional. Default: "Testing SMTP from NuttX"
    CONFIG_EXAMPLES_SENDMAIL_BODY   -    Optional. Default: "Test message sent by NuttX"

  NOTE: This test has not been verified on the NuttX target environment.
  As of this writing, unit-tested in the Cygwin/Linux host environment.

  NOTE 2: This sendmail example only works for the simplest of
  environments.  Virus protection software on your host may have
  to be disabled to allow you to send messages.  Only very open,
  unprotected recipients can be used.  Most will protect themselves
  from this test email because it looks like SPAM.

  Applications using this example will need to enble the following
  netutils libraries in their defconfig file:

    CONFIG_NETUTILS_NETLIB=y
    CONFIG_NETUTILS_SMTP=y

examples/serialblaster
^^^^^^^^^^^^^^^^^^^^^^

  Sends a repeating pattern (the alphabet) out a serial port continuously.
  This may be useful if you are trying run down other problems that you
  think might only occur when the serial port usage is high.

examples/serialrx
^^^^^^^^^^^^^^^^^

  Constant receives serial data.  This is the complement to serialblaster.
  This may be useful if you are trying run down other problems that you
  think might only occur when the serial port usage is high.

examples/serloop
^^^^^^^^^^^^^^^^

  This is a mindlessly simple loopback test on the console.  Useful
  for testing new serial drivers.  Configuration options include:

  * CONFIG_EXAMPLES_SERLOOP_BUFIO
      Use C buffered I/O (getchar/putchar) vs. raw console I/O
      (read/read).

examples/smart
^^^^^^^^^^^^^^

  This is a test of the SMART file systemt that derives from
  examples/nxffs.

  * CONFIG_EXAMPLES_SMART: - Enable the SMART file system example
  * CONFIG_EXAMPLES_SMART_ARCHINIT: The default is to use the RAM MTD
    device at drivers/mtd/rammtd.c.  But an architecture-specific MTD
    driver can be used instead by defining CONFIG_EXAMPLES_SMART_ARCHINIT.  In
    this case, the initialization logic will call smart_archinitialize()
    to obtain the MTD driver instance.
  * CONFIG_EXAMPLES_SMART_NEBLOCKS: When CONFIG_EXAMPLES_SMART_ARCHINIT is not
    defined, this test will use the RAM MTD device at drivers/mtd/rammtd.c
    to simulate FLASH.  In this case, this value must be provided to give
    the nubmer of erase blocks in MTD RAM device.  The size of the allocated
    RAM drive will be: CONFIG_RAMMTD_ERASESIZE * CONFIG_EXAMPLES_SMART_NEBLOCKS
  * CONFIG_EXAMPLES_SMART_MAXNAME: Determines the maximum size of names used
    in the filesystem
  * CONFIG_EXAMPLES_SMART_MAXFILE: Determines the maximum size of a file
  * CONFIG_EXAMPLES_SMART_MAXIO: Max I/O, default 347.
  * CONFIG_EXAMPLES_SMART_MAXOPEN: Max open files.
  * CONFIG_EXAMPLES_SMART_MOUNTPT: SMART mountpoint
  * CONFIG_EXAMPLES_SMART_NLOOPS: Number of test loops. default 100
  * CONFIG_EXAMPLES_SMART_VERBOSE: Verbose output

endif

examples/smart_test
^^^^^^^^^^^^^^^^^^^

  Performs a file-based test on a SMART (or any) filesystem. Validates
  seek, append and seek-with-write operations.

    * CONFIG_EXAMPLES_SMART_TEST=y

  Dependencies:

    * CONFIG_NSH_BUILTIN_APPS=y: This test can be built only as an NSH
      command

examples/telnetd
^^^^^^^^^^^^^^^^

  This directory contains a functional port of the tiny uIP shell.  In
  the NuttX environment, the NuttShell (at apps/nshlib) supercedes this
  tiny shell and also supports telnetd.

    CONFIG_EXAMPLES_TELNETD - Enable the Telnetd example
    CONFIG_NETUTILS_NETLIB, CONFIG_NETUTILS_TELNED - Enable netutils
      libraries needed by the Telnetd example.
    CONFIG_EXAMPLES_TELNETD_DAEMONPRIO - Priority of the Telnet daemon.
      Default: SCHED_PRIORITY_DEFAULT
    CONFIG_EXAMPLES_TELNETD_DAEMONSTACKSIZE - Stack size allocated for the
      Telnet daemon. Default: 2048
    CONFIG_EXAMPLES_TELNETD_CLIENTPRIO- Priority of the Telnet client.
      Default: SCHED_PRIORITY_DEFAULT
    CONFIG_EXAMPLES_TELNETD_CLIENTSTACKSIZE - Stack size allocated for the
      Telnet client. Default: 2048
    CONFIG_EXAMPLES_TELNETD_NOMAC - If the hardware has no MAC address of its
      own, define this =y to provide a bogus address for testing.
    CONFIG_EXAMPLES_TELNETD_IPADDR - The target IP address.  Default 10.0.0.2
    CONFIG_EXAMPLES_TELNETD_DRIPADDR - The default router address. Default
      10.0.0.1
    CONFIG_EXAMPLES_TELNETD_NETMASK - The network mask.  Default: 255.255.255.0

  Also, make sure that you have the following set in the NuttX configuration
  file or else the performance will be very bad (because there will be only
  one character per TCP transfer):

    CONFIG_STDIO_BUFFER_SIZE - Some value >= 64
    CONFIG_STDIO_LINEBUFFER=y

examples/thttpd
^^^^^^^^^^^^^^^

  An example that builds netutils/thttpd with some simple NXFLAT
  CGI programs.  see configs/README.txt for most THTTPD settings.
  In addition to those, this example accepts:

    CONFIG_EXAMPLES_THTTPD_NOMAC    - (May be defined to use software assigned MAC)
    CONFIG_EXAMPLES_THTTPD_DRIPADDR - Default router IP addess
    CONFIG_EXAMPLES_THTTPD_NETMASK  - Network mask

  Applications using this example will need to enable the following
  netutils libraries in the defconfig file:

    CONFIG_NETUTILS_NETLIB=y
    CONFIG_NETUTILS_THTTPD=y

examples/touchscreen
^^^^^^^^^^^^^^^^^^^^

  This configuration implements a simple touchscreen test at
  apps/examples/touchscreen.  This test will create an empty X11 window
  and will print the touchscreen output as it is received from the
  simulated touchscreen driver.

    CONFIG_NSH_BUILTIN_APPS - Build the touchscreen test as
      an NSH built-in function.  Default: Built as a standalone problem
    CONFIG_EXAMPLES_TOUCHSCREEN_MINOR - The minor device number.  Minor=N
      corresponds to touchscreen device /dev/inputN.  Note this value must
      with CONFIG_EXAMPLES_TOUCHSCREEN_DEVPATH.  Default 0.
    CONFIG_EXAMPLES_TOUCHSCREEN_DEVPATH - The path to the touchscreen
      device.  This must be consistent with CONFIG_EXAMPLES_TOUCHSCREEN_MINOR.
      Default: "/dev/input0"
    CONFIG_EXAMPLES_TOUCHSCREEN_NSAMPLES - If CONFIG_NSH_BUILTIN_APPS
      is defined, then the number of samples is provided on the command line
      and this value is ignored.  Otherwise, this number of samples is
      collected and the program terminates.  Default:  Samples are collected
      indefinitely.
    CONFIG_EXAMPLES_TOUCHSCREEN_MOUSE - The touchscreen test can also be
      configured to work with a mouse driver by setting this option.

  The following additional configurations must be set in the NuttX
  configuration file:

    CONFIG_INPUT=y
    (Plus any touchscreen-specific settings).

  The following must also be defined in your apps configuration file:

    CONFIG_EXAMPLES_TOUCHSREEN=y

  The board-specific logic must provide the following interfaces that will
  be called by the example in order to initialize and uninitialize the
  touchscreen hardware:

    int arch_tcinitialize(int minor);
    int arch_tcuninitialize(void);

examples/watchdog
^^^^^^^^^^^^^^^^^

  A simple test of a watchdog timer driver.  Initializes starts the watchdog
  timer.  It pings the watchdog timer for a period of time then lets the
  watchdog timer expire... resetting the CPU is successful.  This
  example can ONLY be built as an NSH built-in function.

  This test depends on these specific Watchdog/NSH configurations settings (your
  specific watchdog hardware settings might require additional settings).

    CONFIG_WATCHDOG- Enables watchdog timer support support.
    CONFIG_NSH_BUILTIN_APPS - Build the watchdog time test as an NSH
      built-in function. Default: Not built!  The example can only be used
      as an NSH built-in application

  Specific configuration options for this example include:

    CONFIG_EXAMPLES_WATCHDOG_DEVPATH - The path to the Watchdog device.
      Default: /dev/watchdog0
    CONFIG_EXAMPLES_WATCHDOG_PINGTIME - Time in milliseconds that the example
      will ping the watchdog before letting the watchdog expire. Default: 5000
      milliseconds
    CONFIG_EXAMPLES_WATCHDOG_PINGDELAY - Time delay between pings in
      milliseconds. Default: 500 milliseconds.
    CONFIG_EXAMPLES_WATCHDOG_TIMEOUT - The watchdog timeout value in
      milliseconds before the watchdog timer expires.  Default:  2000
      milliseconds.
