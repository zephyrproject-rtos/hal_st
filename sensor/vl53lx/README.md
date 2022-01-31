VL53LX
#######

Origin:
   ST Microelectronics
   http://www.st.com/en/embedded-software/stsw-img015.html

Status:
   version 1.2.8

Purpose:
   ST Microelectonics official API to use vl53lx sensor.

Description:
   This library written by STMicroelectronics is dedicated to vl53lx time of flight sensor.

   This library is called from the vl53lx driver and is kept transparent for user.
   User is calling a standard Zephyr driver and then the driver makes some calls to this library.

   In order to fit with Zephyr rules and simplify version updates, we have done a minimum of modifications :
      - keep only API directory
        (remove documentation and samples, see ST website for this)

   In order to use this library, you have to :
      * define CONFIG_HAS_STLIB and CONFIG_VL53LX
      * use NEWLIB_LIBC in prj.conf (needed for abs function)
      * include vl53lx_api.h and vl53lx_platform.h in the driver .h

Dependencies:
   This package depends on Zephyr I2C implementation and is linked statically.
   This library will be used by a standard Zephyr driver.

URL:
   http://www.st.com/en/embedded-software/stsw-img015.html

commit:
   version 1.2.8

Maintained-by:
   External

License:
   BSD-3-Clause

License Link:
   http://www.st.com/en/embedded-software/stsw-img015.html



