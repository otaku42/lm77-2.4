# lm77-2.4
Backport of lm77.c from kernel 2.6.x to 2.4.x

This is a driver for the LM77 for Linux kernel 2.4, a temperature sensor and
thermal window comparator with 0.5 deg resolution made by National
Semiconductor. Actually it's a backport of lm77.c from kernel 2.6.

lm77-2.4 is in production use by the sponsor of the backport effort,
[true global communications GmbH](http://www.tgc.de). It's known to basically
work in conjunction with lm_sensors v2.9.2. However, there is evidence that
there might be some bugs, but I'm not sure if they are caused by the LM77
driver or the I2C bus driver that we use. I never found the time to investigate
this issue so far.

Please note that I didn't actively work on the driver since years. In particular
I didn't track changes that might have happened to lm77.c in kernel 2.6 and later.
You will also be on your own so far regarding the integration of the driver into
the [lm_sensors](https://en.wikipedia.org/wiki/Lm_sensors) package, but that
shouldn't be too hard.
