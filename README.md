tgy
===
tgy is Atmel AVR assembly code for AVR-based 3-phase sensorless motor
electronic speed control (ESC) boards. This work is based on Bernhard
Konze's "tp-18a" code, with significant modifications. Please see
tgy.asm for Bernhard's license.

Features and Changes
--------------------
- 16-bit output PWM with full clock rate resolution (~18kHz PWM with
  a POWER_RANGE of 400 steps)
- Immediate PPM input to PWM output for best possible multicopter
  response (eg: ideal for tricopters, quadcopters, etc., but NOT where
  where slow-start or really any significant current limiting is needed!)
- Accepts 495Hz PPM update rates (minimum 5microseconds PPM low time)
- Optimized interrupt code (very low minimum PWM and reduced full
  throttle bump, and work beyond 120,000 RPM now - test carefully!)
- Configurable board pin assignments by include file
- Improved startup (though heavy hard drives are still a bit dodgy -
  suggestions/patches welcome!)

Supported Hardware
------------------
- TowerPro 18A, Turnigy Basic and Turnigy Plush 18A and 25A
- AfroESC
- HobbyKing BlueSeries w/all nFETs
- HobbyKing BlueSeries w/pFETs (not fully tested, but should work)
- ...any other board that uses a similar hardware configuration and
  compatible AVR8 chip!

Notes
-----
- Newer Turnigy Plush seems to have switched to SiLabs C8051F334, d'oh!
- If it breaks, you to keep both pieces!
- Use at your own risk, and always test without propellers!
- If your cheap ESC has 6 pads and an AVR, it's probably compatible;
  the pads are MOSI, MISO, SCLK, GND, VCC, and RESET. If it has 4 pads,
  it is probably a newer SiLabs-based one, for which this code will not
  work.
- i2c support was pulled out in the fork I started with. Patches welcome,
  though all of the ifdefs before were pretty ugly.
- I build and maintain this in Linux with avra (1.3.0 or newer); I hear
  it still works in AVR studio.

Thoughts
--------
- This doesn't check temperature or battery voltage, but if it did, the
  results would probably be less than desired on a multi-rotor platform.
- It might be a good idea to give up trying to start if the propeller is
  firmly planted in the ground and will never spin. Newer Turnigy Plush,
  Dynam, etc. ESCs seem to do this, which would be nice if the radio
  signal does not stop and has a non-zero throttle. PWM duty cycle will be
  limited to PWR_MAX_RPM1 (25%), which should avoid burning things for a
  little while. On the other hand, being temporarily stopped by a tree and
  then starting again without needing a throttle reset is also a possible
  advantage.
- There seems to be a slight timing difference (1/16th) between startup
  and running mode that throws off heavy, low-current motors (hard drives)
  when switching from startup to running mode. This seems seems to be a
  result of the "wait_OCT1_before_switch" "com_timing" which does not
  happen in the startup mode.

Patches and comments are always welcome! Let me know how it goes!

Code normally pushed to github here: https://github.com/sim-/tgy
