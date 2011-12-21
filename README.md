This tree contains Atmel AVR assembly code for AVR-based 3-phase
sensor-less motor electronic speed control (ESC) boards. This work is
based on Bernhard Konze's "tp-18a" code, with significant modifications.
Please see tgy.asm for Bernhard's license.

Features and Changes
--------------------
- 16MHz operation on most boards
- 16-bit output PWM with full clock rate resolution (~18kHz PWM with
  a POWER_RANGE of 800 steps)
- 24-bit timing and PPM pulse tracking at full clock rate resolution
- ICP-based pulse time recording (on supported hardware) for zero
  throttle jitter
- Immediate PPM input to PWM output for best possible multicopter
  response (eg: ideal for tricopters, quadcopters, etc., but NOT where
  where slow-start or really any significant current limiting is needed!)
- Accepts 495Hz PPM update rates (minimum ~5microseconds PPM low time)
- Optimized interrupt code (very low minimum PWM and reduced full
  throttle bump, and should work beyond 120,000 RPM
- Configurable board pin assignments by include file
- Improved startup (though heavy hard drives are still a bit dodgy -
  suggestions/patches welcome!)
- Forward and reverse commutation supported, including RC-car style
  reverse-neutral-forward PPM ranges, with optional braking

Supported (Tested) Hardware
---------------------------
- afro:
    - AfroESC (http://code.google.com/p/afrodevices/downloads/list)
- bs:
    - Hobby King 60A (F-60A)
- bs_nfet:
    - Hobby King 20A (F-20A)
    - Hobby King 30A (F-30A)
    - Hobby King BlueSeries 30A
- rct50a:
    - RCTimer 50A
- tp:
    - Original TowerPro 17A, 25A
- tgy:
    - Original TowerPro 18A
    - Original Turnigy Basic and Turnigy Plush 10A, 18A, and 25A (rebranded Hobbywing)
    - RCTimer 10A, 18A, 20A, 30A (18A, 20A, 30A are same board with more or less FETs)
    - Hobby King SS 15-18A
- tgy6a:
    - Original Turnigy Plush 6A
- tp_nfet:
    - Newer TowerPro 25A
    - Turnigy TY-P1 25A (HEXFET)

Notes
-----
- If it breaks, you get to keep both pieces!
- Use at your own risk, and always test first without propellers!
- Newer Turnigy Plush seems to have switched to SiLabs C8051F334, d'oh!
- If your cheap ESC has 6 pads and an AVR, it's probably compatible;
  the pads are MOSI, MISO, SCLK, GND, VCC, and RESET. If it has 4 pads,
  it is probably a newer SiLabs-based one, for which this code will not
  work.
- I build and maintain this in Linux with avra (1.3.0 or newer). Patches
  welcome for AVR Studio APS files, etc.
- The TowerPro/Turnigy Plush type boards typically do not come with
  external oscillators, which means their frequency drifts a bit with
  temperature and between boards. Multicopters and RC-car/boat
  controllers (with a neutral deadband) would probably be better on a
  board with an external oscillator. The Mystery/BlueSeries boards
  typically have them, as well as most higher current boards.
- This doesn't yet check temperature or battery voltage. This is not
  desired on multi-rotor platforms; however, people still want to use
  this on planes, cars, boats, etc., so I suppose I'll add it.

Installation
------------
Never just randomly try build targets until one works, especially not
when directly powered from a 4S LiPo! :P Many boards have completely
inverted FET banks and different pin assignments, so toggling one pin
could immediately fry multiple FETs.

The safest arrangement is to use a current-limited bench power supply,
set to a low voltage (6V-7V), for both flashing and initial testing.
If the pinout is wrong and causes a short, the current-limiting causes
the input voltage to drop below the brown-out detection voltage of the
MCU, causing all output pins to go high-impedance in hardware, and an
automatic reset when the voltage comes back.

If you do not have a current-limited supply, you can improvise by using 4
AA batteries or an old NiCd pack or even a LiPo with a 12V light bulb in
series to act as a current limiter. Also, be careful when touching the
board, since it can be quite easy to turn on a FET gate with just your
finger. This should be OK if you have a current-limited supply, since
it should just reset.

Even if the board appears to be one tested by others, make sure yours
has the expected FET pin assignments, inversions, and sense lines! The
assignments for each board type can be found in the .inc files, and
the actual pin mappings can be found in the first few pages of ATmega8
datasheet. This typically requires a voltmeter and, preferably, an
oscilloscope.

When not powered, use a voltmeter to check the path from the MCU pins to
either the FET resistors, transistors, or driver chips, and verify that
they match the assignments in one of the include files. Then power up
the ESC with the original firmware, and check with an oscilloscope the
inversions of the same pins. A voltmeter may be used instead when the
motor is stopped. Be careful not to short FET pins together! If all
voltages are low when the motor is off, nothing is inverted, and the
INIT_Px values in the .inc file should be 0 for all of the FET bits.

Older and smaller boards typically use P-FETs and N-FETs in the H-bridge;
the P-FETs are typically driven by three NPN transistors, while the
N-FETs are driven directly at TTL voltages with just a low resistor.
Medium-sized and newer boards have moved to all-N-FET designs, but
maintain the NPN transistors, and so have inverted P-FET pins from the
MCU. Larger current (>~30A) boards typically have separate FET driver
chips, to which the N-FET _or_ P-FET pins _may_ be inverted (but not both,
since it would blow up before the MCU initializes).

PWM is usually done on the low side of the H-bridge, where high frequency
driving is easiest. If the average voltage increases at the AVR pin as
throttle increases, and drops to 0V when stopped, the low-side FETs are
not inverted; if average voltage decreases, and rises to 5V when stopped,
the low-side FETs are likely inverted. The high side FETs are only
switched at every other motor commutation step, and so switch at a lower
frequency, and should be off 2/3rds of the time. If at 0V when stopped,
and less than 2.5V average when running, the P-FETs are not inverted. If
at 5V when stopped, and more than 2.5V when running, the P-FETs are
inverted. In the case of inverted an FET group, they should be listed in
the INIT_Px values (to turn them ON at boot), and the "on" macros should
use clear instead of set instructions. The inverse applies to the "off"
macros.

There are four sense lines. The three output phases go through resistor
dividers (to bring the voltage down to between 0-5V), and then are
connected to ADC channels which can be accessed by the comparator with
the ADC multiplexer when the ACME (Analog Comparator Multiplexer Enable)
bit is enabled. Some boards use all ADC channels while others put one pin
on AIN1 so that the ADC can sample voltages on other ADC channels while
the comparator samples that phase. A "center tap" is established with a
resistor star and connected to AIN0 on all boards, for detecting the
zero-crossing from the motor back-EMF for timing. You can check which
pins run to which output phases as they will have the lowest resistance
from output phase to AVR pin, and all three will have a common resistance
to the AIN1 pin.

Flashing and Testing
--------------------
Sort out how you want to connect an ISP programming device to the chip.
Most boards have 6 pads in a row for this purpose. You can either solder
wires, or make up some kind of springy-pin connector that touches the
pads or chip pins without needing to solder. This is helpful when
flashing more than one board. See here for some ideas and discussion:
http://www.rcgroups.com/forums/showthread.php?t=1513678

Sort out which software you will use for flashing. You can download AVR
Studio and the AVR Toolchain from www.atmel.com, or "avrdude" on most
OSes. There are plenty of resources on the web for AVR ISP programming.

With the board powered from the current-limited supply, try to read
the stock firmware (flash) _and_ EEPROM from the AVR, to use as a backup.
Most are locked and will still appear to read, but the files will contain
just a series of repeating/increasing digits. If you do manage to get
something, consider yourself lucky!

Write down the stock fuse values, and check that they are sane. Most AVR
programmers have a menu for this, but with avrdude or uisp, Google "AVR
fuse calculator", select the ATmega8 target, and type in the hex values.
If the "watchdog" fuse is enabled, you will want to disable it for now.
The brown-out voltage should be set to 4.0V and enabled (BODEN). Leave
the rest of the fuse values as shipped, and write down the new values.

Flash the desired target .hex file to the AVR, then set the fuses, if
anything needed changing. You can leave the EEPROM unchanged or erased
for now. If you have any errors, check the connections to and voltage at
the chip. Sometimes, a weak power or signal connection can temporarily
work and then fail part-way through programming, giving verification
errors. This can happen particularly if the target chip is powered weakly
by the programmer itself, which then back-feeds to the rest of the
circuit and tries to charge the capacitor, etc.

Once programming is successful, hook up a small motor without propeller
and reset the power. You should hear three increasing beeps. If not, or
if you hear only some beeps, the FET pinout may be incorrect or one or
more FETs may be broken. Repetitive clicking can also indicate that the
pinout is incorrect and causing continuous brown-out resets.

Now, if you attach a valid PPM servo pulse with low-enough pulse length,
you should hear a forth beep indicating that the ESC is armed. If not,
try lowering the trim as far as possible. If it still doesn't work, you
may need to raise the STOP_RC_PULS value in the code.

Once armed, the ESC will try to start the motor if the pulse length
(throttle) is increased. Try running up the motor slowly, and make sure
everything runs smoothly. If on a variable supply, increase the voltage
to the expected running voltage, and try again, with slow and rapid
throttle changes.

Finally, test the ESC in the intended application, with the usual power
source, without anything attached to the motor shaft/bell first. Then,
attach the propeller or gear, etc., LOCK IT DOWN so it doesn't hit you in
the face, and try a slow sweep from idle to full throttle. Finally, try
rapid throttle changes from slow to fast. The ESC and motor should run
smoothly, never lose timing, and the ESC should not reset. If using a
flight control board, you can sometimes tap the board to make it output a
sudden throttle increase.

For debugging reset causes, recent code has different beep sequences for
each AVR reset case. Three increasing beeps indicates a usual startup.
However, if there was a power brownout (such as voltage at the MCU
dropping below 4.0V), there will be a medium and low beep (like a mobile
phone with dead battery). Finally, an external reset, such as after ISP
flashing, will cause just a single beep at the same pitch as the arming
beep.

---

Patches and comments are always welcome! Let me know how it
goes!

Code normally pushed to github here: https://github.com/sim-/tgy
