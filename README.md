zdbrg - Antenna rotator for receiving HRPT/HRIT -LEO satellite transmissions (Meteor M-2, NOAA-15/18/19 & MetOp etc)

* Raspberry Pi Zero (W) as main controller, running a super light buildroot-based OS.
  All wireless features are disabled by default.
* Custom PCB for neatly connecting everything together and handling serial communiaction
* Mostly 3d printed structural parts
*  Antenna center frequency 1700MHz (RHCP)
* EasyCommV2 compatible for use with 'rotctld'
* NEMA17 stepper motors: 0,9deg, 1.5A, 400SPR with full-step torque of 1,2NM.
  A4988 driver circuis.
  Printed 5:1 planetary reduction gears used for increased torque and better resistance
  against non-controlled, free movement while motors are disabled.
