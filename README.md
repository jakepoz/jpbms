Quiescent Current Calculations:
 - ST715M25R Quiescent = 5uA
 - ZXCT1107 Quiescent = 5uA
 - BAT54SW Reverse Leakage = 2uA * 5 = 10uA
 - ST32L053, Stop mode = ~1uA
	   , low power sleep = 4.5uA
	   , low power run = 8uA

 - Probably plan is to have Stop Mode, with 3.5uS wake time, and comparators, so hopefully can get 10uA all together

 - Problem areas
  - Each of VBATT, VSOLAR, CELL4V, and CELL8V have a voltage divider that will use 150uA of current
  - Still noisy ADC, esp with PWM TIM22 running
    - I wonder if this is the case if the leds are removed too?
  - Gate drivers have 150uA queiscent current each, need to have a way to shutdown


Measured in standby mode v1.0, = 68uA Cell1, 90uA Cell2, 100uA Cell3
(With Gate drivers removed)


Things todo:
 - Check on noise in Cell voltage measurements
 - Check voltage drop on cells while balancing, could be a divider between fuse and balance resistor?
 - Write code for the buck boost, you should be able to pump power into a resistor from the battery as a test
 - Have a way to wake on solar panel plugged in?
 - Check current sense

Code:
 - Wake periodically, sample VBATT, if too low, put everything to sleep for a long time
 - If good, sample cell voltages reliably to high accuracy
 - If solar panel available and vbatt low enough, charge the battery
    - MPPT
 - If no solar panel available, run balance algorithm after a while

Other ideas:

ADS1115 - $1.14 on lcsc
https://www.lcsc.com/datasheet/lcsc_datasheet_1809192322_Texas-Instruments-ADS1115IDGSR_C37593.pdf

Maybe do a basic comparator thing with analog multiplexer for balance detection
Might still only want to do comparator balance if the difference is really high enough
