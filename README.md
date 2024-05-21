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
  - Gate drivers have 150uA queiscent current each


Measured in standby mode v1.0, = 68uA Cell1, 90uA Cell2, 100uA Cell3
(With Gate drivers removed)