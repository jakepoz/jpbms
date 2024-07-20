**JPBms**

JPBms is a simple BMS for solar charging and balancing a 3-cell lithium ion battery.

The key thing about it is that it uses no custom ICs, beyond a basic microcontroller. 
That is to say that it depends on no hard-to-find custom ICs that will be obsolete 3 years from now.
It implements a basic charging circuit that can charge a ~12V 3S battery from basically any input power source between
6V-24V, including if that source is a solar panel.


-------------------------------

![image](https://github.com/user-attachments/assets/11286e71-eda3-455f-81e5-7d8d75b5119b)


[v1.3 Schematics](JPBms2-v1.3.pdf)

-----------------------------
- STM32L053 microcontroller
- It's just an inductor with a full set of switches on it, so you can customize it
- Simple resistive balancing
- Single layer board design for easy JLCPCB style production
- Low ~100uA quiescent current
- XT30 solar panel connector, JST-XH battery balance connector

