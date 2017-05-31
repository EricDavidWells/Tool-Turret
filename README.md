May 30, 2017

* Switched to calculating velocity using just math with no reliance on timers and float incrementers that accumulate error 
* Timer is still used to time the pulses but not calculate velocity
* Works perfectly now!

May 21, 2017

* Couldn't get it to turn any motor forever, it just squealed.  
** Combination of turning the current limiter DOWN and turning the voltage on power supply up made it work
** Moving voltage to 24V worked
** Moving voltage to 12V didn't work unless current was limited.  Idk why this is... Only thing I can think of is that too much current heats up the internal wire too much
* Current limiter, measure current 

M1M1M2	microstep  int
000	   1	    0
100	   1/2	    4
010	   1/4	    2
110	   1/8	    6
001	   1/16	    1
101	   1/32	    5
011	   1/32     3
111	   1/32	    7

* if the velocity is higher than the loops max speed, things go bad
** the velocity keeps increasing but the position can't keep up
** the max velocity is reached before the expected position 'p1'
*** this is allright because it just reaches its fastest speed before expected
** on the way down though the velocity decreases faster than expected 