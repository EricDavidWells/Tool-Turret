Arduino Controlled Tool Turret for a Miniature Lathe:

Microstepping Table

M1M1M2	microstep  int

000	   	  1	    	  0

100	  	  1/2	       4

010	       1/4	       2

110	       1/8	       6

001	       1/16	  1

101	       1/32	  5

011	       1/32       3

111	       1/32	  7

* if the velocity is higher than the loops max speed, things go bad
** the velocity keeps increasing but the position can't keep up
** the max velocity is reached before the expected position 'p1'
*** this is allright because it just reaches its fastest speed before expected
** on the way down though the velocity decreases faster than expected 