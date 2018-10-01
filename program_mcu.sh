#!/bin/bash

#sudo avrdude -p m8 -c stk500v2 -P /dev/ttyACM0 -U lfuse:r:low_fuse_val.hex:h -U hfuse:r:high_fuse_val.hex:h
avrdude -p m8 -c stk500v2 -P /dev/ttyACM0 -U flash:w:bin/avr-dht.hex 
