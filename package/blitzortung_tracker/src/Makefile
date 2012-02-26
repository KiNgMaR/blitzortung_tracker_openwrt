# Source: http://www.gargoyle-router.com/wiki/doku.php?id=openwrt_coding

CFLAGS := $(CFLAGS) -std=c99 -Wall -lm -Dbool=int -Dfalse=0 -Dtrue=1 -D_GNU_SOURCE
CFLAGS := $(CFLAGS) -DB4800 -DB9600 -DB19200 -DB25000 -DB38400 -DB50000 -DB115200 -DB230400 -DB250000 -DB500000

tracker: tracker_Linux.o
	$(CC) $(CFLAGS) $(LDFLAGS) tracker_Linux.o -o tracker_Linux

tracker_Linux.o: tracker_Linux.c
	$(CC) $(CFLAGS) $(CFLAGS) -c tracker_Linux.c

clean:
	rm *.o tracker_Linux

