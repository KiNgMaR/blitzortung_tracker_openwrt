# Source: http://www.gargoyle-router.com/wiki/doku.php?id=openwrt_coding

CFLAGS := -std=c99 -Wall -lm -lpthread -Dbool=int -Dfalse=0 -Dtrue=1 -D_GNU_SOURCE -DADDVERSION=\"OpenWrt\" $(CFLAGS)

tracker: blitzortung_tracker.o
	$(CC) $(CFLAGS) $(LDFLAGS) blitzortung_tracker.o -o blitzortung_tracker

blitzortung_tracker.o: blitzortung_tracker.c
	$(CC) $(CFLAGS) $(CFLAGS) -c blitzortung_tracker.c

clean:
	rm *.o blitzortung_tracker

