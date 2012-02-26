# Source: http://www.gargoyle-router.com/wiki/doku.php?id=openwrt_coding

CFLAGS := -std=c99 -Wall -lm -Dbool=int -Dfalse=0 -Dtrue=1 -D_GNU_SOURCE $(CFLAGS)

tracker: tracker_Linux.o
	$(CC) $(CFLAGS) $(LDFLAGS) tracker_Linux.o -o tracker_Linux

tracker_Linux.o: tracker_Linux.c
	$(CC) $(CFLAGS) $(CFLAGS) -c tracker_Linux.c

clean:
	rm *.o tracker_Linux

