/*
    Copyright (C) 2003-2012  Egon Wanke <blitzortung@gmx.org>
    Copyright (C) 2012       Tobias Volgnandt <mail@myblitzortung.org>
    Copyright (C) 2012       Ingmar Runge <ingmar@irsoft.de>


    This program sends the output of the evaluation board as
    udp packets to the servers of blitzortung.org. It supports
    all firmware versions.


     Linux
    =======
    Name this file "blitzortung_tracker.c" and compile it by
    > g++ -Wall -lm -pthread -o blitzortung_tracker blitzortung_tracker.c


     OpenWRT
    =========
    Compiling with the OpenWRT toolchain:
    > gcc -std=c99 -Wall -lm -lpthread -Dbool=int -Dfalse=0 -Dtrue=1 -D_GNU_SOURCE \
       -DADDVERSION=\"OpenWRT\" -o blitzortung_tracker blitzortung_tracker.c

    Download precompiled binaries here: http://blitzortung.ingmar.cc/openwrt/


     Windows
    =========
    If you want a windows version, then you need Cygwin. Compile it by
    > g++ -Wall -lm -lpthread -static -o blitzortung_tracker blitzortung_tracker.c

    You need the file cygwin1.dll in the same directory to run the program!


     Help
    ======

    > ./blitzortung_tracker -h
    or
    > ./blitzortung_tracker --help

    Windows users have to call the command shell, and call it like
    > blitzortung_tracker.exe -h


     Simple Mode
    =============

    If you do not need webserver, watchdog and the parameter updates,
    then you can compile the tracker with the -DSIMPLE_TRACKER option.
    You will then get a much smaller executable.


     Examples
    ==========

    *** PCB 6.8 or higher ***

    Examples of the use for PCB 6 Version 8 with Firmware version 30a or higher:

    > ./blitzortung_tracker -vi -vl -vo -vs - /dev/ttyS0 CharlyAU yzyzyzyz 2
    This command writes sytstem information, log information, board output,
    and sent UDP packets to standard output,
    sets GPS speed to 4800 baud, does not initialize the GPS,
    sets tracker speed to 115200 baud, and uses serial device = /dev/ttyS0,
    username = CharlyAU, password = yzyzyzyz, and region = 2

    > ./blitzortung_tracker -bg 9600 SiRF /dev/ttyUSB0 PeterPim aaaaaaaa 1
    This command set GPS speed to 9600 baud, initializes the GPS with SiRF chipset,
    sets tracker speed to 115200 baud, and uses serial device = /dev/ttyUSB0,
    username = PeterPin, password = aaaaaaaa, and region = 1

    > ./blitzortung_tracker -vi -ll tracker.log -bt 500000 - /dev/ttyS0 CharlyAU yzyzyzyz 2
    This command outputs system information on standard output,
    writes log information to file tracker.log,
    sets GPS speed to 4800 baud, does not initialize the GPS,
    sets tracker speed to 500000 baud, and uses serial device = /dev/ttyS0,
    username = CharlyAU, password = yzyzyzyz, and region = 2

    If you want to controll the GPS output, then turn the yellow jumpers of the
    board by 90 degrees and type the following command:

    > ./blitzortung_tracker -vo -bg 4800 SiRF /dev/ttyUSB0
    This command outputs board output on standard output,
    sets GPS speed to 4800 baud, intializes the GPS with SiRF chipset,
    sets tracker speed to GPS speed (= 4800 baud),
    and uses serial device = /dev/ttyUSB0


    *** PCB 6.7 or lower ***

    Examples of the use for boards before PCB 6 Version 8 with Firmware version
    less than 30a. You should use FW 29, because older ones had some bugs!
    For these boards the tracker baudrate always has to be equal to the GPS baudrate:

    > ./blitzortung_tracker -vi -vl -vo -vs -bg 38400 -bt 38400 SiRF /dev/ttyS0 CharlyAU yzyzyzyz 1
    This command writes sytstem information, log information, board output,
    and sent UDP packets to standard output,
    sets GPS speed to 38400 baud, initializes the GPS,
    sets tracker speed to 38400 baud, and uses serial device = /dev/ttyS0,
    username = CharlyAU, password = yzyzyzyz, and region = 1


    To control the GPS output, turn the yellow jumpers of the board by 90 degrees and
    type the following command which is the same for all boards:

    > ./blitzortung_tracker -vo -bg 4800 SiRF /dev/ttyUSB0
    This command outputs board output on standard output,
    sets GPS speed to 4800 baud, intializes the GPS with SiRF chipset,
    sets tracker speed to GPS speed (= 4800 baud),
    and uses serial device = /dev/ttyUSB0

*/





/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

//General
#define VERSION                 "3"                    // version string send to server; no white spaces; linux/windows-info included automatically
#define SERVER_ADDR_1           "rechenserver.de"      // server address region 1
#define SERVER_ADDR_2           "rechenserver.com"     // server address region 2
#define SERVER_ADDR_3           "rechenserver.org"     // server address region 3
#define SERVER_ADDR_4           "rechenserver.com"     // server address region 4
#define SERVER_PORT_1           8308                   // server port region 1
#define SERVER_PORT_2           8308                   // server port region 2
#define SERVER_PORT_3           8308                   // server port region 3
#define SERVER_PORT_4           8308                   // server port region 4
#define SERVER_DNS_TIMEOUT      3600                   // do DNS lookups every given seconds
#define SHOW_SIGNAL_RATE        2                      // print signal rate to stdout (signals per sec) if higher than given value
#define BOARD_TIMEOUT           5                      // timeout in seconds before watchdog acts when no message from board can be evaluated
#define BOARD_TIMEOUT_RESET     20                     // timeout in seconds before watchdog resets buffers
#define GPS_TIMEOUT_INIT        15                     // when watchdog is on, init GPS only when there is no valid data for given seconds


//Buffers
#define STRING_BUFFER_SIZE      2048                   // maximal buffer size for the strings we use to receive UDP packets
#define INFO_BUFFER_SIZE        128                    // maximal buffer size for the strings we use elsewhere
#define RING_BUFFER_SIZE        30                     // ring buffer size for averaged computation of counter difference
#define BUFFERD_SIGNALS         20                     // how much signals should we save (used for http output)
#define BAUDRATES               10                     // number of different baudrates
#define SERIAL_READ_BUFFER      128                    // buffer for storing serial data (needed for cygwin)


//Default Baudrates
#define DEFAULT_TRACKER_BAUDRATE  115200
#define DEFAULT_GPS_BAUDRATE      4800


//Parameter Updates
#define PARAMETER_REQ_SERVER    "tracker.blitzortung.org"  // server for parameter update requests
#define PARAMETER_REQ_URI       "/u/"              // uri for parameter update requests
#define PARAMETER_REQ_TIMEOUT   3600                    // if no valid data received -> fallback to defaults


//Password
#define PASSWORD_CHECK_URI      "/Scripts_php/check_passwd.php"     // check password url on main server (auto added: ?username=%s&region=%s&password=%s)
#define PASSWORD_CHECK_INTVL    3600                    // re-check password after sec


//HTTP Server and GET-Client
#define HTTP_SERVER_BACKLOG     20                     // Server: how many pending connections queue will hold
#define HTTP_REFRESH_PAGE       900                    // Server: refresh-rate of variables printed on tracker webpage (milliseconds)
#define HTTP_REFRESH_DATA       300                    // Server: refresh rate of data/graphs (milliseconds)
#define HTTP_REFRESH_DATA_NEW   50                     // Server: if new data (new signal) was sent, lower refresh rate (milliseconds)
#define HTTP_REQUEST_BUFFER     (1024*8)               // Client: buffer for sending and storing GET-Requests
#define HTTP_SOCKET_TIMEOUT     5                      // Server&Client: Timeout in seconds





/**********************************************************************************/
/* Following constants are default values for remote changeable parameters        */
/*                                                                                */
/* !!! PLEASE DO NOT CHANGE THESE VALUES!!!                                       */
/*                                                                                */
/* You can ask us to change them remotely if your station doesn't work well with  */
/* this standard values!                                                          */
/**********************************************************************************/


//Update
#define PARAM_UPDATE_SEC        300                    // default update interval


//Calculation
#define SIGNAL_RATE_INTERVAL    5                      // interval for calculating signal rate
#define SMOOTH_FACTOR           7200                   // averaging over given number of seconds
#define POS_PRECISION           0.001000l              // position precision in degree to reach before sending data
#define PPS_PRECISION           0.000001l              // pulse precision to reach before sending data
#define ALT_PRECISION           100.0l                 // altitude precision in meter to reach before sending data


//Interference-Mode
#define MAX_NONZERO_SEC         10                     // maximal number of consecutive seconds with signals
#define MAX_SIGNAL_RATE         20.0                   // max. signals per second in mean interval


//Signal filter
#define FILTER_ENABLE           0                      // 0=disabled, 1=filter only activated before imode starts, 2=always
#define MIN_AMPLITUDE           0                      // minimum amplitude (1 = 0.02V at 8bits per sample)
#define MAX_AMPLITUDE           0                      // maximum amplitude (1 = 0.02V at 8bits per sample)
#define MAX_AMPLITUDE_COUNT     0                      // max. count of max. amplitude
#define SIGNAL_CHECK_BYTES      0                      // check bytes for amplitude check




/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

//Use thread safe functions when needed!
#ifndef SIMPLE_TRACKER
#ifndef _REENTRANT
#define _REENTRANT
#endif
#endif

#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <string.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <signal.h>
#include <syslog.h>
#include <math.h>

#ifndef SIMPLE_TRACKER
#include <dirent.h>
#include <pthread.h>
#include <errno.h>
#endif


/******************************************************************************/
/******************************************************************************/
/******************************************************************************/




// Server settings
struct server_type
{
	char addr[INFO_BUFFER_SIZE];
	int port;
	char *username;
	char *password;
	bool login_ok;
	int sock_id;
	struct sockaddr_in serv_addr;
	long long bytes_sent;
} Server[3];


//
struct flag_type
{
	bool send_to_blitzortung;
	bool verbose_out;
	bool verbose_log;
	bool verbose_sent;
	bool verbose_info;
	bool verbose_http;
	bool verbose_update;
	bool add_server;
	bool SBAS;
	bool syslog;
	bool help;
	bool requests_enabled;
	bool watchdog_enabled;
	bool auto_device;
	int region;
	int http_server_port;
} flag;


//
struct precision_type
{
	long double S_counter;
	long double lat;
	long double lon;
	long double alt;
	long double PPS;
} precision;


//
struct ring_buffer_type
{
	long double S_counter_difference;
	long double lat;
	long double lon;
	long double alt;
	double sat;
} ring_buffer[RING_BUFFER_SIZE], average;


//
struct Time_type
{
	int year;
	int mon;
	int day;
	int hour;
	int min;
	int sec;
	long long nsec;
};

//
struct S_type
{
	bool board_is_sending;
	long long counter;
	char status;
	struct Time_type time;
	long double lat;
	long double lon;
	long double alt;
	long double average_lat;
	long double average_lon;
	long double average_alt;
	long double average_pps;
	int sat;
	double average_sat;
	char firmware_version[INFO_BUFFER_SIZE];
	char tracker_version[INFO_BUFFER_SIZE];
	char tracker_version_long[INFO_BUFFER_SIZE];
} S, last_S;

//
struct L_type
{
	time_t receive_time;
	time_t gps_ok_time;
	long long counter;
	long long nsec;
	int channels;
	int values;
	int bits;
	bool signal_found;
	bool signal_ok;
	char data[STRING_BUFFER_SIZE];
	unsigned char data_numeric[STRING_BUFFER_SIZE];
} L;

//
struct Signal_type
{
	struct Time_type time;
	unsigned char data[STRING_BUFFER_SIZE];
	int channels;
	int values;
	int bits;
} LastGoodSignal, LastSentSignal,
LastMinSignal, LastMaxSignal,
LastIMode1Signal, LastIMode2Signal,
FirstIMode1Signal, FirstIMode2Signal;


//
struct control_type
{
	bool accuracy_ok;
	bool seconds_flow_ok;
	bool time_ok;
	bool pos_ok;
	bool checksum_ok;
	bool faulty;
	bool faulty_rate;
	bool startup_phase;
	bool ringbuffer_reset;
	bool force_send;
	char filter_enabled;
	int nogps_count;
	int signals_per_sec;
	int signals_per_sec_count;
	int signals_per_sec_count_filtered;
	int signals_per_sec_time;
	int signals_per_sec_max;
	int signals_per_sec_max_last;
	float mean_signal_rate_last;
	float mean_signal_rate;
	int nonzero_sec;
	struct ring_buffer_type sum;
	long long last_transmission_time;
} C, last_C;



//
struct LT_type
{
	time_t start_time;
	time_t board_last_timeout;
	time_t reset_time;
	time_t last_update;

	int board_last_timeout_duration;

	int signals;
	int signals_sent;
	int signals_filtered_min;
	int signals_filtered_max;
	int signals_faulty;
	int signals_faulty_rate;

	int sec_accuracy_nok;
	int sec_time_nok;
	int sec_seconds_flow_nok;
	int sec_checksum_nok;
	int sec_pos_nok;
	int sec_gps_nok;
	int sec_faulty;
	int sec_faulty_rate;

	int count_accuracy_nok;
	int count_time_nok;
	int count_seconds_flow_nok;
	int count_checksum_nok;
	int count_pos_nok;
	int count_gps_nok;
	int count_faulty;
	int count_faulty_rate;

	int min_sat;
	int max_sat;

	long long bytes_sent;
	long long packets_sent;

	int parameter_updates;
	int parameter_updates_var;

} LT;


// parameters, that can be changed remotely
struct param_type
{
	time_t last_update;
	time_t last_update_success;
	char message[STRING_BUFFER_SIZE + 1];

	int signal_rate_interval;
	int smooth_factor;
	long double pos_precision;
	long double pps_precision;
	long double alt_precision;
	int max_nonzero_sec;
	float max_signal_rate;
	int min_amplitude;
	int max_amplitude;
	int max_amplitude_count;
	int signal_check_bytes;
	int param_update_sec;
	int filter_enable;
	int send_board_output;
	
	char extra_server_addr[INFO_BUFFER_SIZE + 1];
	int  extra_server_port;
	char extra_server_username[INFO_BUFFER_SIZE + 1];
	char extra_server_password[INFO_BUFFER_SIZE + 1];
} P;


//
struct serial_type
{
	int e;
	int f;
	char device[INFO_BUFFER_SIZE];
	char *echo_device;
	int tracker_baudrate;
	int gps_baudrate;
	int tracker_baudrates[BAUDRATES];
	int gps_baudrates[BAUDRATES];
	char *gps_type;
	char tracker_baudrates_string[INFO_BUFFER_SIZE];
	char gps_baudrates_string[INFO_BUFFER_SIZE];
} serial;


//
struct logfile_type
{
	char *name;
	FILE *fd;
};


//
struct logfiles_type
{
	struct logfile_type out;
	struct logfile_type info;
	struct logfile_type log;
	struct logfile_type sent;
	struct logfile_type http;
	struct logfile_type update;
} logfiles;




/******************************************************************************/
/***** Variables **************************************************************/
/******************************************************************************/


//Single variables
const char *server_addr[] = {"0.0.0.0", SERVER_ADDR_1, SERVER_ADDR_2, SERVER_ADDR_3, SERVER_ADDR_4};
int server_port[] = {0, SERVER_PORT_1, SERVER_PORT_2, SERVER_PORT_3, SERVER_PORT_4};
int ring_buffer_index = 0;

#if SERIAL_READ_BUFFER > 1
unsigned char serial_buf[SERIAL_READ_BUFFER];
int serial_buf_to_read = 0;
int serial_buf_len = 0;
#endif

//Signals
struct Signal_type LastSignal[BUFFERD_SIGNALS];

#ifndef SIMPLE_TRACKER
//Threads
pthread_mutex_t thread_mutex = PTHREAD_MUTEX_INITIALIZER;
#endif

//HTTP stuff
char http_string_buf[STRING_BUFFER_SIZE];
char http_data_buf[STRING_BUFFER_SIZE];
char http_fd;

//Logging
char log_data_buf[STRING_BUFFER_SIZE];



/******************************************************************************/
/***** Macros *****************************************************************/
/******************************************************************************/


//Log types
#define LOG_GENERAL  1
#define LOG_HTTP     2
#define LOG_UPDATE   4
#define LOG_NOSYSLOG 8

//Logging macros for easy sprintf usages
#define print_log(a, ...) { snprintf(log_data_buf, STRING_BUFFER_SIZE, __VA_ARGS__);  write_log_message(log_data_buf, a); }
#define print_error(...)  { snprintf(log_data_buf, STRING_BUFFER_SIZE, __VA_ARGS__);  perror(log_data_buf); }
#define print_http(...)   { snprintf(http_data_buf, STRING_BUFFER_SIZE, __VA_ARGS__); http_send(http_fd, http_data_buf); }



/******************************************************************************/
/***** initialization strings for gps chip sets *******************************/
/******************************************************************************/

//
// initialization for San Jose Navigation moduls, 4800 baud
//
char init_gps_SANAV[] = "\
$PFEC,GPint,GGA01,GLL00,GSA00,GSV00,RMC01,DTM00,VTG00,ZDA00*00\n";

//
// initialization for Garmin moduls
//
char init_gps_Garmin[] = "\
$PGRMO,GPGGA,1*00\r\n\
$PGRMO,GPGSA,0*00\r\n\
$PGRMO,GPGSV,0*00\r\n\
$PGRMO,GPRMC,1*00\r\n\
$PGRMO,GPVTG,0*00\r\n\
$PGRMO,PGRMM,0*00\r\n\
$PGRMO,PGRMT,0*00\r\n\
$PGRMO,PGRME,0*00\r\n\
$PGRMO,PGRMB,0*00\r\n\
$PGRMCE*00\r\n";

char init_gps_Garmin_SBAS_on[] = "$PGRMC1,1,,,,,,,W,,,,,*00\r\n";
char init_gps_Garmin_SBAS_off[] = "$PGRMC1,1,,,,,,,N,,,,,*00\r\n";

char init_gps_Garmin_4800[] = "$PGRMC,,,,,,,,,,3,,2,4,*00\r\n";
char init_gps_Garmin_9600[] = "$PGRMC,,,,,,,,,,4,,2,4,*00\r\n";
char init_gps_Garmin_19200[] = "$PGRMC,,,,,,,,,,5,,2,4,*00\r\n";
char init_gps_Garmin_38400[] = "$PGRMC,,,,,,,,,,8,,2,4,*00\r\n";

//
// initialization for SiRF moduls
//
char init_gps_SiRF[] = "\
$PSRF103,00,00,01,01*00\r\n\
$PSRF103,01,00,00,01*00\r\n\
$PSRF103,02,00,00,01*00\r\n\
$PSRF103,03,00,00,01*00\r\n\
$PSRF103,04,00,01,01*00\r\n\
$PSRF103,05,00,00,01*00\r\n\
$PSRF103,06,00,00,01*00\r\n\
$PSRF103,08,00,00,01*00\r\n";

char init_gps_SiRF_SBAS_on[] = "$PSRF151,01*00\r\n";
char init_gps_SiRF_SBAS_off[] = "$PSRF151,00*00\r\n";

char init_gps_SiRF_4800[] = "$PSRF100,1,4800,8,1,0*00\r\n";
char init_gps_SiRF_9600[] = "$PSRF100,1,9600,8,1,0*00\r\n";
char init_gps_SiRF_19200[] = "$PSRF100,1,19200,8,1,0*00\r\n";
char init_gps_SiRF_38400[] = "$PSRF100,1,38400,8,1,0*00\r\n";




/******************************************************************************/
/***** time functions *********************************************************/
/******************************************************************************/

//
// convert utc calender time to epoche nanoseconds
//
long long utc_ctime_to_ensec(int year, int mon, int day, int hour, int min, long double sec_nsec)
{
	int sec = (int)sec_nsec;
	long long nsec = (long long)(sec_nsec * 1000000000ll) - sec * 1000000000ll;
	struct tm t;
	t.tm_year = year - 1900;
	t.tm_mon = mon - 1;
	t.tm_mday = day;
	t.tm_hour = hour;
	t.tm_min = min;
	t.tm_sec = (int)sec;
	time_t esec = timegm(&t);
	return ((long long)esec * 1000000000ll + nsec);
}


//
// convert epoche nanoseconds to utc calender time
//
void ensec_to_utc_ctime(long long ensec, int *year, int *mon, int *day, int *hour, int *min, long double *sec_nsec)
{
	time_t esec = ensec / 1000000000ll;
	struct tm *t = gmtime(&esec);
	*year = t->tm_year + 1900;
	*mon = t->tm_mon + 1;
	*day = t->tm_mday;
	*hour = t->tm_hour;
	*min = t->tm_min;
	*sec_nsec = t->tm_sec + (ensec % 1000000000ll) / 1000000000.0l;
}

//
// return epoche nanoseconds
//
long long ensec_time()
{
	struct timeval t;
	gettimeofday(&t, (struct timezone *)0);
	return ((long long)(t.tv_sec) * 1000000000ll + (long long)t.tv_usec * 1000ll);
}


//
// compute difference between two time_type in seconds
//
long long time_diff_nsec(struct Time_type *a, struct Time_type *b)
{
	long long start, end;

	start =    utc_ctime_to_ensec(a->year, a->mon, a->day, a->hour, a->min, a->sec + a->nsec / 1000000000ll);
	end   =    utc_ctime_to_ensec(b->year, b->mon, b->day, b->hour, b->min, b->sec + b->nsec / 1000000000ll);

	return end - start;
}


/******************************************************************************/
/***** write log message ******************************************************/
/******************************************************************************/

//
// general logging
// called by macro!
//
void write_log_message(const char *text, int type)
{
	int year, mon, day, min, hour;
	long double sec_nsec;
	char buf [STRING_BUFFER_SIZE];

	ensec_to_utc_ctime(ensec_time(), &year, &mon, &day, &hour, &min, &sec_nsec);
	sprintf(buf, "%04d-%02d-%02d %02d:%02d:%02.0Lf, PID: %d,", year, mon, day, hour, min, sec_nsec, (int)getpid());


	if(type & LOG_GENERAL)
	{
		if(logfiles.log.name != NULL)
		{
			fprintf(logfiles.log.fd, "%s %s", buf, text);
			fflush(logfiles.log.fd);
		}

		if(flag.verbose_log)
		{
			printf("%s %s", buf, text);
			fflush(stdout);
		}
	}

	if(type & LOG_HTTP)
	{
		if(logfiles.http.name != NULL)
		{
			fprintf(logfiles.http.fd, "%s %s", buf, text);
			fflush(logfiles.http.fd);
		}

		if(flag.verbose_http)
		{
			printf("%s HTTP-Server: %s", buf, text);
			fflush(stdout);
		}
	}

	if(type & LOG_UPDATE)
	{
		if(logfiles.update.name != NULL)
		{
			fprintf(logfiles.update.fd, "%s %s", buf, text);
			fflush(logfiles.update.fd);
		}

		if(flag.verbose_update)
		{
			printf("%s Parameter-Update: %s", buf, text);
			fflush(stdout);
		}
	}


	if(flag.syslog && !(type & LOG_NOSYSLOG) )
	{
		openlog("blitzortung", LOG_CONS | LOG_PID, LOG_USER);

		if(type & LOG_GENERAL)
			syslog(LOG_INFO, "%s", text);
		else if(type & LOG_HTTP)
			syslog(LOG_INFO, "HTTP-Server: %s", text);
		else if(type & LOG_UPDATE)
			syslog(LOG_INFO, "Parameter Update: %s", text);

		closelog();
	}

}




//
//set socket timeout
//
void set_socket_timeout(int sock)
{
	struct timeval timeout;
	timeout.tv_sec = HTTP_SOCKET_TIMEOUT;
	timeout.tv_usec = 0;

	if(setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(timeout)) < 0)
		print_error("Couldn't set socket receive timeout");

	if(setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout, sizeof(timeout)) < 0)
		print_error("Couldn't set socket send timeout");

}



/******************************************************************************/
/***** serial communication wrappers   ****************************************/
/******************************************************************************/

//
// open serial port
//
int serial_open(const char *path)
{
	int fd;

	fd = open(path, O_RDWR | O_NOCTTY);
	if(fd < 0)
	{
		print_error("Could not open serial port %s ", path);
	}

	return fd;
}

//
// close serial port
//
int serial_close(int fd)
{
	if(fd >= 0)
	{
		int r = close(fd);

		//wait some time (prevents "Permission denied" error under windows)
		sleep(1);

		return r;
	}
	else
		return -1;
}


//
// close all ports
//
void serial_close_all()
{
	serial_close(serial.e);
	serial_close(serial.f);
	serial.e = -1;
	serial.f = -1;
}

//
// read from serial port
//
int serial_read(int fd, void *buf, int len)
{
	if(fd < 0)
	{
		return -1;
	}

	return read(fd, buf, (size_t)len);
}


//
// write to serial port
//
int serial_write(int fd, void *buf, int len)
{
	if(fd < 0)
	{
		fprintf(stderr, "serial_write: invalid fd %d\n", fd);
		return -1;
	}

	return (int)write(fd, buf, (size_t)len);
}


//
// set baudrate of open tty
//
void serial_set_baudrate(int port, int baudrate)
{
	struct termios tio;

	if(port < 0)
	{
		fprintf(stderr, "serial_set_baudrate: invalid fd %d\n", port);
		return;
	}

	speed_t speed = B38400;

	//raw mode for serial port
	cfmakeraw(&tio);

	tio.c_iflag = 0;
	tio.c_oflag = 0;
	tio.c_cflag &= ~CSIZE;
	tio.c_cflag |= CS8 | CLOCAL | CREAD;
	tio.c_cc[VTIME] = 1;
	tio.c_cc[VMIN] = SERIAL_READ_BUFFER;

	switch(baudrate)
	{
	case 4800: speed = B4800; break;
	case 9600: speed = B9600; break;
	case 19200: speed = B19200; break;
	case 38400: speed = B38400; break;
	case 115200: speed = B115200; break;
#ifdef B250000
	case 250000: speed = B250000; break;
#endif
#ifdef B500000
	case 500000: speed = B500000; break;
#endif
#ifdef B2500000
	case 2500000: speed = B2500000; break;
#endif
	default:
		fprintf(stderr, "Baudrate %d is not possible with your hardware\n", baudrate);
		break;
	}

	if(cfsetospeed(&tio, speed) == -1)
	{
		print_error("Don't know baudrate %d", baudrate);
	}

	if(tcsetattr(port, TCSANOW, &tio) == -1)
	{
		print_error("Could not set baudrate %d", baudrate);
	}

}

//
// Reads data from main serial input and stores into global buffer
//
int serial_read_buf(unsigned char *c)
{
#if SERIAL_READ_BUFFER <= 1
	return serial_read(serial.f, c, 1);
#else
	
	while(serial_buf_to_read <= 0)
	{
		serial_buf[0] = 0;
		serial_buf_to_read = serial_read(serial.f, serial_buf, SERIAL_READ_BUFFER);
		serial_buf_len = serial_buf_to_read;
		
		if (serial_buf_to_read == -1)
			break;
		else if (serial_buf_to_read == 0)
			usleep(10000);
	}

	if (serial_buf_to_read == -1)
		return 0;
	
	*c = serial_buf[serial_buf_len - serial_buf_to_read];
	serial_buf_to_read--;
	return 1;
	
#endif
}

/******************************************************************************/
/***** initialization *********************************************************/
/******************************************************************************/

//
// initialize serial struct
//
void init_struct_serial_type()
{
	int i;

	serial.f = -1;
	serial.e = -1;

	serial.tracker_baudrate = DEFAULT_TRACKER_BAUDRATE;
	serial.gps_baudrate = DEFAULT_GPS_BAUDRATE;

	serial.device[0] = '\0';
	serial.echo_device = NULL;



	for(i = 0; i < BAUDRATES; i++)
	{
		serial.tracker_baudrates[i] = 0;
		serial.gps_baudrates[i] = 0;
	}

	serial.tracker_baudrates[0] = 4800;
	serial.tracker_baudrates[1] = 9600;
	serial.tracker_baudrates[2] = 19200;
	serial.tracker_baudrates[3] = 38400;
	serial.tracker_baudrates[4] = 115200;
#ifdef B250000
	serial.tracker_baudrates[5] = 250000;
#endif
#ifdef B500000
	serial.tracker_baudrates[6] = 500000;
#endif
#ifdef B2500000
	serial.tracker_baudrates[7] = 2500000;
#endif


	serial.gps_baudrates[0] = 4800;
	serial.gps_baudrates[1] = 9600;
	serial.gps_baudrates[2] = 19200;
	serial.gps_baudrates[3] = 38400;


	serial.tracker_baudrates_string[0] = 0;
	serial.gps_baudrates_string[0] = 0;
	char buf[10];
	for(i = 0; i < BAUDRATES; i++)
	{
		if(serial.tracker_baudrates[i] > 0)
		{
			sprintf(buf, "%s%d", i > 0 ? ", " : "", serial.tracker_baudrates[i]);
			strcat(serial.tracker_baudrates_string, buf);
		}

		if(serial.gps_baudrates[i] > 0)
		{
			sprintf(buf, "%s%d", i > 0 ? ", " : "", serial.gps_baudrates[i]);
			strcat(serial.gps_baudrates_string, buf);
		}
	}

}


//
// Get the maximum signal rate (signals per sec) according to baudrate
//
float get_max_signal_rate()
{
	float max_signal_rate;
	max_signal_rate = (float)serial.tracker_baudrate / 500. / 7.5;

	if(max_signal_rate > MAX_SIGNAL_RATE)
		max_signal_rate = MAX_SIGNAL_RATE;

	return max_signal_rate;
}


//
// initialize Server type
//
void init_struct_server_type(struct server_type *s)
{
	s->addr[0] = 0;
	s->port = SERVER_PORT_1;
	s->sock_id = -1;
	s->username = NULL;
	s->password = NULL;
	s->login_ok = false;
	s->bytes_sent = 0;
}

//
// initialize Time type
//
void init_struct_Time_type(struct Time_type *t)
{
	t->year = 0;
	t->mon = 0;
	t->day = 0;
	t->hour = 0;
	t->min = 0;
	t->sec = 0;
	t->nsec = 0;
}



//
// initialize flag struct
//
void init_struct_flag_type()
{
	memset(&flag, 0, sizeof(flag));
	flag.requests_enabled = true;
	flag.watchdog_enabled = true;
}



//
// initialize logfile struct
//
void init_struct_logfile_type()
{
	logfiles.log.name = NULL;
	logfiles.log.fd = NULL;
	logfiles.out.name = NULL;
	logfiles.out.fd = NULL;
	logfiles.sent.name = NULL;
	logfiles.sent.fd = NULL;
	logfiles.http.name = NULL;
	logfiles.http.fd = NULL;
	logfiles.update.name = NULL;
	logfiles.update.fd = NULL;
}



//
// initialize S type
//
void init_struct_S_type(struct S_type *s)
{
	s->counter = -1;
	s->status = '-';
	for(int i = 0; i < INFO_BUFFER_SIZE; i++)
	{
		s->firmware_version[i] = 0;
		s->tracker_version[i] = 0;
		s->tracker_version_long[i] = 0;
	}

	s->lat = 0;
	s->lon = 0;
	s->alt = 0;
	s->average_lat = 0;
	s->average_lon = 0;
	s->average_alt = 0;
	s->sat = 0;
	s->average_sat = 0;
	s->board_is_sending = false;

	init_struct_Time_type(&s->time);
}



//
// initialize C type
//
void init_struct_C_type(struct control_type *c)
{
	c->accuracy_ok = false;
	c->seconds_flow_ok = false;
	c->time_ok = false;
	c->pos_ok = false;
	c->checksum_ok = false;
	c->faulty = false;
	c->faulty_rate = false;
	c->startup_phase = true;
	c->ringbuffer_reset = false;
	c->force_send = false;
	c->filter_enabled = 0;

	c->nogps_count = 0;
	c->signals_per_sec = 0;
	c->signals_per_sec_count = 0;
	c->signals_per_sec_count_filtered = 0;
	c->signals_per_sec_time = 0;
	c->signals_per_sec_max = 0;
	c->mean_signal_rate_last = 0;
	c->signals_per_sec_max_last = 0;
	c->mean_signal_rate = 0;
	c->nonzero_sec = 0;
	c->last_transmission_time = 0ll;

}



//
// initialize L type
//
void init_struct_L_type()
{
	L.counter = 0.;
	L.nsec = 0.;
	L.channels = 0;
	L.values = 0;
	L.bits = 0;
	L.signal_found = false;
	L.signal_ok = false;
	L.data[0] = 0;
	L.data_numeric[0] = 0;
	L.receive_time = time(NULL);
	L.gps_ok_time = time(NULL);
}



//
// initialize LT (Longtime) type
//
void init_struct_LT_type()
{	
	LT.reset_time = 0;
	LT.last_update = 0;

	LT.board_last_timeout = 0;
	LT.board_last_timeout_duration = 0;

	LT.signals = 0;
	LT.signals_sent = 0;
	LT.signals_filtered_min = 0;
	LT.signals_filtered_max = 0;
	LT.signals_faulty = 0;
	LT.signals_faulty_rate = 0;

	LT.sec_accuracy_nok = 0;
	LT.sec_seconds_flow_nok = 0;
	LT.sec_checksum_nok = 0;
	LT.sec_pos_nok = 0;
	LT.sec_gps_nok = 0;
	LT.sec_time_nok = 0;
	LT.sec_faulty = 0;
	LT.sec_faulty_rate = 0;

	LT.count_accuracy_nok = 0;
	LT.count_seconds_flow_nok = 0;
	LT.count_checksum_nok = 0;
	LT.count_pos_nok = 0;
	LT.count_gps_nok = 0;
	LT.count_time_nok = 0;
	LT.count_faulty = 0;
	LT.count_faulty_rate = 0;

	LT.min_sat = 999;
	LT.max_sat = 0;

	LT.bytes_sent = 0;
	LT.packets_sent = 0;

	LT.parameter_updates = 0;
	LT.parameter_updates_var = 0;
}


//
// initialize P (parameter) type
// has to be called after baudrate parameter has been parsed!
//
void init_struct_P_type(struct param_type *p)
{
	p->last_update = 0;
	p->last_update_success = 0;
	p->message[0] = '\0';

	p->signal_rate_interval = SIGNAL_RATE_INTERVAL;
	p->smooth_factor = SMOOTH_FACTOR;
	p->pos_precision = POS_PRECISION;
	p->pps_precision = PPS_PRECISION;
	p->alt_precision = ALT_PRECISION;
	p->max_nonzero_sec = MAX_NONZERO_SEC;
	p->max_signal_rate = get_max_signal_rate();
	p->min_amplitude = MIN_AMPLITUDE;
	p->max_amplitude = MAX_AMPLITUDE;
	p->max_amplitude_count = MAX_AMPLITUDE_COUNT;
	p->signal_check_bytes = SIGNAL_CHECK_BYTES;
	p->param_update_sec = PARAM_UPDATE_SEC;
	p->filter_enable = FILTER_ENABLE;
	p->send_board_output = 0;
	
	p->extra_server_addr[0] = '\0';
	p->extra_server_port = SERVER_PORT_1;

	strcpy(p->extra_server_username, Server[0].username);
	strcpy(p->extra_server_password, Server[0].password);
}


//
// initialize Signal type
//
void init_struct_Signal_type(struct Signal_type *s)
{
	init_struct_Time_type(&s->time);

	for(int i = 0; i <= STRING_BUFFER_SIZE; i++)
		s->data[i] = 128;

	s->channels = 0;
	s->values = 0;
	s->bits = 0;

}


//
//
//
char int_to_hex(int b)
{
	b &= 0x0F;
	if(b > 9)
	{
		return (b + 'A' - 10);
	}
	else
	{
		return (b + '0');
	}
}

//
//
//
char hex_to_int(char c)
{
	if((c >= '0') && (c <= '9'))
	{
		return (c - '0');
	}
	else
	{
		return (c - 'A' + 10);
	}
}


//
//
//
long double fabsl(long double x)
{
	return x < 0. ? -x : x;
}

//
// computes NMEA checksums between every $ and *
//
void fill_checksum(char *buf)
{
	unsigned int i;
	int c = 0;

	for(i = 0; i < strlen(buf); i++)
	{
		if(buf[i] == '$')
		{
			c = 0;
		}
		else if(buf[i] == '*')
		{
			i++;
			buf [i] = int_to_hex(c >> 4);
			i++;
			buf [i] = int_to_hex(c);
		}
		else
		{
			c ^= buf[i];
		}
	}
}

int compute_checksum(const char *buf)
{
	unsigned int i;
	int c = 0;

	for(i = 0; i < strlen(buf); i++)
	{
		if(buf[i] == '$')
		{
			c = 0;
		}
		else if(buf[i] == '*')
		{
			return (c);
		}
		else
		{
			c ^= buf[i];
		}
	}
	return (c);
}


//
// initialize the GPS module
//
bool init_gps(struct serial_type serial)
{
	char init_string [STRING_BUFFER_SIZE];

	strcpy(P.message, "Initializing GPS...");

	if(strcmp(serial.gps_type, "SANAV") == 0)
	{
		strcpy(init_string, init_gps_SANAV);
		if(flag.SBAS)
		{
			fprintf(stderr, "Do not know how to initialize GPS chip set SANAV with SBAS support!\n");
			exit(EXIT_FAILURE);
		}
		if(serial.gps_baudrate != 4800)
		{
			fprintf(stderr, "Do not know how to initialize GPS chip set SANAV with %d Baud!\n", serial.gps_baudrate);
			exit(EXIT_FAILURE);
		}
	}

	else if(strcmp(serial.gps_type, "Garmin") == 0)
	{
		strcpy(init_string, init_gps_Garmin);
		if(serial.gps_baudrate == 4800)
		{
			strcat(init_string, init_gps_Garmin_4800);
		}
		else if(serial.gps_baudrate == 9600)
		{
			strcat(init_string, init_gps_Garmin_9600);
		}
		else if(serial.gps_baudrate == 19200)
		{
			strcat(init_string, init_gps_Garmin_19200);
		}
		else if(serial.gps_baudrate == 38400)
		{
			strcat(init_string, init_gps_Garmin_38400);
		}
		else
		{
			fprintf(stderr, "Do not know how to initialize GPS chip set Garmin with %d Baud!\n", serial.gps_baudrate);
			exit(EXIT_FAILURE);
		}

		if(flag.SBAS)
		{
			strcat(init_string, init_gps_Garmin_SBAS_on);
		}
		else
		{
			strcat(init_string, init_gps_Garmin_SBAS_off);
		}
	}

	else if(strcmp(serial.gps_type, "SiRF") == 0)
	{
		strcpy(init_string, init_gps_SiRF);
		if(serial.gps_baudrate == 4800)
		{
			strcat(init_string, init_gps_SiRF_4800);
		}
		else if(serial.gps_baudrate == 9600)
		{
			strcat(init_string, init_gps_SiRF_9600);
		}
		else if(serial.gps_baudrate == 19200)
		{
			strcat(init_string, init_gps_SiRF_19200);
		}
		else if(serial.gps_baudrate == 38400)
		{
			strcat(init_string, init_gps_SiRF_38400);
		}
		else
		{
			fprintf(stderr, "Do not know how to initialize GPS chip set SiRF with %d Baud!\n", serial.gps_baudrate);
			exit(EXIT_FAILURE);
		}
		if(flag.SBAS)
		{
			strcat(init_string, init_gps_SiRF_SBAS_on);
		}
		else
		{
			strcat(init_string, init_gps_SiRF_SBAS_off);
		}
	}

	else if(strcmp(serial.gps_type, "-") == 0)
	{
		print_log(LOG_GENERAL, "No GPS type given. No initialization!\n");
		return true;
	}

	else
	{
		fprintf(stderr, "Do not know how to initialize GPS type %s!\n", serial.gps_type);
		fprintf(stderr, "Use SANAV, Garmin, SiRF, or - for no initialization.\n");
		exit(EXIT_FAILURE);
	}

	fill_checksum(init_string);
	//print_log (LOG_GENERAL, init_string);

	int br = 38400;
	for(int b = 0; b < 4; b++)
	{
		int f = serial_open(serial.device);

		if(f < 0)
			return false;

		serial_set_baudrate(f, br);
		print_log(LOG_GENERAL, "initialize GPS chip set %s, %d baud, using %d baud, please wait!\n", serial.gps_type, serial.gps_baudrate, br);
		usleep(1000000);
		serial_write(f, init_string, strlen(init_string));
		usleep(1000000);
		serial_close(f);
		usleep(100000);
		br >>= 1;
	}

	print_log(LOG_GENERAL, "GPS initialized!\n");

	return true;
}




//
// initialize (almost) all variables
//
void init_vars()
{
	init_struct_serial_type();
	init_struct_flag_type();
	init_struct_logfile_type();
	init_struct_L_type();
	init_struct_LT_type();

	init_struct_C_type(&C);
	init_struct_C_type(&last_C);

	init_struct_S_type(&S);
	init_struct_S_type(&last_S);

	init_struct_Time_type(&S.time);

	init_struct_Signal_type(&LastGoodSignal);
	init_struct_Signal_type(&LastMinSignal);
	init_struct_Signal_type(&LastMaxSignal);
	init_struct_Signal_type(&LastIMode1Signal);
	init_struct_Signal_type(&LastIMode2Signal);
	init_struct_Signal_type(&FirstIMode1Signal);
	init_struct_Signal_type(&FirstIMode2Signal);

	for(int i = 0; i < BUFFERD_SIGNALS; i++)
		init_struct_Signal_type(&LastSignal[i]);

	init_struct_server_type(&Server[0]);
	init_struct_server_type(&Server[1]);
	init_struct_server_type(&Server[2]);

	LT.start_time = time(NULL);

}

//
// Update IP address of computing server
//
bool compute_server_updateip(struct server_type *s)
{
	struct hostent *host_info;

	if(strlen(s->addr) > 1 && inet_addr(s->addr) == INADDR_NONE)
	{
		/* host not given by IP but by name */
		host_info = gethostbyname(s->addr);
		if(host_info == NULL)
		{
			print_log(LOG_GENERAL, "gethostbyname (%s) failed, try again in 10 seconds!\n", s->addr);
			sleep(10);
			host_info = gethostbyname(s->addr);
			if(host_info == NULL)
			{
				print_log(LOG_GENERAL, "gethostbyname (%s) failed, try again in 60 seconds!\n", s->addr);
				sleep(60);
				host_info = gethostbyname(s->addr);
				if(host_info == NULL)
				{
					print_log(LOG_GENERAL, "gethostbyname (%s) failed, give up, please check your internet connection!\n", s->addr);
					print_error("gethostbyname (%s)", s->addr);
					close(s->sock_id);

					return false;
				}
			}
		}

		if(memcmp((char *) &s->serv_addr.sin_addr.s_addr, host_info->h_addr, host_info->h_length) != 0)
		{
			memcpy((char *) &s->serv_addr.sin_addr.s_addr, host_info->h_addr, host_info->h_length);
			print_log(LOG_GENERAL, "DNS-Request: IP of %s is %s\n", s->addr, inet_ntoa(s->serv_addr.sin_addr));
		}


	}

	return true;
}

//
// Open Socket to Server
//
void compute_server_connect(struct server_type *s, bool exit_on_failure)
{

	strcpy(P.message, "Connecting to computing server...");

	//Create UDP Socket
	s->sock_id = socket(AF_INET, SOCK_DGRAM, 0);
	if(s->sock_id == -1)
	{
		perror("socket(AF_INET, SOCK_DGRAM, 0)");
		if(exit_on_failure)
			exit(EXIT_FAILURE);
		else
			return;
	}

	s->serv_addr.sin_family = AF_INET;
	s->serv_addr.sin_port = htons(s->port);
	s->serv_addr.sin_addr.s_addr = inet_addr(s->addr);

	if(compute_server_updateip(s))
	{
		print_log(LOG_GENERAL, "Socket ready to send to %s:%d\n", s->addr, s->port);
	}
	else if(exit_on_failure)
		exit(EXIT_FAILURE);

	s->bytes_sent = 0;
}



//
// Close Socket to Server
//
void compute_server_close(struct server_type *s)
{
	if(s->sock_id > 0)
	{
		close(s->sock_id);
		print_log(LOG_GENERAL, "Closed socket for %s:%d, sent %lld bytes\n", s->addr, s->port, s->bytes_sent);
	}

	s->sock_id = -1;
}



/******************************************************************************/
/***** send_strike ************************************************************/
/******************************************************************************/


//
// build string for sending to server (and logging to file)
//
void build_sent_string(char *buf, char *username, char *password)
{
	sprintf(buf, "%04d-%02d-%02d %02d:%02d:%02d.%09lld %.6Lf %.6Lf %.0Lf %s %s %c %d %d %d %d %s %s %s %d/%d %s\n",
	        S.time.year, S.time.mon, S.time.day, S.time.hour, S.time.min, S.time.sec, L.nsec,
	        S.average_lat, S.average_lon, S.average_alt, username, password,
	        S.status, S.sat, L.channels, L.values, L.bits, L.data, S.tracker_version, S.firmware_version,
	        serial.gps_baudrate, serial.tracker_baudrate, serial.gps_type);
}


//
// send signal information
//
void send_strike_to_server(char *buf, struct server_type *server)
{
	if(server->sock_id <= 0)
		return;

	if(sendto(server->sock_id, buf, strlen(buf), 0, (struct sockaddr *) &server->serv_addr, sizeof(struct sockaddr)) == -1)
	{
		print_log(LOG_GENERAL, "error: sendto %s:%d\n", server->addr, server->port);
	}
	else
	{
		server->bytes_sent += strlen(buf);
	}
}

//
// Send signal data to different servers
//
void send_strike()
{

	char buf [STRING_BUFFER_SIZE];
	if((S.time.year < 2012) || (S.time.year > 2031))
	{
		print_log(LOG_GENERAL, "error: year not between 2011 and 2031\n");
		return;
	}
	if((S.time.mon < 1) || (S.time.mon > 12))
	{
		print_log(LOG_GENERAL, "error: month not between 1 and 12\n");
		return;
	}
	if((S.time.day < 1) || (S.time.day > 31))
	{
		print_log(LOG_GENERAL, "error: day not between 1 and 31\n");
		return;
	}
	if((S.time.hour < 0) || (S.time.hour > 23))
	{
		print_log(LOG_GENERAL, "error: hour not between 0 and 23\n");
		return;
	}
	if((S.time.min < 0) || (S.time.min > 59))
	{
		print_log(LOG_GENERAL, "error: minutes not between 0 and 59\n");
		return;
	}
	if((S.time.sec < 0) || (S.time.sec > 59))
	{
		print_log(LOG_GENERAL, "error: seconds not between 0 and 59\n");
		return;
	}
	if((L.nsec < 0) || (L.nsec > 999999999ll))
	{
		print_log(LOG_GENERAL, "error: nanoseconds not between 0 and 999999999\n");
		return;
	}
	if((S.average_lat < -90) || (S.average_lat > 90))
	{
		print_log(LOG_GENERAL, "error: latitude not between -90 and 90\n");
		return;
	}
	if((S.average_lon < -180) || (S.average_lon > 180))
	{
		print_log(LOG_GENERAL, "error: latitude not between -180 and 180\n");
		return;
	}
	if((S.average_alt < -1000) || (S.average_alt > 10000))
	{
		print_log(LOG_GENERAL, "error: altitude not between -1000 and 10000\n");
		return;
	}
	if((S.status != 'A') && (S.status != 'V'))
	{
		print_log(LOG_GENERAL, "error: status not 'A' or 'V'\n");
		return;
	}


#ifndef SIMPLE_TRACKER
	//
	// Lock other threads...
	// especially for Server[2]
	//
	pthread_mutex_lock(&thread_mutex);
#endif


	//
	//Send to Main Server
	//
	if ( !(P.send_board_output&8) )
	{
		build_sent_string(buf, Server[0].username, Server[0].password);
		send_strike_to_server(buf, &Server[0]);
		LT.bytes_sent += strlen(buf);
		LT.packets_sent++;

		if(flag.verbose_sent)
		{
			printf("%s", buf);
			fflush(stdout);
		}
		if(logfiles.sent.name != NULL)
		{
			fprintf(logfiles.sent.fd, "%s", buf);
			fflush(logfiles.sent.fd);
		}
	}
	
	//
	// To additional Server
	//
	if(strlen(Server[1].addr) && !(P.send_board_output&(8 << 4) ) )
	{
		build_sent_string(buf, Server[1].username, Server[1].password);
		send_strike_to_server(buf, &Server[1]);
	}



	//
	// To remote added Server
	// but only if it's another server/port than main server
	//
	if(strlen(Server[2].addr)  && !(P.send_board_output&(8 << 8))
		 && (strcmp(Server[0].addr, Server[2].addr) || Server[0].port != Server[2].port ) )
	{
		build_sent_string(buf, Server[2].username, Server[2].password);
		send_strike_to_server(buf, &Server[2]);
	}

#ifndef SIMPLE_TRACKER
	pthread_mutex_unlock(&thread_mutex);
#endif

}

//
// Send board output to servers
//
void send_board_output_remote(const char *line, bool is_unknown, bool is_signal)
{
	int i;
	bool only_unknown, only_non_signal;
	
	for (i=0; i<3;i++)
	{
		if (strlen(Server[2].addr) <= 0)
			continue;
		
		//send debug?
		if (!(P.send_board_output & (1 << i*4)))
			continue;
		
		//send only unknown sentences
		only_unknown    = P.send_board_output & (2 << i*4);
		
		//send only non-signals
		only_non_signal = P.send_board_output & (4 << i*4);
		
		if (    (only_unknown    && is_unknown) 
			 || (only_non_signal && !is_signal)
			 || (!only_unknown   && !only_non_signal))
		{
			char buf[STRING_BUFFER_SIZE];
			sprintf(buf, "%s %s %s", Server[2].username, Server[2].password, line);
			send_strike_to_server(buf, &Server[i]);
		}

	}
	
}

/******************************************************************************/
/***** evaluate ***************************************************************/
/******************************************************************************/

//
// Calculate nsec from counter difference
//
long long counter_difference_to_nsec(long long counter_difference)
{
	if((int)C.sum.S_counter_difference == 0)
		return 0.;

	if(counter_difference < 0)
	{
		counter_difference += 0x1000000ll;
	}

	return (long long)(counter_difference * RING_BUFFER_SIZE * 1000000000ll) / C.sum.S_counter_difference;
}


//
// log status
//
void log_status()
{


	if((!(last_C.faulty)) && (C.faulty))
		print_log(LOG_GENERAL, "interference mode started, %d signals per second, %d seconds nonzero\n", last_C.signals_per_sec, C.nonzero_sec);
	if((last_C.faulty) && (!(C.faulty)))
		print_log(LOG_GENERAL, "interference mode stopped\n");

	if((!(last_C.faulty_rate)) && (C.faulty_rate))
		print_log(LOG_GENERAL, "interference mode (2) started, mean signal rate greater than %.1f signals per second!\n", P.max_signal_rate);
	if((last_C.faulty_rate) && (!(C.faulty_rate)))
		print_log(LOG_GENERAL, "interference mode (2) stopped\n");


	if(((last_C.faulty) && (C.faulty)) || ((last_C.faulty_rate) && (C.faulty_rate)))
		return;

	if(!(C.checksum_ok))
		print_log(LOG_GENERAL, "Checksum error\n");

	if((!(last_C.time_ok)) && (C.time_ok))
		print_log(LOG_GENERAL, "Time is ok\n");
	if((last_C.time_ok) && (!(C.time_ok)))
		print_log(LOG_GENERAL, "Time is wrong\n");

	if((!(last_C.seconds_flow_ok)) && (C.seconds_flow_ok))
		print_log(LOG_GENERAL, "GPS seconds running, %d %d\n", last_S.time.sec, S.time.sec);
	if((last_C.seconds_flow_ok) && (!(C.seconds_flow_ok)))
		print_log(LOG_GENERAL, "GPS seconds stopped, %d %d\n", last_S.time.sec, S.time.sec);

	if(last_S.status != S.status)
		print_log(LOG_GENERAL, "GPS status changed from '%c' to '%c'\n", last_S.status, S.status);

	if((!(last_C.pos_ok)) && (C.pos_ok))
		print_log(LOG_GENERAL, "Position fixed, lat: %+.6Lf, lon: %+.6Lf, alt: %+.1Lf\n", S.average_lat, S.average_lon, S.average_alt);
	if((last_C.pos_ok) && (!(C.pos_ok)))
		print_log(LOG_GENERAL, "Position lost, lat: %+.6Lf, lon: %+.6Lf, alt: %+.1Lf\n", S.lat, S.lon, S.alt);

	if((!(last_C.accuracy_ok)) && (C.accuracy_ok))
		print_log(LOG_GENERAL, "1PPS signal accuracy ok, counter: %06llX %06llX, %+.0Lf nsec\n", last_S.counter, S.counter, precision.PPS * 1000000000.0l);
	if((last_C.accuracy_ok) && (!(C.accuracy_ok)))
		print_log(LOG_GENERAL, "1PPS signal inaccurate, counter: %06llX %06llX, accuracy: %+.0LF nsec\n", last_S.counter, S.counter, precision.PPS * 1000000000.0l);


	if((!(last_C.filter_enabled)) && (C.filter_enabled))
		print_log(LOG_GENERAL, "Filter enabled!\n");
	if((last_C.filter_enabled) && (!(C.filter_enabled)))
		print_log(LOG_GENERAL, "Filter disabled!\n");


	//First good values -> extra message
	if(!C.startup_phase && C.startup_phase != last_C.startup_phase)
		print_log(LOG_GENERAL, "We have a good average position and PPS counter. Let's go...\n");

	//Message when ringbuffer needs a reset
	if(!C.startup_phase && C.ringbuffer_reset && !last_C.ringbuffer_reset)
		print_log(LOG_GENERAL, "GPS without good position since %d sec - resetting average position!\n", C.nogps_count);

}


//
// Save signal data and time
//
void save_signal(struct Signal_type *s)
{
	int i;

	s->time = S.time;
	s->time.nsec = counter_difference_to_nsec(L.counter - S.counter);
	s->channels = L.channels;
	s->values = L.values > STRING_BUFFER_SIZE ? STRING_BUFFER_SIZE : L.values;
	s->bits = L.bits;
	memcpy(s->data, L.data_numeric, STRING_BUFFER_SIZE);

	//Save last signal in extra buffer, if it is a new signal (= nsec differs)
	if(s->time.nsec != LastSignal[0].time.nsec)
	{
		for(i = BUFFERD_SIGNALS - 1; i > 0; i--)
		{
			LastSignal[i] = LastSignal[i - 1];
		}

		LastSignal[0] = *s;
	}

}



//
// check signal amplitude
//
void evaluate_signal()
{
	L.signal_ok = true;

	//check signal only if valid data was found
	if(L.signal_found && L.data[0] != '-' && L.bits == 8 && L.channels > 0)
	{
		int i, fails1, fails2;
		size_t count;
		char *pos = L.data;
		int check_bytes = P.signal_check_bytes;

		if(check_bytes > L.values)
			check_bytes = L.values;

		//hex string to char-array
		for(count = 0; count < STRING_BUFFER_SIZE; count++)
		{
			sscanf(pos, "%2hhx", &L.data_numeric[count]);
			pos += 2 * sizeof(char);
		}

		if(C.filter_enabled && P.min_amplitude > 0)
		{
			L.signal_ok = false;

			//check only first bytes of the signal
			for(i = 0; i < L.channels * check_bytes; i++)
			{
				if(abs(L.data_numeric[i] - 128) > P.min_amplitude)
				{
					L.signal_ok = true;
					break;
				}
			}

			if(L.signal_ok == false)
			{
				save_signal(&LastMinSignal);
				LT.signals_filtered_min++;
			}

			if(!L.signal_ok && !C.faulty && !C.faulty_rate)
			{
				if (C.signals_per_sec_count_filtered < 3)
				{
					print_log(LOG_GENERAL | LOG_NOSYSLOG, "Filter: signal didn't reach minimum amplitude\n");
				}
				else if (C.signals_per_sec_count_filtered == 3)
				{
					print_log(LOG_GENERAL | LOG_NOSYSLOG, "Filter: signal didn't reach minimum amplitude - stopped logging\n");
				}
			}
		}

		//check maximum amplitudes if previous test was good
		if(C.filter_enabled && L.signal_ok && P.max_amplitude > 0)
		{
			fails1 = 0;
			fails2 = 0;
			for(i = 0; i < L.channels * L.values; i++)
			{
				if(abs(L.data_numeric[i] - 128) >= P.max_amplitude)
				{
					if(i % L.channels == 0)
						fails1++;
					else
						fails2++;

					if(fails1 >= P.max_amplitude_count && (L.channels == 1 || fails2 >= P.max_amplitude_count))
					{
						L.signal_ok = false;
						save_signal(&LastMaxSignal);
						LT.signals_filtered_max++;
						break;
					}


				}
			}

			if(!L.signal_ok && !C.faulty && !C.faulty_rate)
			{
				if (C.signals_per_sec_count_filtered < 3)
				{
					print_log(LOG_GENERAL | LOG_NOSYSLOG, "Filter: signal was too long at max. amplitude\n");
				}
				else if (C.signals_per_sec_count_filtered == 3)
				{
					print_log(LOG_GENERAL | LOG_NOSYSLOG, "Filter: signal was too long at max. amplitude - stopped logging\n");
				}
			}
		}

	}


	//
	//Save signal
	//only assign "good" signals to imode as bad ones don't affect imodes
	//
	if(L.signal_ok)
	{
		save_signal(&LastGoodSignal);

		if(!last_C.faulty && C.faulty)
			save_signal(&FirstIMode1Signal);

		if(C.faulty)
			save_signal(&LastIMode1Signal);

		if(!last_C.faulty_rate && C.faulty_rate)
			save_signal(&FirstIMode2Signal);

		if(C.faulty_rate)
			save_signal(&LastIMode2Signal);

	}
	else
	{
		C.signals_per_sec_count_filtered++;
	}

	return;
}


//
// fill the ring buffer with values
//
void fill_ring_buffer(long long counter_difference)
{
	if(S.lat != 0.0 && S.lon != 0.0 && (3 <= S.sat && S.sat <= 20))
	{
		// fill the ring buffer
		ring_buffer [ring_buffer_index].S_counter_difference = (long double) counter_difference;
		ring_buffer [ring_buffer_index].lat = S.lat;
		ring_buffer [ring_buffer_index].lon = S.lon;
		ring_buffer [ring_buffer_index].alt = S.alt;
		ring_buffer [ring_buffer_index].sat = S.sat;

		ring_buffer_index = (ring_buffer_index + 1) % RING_BUFFER_SIZE;
	}
}




//
// check signal counts and rate for interference mode
//
void evaluate_imode()
{

	//
	// Interference-Mode 1
	//  -> Max. non-zero seconds
	//
	if(C.signals_per_sec == 0)
	{
		C.nonzero_sec = 0;
		C.faulty = false;
		
		if (!C.signals_per_sec_count_filtered) //disable filter
			C.filter_enabled &= 2; //leave info for imode 2!
	}
	else
	{
		C.signals_per_sec_max = C.signals_per_sec_max > C.signals_per_sec ? C.signals_per_sec_max : C.signals_per_sec;
		C.nonzero_sec++;

		// i-mode condition!
		if(P.max_nonzero_sec >= 0 && C.nonzero_sec >= P.max_nonzero_sec && !C.faulty)
		{
			if(!(C.filter_enabled&1) && P.filter_enable == 1)
			{
				//enable filter on imode-condition
				C.filter_enabled |= 1;
			}
			else
			{
				C.faulty = true;

				if(!last_C.faulty && C.faulty)
					C.force_send = true;
			}
		}

		C.signals_per_sec = 0;
	}


	//
	// Interference-Mode 2
	//  -> Max. signal rate
	//
	C.signals_per_sec_time++;
	if(C.signals_per_sec_time >= P.signal_rate_interval)
	{
		C.mean_signal_rate = (float)C.signals_per_sec_count / (float)C.signals_per_sec_time;
		if(C.mean_signal_rate >= (float)SHOW_SIGNAL_RATE)
		{
			print_log(LOG_GENERAL, "%d signals in the last %ds, rate: %.1f/s; max. %d signals in one second\n", C.signals_per_sec_count, C.signals_per_sec_time, C.mean_signal_rate, C.signals_per_sec_max);
		}

		
		if (!C.signals_per_sec_count_filtered) //disable filter
			C.filter_enabled &= 1; //leave info for imode 1!
		
		if(P.max_signal_rate >= 0 && C.mean_signal_rate > P.max_signal_rate && !C.faulty_rate)
		{
			if(!(C.filter_enabled&2) && P.filter_enable == 1)
			{
				//enable filter on imode-condition
				C.filter_enabled |= 2;
			}
			else
			{
				C.faulty_rate = true;

				if(!last_C.faulty_rate && C.faulty_rate)
					C.force_send = true;
			}
		}
		else
		{
			C.faulty_rate = false;
			C.filter_enabled &= 1;
		}
			
		C.mean_signal_rate_last = C.mean_signal_rate;
		C.signals_per_sec_max_last = C.signals_per_sec_max;
		C.signals_per_sec_time = 0;
		C.signals_per_sec_count = 0;
		C.signals_per_sec_count_filtered = 0;
		C.signals_per_sec_max = 0;

	}


}


//
// update longtime vars
//
void update_longtime(bool status_msg)
{

	if(status_msg)
	{

		if(!C.startup_phase && LT.last_update != time(NULL))
		{
			LT.last_update = time(NULL);

			if(!C.accuracy_ok)
				LT.sec_accuracy_nok++;

			if(!C.time_ok)
				LT.sec_time_nok++;

			if(!C.seconds_flow_ok)
				LT.sec_seconds_flow_nok++;

			if(!C.checksum_ok)
				LT.sec_checksum_nok++;

			if(!C.pos_ok)
				LT.sec_pos_nok++;

			if(S.status != 'A')
				LT.sec_gps_nok++;

			if(S.status != '-' && S.sat == last_S.sat && S.sat < LT.min_sat)
				LT.min_sat = S.sat;
		}

		if(C.faulty)
			LT.sec_faulty++;

		if(C.faulty_rate)
			LT.sec_faulty_rate++;

		if(S.status != '-' && S.sat == last_S.sat && S.sat > LT.max_sat && S.sat <= 20)
			LT.max_sat = S.sat;

		if(last_C.accuracy_ok && !C.accuracy_ok)
			LT.count_accuracy_nok++;

		if(last_C.time_ok && !C.time_ok)
			LT.count_time_nok++;

		if(last_C.seconds_flow_ok && !C.seconds_flow_ok)
			LT.count_seconds_flow_nok++;

		if(last_C.checksum_ok && !C.checksum_ok)
			LT.count_checksum_nok++;

		if(last_C.pos_ok && !C.pos_ok)
			LT.count_pos_nok++;

		if(last_S.status == 'V' && S.status == 'A')
			LT.count_gps_nok++;

		if(!last_C.faulty && C.faulty)
			LT.count_faulty++;

		if(!last_C.faulty_rate && C.faulty_rate)
			LT.count_faulty_rate++;

	}
	else if(L.signal_found)
	{
		LT.signals++;

		if(C.faulty && L.signal_ok)
			LT.signals_faulty++;

		if(C.faulty_rate && L.signal_ok)
			LT.signals_faulty_rate++;
	}

}




//
// evaluate received line
//
void evaluate(const char *line)
{
	int year, mon, day, min, hour, sec = 0, lat_deg, lon_deg, satellites = 0, no_param = 0;
	long double lat_min, lon_min, alt = 0.0l;
	char north_south, west_east, status;
	long long counter, counter_difference;
	char firmware_version [INFO_BUFFER_SIZE];
	firmware_version [0] = 0;
	int checksum;
	int A, B;
	double smooth_factor;
	int sec_running;
	bool unknown_sentence = false;

	L.signal_found = false;

	
	//
	// Second sentences
	//
	if((((no_param = sscanf(line, "$S,%llX,%c,%2d%2d%2d,%2d%2d%2d,%2d%Lf,%c,%3d%Lf,%c,%Lf,%d,%3s*%x",
	                        &counter,
	                        &status,
	                        &hour, &min, &sec, &day, &mon, &year,
	                        &lat_deg, &lat_min, &north_south,
	                        &lon_deg, &lon_min, &west_east, &alt,
	                        &satellites,
	                        firmware_version,
	                        &checksum)) == 18) ||
	        ((no_param = sscanf(line, "$BS,%llX,%c,%2d%2d%2d,%2d%2d%2d,%2d%Lf,%c,%3d%Lf,%c,%Lf,%d,%3s*%x",
	                            &counter,
	                            &status,
	                            &hour, &min, &sec, &day, &mon, &year,
	                            &lat_deg, &lat_min, &north_south,
	                            &lon_deg, &lon_min, &west_east, &alt,
	                            &satellites,
	                            firmware_version,
	                            &checksum)) == 18) ||
	        ((no_param = sscanf(line, "$BLSEC,%llX,%c,%2d%2d%2d,%2d%2d%2d,%2d%Lf,%c,%3d%Lf,%c,%Lf,M,%d*%x",
	                            &counter,
	                            &status,
	                            &hour, &min, &sec, &day, &mon, &year,
	                            &lat_deg, &lat_min, &north_south,
	                            &lon_deg, &lon_min, &west_east, &alt,
	                            &satellites,
	                            &checksum)) == 17) ||
	        ((no_param = sscanf(line, "$BLSEC,%2d%2d%2d,%2d%2d%2d,%c,%2d%Lf,%c,%3d%Lf,%c,%llX*%x",
	                            &hour, &min, &sec, &day, &mon, &year,
	                            &status,
	                            &lat_deg, &lat_min, &north_south,
	                            &lon_deg, &lon_min, &west_east,
	                            &counter,
	                            &checksum)) == 15)) &&
	        (C.checksum_ok = (checksum == compute_checksum(line))))
	{


		if(no_param == 15)
		{
			satellites = -1;
			alt = 0.0l;
			strcpy(firmware_version, "15a");
		}
		if(no_param == 17)
		{
			strcpy(firmware_version, "19a");
		}

		last_S = S;
		S.counter = counter;
		S.status = status;
		S.time.hour = hour;
		S.time.min = min;
		S.time.sec = sec;
		S.time.day = day;
		S.time.mon = mon;
		S.time.year = year + 2000;
		S.lat = lat_deg + lat_min / 60.0;
		if(north_south == 'S')
		{
			S.lat = -S.lat;
		}
		S.lon = lon_deg + lon_min / 60.0;
		if(west_east == 'W')
		{
			S.lon = -S.lon;
		}
		S.alt = alt;
		S.sat = satellites;
		strcpy(S.firmware_version, firmware_version);


		if(C.startup_phase)
			C.ringbuffer_reset = true;

		if(last_S.counter >= 0)
		{

			last_C = C;
			C.seconds_flow_ok = (S.time.sec == (last_S.time.sec + 1) % 60);
			C.time_ok = (S.time.year == last_S.time.year || S.time.year  == last_S.time.year + 1)
			            && (S.time.mon  == last_S.time.mon  || S.time.mon - 1 == (last_S.time.mon) % 12)
			            && (S.time.day  == last_S.time.day  || S.time.day - 1 == (last_S.time.day) % 31)
			            && (S.time.hour == last_S.time.hour || S.time.hour  == (last_S.time.hour + 1) % 24)
			            && (S.time.min  == last_S.time.min  || S.time.min   == (last_S.time.min + 1) % 60) ;

			counter_difference = S.counter - last_S.counter;
			if(counter_difference < 0)
			{
				counter_difference += 0x1000000ll;
			}

			//at startup phase, fill ringbuffer always
			if(C.ringbuffer_reset)
				fill_ring_buffer(counter_difference);


			//compute and check GPS positions (3D)
			C.sum.lat = 0.0;
			C.sum.lon = 0.0;
			C.sum.alt = 0.0;
			C.sum.sat = 0.0;
			for(int i = 0; i < RING_BUFFER_SIZE; i++)
			{
				C.sum.lat += ring_buffer [i].lat;
				C.sum.lon += ring_buffer [i].lon;
				C.sum.alt += ring_buffer [i].alt;
				C.sum.sat += ring_buffer [i].sat;
			}
			average.lat = C.sum.lat / RING_BUFFER_SIZE;
			average.lon = C.sum.lon / RING_BUFFER_SIZE;
			average.alt = C.sum.alt / RING_BUFFER_SIZE;
			average.sat = C.sum.sat / RING_BUFFER_SIZE;
			precision.lat = fabsl(S.lat - average.lat);
			precision.lon = fabsl(S.lon - average.lon);
			precision.alt = fabsl(S.alt - average.alt);
			C.pos_ok = (precision.lat + precision.lon < P.pos_precision)
			           && (S.lat != 0.0 && S.lon != 0.0)
			           && (precision.alt < P.alt_precision);

			//compute and check GPS counter (1PPS)
			C.sum.S_counter_difference = 0;
			for(int i = 0; i < RING_BUFFER_SIZE; i++)
				C.sum.S_counter_difference += ring_buffer[i].S_counter_difference;
			average.S_counter_difference = C.sum.S_counter_difference / RING_BUFFER_SIZE;
			precision.S_counter = average.S_counter_difference - (long double)counter_difference;
			precision.PPS = (long double)precision.S_counter / average.S_counter_difference;
			C.accuracy_ok = (fabsl(precision.PPS) < P.pps_precision);


			//position is only ok, when *all* values (mean value) in the
			//ringbuffer are nearby the current value
			if(C.pos_ok && last_C.pos_ok && C.accuracy_ok && C.seconds_flow_ok)
			{
				//normal operation: fill ringbuffer only, when pos-precision is good enough
				if(!C.ringbuffer_reset)
					fill_ring_buffer(counter_difference);


				//lower smooth factor during first minutes/hours
				sec_running = (int)(time(NULL) - LT.start_time);
				if(sec_running < P.smooth_factor)
					smooth_factor = (double)sec_running;
				else
					smooth_factor = (double)P.smooth_factor;


				//smoothed averages
				S.average_lat = (S.average_lat * (smooth_factor - 1) + average.lat) / smooth_factor;
				S.average_lon = (S.average_lon * (smooth_factor - 1) + average.lon) / smooth_factor;
				S.average_alt = (S.average_alt * (smooth_factor - 1) + average.alt) / smooth_factor;
				S.average_sat = (S.average_sat * (smooth_factor - 1) + average.sat) / smooth_factor;
				S.average_pps = (S.average_pps * (smooth_factor - 1) + fabsl(precision.PPS)) / smooth_factor;
			}
			else if(C.ringbuffer_reset)
			{
				S.average_lat = average.lat;
				S.average_lon = average.lon;
				S.average_alt = average.alt;
				S.average_sat = average.sat;
				S.average_pps = 0;
			}

			//
			// check accuracy and GPS-fix
			//
			if(C.accuracy_ok && C.pos_ok)
			{
				C.startup_phase = false;
				C.ringbuffer_reset = false;
				C.nogps_count = 0;
			}
			else
			{
				C.nogps_count++;

				//reset ringbuffer, if no gps-fix for a long time
				if(!C.startup_phase && C.nogps_count > RING_BUFFER_SIZE)
				{
					C.ringbuffer_reset = true;
					C.nogps_count = 0;
				}
			}

			//
			// Remember last good GPS ok for watchdog
			//
			if(S.lat != 0.0 && S.lon != 0.0 && C.seconds_flow_ok && C.time_ok)
			{
				L.gps_ok_time = time(NULL);
			}
			else if(S.lat == 0.0 || S.lon == 0.0)
			{
				L.gps_ok_time = 0;
			}

			//Filter status
			if(P.filter_enable == 2)
				C.filter_enabled = 1;
			else if(P.filter_enable == 0)
				C.filter_enabled = 0;


			//
			//check and set interference-mode
			//
			if (C.accuracy_ok && C.pos_ok && C.seconds_flow_ok && C.time_ok)
			{
				evaluate_imode();
			}

			//
			//log to stdout or logfile
			//
			log_status();

			//
			//update long-time values each second
			//
			update_longtime(true);


			if(flag.verbose_info)
			{
				//Cast (int)counter_difference is for Windows Cygwin Bug
				printf("%d lat: %+.6Lf, lon: %+.6Lf, alt: %+.1Lf, acc: %+.0Lf nsec, sat: %d\n", (int)counter_difference, S.average_lat, S.average_lon, S.average_alt, precision.PPS * 1000000000.0l, (int)S.sat);
				fflush(stdout);
			}

			if(logfiles.info.name != NULL)
			{
				//Cast (int)counter_difference is for Windows Cygwin Bug
				fprintf(logfiles.info.fd, "%d lat: %+.6Lf, lon: %+.6Lf, alt: %+.1Lf, acc: %+.0Lf nsec, sat: %d\n", (int)counter_difference, S.average_lat, S.average_lon, S.average_alt, precision.PPS * 1000000000.0l, (int)S.sat);
				fflush(logfiles.info.fd);
			}


			//send info message
			// only when GPS is ok AND
			// - every 10 minutes
			// - or when forced, i.e. when entering/leaving interference-mode
			long long time = ensec_time();
			if((C.accuracy_ok) && (!C.ringbuffer_reset) && (C.seconds_flow_ok) && (C.time_ok) &&
			        (time > C.last_transmission_time + 600000000000ll || C.force_send)
			  )
			{
				L.channels = 0;
				L.values = 0;
				L.bits = C.nonzero_sec;
				strcpy(L.data, "-");
				send_strike();
				C.last_transmission_time = time;
				C.force_send = false;
			}

		}
	}

	//
	// Lightning sentence
	//
	// BLSIG sentence type 1
	else if((sscanf(line, "$BLSIG,%6llx,%2x,%2x*%x", &L.counter, &A, &B, &checksum) == 4) && (C.checksum_ok = (checksum == compute_checksum(line))))
	{
		sprintf(L.data, "%02x%02x", A, B);
		L.channels = 2;
		L.values = 1;
		L.bits = 8;
		L.signal_found = true;
	}

	// BLSIG sentence type 2
	else if((sscanf(line, "$BLSIG,%6llx,%3x,%3x*%x", &L.counter, &A, &B, &checksum) == 4) && (C.checksum_ok = (checksum == compute_checksum(line))))
	{
		sprintf(L.data, "%03x%03x", A, B);
		L.channels = 2;
		L.values = 1;
		L.bits = 12;
		L.signal_found = true;
	}

	// BLSIG sentence type 3
	else if((sscanf(line, "$BLSEQ,%6llx,%256s*%x", &L.counter, L.data, &checksum) == 3) && (C.checksum_ok = (checksum == compute_checksum(line))))
	{
		L.channels = 2;
		L.values = 64;
		L.bits = 8;
		L.signal_found = true;
	}

	// BD sentence type 1
	else if((sscanf(line, "$BD,%6llx,%256s*%x", &L.counter, L.data, &checksum) == 3) && (C.checksum_ok = (checksum == compute_checksum(line))))
	{
		L.channels = 2;
		L.values = 64;
		L.bits = 8;
		L.signal_found = true;
	}

	// BM sentence type 1
	else if((sscanf(line, "$BM,%6llx,%256s*%x", &L.counter, L.data, &checksum) == 3) && (C.checksum_ok = (checksum == compute_checksum(line))))
	{
		L.channels = 1;
		L.values = 128;
		L.bits = 8;
		L.signal_found = true;
	}

	// L sentence type 1
	else if((sscanf(line, "$L,%6llx,%1024s*%x", &L.counter, L.data, &checksum) == 3) && (C.checksum_ok = (checksum == compute_checksum(line))))
	{
		L.channels = 2;
		L.values = 256;
		L.bits = 8;
		L.signal_found = true;
	}

	// unknown sentence
	else
	{
		print_log(LOG_GENERAL, "unknown sentence: %s", line);
		unknown_sentence = true;
	}


	//Reset Watchdog timer
	if(!unknown_sentence)
		L.receive_time = time(NULL);

	
	//
	// send output of board to servers
	//
	if (P.send_board_output)
	{
		send_board_output_remote(line, unknown_sentence, L.signal_found);
	}


	//
	// send signal if found
	//
	if(L.signal_found)
	{
		//check signal quality
		evaluate_signal();

		//update longtime vars (after evaluate_signal!!)
		update_longtime(false);


		//
		// send only if precision is ok
		//
		if(L.signal_ok)
		{
			C.signals_per_sec++;
			C.signals_per_sec_count++;
			if((C.accuracy_ok) && (!C.startup_phase) && (C.seconds_flow_ok) && (C.time_ok) && (!(C.faulty)) && (!(C.faulty_rate)))
			{
				L.nsec = counter_difference_to_nsec(L.counter - S.counter);
				send_strike();
				C.last_transmission_time = ensec_time();
				LT.signals_sent++;
				LastSentSignal = LastGoodSignal;
			}
		}
	}
}





/******************************************************************************/
/***** Board communication thread *********************************************/
/******************************************************************************/

void *main_thread_exit()
{
	if(!flag.auto_device)
	{
		exit(EXIT_FAILURE);
	}

	return NULL;
}



//
// Serial read/write with board
//
void *main_thread(void *dummy)
{
	int i = 0;
	unsigned char c;
	char buf[STRING_BUFFER_SIZE];
	int fails = 0;
	bool exit_on_failure = true;


	//
	//loops if serial port vanishes (USB disconnect)
	// --> try reconnect
	//
	while(true)
	{
		//
		if(!flag.watchdog_enabled)
		{
			if(!init_gps(serial) && exit_on_failure)
				return main_thread_exit();
		}

		//
		if(serial.echo_device != NULL)
		{
			strcpy(P.message, "Opening COM port ");
			strcat(P.message, serial.echo_device);
			strcat(P.message, " to echo device...");
			serial.e = serial_open(serial.echo_device);

			if(serial.e < 0 && exit_on_failure)
				return main_thread_exit();

			if(serial.e < 0)
			{
				if(exit_on_failure)
					return main_thread_exit();

				print_log(LOG_GENERAL, "Error opening %s\n", serial.echo_device);
			}
			else
			{
				print_log(LOG_GENERAL, "Opened %s\n", serial.echo_device);
			}

			serial_set_baudrate(serial.e, serial.tracker_baudrate);
		}


		//
		strcpy(P.message, "Opening COM port ");
		strcat(P.message, serial.device);
		strcat(P.message, " to board...");
		serial.f = serial_open(serial.device);

		if(serial.f < 0)
		{
			if(exit_on_failure)
				return main_thread_exit();

			print_log(LOG_GENERAL, "Error opening %s\n", serial.device);
			sleep(10);

			continue;
		}
		else
		{
			print_log(LOG_GENERAL, "Opened %s\n", serial.device);
		}

		//
		// If device was opened successfully the first time, never exit any more!
		//
		exit_on_failure = false;

		if(!flag.send_to_blitzortung)
		{
			serial_set_baudrate(serial.f, serial.gps_baudrate);

			while(true)
			{
				if(serial_read_buf(&c))
				{
					if(serial.echo_device != NULL)
					{
						serial_write(serial.e, &c, 1);
					}
					if(flag.verbose_out)
					{
						putchar(c);
					}
					if(logfiles.out.name != NULL)
					{
						fwrite(&c, 1, 1, logfiles.out.fd);
						fflush(logfiles.out.fd);
					}

					fails = 0;
				}
				else
				{
					if(fails++ > 1000)
						break;
					usleep(1000);
				}
			}

		}
		else
		{
			bool delete_msg = true;

			serial_set_baudrate(serial.f, serial.tracker_baudrate);

			strcpy(P.message, "Wait to get first data from board at ");
			strcat(P.message, serial.device);
			strcat(P.message, " ...");

			//the time check is for deleting some old data in the serial buffers
			long long start = ensec_time();
			do
			{
				if(serial_read(serial.f, &c, 1) != 1)
					break;
				usleep(100);
			}
			while(c != '\n' || ensec_time() - start < 500000000ll);

			
			//Main Loop
			while(true)
			{
				if(serial_read_buf(&c))
				{
					if(((c < ' ') || (c > '~')) && (c != '\n'))
					{
						char hex[INFO_BUFFER_SIZE];
						sprintf(hex, "(0x%02X)", c);
						if(serial.echo_device != NULL)
						{
							serial_write(serial.e, hex, 6);
						}
						if(flag.verbose_out)
						{
							fwrite(hex, 6, 1, stdout);
#if __CYGWIN__
							if(c == '\n') //cygwin is slow
#endif
								fflush(stdout);
						}
						if(logfiles.out.name != NULL)
						{
							fwrite(hex, 6, 1, logfiles.out.fd);
							if(c == '\n')
								fflush(logfiles.out.fd);
						}
					}
					else
					{
						if(serial.echo_device != NULL)
						{
							serial_write(serial.e, &c, 1);
						}
						if(flag.verbose_out)
						{
							fwrite(&c, 1, 1, stdout);
#if __CYGWIN__
							if(c == '\n') //cygwin is slow
#endif
								fflush(stdout);
						}
						if(logfiles.out.name != NULL)
						{
							fwrite(&c, 1, 1, logfiles.out.fd);
							if(c == '\n')
								fflush(logfiles.out.fd);
						}
						buf [i] = c;
						if(i < STRING_BUFFER_SIZE - 2)
						{
							i++;
						}
						if(c == '\n')
						{
							buf [i] = 0;
							evaluate(buf);
							i = 0;

							if(delete_msg)
							{
								strcpy(P.message, "");
								delete_msg = false;
							}
						}
					}

					fails = 0;
				}
				else
				{
					if(fails++ > 1000)
						break;
					//usleep(1000);
				}

			}

		}

		serial_close(serial.e);
		serial_close(serial.f);

		//
		strcpy(P.message, "Ooops. Error with serial device. Trying to reconnect...");
		print_log(LOG_GENERAL, "Ooops. Error with serial device %s. Trying to reconnect...\n", serial.device);
		fprintf(stderr, "Ooops. Error with serial device. Trying to reconnect...\n");
		sleep(5);

	}


	return main_thread_exit();
}








#ifndef SIMPLE_TRACKER


/******************************************************************************/
/***** HTTP-GET Request********************************************************/
/******************************************************************************/

//
// Performs a simple GET-Request to a server
//
int http_get_request(char *buf, char *server, char *get)
{
	struct sockaddr_in serv_addr;
	struct hostent *host_info;
	int sock;
	int count, i;
	char buf2[HTTP_REQUEST_BUFFER];
	char *htmlcontent;

	buf[0] = '\0';

	sock = socket(PF_INET, SOCK_STREAM, 0);
	if(sock < 0)
	{
		perror("GET Request: failed to create socket");
		return -1;
	}


	set_socket_timeout(sock);

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(80);
	serv_addr.sin_addr.s_addr = inet_addr(server);

	if(serv_addr.sin_addr.s_addr == INADDR_NONE)
	{
		host_info = gethostbyname(server);
		if(host_info == NULL)
		{
			fprintf(stderr, "GET Request: unknown server: %s\n", server);
			return -1;
		}

		memcpy((char *)&serv_addr.sin_addr, host_info->h_addr, host_info->h_length);
	}

	//Connect
	if(connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
	{
		print_error("GET Request - can't connect to server %s", server);
		return -1;
	}

	// Send GET Request
	snprintf(buf2, HTTP_REQUEST_BUFFER - 100,
	         "GET %s HTTP/1.0\r\n"
	         "Host: %s\r\n"
	         "User-Agent: %s\r\n\r\n",
	         get, server, S.tracker_version_long);

	if(send(sock, buf2, strlen(buf2), MSG_NOSIGNAL) == -1)
	{
		print_error("GET Request - Couldn't send GET request (%s): %s", server, buf2);
		return -1;
	}


	//Save response in buffer
	memset(buf2, '\0', sizeof(buf2));
	i = 0;
	do
	{
		count = recv(sock, &buf2[i], sizeof(buf2) - i - 1, 0);

		if(count == -1)
		{
			print_error("GET Request: Error in getting server response %s %s", server, get);
			return -1;
		}

		i += count;

	}
	while(count > 0 && i < HTTP_REQUEST_BUFFER - 1);

	if(count > 0)
	{
		fprintf(stderr, "GET Request: Response was too long (%s): %s\n", server, get);
		return -1;
	}



	// Check Response Code
	int ver = 0;
	int code = 0;
	char text[31];

	buf2[HTTP_REQUEST_BUFFER - 1] = '\0'; //to be sure, especially strstr later!
	sscanf(buf2, "HTTP/1.%1d %3d %30s", &ver, &code, text);

	if(code != 200)
	{
		fprintf(stderr, "GET Request: Error, Response code %d (%s): %s\n", code, server, get);
		return code;
	}

	//Get Content without headers
	htmlcontent = strstr(buf2, "\r\n\r\n");
	if(htmlcontent == NULL)
	{
		fprintf(stderr, "GET Request: No content (%s): %s\n", server, get);
		return -1;
	}

	strncpy(buf, htmlcontent + 4, HTTP_REQUEST_BUFFER - 500);

	//Close connection
	close(sock);

	return 0;
}




/******************************************************************************/
/***** HTTP-SERVER ************************************************************/
/******************************************************************************/

//
// get sockaddr, IPv4 or IPv6:
//
void *get_in_addr(struct sockaddr *sa)
{
	if(sa->sa_family == AF_INET)
	{
		return &(((struct sockaddr_in *)sa)->sin_addr);
	}

	return &(((struct sockaddr_in6 *)sa)->sin6_addr);
}


//
// Send HTML data to client
//
int http_send(int s, const char *buf)
{
	int total = 0;
	int bytesleft = 0;
	int len = 0;
	int n = 0;

	len = strlen(buf);
	bytesleft = len;

	while(total < len)
	{
		n = send(s, buf + total, bytesleft, MSG_NOSIGNAL);
		if(n == -1)
			break;

		total += n;
		bytesleft -= n;
	}

	return n == -1 ? -1 : 0;
}


//
// The text of the page with parameters, variables...
// NO JS-SCRIPTS IN THERE!!!
//
void http_page_content()
{
	int board_ok = 1;
	int passwd_ok = -1;
	time_t now;
	now = time(NULL);



	if(strlen(P.message) > 0 || C.startup_phase)
	{
		print_http("<div id='status_msg' class='message'>\n");

		if(C.startup_phase && S.board_is_sending)
			print_http("Startup Phase! Please wait at least %d to %d seconds.\n", RING_BUFFER_SIZE, RING_BUFFER_SIZE * 2);

		print_http("%s\n", P.message);
		print_http("</div>\n");
	}

	if(!S.board_is_sending && (!C.startup_phase || now - L.receive_time >= BOARD_TIMEOUT))
	{
		print_http("<div id='no_conn_msg' class='message bad'>\n");

		if(now - L.receive_time >= BOARD_TIMEOUT_RESET)
		{
			print_http("<strong>Board is offline since %ldmin, %ldsec! Check serial connection!</strong>\n", (now - L.receive_time) / 60, now - L.receive_time - ((int)((now - L.receive_time) / 60)) * 60);
			board_ok = -1;
		}
		else
		{
			print_http("<strong>No message from board since %ldsec! Waiting...</strong>\n", now - L.receive_time);
			board_ok = 0;
		}

		print_http("</div>\n");
	}


	if(!C.startup_phase && flag.send_to_blitzortung && !Server[0].login_ok)
	{
		print_http("<div id='password_msg' class='message bad'>\n");
		print_http("Could not validate your username and password for server <strong>%s</strong>, please check! Use only letters and numbers for your password.\n", Server[0].addr);
		print_http("</div>\n");
		passwd_ok = 0;
	}
	else if(Server[0].login_ok)
	{
		passwd_ok = 1;
	}

	print_http("<h2>Current Status <div class='wait_img wait%d'></div></h2>\n", (int)now % 8);

	print_http("<table id='table_status'>\n");

	print_http("<tr>");
	print_http("<td class='%s' colspan='4'>BOARD</td>\n",  board_ok == 1  ? "good" : (board_ok == 0      ? "notgood" : "bad"));
	print_http("<td class='%s' colspan='4'>LOGIN</td>\n",  passwd_ok == 1 ? "good" : (passwd_ok == -1    ? "notdef"  : "bad"));
	print_http("<td class='%s' colspan='4'>GPS</td>\n",    C.pos_ok ?     "good" : (S.status == 'A'  ? "notgood" : "bad"));
	print_http("</tr>");

	print_http("<tr>");
	print_http("<td class='%s' colspan='3'>ACCURACY</td>\n",  C.accuracy_ok ?      "good" : (last_C.accuracy_ok      ? "notgood" : "bad"));
	print_http("<td class='%s' colspan='3'>FLOW</td>\n",      C.seconds_flow_ok ?  "good" : (last_C.seconds_flow_ok  ? "notgood" : "bad"));
	print_http("<td class='%s' colspan='3'>CHECKSUM</td>\n",  C.checksum_ok ?      "good" : (last_C.checksum_ok      ? "notgood" : "bad"));
	print_http("<td class='%s' colspan='3'>TIME</td>\n",      C.time_ok ?          "good" : (last_C.time_ok          ? "notgood" : "bad"));
	print_http("</tr>");

	print_http("<tr>");
	print_http("<td class='%s' colspan='6'>INTERFERENCE 1</td>\n", !C.faulty ?          "good" : (!last_C.faulty           ? "notgood" : "bad"));
	print_http("<td class='%s' colspan='6'>INTERFERENCE 2</td>\n", !C.faulty_rate ?     "good" : (!last_C.faulty_rate      ? "notgood" : "bad"));
	print_http("</tr>");

	print_http("</table>\n");

	//General status
	/*
	print_http("<ul>\n");
	print_http("<li>Board is: %s</li>\n", S.board_is_sending ? "<em class='good'>ONLINE</em>" : "<em class='bad'>OFFLINE</em>");
	print_http("<li>Accuracy: %s</li>\n", C.accuracy_ok ? "<em class='good'>GOOD</em>" : "<em class='bad'>NOT GOOD</em>");
	print_http("<li>Position: %s</li>\n", C.pos_ok ? "<em class='good'>GOOD</em>" : "<em class='bad'>NOT GOOD</em>");
	print_http("<li>Seconds flow: %s</li>\n", C.seconds_flow_ok ? "<em class='good'>GOOD</em>" : "<em class='bad'>NOT GOOD</em>");
	print_http("<li>Time: %s</li>\n", C.time_ok ? "<em class='good'>GOOD</em>" : "<em class='bad'>NOT GOOD</em>");
	print_http("<li>Checksum: %s</li>\n", C.checksum_ok ? "<em class='good'>GOOD</em>" : "<em class='bad'>NOT GOOD</em>");
	print_http("<li>Interference mode: %s</li>\n", C.faulty || C.faulty_rate ? "<em class='bad'>ON</em>" : "<em class='good'>OFF</em>");
	print_http("</ul>\n\n");
	*/

	//Signal Info
	print_http("<h2>Signal info</h2>\n\n");
	print_http("<ul id='signal_info'>\n");
	print_http("<li>Measure interval: <em>%ds</em></li>\n", P.signal_rate_interval);
	print_http("<li>Signal rate: <em>%.1f signals per sec</em></li>\n", C.mean_signal_rate_last);
	print_http("<li>Maximum: <em>%d signals in one sec</em></li>\n", C.signals_per_sec_max_last);
	print_http("<li>Non zero seconds: <em>%d</em></li>\n", C.nonzero_sec);
	print_http("</ul>\n\n");



	//GPS info
	print_http("<h2>Current GPS Information</h2>\n\n");
	print_http("<ul>\n");
	print_http("<li>Status: <em>%c</em></li>\n", S.status);
	print_http("<li>Time: <em>%04d-%02d-%02d %02d:%02d:%02d UTC</em></li>\n", S.time.year, S.time.mon, S.time.day, S.time.hour, S.time.min, S.time.sec);
	print_http("<li>Lat: <em>%.6Lf° (%+.6Lf°)</em></li>", S.lat, S.lat - S.average_lat);
	print_http("<li>Lon: <em>%.6Lf° (%+.6Lf°)</em></li>", S.lon, S.lon - S.average_lon);
	print_http("<li>Height: <em>%.1Lfm (%+.1Lfm)</em></li>\n", S.alt, S.alt - S.average_alt);
	print_http("<li>Satellites: <em>%d</em></li>\n", S.sat);
	print_http("<li>Accuracy: <em>%+.0Lf nsec</em></li>\n", precision.PPS * 1000000000.0l);
	print_http("</ul>\n\n");


	//GPS average
	print_http("<h2>Smoothed GPS Values</h2>\n\n");
	print_http("<ul>\n");
	print_http("<li>Lat: <em>%.6Lf°</em></li><li>Lon: <em>%.6Lf°</em></li><li>Height: <em>%.1Lfm</em></li>\n", S.average_lat, S.average_lon, S.average_alt);
	print_http("<li>Satellites: <em>%.1f</em></li>\n", S.average_sat);
	print_http("<li>Accuracy: <em>%.1Lf nsec</em></li>\n", S.average_pps * 1000000000.0l);
	print_http("</ul>\n\n");




	if(!flag.send_to_blitzortung)
		print_http("<div class='bad'>Tracker runs in \"listen only mode\" and does not send data to Blitzortung.org! Please specifiy username, password and region on command-line!</div>");

	//Longtime
	struct tm *t;
	int sec_running;
	sec_running = (int)(now - LT.start_time);


	print_http("<h2>Longtime");
	print_http(" (<a href='/reset.html' class='reset' onclick='return confirm(\"Do you want to reset all longtime variables to zero?\");'>Reset</a>)");
	print_http("</h2>\n\n");
	print_http("<ul>\n");

	t = gmtime(&LT.start_time);
	print_http("<li>Tracker started: <em>%04d-%02d-%02d %02d:%02d:%02d UTC</em></li>\n", t->tm_year + 1900, t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);

	if(LT.reset_time > 0)
	{
		t = gmtime(&LT.reset_time);
		print_http("<li>Last longtime reset: <em>%04d-%02d-%02d %02d:%02d:%02d UTC</em></li>\n", t->tm_year + 1900, t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);
	}

	print_http("<li>Running for: <em>");
	print_http("%dd ", (int)sec_running / (3600 * 24));
	sec_running = sec_running - ((int)(sec_running / (3600 * 24))) * 3600 * 24;
	print_http("%dh ", ((int)sec_running / 3600));
	sec_running = sec_running - ((int)sec_running / 3600) * 3600;
	print_http("%dmin ", ((int)sec_running / 60));
	sec_running = sec_running - ((int)sec_running / 60) * 60;
	print_http("%ds", sec_running);
	print_http("</em></li>\n");

	if(LT.board_last_timeout)
	{
		t = gmtime(&LT.board_last_timeout);
		print_http("<li>Last board timeout: <em>%04d-%02d-%02d %02d:%02d:%02d (for %ds)</em></li>\n", t->tm_year + 1900, t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec, LT.board_last_timeout_duration);
	}

	if(flag.send_to_blitzortung)
		print_http("<li>Sent to Blitzortung: <em>%lldkB, %lld packets</em></li>\n", LT.bytes_sent / 1024, LT.packets_sent);

	print_http("<li>No. of signals: <em>%d, sent %d</em></li>\n", LT.signals, LT.signals_sent);

	if(LT.signals_filtered_min || LT.signals_filtered_max || P.filter_enable)
	{
		print_http("<li>Signals filtered: <em>weak: %d / strong: %d</em></li>\n", LT.signals_filtered_min, LT.signals_filtered_max);
	}

	print_http("<li>Satellites: <em>%d min, %d max</em></li>\n", LT.min_sat > 100 ? LT.max_sat : LT.min_sat, LT.max_sat);

	if(LT.count_gps_nok)
		print_http("<li>GPS: <em>%d fails for %d sec</em></li>\n", LT.count_gps_nok, LT.sec_gps_nok);

	if(LT.count_accuracy_nok > 0)
		print_http("<li>Accuracy: <em>%d fails for %d sec</em></li>\n", LT.count_accuracy_nok, LT.sec_accuracy_nok);

	if(LT.count_time_nok > 0)
		print_http("<li>Time: <em>%d fails for %d sec</em></li>\n", LT.count_time_nok, LT.sec_time_nok);

	if(LT.count_pos_nok > 0)
		print_http("<li>Position: <em>%d fails for %d sec</em></li>\n", LT.count_pos_nok, LT.sec_pos_nok);

	if(LT.count_seconds_flow_nok > 0)
		print_http("<li>Seconds flow: <em>%d fails for %d sec</em></li>\n", LT.count_seconds_flow_nok, LT.sec_seconds_flow_nok);

	if(LT.count_checksum_nok > 0)
		print_http("<li>Checksum: <em>%d fails for %d sec</em></li>\n", LT.count_checksum_nok, LT.sec_checksum_nok);

	print_http("</ul>\n");



	//Interference-Modes
	print_http("<h2>Interference Modes</h2>\n\n");
	print_http("<ul>");

	if(P.max_nonzero_sec < 0)
	{
		print_http("<li>Enable I-Mode-1 at: <em>Disabled!</em></li>\n");
	}
	else
	{
		print_http("<li>Enable I-Mode-1 at: <em>%d sec of cont. signals</em></li>\n", P.max_nonzero_sec);
		print_http("<li>Longtime stat.: <em>%dx, %d Signals, %ds</em></li>\n", LT.count_faulty, LT.signals_faulty, LT.sec_faulty);
		if(FirstIMode1Signal.time.year)
		{
			print_http("<li>Last enabled: <em>%04d-%02d-%02d %02d:%02d:%02d UTC\n",
			           FirstIMode1Signal.time.year, FirstIMode1Signal.time.mon, FirstIMode1Signal.time.day,
			           FirstIMode1Signal.time.hour, FirstIMode1Signal.time.min, FirstIMode1Signal.time.sec);
			print_http("<br>for %d sec</em></li>", (int)(time_diff_nsec(&FirstIMode1Signal.time, &LastIMode1Signal.time) / 1e9));
		}
	}
	print_http("</ul>\n");

	print_http("<ul>");
	if(P.max_signal_rate < 0)
	{
		print_http("<li>Enable I-Mode-2 at: <em>Disabled!</em></li>\n");
	}
	else
	{
		print_http("<li>Enable I-Mode-2 at: <em>%.1f signals per second</em></li>\n", P.max_signal_rate);
		print_http("<li>Longtime stat.: <em>%dx, %d Signals, %ds</em></li>\n", LT.count_faulty_rate, LT.signals_faulty_rate, LT.sec_faulty_rate);
		if(FirstIMode2Signal.time.year)
		{
			print_http("<li>Last enabled: <em>%04d-%02d-%02d %02d:%02d:%02d UTC\n",
			           FirstIMode2Signal.time.year, FirstIMode2Signal.time.mon, FirstIMode2Signal.time.day,
			           FirstIMode2Signal.time.hour, FirstIMode2Signal.time.min, FirstIMode2Signal.time.sec);
			print_http("<br>for %d sec</em></li>", (int)(time_diff_nsec(&FirstIMode2Signal.time, &LastIMode2Signal.time) / 1e9));
		}
	}
	print_http("</ul>\n\n");






	//General Settings and Parameters
	print_http("<h2>General Settings</h2>\n\n");
	print_http("<ul>");
	print_http("<li>Smooth time: <em>%dmin</em></li>\n", P.smooth_factor / 60);
	print_http("<li>GPS precision: <em>%.4Lf° / Height: %.0Lfm </em></li>\n", P.pos_precision, P.alt_precision);
	print_http("<li>1PPS precision: <em>%.2Lf°s</em></li>\n", P.pps_precision * 1E6);
	print_http("<li>GPS timeout: <em>%ds</em></li>\n", RING_BUFFER_SIZE);
	print_http("</ul>\n\n");





	//Filters
	if(P.filter_enable > 0)
	{
		print_http("<h2>Filter Settings</h2>\n\n");
		print_http("<ul>");
		print_http("<li>Current status: <em>%s</em></li>\n", C.filter_enabled ? "Active" : "Inactive");
		print_http("<li>Min. Amplitude: <em>%.2fV%s</em></li>\n", P.min_amplitude * (2.5 / 128.), P.min_amplitude == 0 ? " (disabled) " : "");
		print_http("<li>Max. Amplitude: <em>%.2fV%s</em></li>\n", P.max_amplitude * (2.5 / 128.), P.max_amplitude == 0 ? " (disabled) " : "");

		if(P.max_amplitude != 0)
			print_http("<li>Max. Ampl. Count: <em>%d</em></li>\n", P.max_amplitude_count);

		if(P.max_amplitude != 0 || P.min_amplitude != 0)
			print_http("<li>Check Bytes: <em>%d</em></li>\n", P.signal_check_bytes);

		print_http("</ul>\n\n");

	}

	//Auto parameter updates
	print_http("<h2>Parameter Updates</h2>\n\n");
	print_http("<ul>");

	if(flag.requests_enabled)
	{
		print_http("<li>URL: <em>")
		print_http("<a href='http://%s%s?username=%s&amp;region=%d&amp;password=' target='_blank'>", PARAMETER_REQ_SERVER, PARAMETER_REQ_URI, Server[0].username, flag.region);
		print_http("%s%s</a></em></li>\n", PARAMETER_REQ_SERVER, PARAMETER_REQ_URI);

		if(P.last_update <= 0)
		{
			print_http("<li>Last update: <em>When board sends data...</em></li>\n");
			print_http("<li>Next update in: <em>?</em></li>\n");
		}
		else
		{
			t = gmtime(&P.last_update);
			print_http("<li>Last update: <em>%04d-%02d-%02d %02d:%02d:%02d</em></li>\n", t->tm_year + 1900, t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);

			if(P.last_update != P.last_update_success)
			{
				t = gmtime(&P.last_update_success);
				print_http("<li>Last success: <em class='bad'>%04d-%02d-%02d %02d:%02d:%02d</em></li>\n", t->tm_year + 1900, t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);
			}

			int next_update = (int)(P.param_update_sec - now + P.last_update);
			print_http("<li>Next update in: <em>%ds</em></li>\n", next_update);

		}

		print_http("<li>Update requests: <em>%d</em></li>\n", LT.parameter_updates);
		print_http("<li>Updated parameters: <em>%d</em></li>\n", LT.parameter_updates_var);

	}
	else
	{
		print_http("<li class='bad'>Disabled!</li>\n");
	}

	print_http("</ul>\n\n");


	//Servers
	if(flag.send_to_blitzortung)
	{
		print_http("<h2>Servers</h2>\n\n");
		print_http("<ul>");
		print_http("<li>Region: <em>%d (", flag.region);
		switch(flag.region)
		{
		case 1:
			print_http("Europe");
			break;
		case 2:
			print_http("Oceania");
			break;
		case 3:
			print_http("North America");
			break;
		case 4:
			print_http("Japan");
			break;
		default:
			print_http("Unknown");
			break;
		}
		print_http(")</em></li>\n");
		print_http("<li>Host: <em>%s:%d</em></li>\n", Server[0].addr, Server[0].port);
		print_http("<li>User: <em>%s</em></li>\n", Server[0].username);

		if(strlen(Server[1].addr) || strlen(Server[2].addr))
			print_http("<li>Traffic: <em>%lldkB</em></li>\n", Server[0].bytes_sent / 1024);

		if(strlen(Server[1].addr))
		{
			print_http("<li>Add. Server Host: <em>%s:%d</em></li>\n", Server[1].addr, Server[1].port);

			if(strlen(Server[1].username) > 0 && strcmp(Server[1].username, Server[0].username) != 0)
			{
				print_http("<li>Add. Server User: <em>%s</em></li>\n", Server[1].username);
			}

			print_http("<li>Add. Server Traffic: <em>%lldkB</em></li>\n", Server[1].bytes_sent / 1024);
		}

		if(strlen(Server[2].addr))
		{
			print_http("<li>Extra Server Host: <em>%s:%d</em></li>\n", Server[2].addr, Server[2].port);

			if(strlen(Server[2].username) > 0 && strcmp(Server[2].username, Server[0].username) != 0)
			{
				print_http("<li>Extra Server User: <em>%s</em></li>\n", Server[2].username);
			}

			print_http("<li>Extra Server Traffic: <em>%lldkB</em></li>\n", Server[2].bytes_sent / 1024);
		}

	}

	print_http("</ul>\n\n");



	//Technical stuff
	print_http("<h2>Technical Stuff</h2>\n\n");
	print_http("<ul>");
	print_http("<li>Tracker Version: <em>%s</em></li>\n", S.tracker_version);
	print_http("<li>Firmware Version: <em>%s</em></li>\n", S.firmware_version);
	print_http("<li>Board Baudrate: <em>%d</em></li>\n", serial.tracker_baudrate);
	print_http("<li>GPS Baudrate: <em>%d</em></li>\n", serial.gps_baudrate);
	print_http("<li>Channels: <em>%d</em></li>\n", LastSignal->channels);
	print_http("<li>Values: <em>%d</em></li>\n", LastSignal->values);
	print_http("<li>Bits Per Value: <em>%d</em></li>\n", LastSignal->bits);
	print_http("<li>Counter (smoothed): <em>%.1Lf Hz</em></li>\n", average.S_counter_difference);
	print_http("<li>Serial Device: <em>%s</em></li>\n", serial.device);
	if(serial.echo_device != NULL)
		print_http("<li>Serial Echo Device: <em>%s</em></li>\n", serial.echo_device);
	
	if(flag.send_to_blitzortung)
	{
		print_http("<li>Statistics: <em><a href='http://www.myblitzortung.org/blitzortung/statistics.php?user=%s&amp;region=%d' target='_blank'>See MyBlitzortung.org</a></em></li>\n", Server[0].username, flag.region);
	}
	
	print_http("</ul>\n\n");


}


//
// Output Javascript data for graphs
//
void http_jsdata(struct Signal_type *Signal, unsigned int last_nsec, int count_signals)
{
	int i = 0, pos_begin = 0;
	unsigned int signal_nsec = (unsigned int)Signal->time.nsec;
	float power;
	struct Signal_type *s;

	//
	print_http("document.title='Blitzortung Tracker - %.1f/s - %02d:%02d:%02d';\n", last_C.mean_signal_rate, S.time.hour, S.time.min, S.time.sec);
	print_http("last_nsec=%d;\n", signal_nsec);
	print_http("signal_data = [ ];\n");


	//
	// Update only if nsec differ from old value and if signal data exists
	//
	if(signal_nsec > 0 && (last_nsec == 0 || last_nsec != signal_nsec))
	{
		//Search for last known signal
		for(i = 1; i < count_signals; i++)
		{
			if(Signal[i].time.nsec == 0)
				break;

			if((unsigned int)Signal[i].time.nsec == last_nsec)
			{
				pos_begin = i - 1;
				break;
			}
		}

		// output signals in their order
		for(i = pos_begin; i >= 0; i--)
		{
			s = &Signal[i];
			power = 1;

			print_http("signal_data[%d] = [ ];\n", i);
			print_http("signal_data[%d]['data'] = [ ];\n", i);


			if(s->bits == 8)
			{
				int c, x, step;
				power = 0;
				step = (s->channels == 2) ? 2 : 1;

				for(c = 0; c < s->channels; c++)
				{
					for(x = c; x < s->values * s->channels; x += step)
					{
						if(x >= STRING_BUFFER_SIZE)
							break;

						//calc. rough power (not exact) for sound
						power += fabsl((s->data[x] - 128.) / 128.) / (float)(s->values * s->channels);

						if(x == c)
							sprintf(http_string_buf, "%.3f", ((float)s->data[x] - 128.) / 128.*2.5);
						else
							sprintf(http_string_buf, "%s,%.3f", http_string_buf, ((float)s->data[x] - 128.) / 128.*2.5);
					}

					//best channel mode?
					if(s->channels == 1 && s->data[s->values - 1] % 2 == 0)
						c = 1;

					print_http("signal_data[%d]['data'][%d] = [%s];\n", i, c, http_string_buf);
				}

				power = sqrt(power * 0.99 + 0.001);
			}

			print_http("signal_data[%d]['vals']=%d;\n", i, s->values);
			print_http("signal_data[%d]['power']=%.2f;\n", i, power);
			print_http("signal_data[%d]['time']='Time: %04d-%02d-%02d %02d:%02d:%02d.%09lld UTC'\n", i,
			           s->time.year, s->time.mon, s->time.day,
			           s->time.hour, s->time.min, s->time.sec,
			           s->time.nsec);

			print_http("window.setTimeout('display_signal(signal_data[%d]);', %.0f);\n", i, 1.3 * (double)((pos_begin - i)*HTTP_REFRESH_DATA_NEW));
		}

		print_http("data_refresh_wait=%d;\n", HTTP_REFRESH_DATA_NEW * (pos_begin + 1));
	}
	else
	{
		print_http("data_refresh_wait=%d;\n", HTTP_REFRESH_DATA);

		if(signal_nsec == 0)
		{
			print_http("signal_data[0]=[ ];\n");
			print_http("signal_data[0]['vals']=0;\n");
			print_http("signal_data[0]['power']=0;\n");
			print_http("signal_data[0]['time']='No signal!'\n");
			print_http("signal_data[0]['data']=[ ];\n");
			print_http("display_signal(signal_data[0]);\n");
		}

	}
}


//
// CSS for mobile devices
//
void http_css_mobile()
{
	print_http("#header { width: 320px; margin: 0; padding: 5px; height: 52px; }\n");
	print_http("#tracker_content { margin-top: 210px; border: 0; width: 315px; }\n");
	print_http("#graph_container { top: 75px; left: 5px; font-size: 9px; }\n");
	print_http("#Graph { height: 155px; width: 315px; }\n");
	print_http(".graph_text { position: absolute; top: 5px; left: 5px; font-size: 9px; }\n");
}


//
// The main HTML-page with body, js-functions, css
//
void http_page_main(int reset)
{
	//HTML HEAD
	print_http("<!DOCTYPE html>\n");
	print_http("<html>\n<head>\n");
	print_http("<meta http-equiv=\"X-UA-Compatible\" content=\"IE=8\" />\n");
	print_http("<title>Blitzortung Tracker</title>\n");

	if(reset > 0)
		print_http("<meta http-equiv='refresh' content='%d; URL=/'>\n", reset);


	//Main CSS
	print_http("<style type='text/css'>\n");
	print_http("body, ul, li, table, select, input { font-family: arial, sans-serif; font-size: 12px; }\n");
	print_http("html { margin: 0px; padding: 0px; }\n");
	print_http("body { position: relative; margin: 0px; padding: 0px; }\n");
	print_http("h1,h2,h3 { color: #111; }\n");
	print_http("h1 { margin: 0px 0px 5px 0px; font-size: 19px;}\n");
	print_http("#header { position: relative; background: #f0f0ff; width: 310px; padding: 5px; margin: 5px; height: 52px; }\n");
	print_http("#set_autorefresh { }\n");
	print_http("h2 { margin: 10px 0px 5px 0px; font-size: 14px; width: 310px; border-bottom: 1px solid #ddf; padding-bottom: 2px; }\n");
	print_http("ul { margin: 3px 0px 3px 20px; padding: 0; list-style-image: url('data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAAoAAAAKCAYAAACNMs+9AAAACXBIWXMAAAsSAAALEgHS3X78AAAAGXRFWHRDb21tZW50AENyZWF0ZWQgd2l0aCBHSU1QV4EOFwAAASVJREFUGNMtzj8vQ1Ech/Hnd+5VNRnEgLcgTQcJiUEMlYjZVEtJiMXshTQECTFIvAKDiTRd7MTQQYKk/g5IW/eec76Ga/8kz4MkKUSFn29luzvKV5eVta/lFRT6PYV+T4pBDoAYYGQEV1+HUgk73odOB8plwECGA4gSZBk2XcEa21j3BR000ecHDJdBKqAzQxIMBthiDRqbuPtbdLSHfr4gcQXEXJFIU0AoScE5eHqAj3cAUgAUIXGo20VnJ3DTRvML2FoDm5gC70ExSJLy+zvlG3X52pzyw6a8zxUUFbJfyQeZJAHEt1di6wq7vEC/PVhagbFxGB0lnZ2ngD4QFLChBD0+o/NTrN0q/hpbJOtb/49mWO5xUcSpSUJ1BpckWJZDpQrAH65rqN1PdyXPAAAAAElFTkSuQmCC');}\n");
	print_http("li { margin: 0; padding: 0; clear: both; }\n");
	print_http(".bo_link { display: block; width: 100%%; height: 100%%; }\n");
	print_http(".bad { color: red; }\n");
	print_http(".notgood { color: #fa0; }\n");
	print_http(".good { color: green; }\n");
	print_http(".notdef { color: #999; }\n");
	print_http("li { position: relative; }\n");
	print_http("li em { float: right; width: 170px; font-weight: bold; font-style:normal; color: #666; }\n");
	print_http("#tracker_content { width: 320px; border-right: 1px solid #f0f0ff; padding: 3px 0 0 5px; }\n");
	print_http("#graph_container { position: absolute; position: absolute; top: 10px; left: 335px; background: #f7f7ff; }\n");
	print_http("#Graph { border:1px solid #888; display: block; }\n");
	print_http(".graph_text { position: absolute; top: 5px; left: 5px; font-size: 12px; }\n");
	print_http("#top_image {width: 100px; height: 50px; border: 1px solid black; position: absolute; left: 212px; top: 5px;}\n");
	print_http("#footer { color: #999; font-size: 10px; margin-top: 20px; padding: 10px; border-top: 1px solid #eee; width: 310px; }\n");
	print_http("#signal_select { margin: 4px; }\n");
	print_http("#signal_select, #signal_beep_check { display: inline-block; }\n");
	print_http(".message { margin: 7px 5px 7px 0px; }\n");
	print_http("#status_msg { color: #fa0; font-weight: bold; }\n");
	print_http("#table_status { margin: 5px 0 5px 0; padding: 0; width: 310px; border-spacing: 4px; text-align: center; }\n");
	print_http("#table_status tr {  }\n");
	print_http("#table_status td { font-weight: bold; padding: 3px 5px; border: 0px solid #eee; white-space: nowrap; width: 25%%;}\n");
	print_http("#table_status .good { background: #f0fff0; border: 1px solid #d0f5d0; }\n");
	print_http("#table_status .bad { background: #fff0f0; border: 1px solid #ffd0d0; }\n");
	print_http("#table_status .notgood { background: #fff9ea; border: 1px solid #fff0e0; }\n");
	print_http("#table_status .notdef { background: #f3f3f3; border: 1px solid #e6e6e6; }\n");

	//Wait animation
	print_http("div.wait0 { width:16px; height: 16px; background-image: url('data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABAAAAAQCAMAAAAoLQ9TAAABd1BMVEX///8AAADU1NSUlJRgYGBAQEBERERubm6ioqLc3NykpKQkJCQoKCgwMDA2NjY+Pj5qamq6urocHBxycnLs7Ozu7u7CwsKKiopQUFBeXl6+vr7Q0NA6OjoWFhaMjIyqqqpcXFx8fHzi4uKGhoYODg5oaGicnJxmZma2trZGRkYKCgqwsLCWlpYYGBgGBgbo6Oj09PR4eHiEhIT29vaCgoKoqKj6+vr8/PzAwMDKysr4+PjW1taurq7w8PDS0tLk5OTe3t7Ozs7Gxsa8vLzg4ODY2Njy8vLa2tp6enq0tLSysrJMTExSUlJaWlpiYmJCQkI8PDzExMR2dnYyMjLq6uoqKiqOjo5YWFgsLCweHh6goKBOTk4SEhKIiIhkZGQ0NDTIyMjMzMzm5ua4uLh+fn6SkpKenp5WVlaQkJBKSkpISEgiIiKmpqYQEBAMDAysrKwEBASampomJiYUFBQ4ODh0dHQuLi4ICAhwcHAgICBUVFRsbGyAgIDRZ4F8AAAAAXRSTlMAQObYZgAAAAFiS0dEAIgFHUgAAAAJcEhZcwAACxMAAAsTAQCanBgAAACYSURBVBjTY2DACsyt3ERF3cwQAmbGbiJJLi62EJ4ahLIScbUWATEcefWg6kIc7YyBlLKWH0wnp5gLkPRW0YYJiDhbAknumBC42V7JQJJHFy5gJQ9SnB6pARNwiRIEkqncpTABQR1HIMnkHVEI4YuJc4B15weUGAXaujnmKIpLQ2S0FfyzfHzj2OUSYXrdOQwl1cU17LF6HADh4xY6P5DSyQAAAABJRU5ErkJggg==');}\n");
	print_http("div.wait1 { width:16px; height: 16px; background-image: url('data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABAAAAAQCAMAAAAoLQ9TAAABd1BMVEX///8AAADU1NSUlJRgYGBAQEBERERubm6ioqLc3NykpKQkJCQoKCgwMDA2NjY+Pj5qamq6urocHBxycnLs7Ozu7u7CwsKKiopQUFBeXl6+vr7Q0NA6OjoWFhaMjIyqqqpcXFx8fHzi4uKGhoYODg5oaGicnJxmZma2trZGRkYKCgqwsLCWlpYYGBgGBgbo6Oj09PR4eHiEhIT29vaCgoKoqKj6+vr8/PzAwMDKysr4+PjW1taurq7w8PDS0tLk5OTe3t7Ozs7Gxsa8vLzg4ODY2Njy8vLa2tp6enq0tLSysrJMTExSUlJaWlpiYmJCQkI8PDzExMR2dnYyMjLq6uoqKiqOjo5YWFgsLCweHh6goKBOTk4SEhKIiIhkZGQ0NDTIyMjMzMzm5ua4uLh+fn6SkpKenp5WVlaQkJBKSkpISEgiIiKmpqYQEBAMDAysrKwEBASampomJiYUFBQ4ODh0dHQuLi4ICAhwcHAgICBUVFRsbGyAgIDRZ4F8AAAAAXRSTlMAQObYZgAAAAFiS0dEAIgFHUgAAAAJcEhZcwAACxMAAAsTAQCanBgAAACdSURBVBjTY2DABqSVNXWR+RwyKlp6DAxmVuYQPhe3rKq2PgODuYGtGYjPxCYkB5ExExEBqWHmFobptbW3BZIsPIIwAWMHeyDJyisCN97aFUiw8YnC+OaOdkCSnV8MJiDiZA1yBZs4TMDO2QVIcgpIyENN0BZzA9GCkgpq0gbGnBYcNi4QGSlFdXZDEzkdG06YXiUNHSM5U0sRrD4HACMhDnu2z62nAAAAAElFTkSuQmCC');}\n");
	print_http("div.wait2 { width:16px; height: 16px; background-image: url('data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABAAAAAQCAMAAAAoLQ9TAAABd1BMVEX///8AAADU1NSUlJRgYGBAQEBERERubm6ioqLc3NykpKQkJCQoKCgwMDA2NjY+Pj5qamq6urocHBxycnLs7Ozu7u7CwsKKiopQUFBeXl6+vr7Q0NA6OjoWFhaMjIyqqqpcXFx8fHzi4uKGhoYODg5oaGicnJxmZma2trZGRkYKCgqwsLCWlpYYGBgGBgbo6Oj09PR4eHiEhIT29vaCgoKoqKj6+vr8/PzAwMDKysr4+PjW1taurq7w8PDS0tLk5OTe3t7Ozs7Gxsa8vLzg4ODY2Njy8vLa2tp6enq0tLSysrJMTExSUlJaWlpiYmJCQkI8PDzExMR2dnYyMjLq6uoqKiqOjo5YWFgsLCweHh6goKBOTk4SEhKIiIhkZGQ0NDTIyMjMzMzm5ua4uLh+fn6SkpKenp5WVlaQkJBKSkpISEgiIiKmpqYQEBAMDAysrKwEBASampomJiYUFBQ4ODh0dHQuLi4ICAhwcHAgICBUVFRsbGyAgIDRZ4F8AAAAAXRSTlMAQObYZgAAAAFiS0dEAIgFHUgAAAAJcEhZcwAACxMAAAsTAQCanBgAAACWSURBVBjTY2DABqyZwyN4omOlYXwbiVDuSN0YlXg1CN/LP9hDUCTEK04rBsx3ZwmAyhhBKC42Zbhh5iDCwzsQLmBlBiRYfGzhAm5uQELd1w0uICIKJMT9HOE6kkACngIcMIEQB5Bue/EgKQhfn8kFZCiDpZGJJ6eVlb5rgrQoRMZRLUwnSt7LOTEErtfS01TbidMAq88BqrkWSL72Kr4AAAAASUVORK5CYII=');}\n");
	print_http("div.wait3 { width:16px; height: 16px; background-image: url('data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABAAAAAQCAMAAAAoLQ9TAAABd1BMVEX///8AAADU1NSUlJRgYGBAQEBERERubm6ioqLc3NykpKQkJCQoKCgwMDA2NjY+Pj5qamq6urocHBxycnLs7Ozu7u7CwsKKiopQUFBeXl6+vr7Q0NA6OjoWFhaMjIyqqqpcXFx8fHzi4uKGhoYODg5oaGicnJxmZma2trZGRkYKCgqwsLCWlpYYGBgGBgbo6Oj09PR4eHiEhIT29vaCgoKoqKj6+vr8/PzAwMDKysr4+PjW1taurq7w8PDS0tLk5OTe3t7Ozs7Gxsa8vLzg4ODY2Njy8vLa2tp6enq0tLSysrJMTExSUlJaWlpiYmJCQkI8PDzExMR2dnYyMjLq6uoqKiqOjo5YWFgsLCweHh6goKBOTk4SEhKIiIhkZGQ0NDTIyMjMzMzm5ua4uLh+fn6SkpKenp5WVlaQkJBKSkpISEgiIiKmpqYQEBAMDAysrKwEBASampomJiYUFBQ4ODh0dHQuLi4ICAhwcHAgICBUVFRsbGyAgIDRZ4F8AAAAAXRSTlMAQObYZgAAAAFiS0dEAIgFHUgAAAAJcEhZcwAACxMAAAsTAQCanBgAAACWSURBVBjTY2DABji5hDPZ4nSYYHxndm82fj5enqwcCN8iLj0jUFTUOSibhwPEV0rxs4HIZOgGSAOpZIE0mF6B3FggqSNsBxPwyssCkkYeBjCBED1dIClnYozqCFNxd1SBhFRnGNPMCkSKyKdBTbWyhWh2N5V3DDE3NxDRFzWDyDg4azg7OVo76FvB9LpxSlvaucDk0QAALRgWqpsIG+sAAAAASUVORK5CYII=');}\n");
	print_http("div.wait4 { width:16px; height: 16px; background-image: url('data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABAAAAAQCAMAAAAoLQ9TAAABd1BMVEX///8AAADU1NSUlJRgYGBAQEBERERubm6ioqLc3NykpKQkJCQoKCgwMDA2NjY+Pj5qamq6urocHBxycnLs7Ozu7u7CwsKKiopQUFBeXl6+vr7Q0NA6OjoWFhaMjIyqqqpcXFx8fHzi4uKGhoYODg5oaGicnJxmZma2trZGRkYKCgqwsLCWlpYYGBgGBgbo6Oj09PR4eHiEhIT29vaCgoKoqKj6+vr8/PzAwMDKysr4+PjW1taurq7w8PDS0tLk5OTe3t7Ozs7Gxsa8vLzg4ODY2Njy8vLa2tp6enq0tLSysrJMTExSUlJaWlpiYmJCQkI8PDzExMR2dnYyMjLq6uoqKiqOjo5YWFgsLCweHh6goKBOTk4SEhKIiIhkZGQ0NDTIyMjMzMzm5ua4uLh+fn6SkpKenp5WVlaQkJBKSkpISEgiIiKmpqYQEBAMDAysrKwEBASampomJiYUFBQ4ODh0dHQuLi4ICAhwcHAgICBUVFRsbGyAgIDRZ4F8AAAAAXRSTlMAQObYZgAAAAFiS0dEAIgFHUgAAAAJcEhZcwAACxMAAAsTAQCanBgAAACfSURBVBjTY2DABuw15AT8FHPcYXxLcXZ1yfToLD8vCN9R2SNH2s3WKZbf3wbED1EzsoDIpMX7WIM0hGnA9HrwMANJLx1OmIBgdjiQzEmzggmICIUCyfwcuECIbBGQFLNJggl45XoDSXcNO5iAn5YykHRLsFCCChTwOoIo/QRLJQMGBnNzc4ZCqOGuTJxKIbbGZnDPWYkkJYkYmGH1OAMA7VgZCbf5idwAAAAASUVORK5CYII=');}\n");
	print_http("div.wait5 { width:16px; height: 16px; background-image: url('data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABAAAAAQCAMAAAAoLQ9TAAABd1BMVEX///8AAADU1NSUlJRgYGBAQEBERERubm6ioqLc3NykpKQkJCQoKCgwMDA2NjY+Pj5qamq6urocHBxycnLs7Ozu7u7CwsKKiopQUFBeXl6+vr7Q0NA6OjoWFhaMjIyqqqpcXFx8fHzi4uKGhoYODg5oaGicnJxmZma2trZGRkYKCgqwsLCWlpYYGBgGBgbo6Oj09PR4eHiEhIT29vaCgoKoqKj6+vr8/PzAwMDKysr4+PjW1taurq7w8PDS0tLk5OTe3t7Ozs7Gxsa8vLzg4ODY2Njy8vLa2tp6enq0tLSysrJMTExSUlJaWlpiYmJCQkI8PDzExMR2dnYyMjLq6uoqKiqOjo5YWFgsLCweHh6goKBOTk4SEhKIiIhkZGQ0NDTIyMjMzMzm5ua4uLh+fn6SkpKenp5WVlaQkJBKSkpISEgiIiKmpqYQEBAMDAysrKwEBASampomJiYUFBQ4ODh0dHQuLi4ICAhwcHAgICBUVFRsbGyAgIDRZ4F8AAAAAXRSTlMAQObYZgAAAAFiS0dEAIgFHUgAAAAJcEhZcwAACxMAAAsTAQCanBgAAACbSURBVBjTY2DABkQS81Pl1ATtYXxO7cJUcRMPYSMxCN/FhkvMwdiYiUtVQArEd7OwsYbIeCkIOwAph2RpmN7U6BwgaWcRAhNwyioFkolO5jABUX5vkApHhAAfG8gMJgOYgHNZHEiYE25GaREzkDRLsocqkYvUZALRxiEixkBKW72YmwsiY+ZmYMXAoJdXwgH3nTnQIjZlaaw+BwBdpRYMfmWHcAAAAABJRU5ErkJggg==');}\n");
	print_http("div.wait6 { width:16px; height: 16px; background-image: url('data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABAAAAAQCAMAAAAoLQ9TAAABd1BMVEX///8AAADU1NSUlJRgYGBAQEBERERubm6ioqLc3NykpKQkJCQoKCgwMDA2NjY+Pj5qamq6urocHBxycnLs7Ozu7u7CwsKKiopQUFBeXl6+vr7Q0NA6OjoWFhaMjIyqqqpcXFx8fHzi4uKGhoYODg5oaGicnJxmZma2trZGRkYKCgqwsLCWlpYYGBgGBgbo6Oj09PR4eHiEhIT29vaCgoKoqKj6+vr8/PzAwMDKysr4+PjW1taurq7w8PDS0tLk5OTe3t7Ozs7Gxsa8vLzg4ODY2Njy8vLa2tp6enq0tLSysrJMTExSUlJaWlpiYmJCQkI8PDzExMR2dnYyMjLq6uoqKiqOjo5YWFgsLCweHh6goKBOTk4SEhKIiIhkZGQ0NDTIyMjMzMzm5ua4uLh+fn6SkpKenp5WVlaQkJBKSkpISEgiIiKmpqYQEBAMDAysrKwEBASampomJiYUFBQ4ODh0dHQuLi4ICAhwcHAgICBUVFRsbGyAgIDRZ4F8AAAAAXRSTlMAQObYZgAAAAFiS0dEAIgFHUgAAAAJcEhZcwAACxMAAAsTAQCanBgAAACWSURBVBjTY2DABgwcLDW0BR1FYPwQR4tkL3mOQi47CF9UOsE9xMxKSSoszBHEN3Ow04fIOJmkJgEpW059mN78IEGQCS7GMAE79gyQQBLcNrc4dpCZIggBX3WQI9zMYQJOPikgW4zhArFZOSDKHCagxu/nDmWyaIeEJBsG+3tB+WXleTG6kUUSNjC9jsqZ3DzpqdZYfQ4ATccZk/hHp8UAAAAASUVORK5CYII=');}\n");
	print_http("div.wait7 { width:16px; height: 16px; background-image: url('data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABAAAAAQCAMAAAAoLQ9TAAABd1BMVEX///8AAADU1NSUlJRgYGBAQEBERERubm6ioqLc3NykpKQkJCQoKCgwMDA2NjY+Pj5qamq6urocHBxycnLs7Ozu7u7CwsKKiopQUFBeXl6+vr7Q0NA6OjoWFhaMjIyqqqpcXFx8fHzi4uKGhoYODg5oaGicnJxmZma2trZGRkYKCgqwsLCWlpYYGBgGBgbo6Oj09PR4eHiEhIT29vaCgoKoqKj6+vr8/PzAwMDKysr4+PjW1taurq7w8PDS0tLk5OTe3t7Ozs7Gxsa8vLzg4ODY2Njy8vLa2tp6enq0tLSysrJMTExSUlJaWlpiYmJCQkI8PDzExMR2dnYyMjLq6uoqKiqOjo5YWFgsLCweHh6goKBOTk4SEhKIiIhkZGQ0NDTIyMjMzMzm5ua4uLh+fn6SkpKenp5WVlaQkJBKSkpISEgiIiKmpqYQEBAMDAysrKwEBASampomJiYUFBQ4ODh0dHQuLi4ICAhwcHAgICBUVFRsbGyAgIDRZ4F8AAAAAXRSTlMAQObYZgAAAAFiS0dEAIgFHUgAAAAJcEhZcwAACxMAAAsTAQCanBgAAACUSURBVBjTY2DABsxt7a3trJUMYHyrEBdOJkcni0AlCN9MVF/UmMFc1FrD0wEsYGBrBZFxl/cSBRlgbAXTG5jmCBIwgxvuomODapuxXCqagIkckGTTg6tjMlQDkkZacTCBKHZBICnNFyMO4dv41YCdFpWtW5EsIioWVhVnAZHh8K8sKuPjz6p2hhuWKsnPxs7BidXnAE/xFfAXZBoQAAAAAElFTkSuQmCC');}\n");
	print_http("div.wait_img { float: right; }\n");

	//Blitzortung.org image (base64 encoded)
	print_http("div.image { background-image: url('data:image/jpg;base64,/9j/4AAQSkZJRgABAQEA8ADwAAD//gAdQ3JlYXRlZCB3aXRoIEdJTVAgb24gYSBNYWMA/+EKFkV4aWYAAElJKgAIAAAAAgAyAQIAFAAAACYAAABphwQAAQAAADoAAABAAAAAMjAxMTowOToxNyAxNDo0MjoyMQAAAAAAAAADAAMBBAABAAAABgAAAAECBAABAAAAagAAAAICBAABAAAAnAkAAAAAAAD/2P/gABBKRklGAAEBAAABAAEAAP/bAEMABgQFBgUEBgYFBgcHBggKEAoKCQkKFA4PDBAXFBgYFxQWFhodJR8aGyMcFhYgLCAjJicpKikZHy0wLSgwJSgpKP/bAEMBBwcHCggKEwoKEygaFhooKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKP/AABEIADAAZAMBIgACEQEDEQH/xAAfAAABBQEBAQEBAQAAAAAAAAAAAQIDBAUGBwgJCgv/xAC1EAACAQMDAgQDBQUEBAAAAX0BAgMABBEFEiExQQYTUWEHInEUMoGRoQgjQrHBFVLR8CQzYnKCCQoWFxgZGiUmJygpKjQ1Njc4OTpDREVGR0hJSlNUVVZXWFlaY2RlZmdoaWpzdHV2d3h5eoOEhYaHiImKkpOUlZaXmJmaoqOkpaanqKmqsrO0tba3uLm6wsPExcbHyMnK0tPU1dbX2Nna4eLj5OXm5+jp6vHy8/T19vf4+fr/xAAfAQADAQEBAQEBAQEBAAAAAAAAAQIDBAUGBwgJCgv/xAC1EQACAQIEBAMEBwUEBAABAncAAQIDEQQFITEGEkFRB2FxEyIygQgUQpGhscEJIzNS8BVictEKFiQ04SXxFxgZGiYnKCkqNTY3ODk6Q0RFRkdISUpTVFVWV1hZWmNkZWZnaGlqc3R1dnd4eXqCg4SFhoeIiYqSk5SVlpeYmZqio6Slpqeoqaqys7S1tre4ubrCw8TFxsfIycrS09TV1tfY2dri4+Tl5ufo6ery8/T19vf4+fr/2gAMAwEAAhEDEQA/APFovC18y7vssjD2q3B4SvpBkWkoH+7Wz4nurmOeyS2uJo90Z+VJCoJ3kdq6DSl082BRLy9vL2N0MjM7lCmDuP3ht5wBjn1r3Xg4xpqo5bnjRbm7I4qbwZfRjc1vJj2BNNj8JXbnCWrsfZTWz4shvdKe3lgvr1be5RmVWuHJUg9M56dDXr+jwIwhZ+SUVs984FTUwsIxjUUrqQryTcX0PBpvCd1CdstpKrZ6GoX8Lzjk27j86+k77QYrnM8BLIwGQB90+/pWNceGLiXd5ML5Hbaef0rD2UehS5nseAzeFrhVzsIHpmsy40OeMkbX4r3W+8JaoSQtnM30Q1mzeAdamXcLVlB/vYH86nkRa5zwyXT5UPzKeKrtbMOqmvbpPhnqj8yBEz6sKhPwuuM/vbmJB7ZJpey8jVOSPEzCfemGLFeqy/CrUrzVpvst2/2O1fZK4t2YL8isDgEZ5bnH3QM+1YOi+Fxqeg2t75+ZJQwIC55ViP6VywdOrOVOD1jv5HQ1OEVJ9Th9horrp/DBjkKliCPaitPYIj2zN/V4bi+NpJbRtJsQglSODuJro/CcsFms/wBv0aeSaVt3mBz8o67QB2znpzzXnlvrUIxmRfxNa1nr9uuMzxj/AIEK9KWJU6SoyWi8zjhGUJc0Tb8XJquv6iZLfS7mO2jXZFGFHA9cZ716vpL8xR5wwRVIPqAK8qsPEtorZNzD+Liuv0PxHYPNG32qHgj+MVNWv7SEYJWSIlGV3J9");
	print_http("T2nQrDyIkuridIYs9W7+2O9amneXJeObckxljt9hXFeJPElpNa6f9juY3tXTCKrDcMcHI9c1r+FdQVoblgSWSFiK5rO1zppmtrN+v2pyMEZOM1zV3qBD4Ldeax9Y1rZOQSeuK5zV9ZKzBwx2sMiqSexTbZ1lxegjrWBrGrx2VpPc3Em2GFGkc+gAya55tfHTdW/4Y8P2fi+xuW1XfLYq/lvCpK7zgEHcD2OOK5cfjaeAoOvV2X5mtCjKvNQjuek/C/wAK7fC1nqWuWq/2xfA3k0DSExpvA8uMr0O1Ao+u7rXzhr0Fx4F+JGseFpwF0+aT7VYANu8tHG5UB9MAj6r719O+HfF0NlY3g8X3ul6dPZv5azSXSRi5gAysuwnKE5II6ZU44ryTxr4Rs/iF4hbxwmqJFZfY9tmJF8tCUZvLldz0Ug5xjvXj/wBo4bDcuIvpUe6W/wBx3PDzqJ0/5TzyW6Bc9KK5ua92Ssu5TtJGVOQceh7iivo076o8yzPNlguWj3iKUptL7ghI2g4Jz6A96sLYzxS4vbe7iVU8xv3JyF6BsHHGcDNekeBPifpfh/wxYaVqOjS33lrdQXLblAktpCJFhHoDKFLH0GBmtDRfjLbw6bH/AG3pUmo6jKlz9smkKlZyZzPAgzyEWViWHcBQOOK8o9BOx5bLpzQh/NgulcLnyzEwKZbA3ZHQnp71t+GvCT66bxopobWDT41a9mvZktkidmKqu5yOSQOvPWu40b4z29pokMWoaZNfasbV/tN1MysJ7lLgz2rHvsRnZiO5x2FZ/g34haL4e0/W4Fn8QJc6nd291LeLFbTSSbEJdWEmV5ldmBwTgDPOam3mX7TyRh3vw91uyZ4p/It54bwWUxkuVCLI0TSrhuhHlqW3DjBX1rNfQvEOnGBp4dWtzPAlwBHHIT5T52t8vY4OM4rtdR+LGn3B1Sa00u6tri8k1S5XEgYJNdQpBG2evyxiTP8AtPwMVZ1P4paRPF4gmsZvE1rf6hZw2Vu3moyW0SQqhQDf1ZgcvjdgnGCTT17i5le9kcHe2lvstZ47jW5Le4JWOR4GHnN/snOD6cEn61oPpclv4dt786NrrI03kG5aZo1dym8Kq7c8KMk9K7NfjDo8N7bXUdhqso+0Wt2bSWVDBZNbQMkSW654UyMCSQDtGME81l6D8U4I9F0y21qbXLi9hXUxcXCSrJmS5iEaSrubkoMjaeBkkHtU8j6tl+17RX3HGGa5upgLHTdXcOA6oJ5HO0k4xheRgEZ74NVtPv8AXbMG+06XVreNWI86F5NoZRkgt04HJ9q9Is/i5plhqFn/AGbaatb2Nvd6YSizKHe0s4SFiJBGS0rMx7YrVvfGOlJ8OJJbjV/9NvdKj05rCyug65lujLcSeUQCsu3cCW4yQAWB4bipLllqvMz53e60PINb1LWvE7PqWote3xt1CSXDKXWNewJAwtaGoap4iv8AS7DToTrB063izHAysy/KoLMMKMqAQec4BHNdTofxG0jR/C8em2ltqwe0TUIYIPMQW92LkFVkuQPvOiHGACDtXBWut8KePNN1/WNbbUdYl0fRzLp8Voj3whngtIPvIowVZWCgsEO4sF4btKo00klFabeXoP2ktddzweW4vopWjmlnSRTgq2QR+Boq74z1o+IPF2tawxb/AE68luFz2VnJUfgMCituaXczsux//9l966L4ZaBb+P/bAEMADgoLDAsJDgwLDBAPDhEVIxcVExMVKx8hGiMzLTY1Mi0xMDg/UUU4PE09MDFGYEdNVFZbXFs3RGNqY1hqUVlbV//bAEMBDxAQFRIVKRcXKVc6MTpXV1dXV1dXV1dXV1dXV1dXV1dXV1dXV1d");
	print_http("XV1dXV1dXV1dXV1dXV1dXV1dXV1dXV1dXV//AABEIADAAZAMBIgACEQEDEQH/xAAfAAABBQEBAQEBAQAAAAAAAAAAAQIDBAUGBwgJCgv/xAC1EAACAQMDAgQDBQUEBAAAAX0BAgMABBEFEiExQQYTUWEHInEUMoGRoQgjQrHBFVLR8CQzYnKCCQoWFxgZGiUmJygpKjQ1Njc4OTpDREVGR0hJSlNUVVZXWFlaY2RlZmdoaWpzdHV2d3h5eoOEhYaHiImKkpOUlZaXmJmaoqOkpaanqKmqsrO0tba3uLm6wsPExcbHyMnK0tPU1dbX2Nna4eLj5OXm5+jp6vHy8/T19vf4+fr/xAAfAQADAQEBAQEBAQEBAAAAAAAAAQIDBAUGBwgJCgv/xAC1EQACAQIEBAMEBwUEBAABAncAAQIDEQQFITEGEkFRB2FxEyIygQgUQpGhscEJIzNS8BVictEKFiQ04SXxFxgZGiYnKCkqNTY3ODk6Q0RFRkdISUpTVFVWV1hZWmNkZWZnaGlqc3R1dnd4eXqCg4SFhoeIiYqSk5SVlpeYmZqio6Slpqeoqaqys7S1tre4ubrCw8TFxsfIycrS09TV1tfY2dri4+Tl5ufo6ery8/T19vf4+fr/2gAMAwEAAhEDEQA/AOSXTJyM+UxqVNLnYcQt+VWtQllV4Vjkdcr0DEd6vW4tzAQJppZVYbiWbG3v34ru9klFSbONNt2Ml9InXkxt+VNXSpm4ERP4Vb1JJrYxuk8wSQEgGQ8V1NqgOwn0B/SlKmklK+4Xd7HFtpcqHDRMDTDpsg/5Zmu/lsVk/eJyD6dqqvpsjZ2IfyrPlQ9TiX0yQD7tV3spF4wa7KbSrrPELn/gJqu2h3rjPlEfWiw9TjmgZeoNMMZHauuPh26P3gB9TTf+Eak/ikVfpzS5Srs5HYabtrpW8MXMt0/lTHyomwx8skDgEd+evPoOapWmmi4so5/M+Zs5GPQ4rJOMm4roaO6VzHwaK1H00qxBP6UVXILmZduUkmMTRqWwD07c1f05o4Q/n2Ts7HO7d09uKwkvE7sKtRX0Y6uv510OpeHIzFJp3Rb1Jbu+uN0drIsajCLiumtjyq9DgD9K5qHUYQc+an51qWmoQFlPmpx7iiU+ZKK6Cad7s620g2KJZHCL796swbWlPl/dzxWRfahE8cHlSqYyOADz75q1p04KSHuqEistbXNIlq6nHmHvWfLPzyaqXV5hzzVC5u8PuzweaY7mm8w9apXV0sMTySNhEUsfoKom+96u2FjDqsMhuctCDtKDIz361nXrRoQc5FQg5uyN/wAP6ZjTYrm9iH2qcGV0LfKM/dUj2GB+dcFeJJoviC60pxiB28yAZztB5A/p+Feh2OqpDDKNVntrd4jtDtKqiROzYPT0x6jiuY1XSodevzri3QSLycQhhtHBO1ifSuT29Onap/Mb8kpLl7GE0gzRVBpsMRkHBxwc0V6FzmsYASQruCsRgnOD0HeniF1bE0cqgDcfkOQPWt/R/ElrYadDaXFm020SJIcj5o2+YIP+BAE+wqe08XRpbr9stWuJ2WTznbGH+cug9lDE5/CuU6LnONblM7kkBxnbtOV5wM1bsNLa9MpV1iS3UGZ5nEYQkkAZPvWxa+L44rNFntXnuTEfMlcg75FkLxE+wJJI71DpevWVhBdoHvhJcyxyNMEjdmwDkHdxyxJ/KlYfN5FOXQb6ElX2RukwhfdINoYqWHPptGc+4qubLULfYXS6j3xiT5VbO09Dx64rXn8UW7m4eK1ljeVrmQYYEB5FVFOfZQ34mn3HiWzdb54X1GKe4hSGM7gVjVUAIxnuR97rjPrTFfyMaWKPEbrJeMknCsy");
	print_http("H5z7dqnNs0dglx9jvSC+wyFyoJxnAGPStYeLLNJo5Vt7lv3kUvlOw2QmNCFEY9NxBJ44FVrPxMi2dvHePeSSoLjzJAwbLSLtVhk9QMjHvSt5lc3kZO+SVwILa6OeQN7NwenQVHDPfxDz7d7mNQT86M2Mgetb8Xiq1gni+zw3UcMcttkBxkxQpgKT7sSTVmXVrRfD7O95+9mtVtzBDKCPml3SNt7NjPJ9eCaGrqzJvrc5a7uL3Uibm4M0/lgBpCMhR27cVPPc6jPbQW6fajBGnyoQSOByeB0/lWlaeILO001baKO5zEs6JHuAjlEnAaQdyAensOlaena3bX11dm4vGtLXfAsQafY6RJ1A7EHAyBznHWlyRVtNg5mcYzzqxV3dWHUHg0VLql4b/AFW7vDn9/MzjPoTx+lFXdiP/2Q==');");

	print_http("</style>\n");

	//Favicon
	print_http("<link rel=\"shortcut icon\" type=\"image/png\" href=\"data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABAAAAAQCAYAAAAf8/9hAAAABmJLR0QA/wD/AP+gvaeTAAAACXBIWXMAAAsSAAALEgHS3X78AAAAB3RJTUUH3AUOETcK7CEO2gAAArpJREFUOMuNkltIU3EAxn/nnM153OapzZU4FbKLMB/KWLbILAjKgopggW9dfPAlUNCCqAehpwjCB4nIJFAouhlIlERhF81AhmQOmVqUtVV28Wy2Otvc/j0tfEjy9/aD7/uePoVFqKur2+n3+3tqamoaMpnM9PT09NS/cqaF4vf7KysrKx8LhHAXuR1erxchBC6X636Fp+JOOpP+0NHR0Qxksh1JQGdWRr3eY/n5+USjUWw2G5qmoSgKiqIw2tiIYRh0d3e/6Orq2prtKK1wAGDGYimYdTjLDeM3TqeDWCyGruvouo4sywwXFaFpGqqqlphM5kQoFBoAkAA8Hk9RU1NT2O12iw3t7VKB08n4iRaiehRd13ny9ClrV68hHAkLbZkmpZJJTp8+4wYiMkBxcbFPVVV8Ph/WvDzCka+8ffcdk8mEkbKgWAqpvd3D8ZERad3acgpCE7yH5gxclAFqa2vvVFVVCcNISJ8/faQw9zHV2n4GXwbx2g5z/vBJkvEws9Ek9r5nomZqkgR4JLCYAOLxOGNjY9LcrxwO2sZQHfM8/7CPVVqQsqovAJSXjoM0Tln0kfRg93kelJTOX75xo14B6O/vvzIwOPx+Bffm9qnfKn7+sHI196TYUnKdVGyZ9CPiIm8whikvzUNPPfZtewlOTboCgcAFaeEPkq+VtPlcWv751cpMS6HIzTFLje3eUDzlmL2VuLx5PpUSR+xHX+WYY/T29rYZhtH1txzu026KNwjhR4hDCDGH6LtomQGWQ7VZ+BEBuxJkcQpWgt0bQeoJr1EmxFtJyFgbACbuLj82s13JROEa/0NAZ2qH/KmzVQ0BFoBfQ+qTgCqPLHxtFvlfI6lcubC+VTQDCYCHQ5aK8t+ZIEvh1PFdjrYW61DWn11dV71to7WepXLp7PpNIJVlfc8unwdQF8v/ASDQFWWa4OheAAAAAElFTkSuQmCC\">\n");


	//Style for Mobily Devices
	print_http("<meta content=\"yes\" name=\"apple-mobile-web-app-capable\" />\n");
	print_http("<meta name=\"viewport\" content=\"width=device-width;\" />\n");
	print_http("<link media=\"only screen and (max-device-width: 480px) and (min-device-width: 320px)\" href=\"m.css\" type=\"text/css\" rel=\"stylesheet\" />\n");
	print_http("<link rel='apple-touch-icon-precomposed' type='image/x-icon' href='data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAHIAAAByCAMAAAC4A3VPAAADAFBMVEUAAQAHAAARAQEDCQwcAwAHCgYLCA8aBQgSBw8CDRUYBxElBwEiCAACEBsAERcDESUBEyEPEQ4oDAAAFR0AFigAFi0wDQMBFzUAGSUrFBEBHyYAHysAHjUCHjgtFwE/EgEaGxgOHSwCIDIpGgAdHB8GJkAGJzk3HBYBKTYRJjk8IxEPLTUBMTxJIBQoKCwLLz0oKSYPLkgOL0JVIAcaK0oSLmgLN0JHLAJiJQEKMJgRNksANJoWOkEdN1MUNnsnN0o0NTk1NjMWPkoZPVMQO5wYRVAdQ1tYNyx/MQMoQl0hRVdsNhsXQ5siQ4w/QkYqR1N6OQxDREEgTVglS2QpS14fTl8xSmVrRQguTIUpVGg3UW0xVmMyVXBKU1k+VHY8V2tDVmxTVFEwXHQwXW40WKM/WneoSggvXbQtXstEYH6ZURs5ZHw6ZnhJY3hxXzZKZXNKY4pPY4VgYl6FVmhDZ4xVY39WZWs+a4SpVxBLaIdKZqJdaWmfYAGSZASaXECHZSFJa7RFc4xScI9IdYd/bFNYcZpec35ecZROdZxcc4lucmxNepVad5ZOe9qsdg1lfaFggJu/chRYhKZXhp5jgbBygJWbfUJqg5d4gYlsg59/gX5biZphhMuhgiZsham6gADHeTXVegxikaO8flhxjr5okrOCjaN4j61xkqV9jbhplK53kLSXjXt4kqiBjcBqmKqJkZqQkY5rn69ynrGDmrd7nL98m9mIm8h1orzNmRWJnMB/orfAm0qCosXInDR3p7nVngCKor+hop+7oHuTp7mFqtPCpGR+sMmGrsiBsMOWqNWDsb2Tq8qNrcq2p5OYq8/jrwmQuL6xsq+Pus2Zt9SQvNamtuSbut6nt9zgukCWwdWgv9utvdDyvgDwvRzBwr+zxNegy+LgxGu8xNmjzNqqyebcw4+rye24yNHOxa/9ygP7zSvx0z6v2O621vO71+7A1/bU1dLK2eS/4/TH4vfQ4PTe5/HW6/Py6MDm6OT98brs9Pjz9PH39u35/Pn9//xgZfC/AAAAAXRSTlMAQObYZgAAAAFiS0dEAIgFHUgAAAAJcEhZcwAACxMAAAsTAQCanBgAAAAHdElNRQfcBRgSOw4wzTPVAAAAHWlUWHRDb21tZW50AAAAAABDcmVhdGVkIHdpdGggR0lNUGQuZQcAABTaSURBVGjenZt7TFt3lsevdqWYQElbNUlrdUbT7TBjLZJRZxip1UqRnH+2G1myzErb/MVIaDQrlVmhCCHa7jZB2zYRk0RAaKbRguJESdwJuFU2DysGYlKeNh0oNLbXwrYABxwTtfiCje0yZu095/x+92U7TbcHfH19fe/v8/ue132ECEKJ9Q4OurzF5nG57rvghUZrYLdL7IbKrly50v/hh/8mPNXsSPOUAEcY5D4zWiuDvFGEJOt/5+k8j5boIZNxHAlKyxDh97qMVKD9T9TKCTMzBOEsD6OqiffxQxm3KjKv9F/ov6K2/vISOWKGzMNe8iaP577WAIlUVxnihfZGtPYLCrG//w+lRDdJnHmylTCLEuiWhDzVaGHW2N6vYn5YTJwLeL3lSeppqDwLv5JIl0blKQ60NLVaGjmzH5lFzg0EAl6Pz+eb+X7TRFNhSsRbQOyXNFpO97VaJJ39/cXMICA9iKTF95EB5JHqRJtB3LGySEtra5PFckqNVPk2Gg+Q+VQ243uK2pJecAuRV2SRPJ7gUxmp");
	print_http("yiExHpSZnEteVvhFRMa8rwYOslgisoHR2Fu/hNO4Nrodj4cRCD8BhOIiwLA+rr1sSKXkuXULRVLy9MuwpqYmiwk9q0Zy1wZjCSIq5pPU0gQowCUa70sJJCcPps+tK5bGm33I6nvw4HSDyWRq79fY7wgZjwOSmOFwSEMNMNFFMtXFefv2/Vu3mEhwK6z1W/qWt25aLH1by1vLTYhUpY8sMxYX4/EYUGVT6ZVzqiwR7BYZ8y3VyHjf+HKjZXy5tW+rj5DXr1y5fv261BSYX+PiZjwWjUaRFgqFVB4OlUmfIqeOoLlG2Pvt29cbxxtbx1ubxscPNy3fRKTqpIKGMsPR2EYqEYsxYgCZfBkKFVeMd25ubn5+Hl/zc8zGmI1wu9F48/RH04uj0xMNTcuosrEIiUkbCoWjG6l0IkYSQxzGTEJC+6Xx54Pc/EG/f05lhLwNKm809n11b/Hbbxe/HZ1+jLFsvHLjluY8KggeGDkcjqbS8Wg4pLUAy6AAH9aPFpShGiQwwbkYzsaWe4tL35IttiDyOhGVNvwhIUOh6EYa0xZXaYEkv8pKmXwblz83J3n2Pz4ZXeTExU8AaenHxBrhQQf7WEKCb0VMIdKM5aIMLSM51A9fKttkppeN2N7CkIuLo6MtJqjQUxJtjLnDLvhCnBnFeGLahrBMtMw5DVJSTjQpzLhCyKaJ0UXkQSxPm1r7LI0j7IKGjsdfCRmKUhLJTEASNB4LE9LrL29er09CMmpj04PpewC8N7G1NW5qaG0wneKT5RLiAiakRAXotsjqM8xlxuPBcFlWGB2MK6qgw8Cfm1qXlyempyceb+3kd06bMIFG/DiObAKrAsm50QQqLUKGeQTLmGYSYHPtpgc7W8tgQCwUHliQ+UfgbGqRWAwhVUhjktBYHGcXi8U4IFbOaAd6B/ujqW83D8ytnV0gFnZP12HSjilEWBGYbwIBGbmWjaPcqESIS2PGlKFhsVlq8finJstyoZDfZUCwcVRZ1+4vQRJWkZkVxQS8F6uJMZZCS6sNN4xZ6vp2AZTnwEJ+y1KHdiqmEBVkmHcBYInbqWw6kSiBxmKkhXCJBBAzgMpkZOJsY51pHPTld3fzaLu7O1tNdRIzEcezZIynT8CP5yglnBsbYiq7XY4JpARDbhMtk8vlsgya2DxVV2cZ3wLcDjgWvLsFhicTZH6qRIhlLLZTJWs5M5HYKIYmuG1uAjGdTW+ngQhr+CkxCWM3Le+QPEnk1lbfYWI2ssxIyEh+7udABs1m0xDTjXJIER2LpO10Nodv27j1T3WH0a8ag+S1wGZgfh5VI71SbUbVQlNZsJQIgqU4MoHiJgcryPQ2bms3mVp38kW2u9NnamgAZDuckWOyYwOqDqRAgSVmIXkBqiIqJqnM4Dt89ptMh8fzpbbc0NAAMk2TkquEGan7aDofITcSUC/gWnGDHKKJJqQPvNTIP8kidzXe3WkEHsi8QMfDwAJcXiiX6GokMjegWDbAH6JYjARnMiQLZiIRazTVcZFa5G4rQ7aHYuxqR1ATfWoiIVO5olqREghFIjOLMhH5ad1hy1ZZ5GmGtPjDrHUKeCWl3AqUIrEnxMq4VZSRWRQchjbQxJNnR5u2fXV1hwFpGomyXl3GsVElmFgqokom+I+wYnqbISGW6FisEJPp9G5Z5M0aUlk3GQvQRaQKKXmV6yTixjZEU9sQKFvFhChyJPk1Zqk5bOl7MrKmpqZuhN8MEBKplLLqKiGiCDITJdGUSxOR5NeIqabOcjNf3rE1IJGQeBkXCgkUyBlfSBYaVdWlKEKwtjc2ZKSI4qQXKxIWyqaWlsM3y6rE9HntNWi1I+w+IKSJpTp5GHI7BSeVbbnXYrHAR");
	print_http("k7E1o5InMG9haWOpmWOVAH/96+PWkBizWsNTT68DUDHlimRNe5XRoTCA8+GOTDGo0iFAgnLRIqPv1haWupoeLBbrPK7pW8uvoKRbLj4OMTu5gTfzIy6C6gqhBHxVCEyJOncRInMeCgfP370aHQBkAstJvKt2q/fwUyAWPNKxzcPfXQnFxBKEhZ5lDuMiK1dlFp7TIwlmEa5RtKPRxfAltBGa+v6trRnkscLC12vg19bhpYestuNgBTLoo5ObQAsm8ls42kTqpiIWJisMlErIR99sSTbxZqa1mV16mw9WlgY6urouDi0tLT4tYQs11xTjAgdlJIDiJQ/m6JcmSIyya9q5GJLTU3Tlhr5+AtgDg3RPtN/CfD00QYxyhVuEzBLfR2aMQrdZKfYGIsmXRdA8jxWIb+5+FpNzQO1YyeGFpZkx09/jXcKgrY6qDLIo2i5TDadEtn9tchjyZBpkRVtLr29PLSkso4ai9qz4y1DC6pvH34Ft6osfTQdZ4MRM/ibgrE3MF0ZMSqydkANjxybFh+rkd8sfqb26/hhLXLp4V98gPTIRJ4yXGJqe1vcxmSlqwJeJCJ4Vkyx9oPI7LY4oVG59FBBbvXVvXJU8+3CtNfjETweT8AnIRmMQogXIBBG/FHfHkA7ICQ5Ng0ixXtfaJBDE8vs+m7rpgXKUY1cWJye9I54Bc+MD5FcZJblDJbixgZlE9xgi3Cei0n3C6Ji2H/F5dEFLbKjobXvZt/pJlPNnj17XumQHbswND1OT08glh7stVxkBjJ1e0PhhaKBEGVOWNXYJWwar8bW70EJSCmJAx//+Z7nkUYmIxeG7o3z+2yIpSfA7i2RmBZZBjEe3jP4Q3ArqTgWJIsxOXlApbj81fTFS5cuIRhG/qKr4+fPP1/D7flXjnbR5qHRiUnpYYGAT+gCMlJU86Dx+/wBv4SEQhF5NMkoAincFhn/7JOOri7ADnUdP9pw6sL1z8kunGo/ehy3Xrr4GZfocrkEdtpkoSQk9yg9TcOLana/rLq/FCWRmGOpFJ3ZouHJ8c86jh8/3nH09AhdGrPm5j199HhX1/FPbtKDBPaPLYLHtxYKhBhRQoYkovyMQHNLm4beTr1JZE6Bbojdwj9586OWj/p86mee3pG+lpaWj/7b65GJgPQE+PkR/ZrgRDjO41U/BFEjUWaKFRK4NRpT7uLhXsPPWOwN+5trhD8tcSlIX2htTbrvwVvZED3+9Xjnih72yFQkpSQiSmTPbfjtOKHoSRV79hIM+jVEUhkKraWoDdB9bIge7QPRrX3YozBVSGyCMUkjk+f1BtT/fjUXnPND/cvIwUGB+l10TWR+jcpIN9gcwxYzKWU5kj3JCMgiS/5J0B8E1yqBJGTARw98YNoZtUg3s1Kd1OIZkt9kBAKSxBKg1x2cctMDW4ZDAyTlKxIlkfjPXGNu2UrCSVebsDudXcJKGP3eJyBlgRKSPdbSEkfcbg1TDcVTGFRlmpDy8yaQOKeBjWG2ur1Bt9tVggywdCUkJ3pHnG4nwJzFziUkF5lgROlZHj3GYxnDmC7wptstIwcHVSoBCUOklUB6RpxOIjqdZXxL3QD6gEpjgD+J9M6NcY0uNyO6gzCIy2kfHNQiJZES0eWC/YZBqLOMa6kyqfXIItGn7PmoGyPicrkpFVzgpfAUTtwJJIlqF8CvGEhyKycyJNlwqUw6n1CJKIFkREQyGxkjFzndc+55XEGc3Y4vMAFEpjIZ7DvhkKSRIYeH8VUkky68otBh4TqMRGJvY09j3e6xMYYaQ2eiY+fczqkpt50bk/oxIDdY5qiJdoeDJMoy5zRMkSUsXvoF5GfSuBsXRwZE5xxLCbsM/RhMCERTdNcaUh");
	print_http("FdTgcxnVqm5NkoJawcSYWoAgIRFgxpV5hA7BVC0WxmM0pxlJGDoFJhDjPkPGNi9wGRmYQWWUKE2NxxOr1O95SkshesB34EDOUmekglUoPkzFlgMiSJ3Pxeoku1ErymIHvAetGxgBSzSbQ1hiwUCg7HSqGQJL8Mc+b8PBJikD5iiUiEOrUGEGJCyxtUEXt6KJab0az0zNYHRF+hkHE4MoXCitOdKxQikk6SGcYnT1ylRAwXCjkFSSy7tHTa3VNOiUgvIbAGU47CyHBDAMskuDUJMJtjeGx22Dmbg8GcvFKYTqxLDdLtBodkOM/B88ThsEtZ45xiIpnG3rOCJ4SNIFfIRwPeNYYE8pjNubKy4ogAPZdcjQyvrq6S5xGZhu9zabyWxg3JQjIJDsmsJmedK+tJyPXI+vodiMv6yliykJsFWtCxns9H7OvrK4A8ewZO0dG1VDZfyInRELh3DRwLfIdtFr3L3Z1085UCEHNsLR32rwI6D7vx78bc8GHY4YTDxxxj8B3teQeQtAfMPgPEzrOCD6/WU3ncg/vVmweYDbPHkUSHJ5Ozs6AHH9En5/24hXSGQSBsCa+s0Hsy45iFA50OiEXGYZ/FbbiI2O3ohMl1XO/p7QTkfZfL401KKgBpR5jNlizkIwMDsPc8K5iVPI4xDxuycLaEXWN+HIpiDBPA5hHJwywdETzcAYSM3QZ7TNom4WtbTzccMtLbefZspzAInvRgyKDPgkt8djt8iHTD3vkx2wBw7g4MDNBA4F83SMjD/YmIyDCsRxBJDgEDR69Qca3bEDZrs4E3HN1JXO2xwSE9nZ2o0g5MD4wPbQ99HrIPZgv5yW7YIzdwfhiXaCCgsAqjg19zgASV8AaLWaxbnCMi8R1heYBhadOyB/0atHWjKzrJhF5otpgw0bVMDFXih9yfT96BPc6fRydJxAj2Ba4SZp+OrVA1uoedmOGIJBgckr/TjYfb0KWZnm4s8Lt3YJnkSGi0g9huqCoLWdfgCE7nJDhp/fx5WOaTqwPDmC6QQStudwZDgB9jwSS5GjpAAb+NUPPIYXLmurtxrt3d5GNcL+AhhQgRPxCw2frk7BkZHMS9O0/CoWPn0bGw+8CqXCzuWVYSuVgsCKOsEJK+BpeuEBsO6EZls7Sc7OnEYBYyO5A9hGwTzp450zu5vr6eTGazQZfd3gMf7nSejERWz4MNRFZX5wfmV5lh5Nz+dBJOJPFgMJlOzhJyYBbaxh2bzTa2MmmD9TFQubJi6+yORGAJdicS80K4HJ1nO9va2oTODz44A9TeQdfaCpy/e6FYwU6SIfT8tWvXBrg577rdU/PzSAPk/BSdPe46IKFt3LolY2GjN1t3r9ce4aFEZFvbex90njnTY7+/PmyH7nCWth9TMQcUQ+TsVDAsgsr5qampu+67d9k3amKn1jKZJHaL3H+1HWtDE37fhtAPzvZ/PW8jIG0+dkxBKsxrd8FAZjiW3fyf+S+/xI/XyM4D7gnEbtYh16+zkQH5k5/84le/OnToyL/87sihI0f+8a23fot27Nixfy8jExl3vwwGEfmljGQatS7ldhKZc/6xP3e23ZCRz6HtBzuwXzFY/7tX/x7ncuTIIbQj+H7kEMynzeZwuP2AnEIiKEfYyZNstPfee5tmzOYN9nuwPwy2vf32b9955z/f5shqsOd+mO1/9tlnD7z44k9/+ptD//zbt96i2fwG7Rcvoh04sP/VV+VZ4wEvMXuZFi+//MuXf/nrX//DIWHvnr1g1dXl0UWbXnru2Wf3H3hRtgMq208EifNCsVXv2/cC/IIJCNzDuHsVOE1Amkc1/ShiYXxGk/SoUC/hxKo5Zl/1C8TZt28fEMn27tsrCHs");
	print_http("U09KrS0yG7gcfEwxf1dXyES8ArFpRtndfNSD2okL4ci+DChrk/8v2st+9fLJspn+796kGyIofantU73t+vAllmU87Cqhltn3/Idx0QlnkD5hqRclY7MCKJ+N0ZPofjSzDKz5eEw6E6fVGQ61B+PEyNVZZWamrLB1AcqZOf9RsqK21nms+If1RpXTYD2dqcEyBUac3VJbkGu6gN58YNZvNJ4a+fSgoSDxO94xO9xSklK7ycOQwg8FcX3/iqtmg1xf7S1eBEzI2m83Wy9PnLilIAYlVRrPRQO7RG56UL0XjEU9vMNZPNJs77pmf0evxcJ2BkXW1RuObetgIe5yYmH539OFl9Z8B41zqr1qNOmJWqoTyVa04JSMgIYzGE2bzQaMRP2FSmj+q0gFWpzd3jV411tfW1hrNV7/767lLl7V/7AyzqZ8+9zOduYqHFHVVwATKZWUlZfwzOH1jvdXafBkcV2+2NlvN4CmjFdxsbgaXHYTPRjN+gu+am68W/0k3ONZq/VmVWV9RKccTfEzIv6nUadOzkuWMsf4Na/PE5XMdMLTR+m6z1VgJWVRhbG4+N31v6DgC6w0QSmO9+Q3rudI/XDfW1xsPgmd45spexQ+GZl2lTi4DTLWD9Qac+9Wu0Uej714GYW9cftdq1FeZjZU6o858aeiT1/UVtQbwfyU432g0GMr+fX7X6EGz9aBOf1CNpLAZzFUn6nVVB3VUf6ixvrne2AX+vLx4z2q+2twMapv/qdn6r9YK/RtvmK215vpag+H1WohvpbUetD7pPyHU64wH9c8YjdoWokd9uqMg6wQmCWBRJ+CrsBrPvQ+hMjc3QzDfBasHt1qNhorXrbUGzFaj3vA+zOb7/rMF5oxOU5lSwkIZHaw6ceIgJaPxTSOVcpUBAvZmFQQFPhx9/9z7ugr9ueZKOECvo0NhhrVP/y8lWFxMn6YwcSKQHYaKZ8w6Q70ROg2pNVTBqHpqAlgaEHuDer4VpeP/H13w4IrqqHijAAAAAElFTkSuQmCC'>");


	//BODY
	print_http("</head>\n\n<body>\n");


	//Header / Image
	print_http("<div id='header'>\n");
	print_http("<div class='image' id='top_image'><a href='http://www.blitzortung.org' target='_blank' class='bo_link' title='Blitzortung'></a></div>\n");
	print_http("<h1>Blitzortung Tracker</h1>\n");
	print_http("<div id='set_autorefresh'>");
	print_http("Auto refresh: ");
	print_http("<input type='radio' name='autorefresh' id='refron'  onclick='start_refresh();' checked><label for='refron'>&nbsp;On</label>&nbsp;&nbsp;&nbsp;\n");
	print_http("<input type='radio' name='autorefresh' id='refroff' onclick='stop_refresh();'><label for='refroff'>&nbsp;Off</label>&nbsp;&nbsp;&nbsp;\n");
	print_http("</div>\n\n");
	print_http("</div>\n");

	print_http("<div id='tracker_content'>\n")
	http_page_content();
	print_http("</div>\n\n");


	// Graph
	print_http("<div id='graph_container'>\n");
	print_http("<canvas id='Graph' width='1024' height='600'></canvas>\n");
	print_http("<div class='graph_text' id='graph1_text'></div>\n");

	//Graph select
	print_http("<div id='signal_select'>");
	print_http("Show Signal: ");
	print_http("<select onchange='change_graph_type(this.value);'>\n");
	print_http("<option value=''>Last Signal</option>");
	print_http("<option value='sent'>Last Sent Signal</option>");
	print_http("<option value='good'>Last Non-Filtered Signal</option>");
	print_http("<option value='imode1_first'>Last I-Mode 1: First Signal</option>");
	print_http("<option value='imode1_last'>Last I-Mode 1: Last Signal</option>");
	print_http("<option value='imode2_first'>Last I-Mode 2: First Signal</option>");
	print_http("<option value='imode2_last'>Last I-Mode 2: Last Signal</option>");
	print_http("<option value='filter_min'>Last Filtered (min)</option>");
	print_http("<option value='filter_max'>Last Filtered (max)</option>");
	print_http("</select>\n");
	print_http("</div>\n");

	print_http("<div id='signal_beep_check'>");
	print_http("<input type='checkbox' id='dobeep'>");
	print_http("<label for='dobeep'>Beep</label>\n");
	print_http("</div>\n");


	print_http("</div>\n");



	//Footer
	print_http("<div id='footer'>\n");
	print_http("%s<br>", S.tracker_version_long);
	print_http("&copy; 2012 by Blitzortung.org<br>\n");
	print_http("</div>\n");


	/**** Scripts *****/
	print_http("<script> \n\n");

	//Graph
	print_http("function Graph(a){this.canvas=a.canvas;this.minX=a.minX;this.minY=a.minY;this.maxX=a.maxX;this.maxY=a.maxY;this.context=this.canvas.getContext(\"2d\");this.centerY=Math.abs(this.minY/(this.maxY-this.minY))*this.canvas.height;this.centerX=Math.abs(this.minX/(this.maxX-this.minX))*this.canvas.width;this.iteration=.1;this.numXTicks=10;this.numYTicks=10;this.xTickHeight=20;this.yTickWidth=20;this.scaleX=this.canvas.width/(this.maxX-this.minX);this.scaleY=this.canvas.height/(this.maxY-this.minY);this.axisColor=\"#aaa\";this.drawXAxis();this.drawYAxis();this.drawXAxisTicks();this.drawYAxisTicks()}Graph.prototype.drawXAxis=function(){var a=this.context;a.beginPath();a.moveTo(0,this.centerY);a.lineTo(this.canvas.width,this.centerY);a.strokeStyle=this.axisColor;a.lineWidth=2;a.stroke()};Graph.prototype.drawYAxis=function(){var a=this.context;a.beginPath();a.moveTo(this.centerX,0);a.lineTo(this.centerX,this.canvas.height);a.strokeStyle=this.axisColor;a.lineWidth=2;a.stroke()};Graph.prototype.drawXAxisTicks=function(){var a=this.context;var b=this.canvas.width/this.numXTicks;for(var c=b;c<this.canvas.width;c+=b){a.beginPath();a.moveTo(c,this.centerY-this.xTickHeight/2);a.lineTo(c,this.centerY+this.xTickHeight/2);a.strokeStyle=this.axisColor;a.lineWidth=2;a.stroke()}};");
	print_http("Graph.prototype.drawYAxisTicks=function(){var a=this.context;var b=this.canvas.height/this.numYTicks;for(var c=b;c<this.canvas.height;c+=b){a.beginPath();a.moveTo(this.centerX-this.yTickWidth/2,c);a.lineTo(this.centerX+this.yTickWidth/2,c);a.strokeStyle=this.axisColor;a.lineWidth=2;a.stroke()}};Graph.prototype.drawEquation=function(a,b,c){var d=this.canvas;var e=this.context;e.save();this.transformContext();e.beginPath();e.moveTo(this.minX,a(this.minX));for(var f=this.minX+this.iteration;f<=this.maxX;f+=this.iteration){e.lineTo(f,a(f))}e.restore();e.lineJoin=\"round\";e.lineWidth=c;e.strokeStyle=b;e.stroke()};Graph.prototype.drawArray=function(a,b,c){var d=this.canvas;var e=this.context;e.save();this.transformContext();e.beginPath();e.moveTo(this.minX,a[0]);var f;for(f in a){e.lineTo(f,a[f])}e.restore();e.lineJoin=\"round\";e.lineWidth=c;e.strokeStyle=b;e.stroke()};Graph.prototype.transformContext=function(){var a=this.canvas;var b=this.context;this.context.translate(this.centerX,this.centerY);b.scale(this.scaleX,-this.scaleY)}\n\n");

	//Canvas for Graph
	print_http("var graph_canvas = document.getElementById('Graph');\n");

	//Draw Graph from Array
	print_http("var signal_data = [ ];\n");
	print_http("function display_signal(data) {\n");
	print_http("try {\n");
	print_http(" graph_clear();");
	print_http(" document.getElementById('graph1_text').innerHTML=data['time'];\n");
	print_http(" var SignalGraph = new Graph({canvas: graph_canvas, minX: 0, minY: -2.5, maxX: data['vals'], maxY: 2.5});\n");
	print_http(" SignalGraph.drawEquation(function(x){return  0.45;}, 'blue', 1);\n");
	print_http(" SignalGraph.drawEquation(function(x){return -0.45;}, 'blue', 1);\n");
	print_http(" for (var channel in data['data']) {\n");
	print_http(" SignalGraph.drawArray(data['data'][channel], channel == 0 ? '#f00' : '#0f0' , 2);\n");
	print_http(" }\n");
	print_http(" playsound(data['power']);\n");
	print_http("} catch (err) {} \n");
	print_http("}\n\n");

	//clear the canvas
	print_http("function graph_clear() {");
	print_http("var ctx=graph_canvas.getContext(\"2d\");");
	print_http("ctx.save();");
	print_http("ctx.setTransform(1, 0, 0, 1, 0, 0);");
	print_http("ctx.clearRect(0, 0, graph_canvas.width, graph_canvas.height);");
	print_http("ctx.restore();");
	print_http("}\n\n");

	//Sound
	print_http("function playsound(volume) { \n");
	print_http("if(document.getElementById('dobeep').checked==true){\n");
	print_http("var sound=document.getElementById('sound');\n");
	print_http("sound.autoplay=true;\n");
	print_http("sound.volume=volume;\n");
	print_http("sound.load(); \n");
	print_http("}\n");
	print_http("}\n");

	print_http("if (navigator.userAgent.match(/like Mac OS X/i)) {\n");
	print_http("document.getElementById('signal_beep_check').style.display='none';\n");
	print_http("}\n");

	//Refresh Script
	print_http("var last_nsec=0;");
	print_http("var signal_type='';");
	print_http("var refresh=false;");
	print_http("var data_refresh_wait=%d;", HTTP_REFRESH_DATA);
	print_http("function change_graph_type(t) {");
	print_http("signal_type = t;\n");
	print_http("graph_clear();\n");
	print_http("last_nsec=0;\n");
	print_http("if (!refresh) signal_refresh(true);\n");
	print_http("}\n");

	//page refresh
	print_http("function page_refresh(manual) {");
	print_http("if (!refresh && !manual) return;\n");
	print_http("var xmlHttp = new XMLHttpRequest;");
	print_http("xmlHttp.open('GET','data.html',true);");
	print_http("xmlHttp.onreadystatechange=function(){");
	print_http("if(xmlHttp.readyState==4){");
	print_http("document.getElementById(\"tracker_content\").innerHTML=xmlHttp.responseText;");
	print_http("if (refresh) window.setTimeout(\"page_refresh();\",%d)}};", HTTP_REFRESH_PAGE);
	print_http("xmlHttp.send(null)");
	print_http("}\n");

	//signal refresh
	print_http("function signal_refresh(manual) {");
	print_http("if (!refresh && !manual) return;\n");
	print_http("var xmlHttp = new XMLHttpRequest();");
	print_http("xmlHttp.open('GET', 'data.js?'+signal_type+'&nsec='+last_nsec, true);");
	print_http("xmlHttp.onreadystatechange=function(){");
	print_http("if(xmlHttp.readyState == 4){");
	print_http("eval(xmlHttp.responseText);");
	print_http("if (refresh) window.setTimeout(\"signal_refresh();\",data_refresh_wait)}};");
	print_http("xmlHttp.send(null);");
	print_http("}\n");

	//start refresh
	print_http("function start_refresh() {");
	print_http("if (refresh) return true;\n");
	print_http("refresh=document.getElementById(\"refron\").checked;\n");
	print_http("page_refresh();\n");
	print_http("signal_refresh();\n");
	print_http("return true;\n");
	print_http("}\n");

	print_http("function stop_refresh() {");
	print_http("refresh=false;\n");
	print_http("}\n");

	print_http("start_refresh();\n");
	
	//output last signal in html page itself
	http_jsdata(LastSignal, 0, 1);
	
	print_http("</script> \n\n");



	//Audio
	print_http("<audio id='sound' style='display:none' >\n");
	print_http("<source src='data:audio/mp3;base64,//tQxAAAAAAAAAAAAAAAAAAAAAAASW5mbwAAAA8AAAACAAACcQCAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICA//////////////////////////////////////////////////////////////////8AAAA5TEFNRTMuOTlyAaUAAAAAAAAAABRAJAKZQgAAQAAAAnG7eA+3AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAP/7UMQAAApkZ0p0/AABdp+sqxlAAAADU0vm/o8ViGGgPWPWTs63NsL4JIEgLTmEZlOZzgI6P8WVsLkGMICGXHU3d+3qpSRuNxunp7eff3nXp6e2fLh/nOUd8QHP/8EHFw//8uf4P/4PtgAAAASSSCQqIRAMnzRAcO1Ze9ihSBIQmPYaGRFmE0EgpGAkHA0oMUMQYApsRxoAccKZaL2l/kaLMNkUTguVtxbk09lrKbq0C4/Wf/1Wbf9H+t/63SX8xTu3/+zIKSkkUSiABG03EJL/+1LEBQALnO1ruGmAETipIlcpMABIJAQLR7/SY48HNoaDiBnNHoT4TgfwyIsCMpRiFSQPkWrd1MXDI0WTqKNZIdAlXqUk+kpZkNFfWl/32Lyb3rJ3urfTZTGJu11vlI/8UwfW7KVf0AQCAAAFoPm8BA8AQH4CRQGKm+AJEBiPhsoBiQUD+ILCOhpHP8yNlmSf/tcxMUP/3SSR1//+kksyMnR///RYySdtFbf//+kksySHa///lUxBTUUzLjk5LjNVVVVVVVVVVVVVVVVVVVVVVQ==' type='audio/mp3'>\n");
	print_http("<source src='data:audio/ogg;base64,T2dnUwACAAAAAAAAAABSYAAAAAAAAOQ+p+MBHgF2b3JiaXMAAAAAAUSsAAAAAAAAgLsAAAAAAAC4AU9nZ1MAAAAAAAAAAAAAUmAAAAEAAAASX0hGDy3/////////////////MgN2b3JiaXMdAAAAWGlwaC5PcmcgbGliVm9yYmlzIEkgMjAwNzA2MjIAAAAAAQV2b3JiaXMfQkNWAQAAAQAYY1QpRplS0kqJGXOUMUaZYpJKiaWEFkJInXMUU6k515xrrLm1IIQQGlNQKQWZUo5SaRljkCkFmVIQS0kldBI6J51jEFtJwdaYa4tBthyEDZpSTCnElFKKQggZU4wpxZRSSkIHJXQOOuYcU45KKEG4nHOrtZaWY4updJJK5yRkTEJIKYWSSgelU05CSDWW1lIpHXNSUmpB6CCEEEK2IIQNgtCQVQAAAQDAQBAasgoAUAAAEIqhGIoChIasAgAyAAAEoCiO4iiOIzmSY0kWEBqyCgAAAgAQAADAcBRJkRTJsSRL0ixL00RRVX3VNlVV9nVd13Vd13UgNGQVAAABAEBIp5mlGiDCDGQYCA1ZBQAgAAAARijCEANCQ1YBAAABAABiKDmIJrTmfHOOg2Y5aCrF5nRwItXmSW4q5uacc845J5tzxjjnnHOKcmYxaCa05pxzEoNmKWgmtOacc57E5kFrqrTmnHPGOaeDcUYY55xzmrTmQWo21uaccxa0pjlqLsXmnHMi5eZJbS7V5pxzzjnnnHPOOeecc6oXp3NwTjjnnHOi9uZabkIX55xzPhmne3NCOOecc84555xzzjnnnHOC0JBVAAAQAABBGDaGcacgSJ+jgRhFiGnIpAfdo8MkaAxyCqlHo6ORUuoglFTGSSmdIDRkFQAACAAAIYQUUkghhRRSSCGFFFKIIYYYYsgpp5yCCiqppKKKMsoss8wyyyyzzDLrsLPOOuwwxBBDDK20EktNtdVYY62555xrDtJaaa211koppZR");
	print_http("SSikIDVkFAIAAABAIGWSQQUYhhRRSiCGmnHLKKaigAkJDVgEAgAAAAgAAADzJc0RHdERHdERHdERHdETHczxHlERJlERJtEzL1ExPFVXVlV1b1mXd9m1hF3bd93Xf93Xj14VhWZZlWZZlWZZlWZZlWZZlWYLQkFUAAAgAAIAQQgghhRRSSCGlGGPMMeegk1BCIDRkFQAACAAgAAAAwFEcxXEkR3IkyZIsSZM0S7M8zdM8TfREURRN01RFV3RF3bRF2ZRN13RN2XRVWbVdWbZt2dZtX5Zt3/d93/d93/d93/d93/d1HQgNWQUASAAA6EiOpEiKpEiO4ziSJAGhIasAABkAAAEAKIqjOI7jSJIkSZakSZ7lWaJmaqZneqqoAqEhqwAAQAAAAQAAAAAAKJriKabiKaLiOaIjSqJlWqKmaq4om7Lruq7ruq7ruq7ruq7ruq7ruq7ruq7ruq7ruq7ruq7ruq7rui4QGrIKAJAAANCRHMmRHEmRFEmRHMkBQkNWAQAyAAACAHAMx5AUybEsS9M8zdM8TfRET/RMTxVd0QVCQ1YBAIAAAAIAAAAAADAkw1IsR3M0SZRUS7VUTbVUSxVVT1VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVTVN0zRNIDRkJQAABADAYo3B5SAhJSXl3hDCEJOeMSYhtV4hBJGS3jEGFYOeMqIMct5C4xCDHggNWREARAEAAMYgxxBzyDlHqZMSOeeodJQa5xyljlJnKcWYYs0oldhSrI1zjlJHraOUYiwtdpRSjanGAgAAAhwAAAIshEJDVgQAUQAAhDFIKaQUYow5p5xDjCnnmHOGMeYcc44556B0UirnnHROSsQYc445p5xzUjonlXNOSiehAACAAAcAgAALodCQFQFAnACAQZI8T/I0UZQ0TxRFU3RdUTRd1/I81fRMU1U90VRVU1Vt2VRVWZY8zzQ901RVzzRV1VRVWTZVVZZFVdVt03V123RV3ZZt2/ddWxZ2UVVt3VRd2zdV1");
	print_http("/Zd2fZ9WdZ1Y/I8VfVM03U903Rl1XVtW3VdXfdMU5ZN15Vl03Vt25VlXXdl2fc103Rd01Vl2XRd2XZlV7ddWfZ903WF35VlX1dlWRh2XfeFW9eV5XRd3VdlVzdWWfZ9W9eF4dZ1YZk8T1U903RdzzRdV3VdX1dd19Y105Rl03Vt2VRdWXZl2fddV9Z1zzRl2XRd2zZdV5ZdWfZ9V5Z13XRdX1dlWfhVV/Z1WdeV4dZt4Tdd1/dVWfaFV5Z14dZ1Ybl1XRg+VfV9U3aF4XRl39eF31luXTiW0XV9YZVt4VhlWTl+4ViW3feVZXRdX1ht2RhWWRaGX/id5fZ943h1XRlu3efMuu8Mx++k+8rT1W1jmX3dWWZfd47hGDq/8OOpqq+brisMpywLv+3rxrP7vrKMruv7qiwLvyrbwrHrvvP8vrAso+z6wmrLwrDatjHcvm4sv3Acy2vryjHrvlG2dXxfeArD83R1XXlmXcf2dXTjRzh+ygAAgAEHAIAAE8pAoSErAoA4AQCPJImiZFmiKFmWKIqm6LqiaLqupGmmqWmeaVqaZ5qmaaqyKZquLGmaaVqeZpqap5mmaJqua5qmrIqmKcumasqyaZqy7LqybbuubNuiacqyaZqybJqmLLuyq9uu7Oq6pFmmqXmeaWqeZ5qmasqyaZquq3meanqeaKqeKKqqaqqqraqqLFueZ5qa6KmmJ4qqaqqmrZqqKsumqtqyaaq2bKqqbbuq7Pqybeu6aaqybaqmLZuqatuu7OqyLNu6L2maaWqeZ5qa55mmaZqybJqqK1uep5qeKKqq5ommaqqqLJumqsqW55mqJ4qq6omea5qqKsumatqqaZq2bKqqLZumKsuubfu+68qybqqqbJuqauumasqybMu+78qq7oqmKcumqtqyaaqyLduy78uyrPuiacqyaaqybaqqLsuybRuzbPu6aJqybaqmLZuqKtuyLfu6LNu678qub6uqrOuyLfu67vqucOu6MLyybPuqrPq6K9u6b+sy2/Z9RNOUZVM");
	print_http("1bdtUVVl2Zdn2Zdv2fdE0bVtVVVs2TdW2ZVn2fVm2bWE0Tdk2VVXWTdW0bVmWbWG2ZeF2Zdm3ZVv2ddeVdV/XfePXZd3murLty7Kt+6qr+rbu+8Jw667wCgAAGHAAAAgwoQwUGrISAIgCAACMYYwxCI1SzjkHoVHKOecgZM5BCCGVzDkIIZSSOQehlJQy5yCUklIIoZSUWgshlJRSawUAABQ4AAAE2KApsThAoSErAYBUAACD41iW55miatqyY0meJ4qqqaq27UiW54miaaqqbVueJ4qmqaqu6+ua54miaaqq6+q6aJqmqaqu67q6Lpqiqaqq67qyrpumqqquK7uy7Oumqqqq68quLPvCqrquK8uybevCsKqu68qybNu2b9y6ruu+7/vCka3rui78wjEMRwEA4AkOAEAFNqyOcFI0FlhoyEoAIAMAgDAGIYMQQgYhhJBSSiGllBIAADDgAAAQYEIZKDRkRQAQJwAAGEMppJRSSimllFJKKaWUUkoppZRSSimllFJKKaWUUkgppZRSSimllFJKKaWUUkoppZRSSimllFJKKaWUUkoppZRSSimllFJKKaWUUkqppJRSSimllFJKKaWUUkoppZRSSimllFJKKaWUUkoppZRSSimllFJKKaWUUkoppZRSSimllFJKKaWUUkoppZRSSimllFJKKaWUUkoppZRSSimllFJKKaWUUkoppZRSSimllFJKKaWUUkoppZRSSimllFJKKaWUUkoppZRSSimllFJKKaWUUkoppZRSSimllFJKKaWUUkoppZRSSimllFJKKaWUUkoppZRSSimllFJKKaWUUkoppZRSSimllFJKKaWUUkoppZRSSimllFJKKaWUUkoppZRSSimllFJKKaWUUkoplVJKKaWUUkoppZRSSimllFJKKaWUUkoppZRSSimllFJKKaWUUkoppZRSSimllFJKKaWUUkoppZRSSimllFJKKaWUUkoppZRSSimllFJKKaWUUkoppZRSCgCQinAAkHowoQwUGrISA");
	print_http("EgFAACMUUopxpyDEDHmGGPQSSgpYsw5xhyUklLlHIQQUmktt8o5CCGk1FJtmXNSWosx5hgz56SkFFvNOYdSUoux5ppr7qS0VmuuNedaWqs115xzzbm0FmuuOdecc8sx15xzzjnnGHPOOeecc84FAOA0OACAHtiwOsJJ0VhgoSErAYBUAAACGaUYc8456BBSjDnnHIQQIoUYc845CCFUjDnnHHQQQqgYc8w5CCGEkDnnHIQQQgghcw466CCEEEIHHYQQQgihlM5BCCGEEEooIYQQQgghhBA6CCGEEEIIIYQQQgghhFJKCCGEEEIJoZRQAABggQMAQIANqyOcFI0FFhqyEgAAAgCAHJagUs6EQY5Bjw1BylEzDUJMOdGZYk5qMxVTkDkQnXQSGWpB2V4yCwAAgCAAIMAEEBggKPhCCIgxAABBiMwQCYVVsMCgDBoc5gHAA0SERACQmKBIu7iALgNc0MVdB0IIQhCCWBxAAQk4OOGGJ97whBucoFNU6iAAAAAAAAwA4AEA4KAAIiKaq7C4wMjQ2ODo8AgAAAAAABYA+AAAOD6AiIjmKiwuMDI0Njg6PAIAAAAAAAAAAICAgAAAAAAAQAAAAICAT2dnUwAEJwQAAAAAAABSYAAAAgAAAO9qF9AEICB5ixxjK/rpebtgF47avlPWGWXuJ9lmIjHlrHey7vT7JG0WHGU5/eznX12gOjQoyOqo0ijKbgQ40kGb4+tr2JeKB6l66h46Nvlnv8L2X9Cnv+BX4cc4PR5fvBdn9SbbCcAxADIAd2u4EonPyO0ex3xNPghsbgjl491XdUF4FFSJy/8CKIkHwPB8G7EAOiYwe34eBwBoo4h4/iscna9YWB0WbEfxbkADIAFGGWUSKLh2su1DW6m9wCaQtN4BfsgN9t/f9KZb9Z3BmVhmoQft0Lmnxx4AYClgEiKzQPO/RxPfQ0KcsvYMTc7+haxNqtVyyOisnF94+izPglecUuxAcNwGfNA2h4U/uJowiUbGJIrXlJdjWno2vROSMoG");
	print_http("w9vRFLVCGnBgHFz4AzZ7zv749QNZKnx36bgGcba8OEs6wPPmsXIwONIBfAA==' type='audio/ogg'>\n");
	print_http("</audio>\n");


	//FOOTER
	print_http("</body></html>");


}

//Header
void http_header(int code, const char *content)
{
	if(code == 200)
	{
		print_http("HTTP/1.1 200 OK\r\n");
	}
	else if(code == 404)
	{
		print_http("HTTP/1.1 404 Not Found\r\n");
	}

	print_http("Connection: close\r\n");
	print_http("Server: Blitzortung Tracker\r\n");
	print_http("Cache-Control: no-cache,must-revalidate\r\n");
	print_http("Pragma: no-cache\r\n");

	if(strcmp(content, "html") == 0)
	{
		print_http("Content-Type: text/html; charset=ISO-8859-1\r\n");
	}
	else if(strcmp(content, "js") == 0)
	{
		print_http("Content-Type: text/javascript; charset=ISO-8859-1\r\n");
	}
	else if(strcmp(content, "css") == 0)
	{
		print_http("Content-Type: text/css; charset=ISO-8859-1\r\n");
	}

}


//
//Launches a simple HTTP-Server on specified port
//
void *http_server_thread(void *port)
{
	int sockfd;
	struct sockaddr_in serv_addr;
	struct sockaddr_storage client_addr;
	socklen_t sin_size;
	char s[INET6_ADDRSTRLEN];
	int yes = 1;
	char get[INFO_BUFFER_SIZE];

	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if(sockfd < 0)
	{
		print_error("HTTP-Server: Error building Socket FD");
		exit(EXIT_FAILURE);
	}

	if(setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int)) == -1)
	{
		print_error("HTTP-Server: Error setsockopt");
		exit(EXIT_FAILURE);
	}

	set_socket_timeout(sockfd);

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = INADDR_ANY;
	serv_addr.sin_port = htons(*(int *)port);

	//bind socket
	if(bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
	{
		print_error("HTTP-Server: Error in Binding");
		exit(EXIT_FAILURE);
	}

	//start listen
	if(listen(sockfd, HTTP_SERVER_BACKLOG) == -1)
	{
		print_error("HTTP-Server: Error listen");
		exit(EXIT_FAILURE);
	}


	print_log(LOG_HTTP | LOG_GENERAL, "HTTP-Server started on port %d!\n", *(int *)port);

	//main accept() loop
	while(1)
	{
		sin_size = sizeof(client_addr);
		http_fd = accept(sockfd, (struct sockaddr *)&client_addr, &sin_size);
		if(http_fd == -1)
		{
			if(errno != EAGAIN)
				print_error("HTTP-Server (Accept)");
			continue;
		}

		//read client request
		*http_data_buf = 0;
		if(recv(http_fd, http_data_buf, STRING_BUFFER_SIZE, 0) < 1 ||
		        sscanf(http_data_buf, "GET %99s HTTP", get) != 1)
		{
			close(http_fd);
			continue;
		}

		inet_ntop(client_addr.ss_family,
		          get_in_addr((struct sockaddr *)&client_addr),
		          s, sizeof s);

		print_log(LOG_HTTP, "%s - GET %s\n", s, get);


		if(strncmp(get, "/data.html", 10) == 0)
		{
			http_header(200, "html");
			print_http("\r\n");
			http_page_content();
		}
		else if(strncmp(get, "/data.js", 8) == 0)
		{
			http_header(200, "js");
			print_http("\r\n");

			//Get nsec parameter var
			unsigned int nsec = 0;
			char *p;
			get[INFO_BUFFER_SIZE - 1] = '\0';
			p = strstr(get, "nsec=");
			if(p != NULL)
				sscanf(p + 5, "%d", &nsec);

			if(strstr(get, "?sent") != NULL)         http_jsdata(&LastSentSignal, nsec, 1);
			else if(strstr(get, "?good") != NULL)         http_jsdata(&LastGoodSignal, nsec, 1);
			else if(strstr(get, "?imode1_last") != NULL)  http_jsdata(&LastIMode1Signal, nsec, 1);
			else if(strstr(get, "?imode1_first") != NULL) http_jsdata(&FirstIMode1Signal, nsec, 1);
			else if(strstr(get, "?imode2_last") != NULL)  http_jsdata(&LastIMode2Signal, nsec, 1);
			else if(strstr(get, "?imode2_first") != NULL) http_jsdata(&FirstIMode2Signal, nsec, 1);
			else if(strstr(get, "?filter_min") != NULL)   http_jsdata(&LastMinSignal, nsec, 1);
			else if(strstr(get, "?filter_max") != NULL)   http_jsdata(&LastMaxSignal, nsec, 1);
			else http_jsdata(LastSignal, nsec, BUFFERD_SIGNALS);
		}
		else if(strncmp(get, "/m.css", 6) == 0)
		{
			http_header(200, "css");
			print_http("\r\n");
			http_css_mobile();
		}
		else if(strncmp(get, "/reset.html", 11) == 0)
		{
			init_struct_LT_type();
			LT.reset_time = time(NULL);
			http_header(200, "html");
			print_http("\r\n");
			http_page_main(1);
		}
		else if(strcmp(get, "/") == 0)
		{
			http_header(200, "html");
			print_http("\r\n");
			http_page_main(0);
		}
		else
		{
			http_header(404, "html");
			print_http("\r\n");
			print_http("<h1>404 Page Not Found</h1>");
			print_http("<hr>%s", S.tracker_version_long);
		}

		http_send(http_fd, "");
		close(http_fd);
	}

}



//
// check password for calculation server
//
void compute_server_check_login(struct server_type *s)
{
	char response_buf[HTTP_REQUEST_BUFFER];
	char uri[STRING_BUFFER_SIZE];
	int message_code = 0;

	//no login-data --> login not needed --> success
	if(s->username == NULL && s->password == NULL)
	{
		print_log(LOG_GENERAL, "Server %s, empty login data\n", s->addr);
		s->login_ok = true;
		return;
	}

	sprintf(uri, "%s?username=%s&region=%d&password=%s", PASSWORD_CHECK_URI, s->username, flag.region, s->password);
	http_get_request(response_buf, s->addr, uri);

	if(!sscanf(response_buf, "Message %d", &message_code))
	{
		print_log(LOG_GENERAL, "Server %s: Could not validate login data!\n", s->addr);
		fprintf(stderr, "Could not validate login data for server %s\n", s->addr);
		s->login_ok = true;
	}
	else
	{
		if(message_code == 205)
		{
			print_log(LOG_GENERAL, "Server %s: Password accepted for user %s\n", s->addr, s->username);
			s->login_ok = true;
		}
		else
		{
			print_log(LOG_GENERAL, "Server %s: *** PASSWORD DENIED *** for user %s, message code %d\n", s->addr, s->username, message_code);
			s->login_ok = false;
		}
	}

}

		
/******************************************************************************/
/***** Parameter Update Requests **********************************************/
/******************************************************************************/

//
// Request for parameter update over HTTP
//
void *parameter_updates_thread(void *dummy)
{
	char response_buf[HTTP_REQUEST_BUFFER];
	char uri[STRING_BUFFER_SIZE];
	int i;
	struct param_type P_temp, P_new;
	int parameters_changed = 0, parameters_changed_last = 0;
	int error_code = 0;
	
	
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)


	while(strlen(Server[0].addr) == 0 || !S.board_is_sending)
	{
		sleep(1);
		continue;
	}

	sleep(5);
	print_log(LOG_UPDATE, "Thread started\n");

	while(true)
	{

		//main URI
		sprintf(uri, "%s?username=%s&region=%d&password=%s&server=%s", 
				PARAMETER_REQ_URI, Server[0].username, flag.region, 
				Server[0].password, Server[0].addr);

		//C-type bool variables
		sprintf(uri, "%s&c[acc]=%d&c[flow]=%d&c[time]=%d&c[pos]=%d&c[sum]=%d&c[fault1]=%d&c[fault2]=%d&c[filt]=%d",
		        uri, C.accuracy_ok ? 1 : 0, C.seconds_flow_ok ? 1 : 0, C.time_ok ? 1 : 0, C.pos_ok ? 1 : 0,
		        C.checksum_ok ? 1 : 0, C.faulty ? 1 : 0, C.faulty_rate ? 1 : 0, C.filter_enabled);

		//C-type number variables
		sprintf(uri, "%s&c[nogps]=%d&c[sps]=%d&c[srate]=%.1f&c[smax]=%d&c[sfilt]=%d&c[nonzero]=%d",
		        uri, C.nogps_count, C.signals_per_sec, C.mean_signal_rate_last, C.signals_per_sec_max_last,
		        C.signals_per_sec_count_filtered, C.nonzero_sec);

		//LT-type times
		sprintf(uri, "%s&ltt[s]=%d&ltt[o]=%d&ltt[r]=%d&ltt[u]=%d",
		        uri, (int)LT.start_time, (int)LT.board_last_timeout, (int)LT.reset_time, (int)LT.last_update);

		//LT-type signal numbers
		sprintf(uri, "%s&lts[count]=%d&lts[sent]=%d&lts[filtmin]=%d&lts[filtmax]=%d&lts[fault1]=%d&lts[fault2]=%d",
		        uri, LT.signals, LT.signals_sent, LT.signals_filtered_min, LT.signals_filtered_max,
		        LT.signals_faulty, LT.signals_faulty_rate);

		//LT-type second numbers
		sprintf(uri, "%s&ltsec[acc]=%d&ltsec[time]=%d&ltsec[flow]=%d&ltsec[sum]=%d&ltsec[pos]=%d&ltsec[gps]=%d&ltsec[fault1]=%d&ltsec[fault2]=%d",
		        uri, LT.sec_accuracy_nok, LT.sec_time_nok, LT.sec_seconds_flow_nok, LT.sec_checksum_nok,
		        LT.sec_pos_nok, LT.sec_gps_nok, LT.sec_faulty, LT.sec_faulty_rate);

		//LT-type count numbers
		sprintf(uri, "%s&ltc[acc]=%d&ltc[time]=%d&ltc[flow]=%d&ltc[sum]=%d&ltc[pos]=%d&ltc[gps]=%d&ltc[fault1]=%d&ltc[fault2]=%d",
		        uri, LT.count_accuracy_nok, LT.count_time_nok, LT.count_seconds_flow_nok, LT.count_checksum_nok,
		        LT.count_pos_nok, LT.count_gps_nok, LT.count_faulty, LT.count_faulty_rate);

		//LT-type other
		sprintf(uri, "%s&lto[board_last_to_sec]=%d&lto[packets_sent]=%lld&lto[parm_up]=%d&lto[parm_up_vars]=%d",
		        uri, LT.board_last_timeout_duration, LT.packets_sent,
		        LT.parameter_updates, LT.parameter_updates_var);

		//Others
		sprintf(uri, "%s&fw=%s",
				uri, S.firmware_version);

		print_log(LOG_UPDATE, "GET http://%s?username=%s&region=%d&password=********\n", PARAMETER_REQ_SERVER, Server[0].username, flag.region);
		error_code = http_get_request(response_buf, (char *)PARAMETER_REQ_SERVER, uri);
	

		if(error_code == 0)
		{
			int auth_error = 0;
			bool server_changed = false;
			bool newline = true;
			int len = (int)strlen(response_buf);
			
			parameters_changed = 0;
			i = -1;
			init_struct_P_type(&P_new);
			init_struct_P_type(&P_temp);

			while(i < len - 1)
			{
				i++;

				if(response_buf[i] == '#')
				{
					continue;
				}
				else if (response_buf[i] == '\n')
				{
					newline = true;
					continue;
				}
				else if(newline)
				{
					newline = false;
					
					//
					// Check if login/password is ok
					// We use it here for checking the password for the main server,
					// so we don't need compute_server_check_login() for that
					//
						
					if (sscanf(&response_buf[i], "Error %d", &auth_error))
					{
						//Login/Password is wrong!
						if (auth_error == 200)
						{
							Server[0].login_ok = false;
						}
						
						print_log(LOG_UPDATE | LOG_GENERAL, 
							"Server %s: Authentication failure for user %s, error code %d\n", 
							Server[0].addr, Server[0].username, auth_error);

						break;
					}
					else
					{
						Server[0].login_ok = true;
					}
					
					
					//
					// Search for parameters and assign new values
					//

					if(sscanf(&response_buf[i], "signal_rate_interval=%d", &P_temp.signal_rate_interval))
						P_new.signal_rate_interval = P_temp.signal_rate_interval;

					if(sscanf(&response_buf[i], "param_update_sec=%d", &P_temp.param_update_sec))
						P_new.param_update_sec = P_temp.param_update_sec;

					if(sscanf(&response_buf[i], "smooth_factor=%d", &P_temp.smooth_factor))
						P_new.smooth_factor = P_temp.smooth_factor;

					if(sscanf(&response_buf[i], "pos_precision=%Lf", &P_temp.pos_precision))
						P_new.pos_precision = P_temp.pos_precision;

					if(sscanf(&response_buf[i], "pps_precision=%Lf", &P_temp.pps_precision))
						P_new.pps_precision = P_temp.pps_precision;

					if(sscanf(&response_buf[i], "alt_precision=%Lf", &P_temp.alt_precision))
						P_new.alt_precision = P_temp.alt_precision;

					if(sscanf(&response_buf[i], "max_nonzero_sec=%d", &P_temp.max_nonzero_sec))
						P_new.max_nonzero_sec = P_temp.max_nonzero_sec;

					if(sscanf(&response_buf[i], "max_signal_rate=%f", &P_temp.max_signal_rate))
						P_new.max_signal_rate = P_temp.max_signal_rate;

					if(sscanf(&response_buf[i], "filter_enable=%d", &P_temp.filter_enable))
						P_new.filter_enable = P_temp.filter_enable;

					if(sscanf(&response_buf[i], "min_amplitude=%d", &P_temp.min_amplitude))
						P_new.min_amplitude = P_temp.min_amplitude;

					if(sscanf(&response_buf[i], "max_amplitude=%d", &P_temp.max_amplitude))
						P_new.max_amplitude = P_temp.max_amplitude;

					if(sscanf(&response_buf[i], "max_amplitude_count=%d", &P_temp.max_amplitude_count))
						P_new.max_amplitude_count = P_temp.max_amplitude_count;

					if(sscanf(&response_buf[i], "signal_check_bytes=%d", &P_temp.signal_check_bytes))
						P_new.signal_check_bytes = P_temp.signal_check_bytes;
					
					if(sscanf(&response_buf[i], "send_board_output=%d", &P_temp.send_board_output))
						P_new.send_board_output = P_temp.send_board_output;

					if(sscanf(&response_buf[i], "extra_server_port=%d", &P_temp.extra_server_port))
						P_new.extra_server_port = P_temp.extra_server_port;

					if(sscanf(&response_buf[i], "extra_server_addr=%"STR(INFO_BUFFER_SIZE)"s", P_temp.extra_server_addr))
						strcpy(P_new.extra_server_addr, P_temp.extra_server_addr);

					if(sscanf(&response_buf[i], "extra_server_username=%"STR(INFO_BUFFER_SIZE)"s", P_temp.extra_server_username))
						strcpy(P_new.extra_server_username, P_temp.extra_server_username);

					if(sscanf(&response_buf[i], "extra_server_password=%"STR(INFO_BUFFER_SIZE)"s", P_temp.extra_server_password))
						strcpy(P_new.extra_server_password, P_temp.extra_server_password);

					if(sscanf(&response_buf[i], "message=%"STR(STRING_BUFFER_SIZE)"[^\n]", P_temp.message))
						strcpy(P_new.message, P_temp.message);

				}
			}


			//
			// Check new parameters
			//
			if(0 >= P_new.signal_rate_interval || P_new.signal_rate_interval > 3600)
			{
				print_log(LOG_UPDATE, "signal_rate_interval=%d not possible!\n", P_new.signal_rate_interval);
				P_new.signal_rate_interval = SIGNAL_RATE_INTERVAL;
			}
			else if(P_new.signal_rate_interval != P.signal_rate_interval)
			{
				print_log(LOG_UPDATE, "signal_rate_interval changed %d -> %d\n", P.signal_rate_interval, P_new.signal_rate_interval);
				parameters_changed++;
			}

			if(10 > P_new.param_update_sec || P_new.param_update_sec > 3600 * 24)
			{
				print_log(LOG_UPDATE, "param_update_sec=%d not possible!\n", P_new.param_update_sec);
				P_new.param_update_sec = PARAM_UPDATE_SEC;
			}
			else if(P_new.param_update_sec != P.param_update_sec)
			{
				print_log(LOG_UPDATE, "param_update_sec changed %d -> %d\n", P.param_update_sec, P_new.param_update_sec);
				parameters_changed++;
			}


			if(10 >= P_new.smooth_factor || P_new.smooth_factor > 3600 * 24 * 30)
			{
				print_log(LOG_UPDATE, "smooth_factor=%d not possible!\n", P_new.smooth_factor);
				P_new.smooth_factor = SMOOTH_FACTOR;
			}
			else if(P_new.smooth_factor != P.smooth_factor)
			{
				print_log(LOG_UPDATE, "smooth_factor changed %d -> %d\n", P.smooth_factor, P_new.smooth_factor);
				parameters_changed++;
			}


			if(0.000000000001l >= P_new.pos_precision || P_new.pos_precision > 1.0l)
			{
				print_log(LOG_UPDATE, "pos_precision=%.9Lf not possible!\n", P_new.pos_precision);
				P_new.pos_precision = POS_PRECISION;
			}
			else if(P_new.pos_precision != P.pos_precision)
			{
				print_log(LOG_UPDATE, "pos_precision changed %.9Lf -> %.9Lf\n", P.pos_precision, P_new.pos_precision);
				parameters_changed++;
			}


			if(0.000000000001l >= P_new.pps_precision || P_new.pps_precision > 1.0l)
			{
				print_log(LOG_UPDATE, "pps_precision=%.9Lf not possible!\n", P_new.pps_precision);
				P_new.pps_precision = PPS_PRECISION;
			}
			else if(P_new.pps_precision != P.pps_precision)
			{
				print_log(LOG_UPDATE, "pps_precision changed %.9Lf -> %.9Lf\n", P.pps_precision, P_new.pps_precision);
				parameters_changed++;
			}


			if(0.0001l >= P_new.alt_precision || P_new.alt_precision > 100000.0l)
			{
				print_log(LOG_UPDATE, "alt_precision=%.9Lf not possible!\n", P_new.alt_precision);
				P_new.alt_precision = ALT_PRECISION;
			}
			else if(P_new.alt_precision != P.alt_precision)
			{
				print_log(LOG_UPDATE, "alt_precision changed %.9Lf -> %.9Lf\n", P.alt_precision, P_new.alt_precision);
				parameters_changed++;
			}


			if(-1 > P_new.max_nonzero_sec || P_new.max_nonzero_sec > 3600)
			{
				print_log(LOG_UPDATE, "max_nonzero_sec=%d not possible!\n", P_new.max_nonzero_sec);
				P_new.max_nonzero_sec = MAX_NONZERO_SEC;
			}
			else if(P_new.max_nonzero_sec != P.max_nonzero_sec)
			{
				print_log(LOG_UPDATE, "max_nonzero_sec changed %d -> %d\n", P.max_nonzero_sec, P_new.max_nonzero_sec);
				parameters_changed++;
			}


			if(-1. > P_new.max_signal_rate || P_new.max_signal_rate > 1000.)
			{
				print_log(LOG_UPDATE, "max_signal_rate=%.1f not possible!\n", P_new.max_signal_rate);
				P_new.max_signal_rate = get_max_signal_rate();
			}
			else if(P_new.max_signal_rate != P.max_signal_rate)
			{
				print_log(LOG_UPDATE, "max_signal_rate changed %.1f -> %.1f\n", P.max_signal_rate, P_new.max_signal_rate);
				parameters_changed++;
			}

			if(0 > P_new.filter_enable || P_new.filter_enable > 2)
			{
				print_log(LOG_UPDATE, "filter_enable=%d not possible!\n", P_new.filter_enable);
				P_new.filter_enable = FILTER_ENABLE;
			}
			else if(P_new.filter_enable != P.filter_enable)
			{
				print_log(LOG_UPDATE, "filter_enable changed %d -> %d\n", P.filter_enable, P_new.filter_enable);
				parameters_changed++;
			}

			if(0 > P_new.min_amplitude || P_new.min_amplitude > 10000)
			{
				print_log(LOG_UPDATE, "min_amplitude=%d not possible!\n", P_new.min_amplitude);
				P_new.min_amplitude = MIN_AMPLITUDE;
			}
			else if(P_new.min_amplitude != P.min_amplitude)
			{
				print_log(LOG_UPDATE, "min_amplitude changed %d -> %d\n", P.min_amplitude, P_new.min_amplitude);
				parameters_changed++;
			}


			if(0 > P_new.max_amplitude || P_new.max_amplitude > 10000)
			{
				print_log(LOG_UPDATE, "max_amplitude=%d not possible!\n", P_new.max_amplitude);
				P_new.max_amplitude = MAX_AMPLITUDE;
			}
			else if(P_new.max_amplitude != P.max_amplitude)
			{
				print_log(LOG_UPDATE, "max_amplitude changed %d -> %d\n", P.max_amplitude, P_new.max_amplitude);
				parameters_changed++;
			}


			if(0 > P_new.max_amplitude_count || P_new.max_amplitude_count > 10000)
			{
				print_log(LOG_UPDATE, "max_amplitude_count=%d not possible!\n", P_new.max_amplitude_count);
				P_new.max_amplitude_count = MAX_AMPLITUDE_COUNT;
			}
			else if(P_new.max_amplitude_count != P.max_amplitude_count)
			{
				print_log(LOG_UPDATE, "max_amplitude_count changed %d -> %d\n", P.max_amplitude_count, P_new.max_amplitude_count);
				parameters_changed++;
			}


			if(0 > P_new.signal_check_bytes || P_new.signal_check_bytes > 10000)
			{
				print_log(LOG_UPDATE, "signal_check_bytes=%d not possible!\n", P_new.signal_check_bytes);
				P_new.signal_check_bytes = SIGNAL_CHECK_BYTES;
			}
			else if(P_new.signal_check_bytes != P.signal_check_bytes)
			{
				print_log(LOG_UPDATE, "signal_check_bytes changed %d -> %d\n", P.signal_check_bytes, P_new.signal_check_bytes);
				parameters_changed++;
			}

			
			if(0 > P_new.send_board_output || P_new.send_board_output > 65536)
			{
				print_log(LOG_UPDATE, "send_board_output=%d not possible!\n", P_new.send_board_output);
				P_new.send_board_output = 0;
			}
			else if(P_new.send_board_output != P.send_board_output)
			{
				print_log(LOG_UPDATE, "send_board_output changed %d -> %d\n", P.send_board_output, P_new.send_board_output);
			}
			
			/*** Extra Server ***/

			if(0 > P_new.extra_server_port || P_new.extra_server_port > 65536)
			{
				print_log(LOG_UPDATE, "extra_server_port=%d not possible!\n", P_new.extra_server_port);
				P_new.extra_server_port = 0;
			}
			else if(P_new.extra_server_port != P.extra_server_port)
			{
				server_changed = true;
				print_log(LOG_UPDATE, "extra_server_port changed %d -> %d\n", P.extra_server_port, P_new.extra_server_port);
			}


			if(strcmp(P_new.extra_server_addr, P.extra_server_addr) != 0)
			{
				server_changed = true;
				print_log(LOG_UPDATE, "extra_server_addr changed '%s' -> '%s'\n", P.extra_server_addr, P_new.extra_server_addr);
				parameters_changed++;
			}

			if(strcmp(P_new.extra_server_username, P.extra_server_username) != 0)
			{
				server_changed = true;
				print_log(LOG_UPDATE, "extra_server_username changed '%s' -> '%s'\n", P.extra_server_username, P_new.extra_server_username);
				parameters_changed++;
			}

			if(strcmp(P_new.extra_server_password, P.extra_server_password) != 0)
			{
				server_changed = true;
				print_log(LOG_UPDATE, "extra_server_password changed '********' -> '********'\n");
				parameters_changed++;
			}

			/*** Message ***/
			if(strcmp(P_new.message, P.message) != 0)
			{
				print_log(LOG_UPDATE, "message changed '%s' -> '%s'\n", P.message, P_new.message);
				parameters_changed++;
			}


			//
			// Update the parameters
			//
			P = P_new;
			strcpy(P.extra_server_addr, P_new.extra_server_addr);
			strcpy(P.message, P_new.message);
			LT.parameter_updates_var += parameters_changed;


			//
			if(parameters_changed > 0)
			{
				parameters_changed_last = parameters_changed;
			}


			//
			// Connect to Extra Server
			//
			if(server_changed)
			{
				compute_server_close(&Server[2]);

				if(strlen(P.extra_server_addr) > 0 && P.extra_server_port > 0)
				{
					strncpy(Server[2].addr, P.extra_server_addr, INFO_BUFFER_SIZE);
					Server[2].port = P.extra_server_port;
					Server[2].username = P.extra_server_username;
					Server[2].password = P.extra_server_password;
					compute_server_connect(&Server[2], false);
					strcpy(P.message, "");
				}
				else
				{
					Server[2].addr[0] = '\0';
					Server[2].port = 0;
					Server[2].username = Server[0].username;
					Server[2].password = Server[0].password;
				}

			}

			LT.parameter_updates++;
			P.last_update_success = time(NULL);

		}
		else
		{

			print_log(LOG_UPDATE, "Parameter Request Failed, Error Code %d!\n", error_code);

			//
			// Fallback to defaults!
			//
			if(time(NULL) - P.last_update_success > PARAMETER_REQ_TIMEOUT && parameters_changed_last > 0)
			{
				print_log(LOG_GENERAL | LOG_UPDATE, "Failed to get new parameters since %ds! Reset to standard vars.\n", (int)(time(NULL) - P.last_update_success));
				parameters_changed_last = 0;

				//
				// Reset to defaults (save last success)
				//
				init_struct_P_type(&P_new);
				P_new.last_update_success = P.last_update_success;
				P = P_new;
			}


		}

		P.last_update = time(NULL);
		sleep(P.param_update_sec);
	}

	return NULL;
}



/******************************************************************************/
/***** Watchdog ***************************************************************/
/******************************************************************************/



//
// Checks if board sends output, if not prints message
// and reset some vars
//
void watchdog(pthread_t *pmain_thread_id)
{
	time_t now, last_msg, last_pw_check, last_dns_check;
	time_t last_time = 0;
	bool first_start = true;

	//for auto device search
	bool search_device = false;
	DIR *hdir;
	struct dirent *entry;
	char serial_device_last[INFO_BUFFER_SIZE];
	serial_device_last[0] = '\0';

	print_log(LOG_GENERAL, "Watchdog started.\n");

	if(!flag.auto_device)
	{
		//normal behavior
		pthread_create(pmain_thread_id, NULL, main_thread, NULL);

		while(serial.f < 0)
		{
			L.receive_time = time(NULL);
			sleep(1);
		}

		sleep(1);
	}
	else
	{
		L.receive_time = time(NULL) - 2;
		search_device = true;
	}

	last_msg       = time(NULL);
	last_pw_check  = time(NULL);
	last_dns_check = time(NULL);

	while(true)
	{

		now = time(NULL);
		
		//
		// Check if system time has been changed completely
		// If so, we have to adjust the longtime values
		// and don't execute the board timeout checking
		//
		if (now > 0 && last_time > 0 && abs(last_time - now) > 1800)
		{
			print_log(LOG_GENERAL, "System time has been adjusted!\n");
			LT.start_time  += now - last_time;
			L.receive_time += now - last_time;
			L.gps_ok_time  += now - last_time;
			usleep(100000);
			last_time = now;
			continue;
		}
		
		last_time = now;
		

		//
		// Check board timeouts
		//
		if(now - L.receive_time <= 1)
		{

			//Auto device: found correct port!
			if(search_device && !S.board_is_sending)
			{
				print_log(LOG_GENERAL, "Found Board!\n");
				search_device = false;
			}


			if(!first_start && !S.board_is_sending)
			{
				print_log(LOG_GENERAL, "Received messge after board didn't send valid data for %ds...\n", LT.board_last_timeout_duration);
			}
			else if(first_start && !last_S.board_is_sending && !S.board_is_sending)
			{
				print_log(LOG_GENERAL, "First message received from board!\n");
				last_S.board_is_sending = true;
				L.gps_ok_time = time(NULL);
			}
			else if(now - L.gps_ok_time > GPS_TIMEOUT_INIT && S.board_is_sending)
			{
				//GPS is not ok -> close main thread, init gps and start main thread again

				print_log(LOG_GENERAL, "No valid GPS! Initializing GPS...\n");

				if(*pmain_thread_id > 0)
					pthread_cancel(*pmain_thread_id);

				serial_close_all();
				init_gps(serial);
				pthread_create(pmain_thread_id, NULL, main_thread, NULL);
				sleep(GPS_TIMEOUT_INIT);
			}

			S.board_is_sending = true;
			first_start = false;

		}
		else if(search_device && !S.board_is_sending)
		{

			//
			// automatic detection of device
			//

			if(*pmain_thread_id > 0)
			{
				pthread_cancel(*pmain_thread_id);
			}

			serial_close_all();

			bool found = false;
			hdir = opendir("/dev/");

			while((entry = readdir(hdir)))
			{
				if(strncmp(entry->d_name, "ttyS", 4) == 0 || strncmp(entry->d_name, "ttyUSB", 6) == 0)
				{
					found = true;

					if(serial_device_last[0] != '\0')
					{
						if(strcmp(serial_device_last, entry->d_name) == 0)
						{
							//found last device, try next device
							serial_device_last[0] = 0;
						}

						continue;
					}

					sprintf(serial.device, "/dev/%s", entry->d_name);
					strcpy(serial_device_last, entry->d_name);
					print_log(LOG_GENERAL, "Trying to find board on %s \n", serial.device);
					strcpy(P.message, "Trying to find board on ");
					strcat(P.message, serial.device);
					pthread_create(pmain_thread_id, NULL, main_thread, NULL);

					sleep(3);

					break;
				}
			}

			if(!found)
			{
				print_log(LOG_GENERAL, "No serial ttyS* or ttyUSB* device found! Waiting 10 seconds...\n");
				strcpy(P.message, "Did not found any serial device (ttyS* or ttyUSB*)! Please plug in USB...");
				sleep(10);
				strcpy(P.message, "");
			}
		}
		else if(now - L.receive_time >= BOARD_TIMEOUT)
		{
			//
			// major timeout
			//

			if(S.board_is_sending)
			{
				//timeout, after board was working fine -> message only
				//sometimes load on the computer can cause those timeouts, so print message only
				print_log(LOG_GENERAL, "Receiving no valid data from board since %ldsec! This can also happen if you computer is overloaded. Waiting...\n", now - L.receive_time);
				last_msg = time(NULL);
			}
			else if(now - last_msg >= BOARD_TIMEOUT_RESET)
			{

				C.pos_ok = false;
				C.accuracy_ok = false;
				C.time_ok = false;
				C.seconds_flow_ok = false;
				C.checksum_ok = false;

				//timeout was too long -> maybe board was powered off?
				// -> reset Ringbuffer!
				C.ringbuffer_reset = true;
				C.startup_phase = true;

				print_log(LOG_GENERAL, "ERROR: Received no data from board since %ldmin, %ldsec! Check serial connection!\n", (now - L.receive_time) / 60, now - L.receive_time - ((int)((now - L.receive_time) / 60)) * 60);
				last_msg = time(NULL);

				if(flag.auto_device)
					search_device = true;
			}

			if(!first_start)
			{
				LT.board_last_timeout = L.receive_time;
				LT.board_last_timeout_duration = now - L.receive_time;
			}

			S.board_is_sending = false;
		}


		//
		//Check password of main server, so that user can see message if it's wrong
		//
		if(now - last_pw_check >= PASSWORD_CHECK_INTVL && !flag.requests_enabled)
		{
			last_pw_check = time(NULL);
			print_log(LOG_GENERAL, "Checking password for %s:%d...\n", Server[0].addr, Server[0].port);
			compute_server_check_login(&Server[0]);
		}


		//
		//Check for new IP adresses of each computing server
		//
		if(now - last_dns_check >= SERVER_DNS_TIMEOUT)
		{
			last_dns_check = time(NULL);
			compute_server_updateip(&Server[0]);
			compute_server_updateip(&Server[1]);
			compute_server_updateip(&Server[2]);
		}

		usleep(500000);
	}

	return;
}


#endif //End ifndef SIMPLE_TRACKER




/******************************************************************************/
/***** main *******************************************************************/
/******************************************************************************/

int main(int argc, char **argv)
{

	char program_name [STRING_BUFFER_SIZE];
	program_name[0] = 0;
	if(argc > 0)
	{
		strcpy(program_name, argv[0]);
		argc--;
		argv++;
	}

	init_vars();



	//
	// Version information
	//

#ifdef ADDVERSION
	strncpy(S.tracker_version, "XLT&nbsp;"VERSION"&nbsp;"ADDVERSION, INFO_BUFFER_SIZE);
	strncpy(S.tracker_version_long, "Blitzortung.org Linux Tracker XT "VERSION" "ADDVERSION, INFO_BUFFER_SIZE);
#elif __CYGWIN__
	strncpy(S.tracker_version, "XWT&nbsp;"VERSION, INFO_BUFFER_SIZE);
	strncpy(S.tracker_version_long, "Blitzortung.org Windows Tracker XT "VERSION" (Cygwin)", INFO_BUFFER_SIZE);
#else
	strncpy(S.tracker_version, "XLT&nbsp;"VERSION, INFO_BUFFER_SIZE);
	strncpy(S.tracker_version_long, "Blitzortung.org Linux Tracker XT "VERSION, INFO_BUFFER_SIZE);
#endif


	bool flag_found;
	do
	{
		flag_found = false;
		if((argc > 0) && ((strcmp(argv[0], "-bt") == 0) || (strcmp(argv[0], "--baudrate") == 0)))
		{
			bool baudrate_ok = false;
			int i = 0;

			flag_found = true;
			serial.tracker_baudrate = atoi(argv[1]);
			argc -= 2;
			argv += 2;

			for(i = 0; i < BAUDRATES; i++)
			{
				if(serial.tracker_baudrate > 0 && serial.tracker_baudrate == serial.tracker_baudrates[i])
				{
					baudrate_ok = true;
					break;
				}
			}

			if(!baudrate_ok)
			{
				fprintf(stderr, "Baudrate %d not possible with your hardware/driver, use %s!\n", serial.tracker_baudrate, serial.tracker_baudrates_string);
				exit(EXIT_FAILURE);
			}

		}
		if((argc > 0) && ((strcmp(argv[0], "-bg") == 0) || (strcmp(argv[0], "--baudrate_gps") == 0)))
		{
			flag_found = true;
			serial.gps_baudrate = atoi(argv[1]);
			argc -= 2;
			argv += 2;
		}
		if((argc > 0) && ((strcmp(argv[0], "-L") == 0) || (strcmp(argv[0], "--syslog") == 0)))
		{
			flag_found = true;
			flag.syslog = true;
			argc -= 1;
			argv += 1;
		}
		if((argc > 0) && ((strcmp(argv[0], "-ll") == 0) || (strcmp(argv[0], "--log_log") == 0)))
		{
			flag_found = true;
			logfiles.log.name = argv[1];
			argc -= 2;
			argv += 2;
		}
		if((argc > 0) && ((strcmp(argv[0], "-li") == 0) || (strcmp(argv[0], "--log_info") == 0)))
		{
			flag_found = true;
			logfiles.info.name = argv[1];
			argc -= 2;
			argv += 2;
		}
		if((argc > 0) && ((strcmp(argv[0], "-lo") == 0) || (strcmp(argv[0], "--log_output") == 0)))
		{
			flag_found = true;
			logfiles.out.name = argv[1];
			argc -= 2;
			argv += 2;
		}
		if((argc > 0) && ((strcmp(argv[0], "-ls") == 0) || (strcmp(argv[0], "--log_sent") == 0)))
		{
			flag_found = true;
			logfiles.sent.name = argv[1];
			argc -= 2;
			argv += 2;
		}
		if((argc > 0) && ((strcmp(argv[0], "-lh") == 0) || (strcmp(argv[0], "--log_http") == 0)))
		{
			flag_found = true;
			logfiles.http.name = argv[1];
			argc -= 2;
			argv += 2;
		}
		if((argc > 0) && ((strcmp(argv[0], "-lu") == 0) || (strcmp(argv[0], "--log_update") == 0)))
		{
			flag_found = true;
			logfiles.update.name = argv[1];
			argc -= 2;
			argv += 2;
		}
		if((argc > 0) && ((strcmp(argv[0], "-vl") == 0) || (strcmp(argv[0], "--verbose_log") == 0)))
		{
			flag_found = true;
			flag.verbose_log = true;
			argc -= 1;
			argv += 1;
		}
		if((argc > 0) && ((strcmp(argv[0], "-vi") == 0) || (strcmp(argv[0], "--verbose_info") == 0)))
		{
			flag_found = true;
			flag.verbose_info = true;
			argc -= 1;
			argv += 1;
		}
		if((argc > 0) && ((strcmp(argv[0], "-vo") == 0) || (strcmp(argv[0], "--verbose_output") == 0)))
		{
			flag_found = true;
			flag.verbose_out = true;
			argc -= 1;
			argv += 1;
		}
		if((argc > 0) && ((strcmp(argv[0], "-vs") == 0) || (strcmp(argv[0], "--verbose_sent") == 0)))
		{
			flag_found = true;
			flag.verbose_sent = true;
			argc -= 1;
			argv += 1;
		}
		if((argc > 0) && ((strcmp(argv[0], "-vu") == 0) || (strcmp(argv[0], "--verbose_update") == 0)))
		{
			flag_found = true;
			flag.verbose_update = true;
			argc -= 1;
			argv += 1;
		}
		if((argc > 0) && ((strcmp(argv[0], "-e") == 0) || (strcmp(argv[0], "--echo") == 0)))
		{
			flag_found = true;
			serial.echo_device = argv [1];
			argc -= 2;
			argv += 2;
		}
		if((argc > 0) && ((strcmp(argv[0], "-as") == 0) || (strcmp(argv[0], "--add_server") == 0)))
		{
			flag_found = true;
			flag.add_server = true;
			strncpy(Server[1].addr, argv [1], INFO_BUFFER_SIZE);
			argc -= 2;
			argv += 2;
		}
		if((argc > 0) && ((strcmp(argv[0], "-ap") == 0) || (strcmp(argv[0], "--add_port") == 0)))
		{
			flag_found = true;
			Server[1].port = atoi(argv [1]);
			argc -= 2;
			argv += 2;
		}
		if((argc > 0) && ((strcmp(argv[0], "-au") == 0) || (strcmp(argv[0], "--add_username") == 0)))
		{
			flag_found = true;
			Server[1].username = argv[1];
			argc -= 2;
			argv += 2;
		}
		if((argc > 0) && ((strcmp(argv[0], "-aw") == 0) || (strcmp(argv[0], "--add_password") == 0)))
		{
			flag_found = true;
			Server[1].password = argv[1];
			argc -= 2;
			argv += 2;
		}
		if((argc > 0) && ((strcmp(argv[0], "-s") == 0) || (strcmp(argv[0], "--SBAS") == 0)))
		{
			flag_found = true;
			flag.SBAS = true;
			argc -= 1;
			argv += 1;
		}
		if((argc > 0) && ((strcmp(argv[0], "-p") == 0) || (strcmp(argv[0], "--server_port") == 0)))
		{
			flag_found = true;
			flag.http_server_port = atoi(argv[1]);
			argc -= 2;
			argv += 2;
		}
		if((argc > 0) && ((strcmp(argv[0], "-vh") == 0) || (strcmp(argv[0], "--verbose-http") == 0)))
		{
			flag_found = true;
			flag.verbose_http = true;
			argc -= 1;
			argv += 1;
		}
		if((argc > 0) && strcmp(argv[0], "-V") == 0)
		{
			flag_found = true;
			flag.verbose_log = true;
			flag.verbose_update = true;
			flag.verbose_http = true;
			argc -= 1;
			argv += 1;
		}
		if((argc > 0) && strcmp(argv[0], "-VV") == 0)
		{
			flag_found = true;
			flag.verbose_log = true;
			flag.verbose_info = true;
			flag.verbose_out = true;
			flag.verbose_sent = true;
			flag.verbose_update = true;
			flag.verbose_http = true;
			argc -= 1;
			argv += 1;
		}
		if((argc > 0) && ((strcmp(argv[0], "-n") == 0) || (strcmp(argv[0], "--no-requests") == 0)))
		{
			flag_found = true;
			flag.requests_enabled = false;
			argc -= 1;
			argv += 1;
		}
		if((argc > 0) && ((strcmp(argv[0], "-d") == 0) || (strcmp(argv[0], "--disable-watchdog") == 0)))
		{
			flag_found = true;
			flag.watchdog_enabled = false;
			argc--;
			argv++;
		}
		if((argc > 0) && ((strcmp(argv[0], "-h") == 0) || (strcmp(argv[0], "--help") == 0)))
		{
			flag_found = true;
			flag.help = true;
			argc--;
			argv++;
		}
	}
	while(flag_found);


#ifdef SIMPLE_TRACKER
	if((flag.help) || (!((argc == 2) || (argc == 5))))
#else
	if((flag.help) || (!((argc == 2) || (argc == 4) || (argc == 5))))
#endif
	{
		//
		// Version info output
		//
		fprintf(stderr, "\n");
		fprintf(stderr, "****** %s ******\n", S.tracker_version_long);
		fprintf(stderr, "\n");
		//
		// Help
		//
		fprintf(stderr, "Usage: \n");
		fprintf(stderr, "%s [options] gps_type serial_device [username password region]\n", program_name);
		fprintf(stderr, "\n");
		fprintf(stderr, "Main parameters:\n");
		fprintf(stderr, "  gps_type         : gps type (SANAV, Garmin, or SiRF)\n");
		fprintf(stderr, "                     use '-' for no initialization\n");
#ifdef __CYGWIN__
		fprintf(stderr, "  serial_device    : serial device (1 for COM1, 2 for COM2...)\n");
#else
		fprintf(stderr, "  serial_device    : serial device (examples: /dev/ttyS0 or /dev/ttyUSB0)\n");
#endif
		fprintf(stderr, "  username         : username (example: PeterPim) \n");
		fprintf(stderr, "  password         : password (example: xxxxxxxx)\n");
		fprintf(stderr, "  region           : region (1 = Europe, 2 = Oceanien, 3 = USA, 4 = Japan)\n");
		fprintf(stderr, "\n");
		fprintf(stderr, "Common options:\n");
		fprintf(stderr, "  -bt baudrate     : tracker baudrate (alternative: --baudrate)\n");
		fprintf(stderr, "                     possible values:\n");
		fprintf(stderr, "                     %s\n", serial.tracker_baudrates_string);
		fprintf(stderr, "                     default = %d\n", DEFAULT_TRACKER_BAUDRATE);
		fprintf(stderr, "  -bg baudrate     : gps baudrate (alternative: --baudrate_gps)\n");
		fprintf(stderr, "                     possible values: %s (default = %d)\n", serial.gps_baudrates_string, DEFAULT_GPS_BAUDRATE);
		fprintf(stderr, "  -s               : activate SBAS \"WAAS/EGNOS/MSAS\" (alternative: --SBAS)\n");
#ifndef SIMPLE_TRACKER
		fprintf(stderr, "  -p port          : activate HTTP Server on <port> (alternative: --server_port)\n");
		fprintf(stderr, "                     Note: Do not forward this port to the internet unless you\n");
		fprintf(stderr, "                     know what you are doing!\n");
#endif
		fprintf(stderr, "\n");
		fprintf(stderr, "Additional server options:\n");
		fprintf(stderr, "  -as udp_server   : additional UDP server hostname (alternative: --add_server)\n");
		fprintf(stderr, "  -ap udp_port     : port for additional UDP server (alternative: --add_port)\n");
		fprintf(stderr, "  -au username     : additional username (alternative: --add_username)\n");
		fprintf(stderr, "  -aw password     : additional password (alternative: --add_password)\n");
		fprintf(stderr, "\n");
		fprintf(stderr, "Logging options:\n");
		fprintf(stderr, "  -ll file         : write log messages to file (alternative: --log_log)\n");
		fprintf(stderr, "  -li file         : write system information to file (alternative: --log_info)\n");
		fprintf(stderr, "  -lo file         : write board output to file (alternative: --log_output)\n");
		fprintf(stderr, "  -ls file         : write sent information to file (alternative: --log_sent)\n");
#ifndef SIMPLE_TRACKER
		fprintf(stderr, "  -lu file         : write param. updates to file (alternative: --log_update)\n");
		fprintf(stderr, "  -lh file         : write HTTP log to file (alternative: --log_http)\n");
#endif
		fprintf(stderr, "  -L               : write log messages to syslog (altervative: --syslog)\n");
		fprintf(stderr, "\n");
		fprintf(stderr, "Verbose modes, log to stdout:\n");
		fprintf(stderr, "  -vl              : general log messages (alternative: --verbose_log)\n");
		fprintf(stderr, "  -vi              : system information (alternative: --verbose_info)\n");
		fprintf(stderr, "  -vo              : board output (alternative: --verbose_output)\n");
		fprintf(stderr, "  -vs              : sent information (alternative: --verbose_sent)\n");
#ifndef SIMPLE_TRACKER
		fprintf(stderr, "  -vu              : show parameter updates (alternative: --verbose_update)\n");
		fprintf(stderr, "  -vh              : HTTP Server (alternative: --verbose_http)\n");
		fprintf(stderr, "  -V               : same as -vl -vu -vh\n");
#endif
		fprintf(stderr, "  -VV              : enable all verbose modes\n");
		fprintf(stderr, "\n");
		fprintf(stderr, "Other options:\n");
#ifndef SIMPLE_TRACKER
		fprintf(stderr, "  -n               : no auto. parameter requests (alternative: --no-requests)\n");
		fprintf(stderr, "                     otherwise parameter will be requested from URL:\n");
		fprintf(stderr, "                     http://%s%s\n", PARAMETER_REQ_SERVER, PARAMETER_REQ_URI);
		fprintf(stderr, "  -d               : disable watchdog (alternative: --disable-watchdog)\n");
#endif
		fprintf(stderr, "  -e serial_device : serial device for input echo (alternative: --echo)\n");
		fprintf(stderr, "  -h               : print this help text (alternative: --help)\n");
		fprintf(stderr, "\n");
//		fprintf(stderr, "Ask for help at http://forum.blitzortung.org \n");
//		fprintf(stderr, "\n");

		exit(EXIT_FAILURE);
	}


	//
	if(logfiles.log.name != NULL)
	{
		logfiles.log.fd = fopen(logfiles.log.name, "w");
		if(logfiles.log.fd < 0)
		{
			print_error("fopen (%s, \"w\")", logfiles.out.name);
			exit(EXIT_FAILURE);
		}
	}
	if(logfiles.info.name != NULL)
	{
		logfiles.info.fd = fopen(logfiles.info.name, "w");
		if(logfiles.info.fd < 0)
		{
			print_error("fopen (%s, \"w\")", logfiles.info.name);
			exit(EXIT_FAILURE);
		}
	}
	if(logfiles.out.name != NULL)
	{
		logfiles.out.fd = fopen(logfiles.out.name, "w");
		if(logfiles.out.fd < 0)
		{
			print_error("fopen (%s, \"w\")", logfiles.out.name);
			exit(EXIT_FAILURE);
		}
	}
	if(logfiles.sent.name != NULL)
	{
		logfiles.sent.fd = fopen(logfiles.sent.name, "w");
		if(logfiles.sent.fd < 0)
		{
			print_error("fopen (%s, \"w\")", logfiles.sent.name);
			exit(EXIT_FAILURE);
		}
	}

#ifndef SIMPLE_TRACKER
	if(logfiles.http.name != NULL)
	{
		logfiles.http.fd = fopen(logfiles.http.name, "w");
		if(logfiles.http.fd < 0)
		{
			print_error("fopen (%s, \"w\")", logfiles.out.name);
			exit(EXIT_FAILURE);
		}
	}
	if(logfiles.update.name != NULL)
	{
		logfiles.update.fd = fopen(logfiles.update.name, "w");
		if(logfiles.update.fd < 0)
		{
			print_error("fopen (%s, \"w\")", logfiles.out.name);
			exit(EXIT_FAILURE);
		}
	}
#endif


	//
	if(argc == 5)
	{
		flag.send_to_blitzortung = true;
		flag.region = atoi(argv[4]);
		strncpy(serial.device, argv[1], INFO_BUFFER_SIZE);
		Server[0].username = argv[2];
		Server[0].password = argv[3];
	}
	else
	{
		flag.send_to_blitzortung = false;
		strncpy(serial.device, argv[1], INFO_BUFFER_SIZE);

		if(serial.echo_device == NULL && logfiles.out.name == NULL && !flag.verbose_out)
		{
			fprintf(stderr, "It doesn't make sense to start the tracker with these options!\n");
			fprintf(stderr, "Either set the login data or define a logging argument to get the messages of your board.\n");
			exit(EXIT_FAILURE);
		}
	}

	//
	serial.gps_type = argv[0];


	if(flag.send_to_blitzortung)
	{
		//Additional Server
		if(Server[1].username == NULL)
			Server[1].username = Server[0].username;

		if(Server[1].password == NULL)
			Server[1].password = Server[0].password;


		if(strcmp(Server[0].password, "xxxxxxxx") == 0 || strcmp(Server[1].password, "xxxxxxxx") == 0)
		{
			fprintf(stderr, "The default password \"xxxxxxxx\" can not be used, please change your password!\n");
			exit(EXIT_FAILURE);
		}
		if((flag.region < 1) || (flag.region > 4))
		{
			fprintf(stderr, "Illegal region (1 = Europe, 2 = Oceanien, 3 = USA, 4 = Japan)!\n");
			exit(EXIT_FAILURE);
		}

	}



	//
	// Everything ok with the command line arguments...
	//////////////////////////////////////////////////////////////////



	//
	// Info message
	//
//	fprintf(stdout, "\n");
//	fprintf(stdout, "*** Please report bugs in our forum at http://forum.blitzortung.org ***\n");
//	fprintf(stdout, "\n");


	//
	// Version info output
	//
	print_log(LOG_GENERAL, "%s\n", S.tracker_version_long);



	// no serial device was given!
	if(argv[1][0] == '-')
	{
		flag.auto_device = true;
		serial.device[0] = 0;

		if(!flag.watchdog_enabled || !flag.send_to_blitzortung)
		{
			fprintf(stderr, "Watchdog disabled or not sending to BLitzortung! Automatic detection of serial device not possible!\n");
			exit(EXIT_FAILURE);
		}

		fprintf(stdout, "\n");
		fprintf(stdout, "*****\n");
		fprintf(stdout, "WARNING: Trying to search for the serial device automatically!\n");
		fprintf(stdout, "This feature is experimental and may interfere with other serial devices!\n");
		fprintf(stdout, "Continuing in 10 seconds. Press Ctrl+C to cancel...\n");
		fprintf(stdout, "*****\n\n");
		sleep(10);
	}



	//
	// Emulate Windows COM port numbering
	//
#ifdef __CYGWIN__
	int comport;
	char win_serial[INFO_BUFFER_SIZE], win_serial_echo[INFO_BUFFER_SIZE];

	comport = atoi(serial.device);
	if(comport > 0)
	{
		snprintf(win_serial, INFO_BUFFER_SIZE, "/dev/ttyS%d", comport - 1);
		strcpy(serial.device, win_serial);
	}

	if(serial.echo_device != NULL)
	{
		comport = atoi(serial.echo_device);
		if(comport > 0)
			snprintf(win_serial_echo, INFO_BUFFER_SIZE, "/dev/ttyS%d", comport - 1);
		serial.echo_device = win_serial_echo;
	}
#endif




	if(flag.send_to_blitzortung)
	{

		// Use a maximum signal rate that is calculated from the baudrate as initial value
		init_struct_P_type(&P);
		print_log(LOG_GENERAL, "Baudrate %d -> set max. signal rate to %.1f signals per sec.\n", serial.tracker_baudrate, get_max_signal_rate());

		//Connect to Main Server
		Server[0].port = server_port[flag.region];
		strncpy(Server[0].addr, server_addr[flag.region], INFO_BUFFER_SIZE);
		strcpy(P.message, "Connect to main server...");
		compute_server_connect(&Server[0], true);
#ifndef SIMPLE_TRACKER
		if (!flag.requests_enabled)
		{
			strcpy(P.message, "Checking login data...");
			compute_server_check_login(&Server[0]);
		}
		strcpy(P.message, "");
#endif

		//Connect to Additional Server
		if(strlen(Server[1].addr) > 0 && Server[1].port > 0)
		{
			strcpy(P.message, "Connect to additional server...");
			compute_server_connect(&Server[1], true);
#ifndef SIMPLE_TRACKER
			compute_server_check_login(&Server[1]);
#endif
		}

	}




	/* -------------------------------------------------------------------------------------------- */
	/*    The main stuff                                                                            */
	/* -------------------------------------------------------------------------------------------- */


#ifdef SIMPLE_TRACKER
	main_thread(NULL);
#else

	//
	//Start threads only when sending to Blitzortung
	//
	if(flag.send_to_blitzortung)
	{
		int c = 0;
		pthread_key_t thread_key;
		pthread_t main_thread_id = 0;
		pthread_t http_server_thread_id = 0;
		pthread_t parameter_updates_thread_id = 0;
		pthread_key_create(&thread_key, NULL);


		//HTTP-Server-Thread
		if(flag.http_server_port > 0 && flag.http_server_port < 65536)
		{
			pthread_create(&http_server_thread_id, NULL, http_server_thread, &flag.http_server_port);
		}

		//Parameter Update Request Thread
		if(flag.requests_enabled)
		{
			pthread_create(&parameter_updates_thread_id, NULL, parameter_updates_thread, &c);
		}


		//different behavior if watchdog is enabled!!!
		if(flag.watchdog_enabled)
		{
			watchdog(&main_thread_id);
		}
		else
		{
			// Watchdog disabled -> tell all that board is sending and return
			S.board_is_sending = true;
			last_S.board_is_sending = true;
			main_thread(NULL);
		}

	}
	else
	{
		main_thread(NULL);
	}
#endif


	serial_close(serial.e);
	serial_close(serial.f);

	compute_server_close(&Server[0]);
	compute_server_close(&Server[1]);
	compute_server_close(&Server[2]);

	exit(EXIT_SUCCESS);
}
