/********************************************************/
/*               Uart Controller v 1.0.                 */
/*   The program configures communication on UART.      */
/*   Reads keystrokes and sends them to the connected   */
/*   device. Send packet size = 1 byte.                 */
/*   Accepts an 8-byte packet in response. The data     */
/*   packet is being parsed. The received information	*/
/*   is visualized and printed.                         */
/*                                                      */
/* by TeeGeRoN, 2021.                                   */
/********************************************************/

/*******************************************/
/*  The program receives 8 bytes of data:  */
/*                                         */
/*    [L1] [L2] [L3] [L4] [(X)(X)(X)(X)]   */
/*                                         */
/*    L1/L2/L3/L4 - the state of 4 leds    */
/*    (1-on/0-off)                         */
/*    XXXX - ADC value from temperature    */
/*    sensor LM335.                        */
/*******************************************/


#include <unistd.h> 
#include <termios.h>
#include <signal.h>
#include <stdlib.h>
#include <ncurses.h>
#include <fcntl.h>   /* File Control Definitions */
#include <sys/ioctl.h>
#include <time.h>  
#include <errno.h>   /* Error number definitions */ 

WINDOW * menuWin;
WINDOW * inpWin;
WINDOW * ledWin;
WINDOW * tempWin;
WINDOW * infoWin;

/* It doesn't matter what the symbol is. 
 * The frame in the program is made by filling the contours 
 * with symbols and filling them and the background with 
 * the same color, which gives the effect of a solid thick border. */
char symBorder = '#'; 

char read_buffer[8];   /* Buffer to store the data received*/
char lastData[8]; 
char write_buffer[1];   /* Buffer to store the data received */
int  bytes_read = 0;    /* Number of bytes read by the read() system call */

struct config {
	const char *tty;
	int baud;
};

static struct config config = {
	.tty = NULL,
	.baud = 0,
};

static int fd; /*File Descriptor*/

/* Prototypes */
static int get_baud(int baud);
int init_uart(const char * device, int baud);
void parse_options(int argc, char **argv);
void sig_winch(int signo);
void show_help(const char *progname);
void opt_fail(const char *err);
int kbhit(void);
void printBorder();
void printDef();
void print_led_state();
void print_temp();
void print_info();
void print_time_date (struct tm* tm_info);
void menuUART();
float calcTempEXT(uint16_t inputADC_EXT);

int main(int argc, char *argv[])
{	
	/* Checking arguments */
	parse_options(argc, argv);
	/* Init uart. Set param`s */
	if (init_uart(config.tty, config.baud) <= 0) {
		printf ("Error open COM port %s\n", config.tty);
		return -1;
	}

	/* Init ncurses */
	initscr(); 	
	/* Init colors */
	if(has_colors() == FALSE)
		{	
			endwin();
			printf("Your terminal does not support color\n");
			exit(1);
		}
	start_color();		
	/* Color presets */
	//init_pair(1, COLOR_RED, COLOR_BLACK);	 // Text red / BG black (for debug)
	init_pair(2, COLOR_BLUE, COLOR_BLUE);	 // Text blue / BG blue (for border)
	init_pair(3, COLOR_WHITE, COLOR_WHITE);  // Text white / BG white (for border)
	init_pair(4, COLOR_BLACK, COLOR_WHITE);  // Text black / BG white (for header and show ON state)
	init_pair(5, COLOR_WHITE, COLOR_BLACK);  // Text white / BG black (disp all info and show OFF state)
	init_pair(10, COLOR_WHITE, COLOR_RED);	 // Text white / BG red (led red ON)
	init_pair(11, COLOR_WHITE, COLOR_GREEN); // Text white / BG green (led green ON)
	init_pair(12, COLOR_WHITE, COLOR_YELLOW);// Text white / BG blue (led orange ON)
	init_pair(13, COLOR_WHITE, COLOR_CYAN);	 // Text white / BG blue (led blue ON)
	init_pair(14, COLOR_BLACK, COLOR_RED);	 // Text black / BG blue (led red OFF)
	init_pair(15, COLOR_BLACK, COLOR_GREEN); // Text black / BG blue (led green OFF)
	init_pair(16, COLOR_BLACK, COLOR_YELLOW);// Text black / BG blue (led orange OFF)
	init_pair(17, COLOR_BLACK, COLOR_CYAN);	 // Text black / BG blue (led blue OFF)

	/* Init windows(height, width, coord_Y, coord_X) */
	inpWin = newwin(10,20,18,0);
	menuWin = newwin(15, 28, 0, 0);
	ledWin = newwin(4, 22, 2, 3);
	tempWin = newwin(1, 22, 7, 3);
	infoWin = newwin(4, 22, 9, 3);

	/* Some settings for keyboard input*/
	signal(SIGWINCH, sig_winch);
	cbreak();
	noecho();
    nodelay(inpWin, TRUE);
    scrollok(inpWin, TRUE);
    isendwin();
	curs_set(0); //hide cursor
	refresh();

	/* The basic logic of the program. 
	 * Exchange of data on UART and display of information.
	 */
	menuUART();
	
	/* Close windows */
	endwin(); 
	/* Return buffer and terminal to default view */
	setvbuf(stdout, NULL, _IOLBF, 0);
  	setvbuf(stderr, NULL, _IONBF, 0);
  	/* Close the serial port */
	close(fd); 
	return 0;
}

void menuUART()
{
	time_t timer;
	struct tm* tm_info;
	/* Print borders and default static text */
	printBorder();
	printDef();
	/* Since the name of the device and its speed does 
	 * not change, it is enough to call the function only once. */
	print_info();
	while (1) 
	{
		char inputKey;
		/* Checks for a character on the keyboard. */
		if (kbhit()) 
		{
	        inputKey = wgetch(inpWin);
			write(fd,&inputKey,1);
	        wrefresh(inpWin);
	        if (inputKey == 'q')
				return;
		}
		/* If there is data in the buffer then read them */
		/* To run the program without 'hanging' on read() */
		int bytes;
		ioctl(fd, FIONREAD, &bytes);
		if (bytes > 0)
			/* Read the data */
			bytes_read = read(fd,&read_buffer,8); 
		for (int i = 0; i < bytes_read; i++)
			lastData[i] = read_buffer[i];
		print_led_state();
		print_temp();
		/* Get time from your PC */
		time(&timer);
		tm_info = localtime(&timer);
		print_time_date(tm_info);

		fflush(stdout);
		usleep(100000);
	}
}
static int get_baud(int baud)
{
	switch (baud) {
	case 9600:
		return B9600;
	case 19200:
		return B19200;
	case 38400:
		return B38400;
	case 57600:
		return B57600;
	case 115200:
		return B115200;
	case 230400:
		return B230400;
	case 460800:
		return B460800;
	case 500000:
		return B500000;
	case 576000:
		return B576000;
	case 921600:
		return B921600;
	case 1000000:
		return B1000000;
	case 1152000:
		return B1152000;
	case 1500000:
		return B1500000;
	case 2000000:
		return B2000000;
	case 2500000:
		return B2500000;
	case 3000000:
		return B3000000;
	case 3500000:
		return B3500000;
	case 4000000:
		return B4000000;
	default: 
		return -1;
	}
}

void show_help(const char *progname)
{
	printf("usage: %s [-h] -d tty  [-b baudrate] \n", progname);
	printf("\n"
	"-d\t\t tty filedevice\n"
	"-b\t\t Baud rate, 115200, etc (115200 is default)\n"
	"-h\t\t This help\n"
	"\n");
}

void opt_fail(const char *err)
{
	fprintf(stderr, "Error: %s\n", err);
	exit(1);
}

void parse_options(int argc, char **argv)
{
	int c;
	if (argc == 1) {
		show_help(argv[0]);
		exit(0);
	}
	while ((c = getopt(argc, argv, "hgrd:a:f:ib:s")) != -1) {
		switch (c) {
		case 'b':
			config.baud = atoi(optarg);
			break;
		case 'd':
			if (optarg != NULL) {
				config.tty = optarg;
			} else {
				opt_fail("Invalid -d argument!\n");
			}
			break;
		case '?':
		case 'h':
		default:
			show_help(argv[0]);
			exit(0);
			break;
		}
	}
}


int init_uart(const char * device, int baud)
{
	/**
	* @brief  inittialize uart
	* @param  device : string value represents com device, examle: "/dev/ttyUSB0"
	* @retval file decriptor of initialized port (<0 if failed)
	*/

	/* Opening the Com Port */
	fd = open(device, O_RDWR | O_NOCTTY);
			   	/* O_RDWR   - Read/Write access to serial port       */
				/* O_NOCTTY - No terminal will control the process   */
	/* Could not open the port */
	if (fd < 0){	
		printf("open_port: Unable to open %s\n", device);
		return fd;
	}
	
	/* Setting the Attributes of the serial port using termios structure */
	/* Create the structure */
	struct termios SerialPortSettings;	
	/* Get the current attributes of the Serial port */
	tcgetattr(fd, &SerialPortSettings);	
	/* Setting the Baud rate */
	if(0 != baud) 
	{
		// converts integer baud to Linux define
		cfsetispeed(&SerialPortSettings,get_baud(baud));	/* Set Read Speed as [baud] */
		cfsetospeed(&SerialPortSettings,get_baud(baud));	/* Set Write Speed as [baud] */
	} else 
	{
		cfsetispeed(&SerialPortSettings,B115200);	/* Set Default Read Speed */
		cfsetospeed(&SerialPortSettings,B115200);	/* Set Default Write Speed */
	}
	/* 8N1 Mode */
	SerialPortSettings.c_cflag &= ~PARENB;   /* Disables the Parity Enable bit(PARENB),So No Parity   */
	SerialPortSettings.c_cflag &= ~CSTOPB;   /* CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit */
	SerialPortSettings.c_cflag &= ~CSIZE;	 /* Clears the mask for setting the data size             */
	SerialPortSettings.c_cflag |=  CS8;      /* Set the data bits = 8                                 */
		
	SerialPortSettings.c_cflag &= ~CRTSCTS;         /* No Hardware flow Control                         */
	SerialPortSettings.c_cflag |= CREAD | CLOCAL;	/* Enable receiver,Ignore Modem Control lines       */ 
		
	SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);	/* Disable XON/XOFF flow control both i/p and o/p */
	SerialPortSettings.c_oflag &= ~OPOST;	/*No Output Processing*/
		
	// Choosing Raw Input
	SerialPortSettings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);	/* Choosing Raw Input */

	/* Setting Time outs */
	SerialPortSettings.c_cc[VMIN] = 4*8; /* Read at least 8 characters */
	SerialPortSettings.c_cc[VTIME] = 10; /* Wait indefinetly */

	/* Set the attributes to the termios structure */
	if((tcsetattr(fd,TCSANOW,&SerialPortSettings)) != 0) 
	    printf("\nERROR in Setting attributes!\n");
	else
        printf("\n  BaudRate = %d \n  StopBits = 1 \n  Parity   = none", baud);		
}

int kbhit(void)
{
    int ch = wgetch(inpWin);

    if (ch != ERR) {
        ungetch(ch);
        return 1;
    } else {
        return 0;
    }
}

void sig_winch(int signo)
{
	struct winsize size;
	ioctl(fileno(stdout), TIOCGWINSZ, (char *) &size);
	resizeterm(size.ws_row, size.ws_col);
}

void printBorder()
{
	/* External border */
	wattron(menuWin, COLOR_PAIR(3));	// White border
	box(menuWin, symBorder, symBorder);

	wmove(menuWin, 0, 0);
	wattron(menuWin, COLOR_PAIR(4) | A_BOLD);
	wprintw(menuWin, "       UART CONTROLLER      ");
	wmove(menuWin, 14, 0);
	wattroff(menuWin, A_BOLD);
	wprintw(menuWin, "      press q  to quit      ");

	/* Initial coords for internal border */
	int startX = 1;
	int startY = 1;
	/* Internal border */
	wattron(menuWin, COLOR_PAIR(2));
	wmove(menuWin, startY + 0, startX + 0);
	whline(menuWin,symBorder, 25); 				// top
	wvline(menuWin,symBorder, 13); 				// lside
	wmove(menuWin, startY + 0, startX + 1);
	wvline(menuWin,symBorder, 13); 				// lside
	wmove(menuWin, startY + 0, startX + 24);
	wvline(menuWin,symBorder, 13); 				// rside
	wmove(menuWin, startY + 0, startX + 25);
	wvline(menuWin,symBorder, 13); 				// rside
	wmove(menuWin, startY + 5, startX + 0);
	whline(menuWin,symBorder, 25); 				// inside
	wmove(menuWin, startY + 7, startX + 0);
	whline(menuWin,symBorder, 25); 				// inside
	wmove(menuWin, startY + 12, startX + 0);
	whline(menuWin,symBorder, 25); 				// bottom
	/* Show changes */
	wrefresh(menuWin);
}

void printDef()
{
	/* Led internal border */
	wattron(ledWin, COLOR_PAIR(2));
	wmove(ledWin, 0, 8);
	wvline(ledWin,symBorder, 5);
	wmove(ledWin, 0, 12);
	wvline(ledWin,symBorder, 5);

	/* Led default text */
	wattron(ledWin, COLOR_PAIR(5));
	wmove(ledWin, 0, 13);
	wprintw(ledWin, " Press 1 ");
	wmove(ledWin, 1, 13);
	wprintw(ledWin, " Press 2 ");
	wmove(ledWin, 2, 13);
	wprintw(ledWin, " Press 3 ");
	wmove(ledWin, 3, 13);
	wprintw(ledWin, " Press 4 ");

	/* Temp default text */
	wattron(tempWin, COLOR_PAIR(5));
	wmove(tempWin, 0, 0);
	wprintw(tempWin, " Temperature: ");

	/* Info default text */
	wattron(infoWin, COLOR_PAIR(5));
	wmove(infoWin, 0, 0);
	wprintw(infoWin, " Device: ");
	wmove(infoWin, 1, 0);
	wprintw(infoWin, " Speed: ");
	wmove(infoWin, 2, 0);
	wprintw(infoWin, " Date: ");
	wmove(infoWin, 3, 0);
	wprintw(infoWin, " Time: ");
	/* Show changes */
	wrefresh(ledWin);
	wrefresh(tempWin);
	wrefresh(infoWin);
}

void print_led_state() 
{
	const char * textLed[] = 
		{
			" Red    ",
			" Green  ",
			" Orange ",
			" Blue   "
		};
	for (int i = 0; i < 4; i++)
	{
		wmove(ledWin, i, 9);
		if (lastData[i] == '1')
		{
			wattron(ledWin, COLOR_PAIR(4));
			wprintw(ledWin, "%s", "ON ");
			wattron(ledWin, COLOR_PAIR(10+i));
		}
		else
		{
			wattron(ledWin, COLOR_PAIR(5));
			wprintw(ledWin, "%s", "OFF");
			wattron(ledWin, COLOR_PAIR(14+i));
		}
		wmove(ledWin, i, 0);
		wprintw(ledWin, "%s", textLed[i]);
		wattroff(ledWin, COLOR_PAIR);
		wrefresh(ledWin);
	}
}

float calcTempEXT(uint16_t inputADC_EXT)
{
	/*
	 * We get the inverted value of the ADC.
	 * (the higher the temperature, the lower the value)
	 * -24 *C = 2.50v
	 *  0  *С = 2.02v(Vo)
	 *  50 *C = 1.02v
	 *  100*C = 0.02v
	 *  => 1*C = 0.02v
	 * voltage = inputADC_EXT*2.98/4096;
	 * where 2.98 is voltage on the sensor;
	 * temp*C = (Vo - voltage) / 0.02
	 */
	return (2.02 - (inputADC_EXT*2.98/(float)4096)) / 0.02;
}

void print_temp()
{
	/* We get ADC value from board from LM335 in char-type,
	 * convert it to int-type and calculate to get real
	 * temperature. */
	int mult = 1000;
	int tempADC = 0;
	wattron(tempWin, COLOR_PAIR(5));
	wmove(tempWin, 0, 14);
	if (bytes_read == 0)
		wprintw(tempWin, "No Data");
	if (bytes_read == 8)
	{
		for (int i = 4; i < 8; i++)
		{
			tempADC += (int)(lastData[i] - '0')*mult;
			mult /= 10;
		}
		wprintw(tempWin, "%.1f *C", calcTempEXT(tempADC));
	}
	wrefresh(tempWin);
}

void print_info()
{
	/* Print info about program settings(name/speed) */
	wattron(infoWin, COLOR_PAIR(5));
	wmove(infoWin, 0, 9);
	wprintw(infoWin, "%s", config.tty);
	wmove(infoWin, 1, 12);
	wprintw(infoWin, "%li", config.baud);
	wrefresh(infoWin);
}

void print_time_date (struct tm* tm_info) 
{
	/* Print time and date from your computer. */
	wattron(infoWin, COLOR_PAIR(5));
	char buffer[12];
	strftime(buffer, 12, "%d.%m.%y", tm_info);
	wmove(infoWin, 2, 11);
	wprintw(infoWin, "%s", buffer);
	strftime(buffer, 10, "%H:%M:%S", tm_info);
	wmove(infoWin, 3, 11);
	wprintw(infoWin, "%s", buffer);
	wrefresh(infoWin);
}




