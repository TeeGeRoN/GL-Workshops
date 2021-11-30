/********************************************************/
/*          Uart Controller led16 Edition v 1.0.		*/
/*   The program configures communication on UART. 		*/
/*   Reads keystrokes and sends them to the connected 	*/
/*	 device. Send packet size = 1 or 6 byte.			*/
/*   Accepts an 76-byte packet in response. The data 	*/
/* 	 packet is being parsed. The received information	*/
/*   is visualized and printed.							*/
/*														*/
/*	 by TeeGeRoN, 2021.									*/
/********************************************************/

/***************************************************/
/*     The program receives 76 bytes of data:      */
/*  Every 4 bytes is a 4-character variable [xxxx] */
/*                                                 */
/* 	  x16[leds][Frequency][dutyCycle][SleepMode]   */
/*										           */
/*    x16[leds] - duty cycle of 16 leds separately */
/*				(0-4095);			               */
/*    [Frequency] - frequency value from PCA9685   */
/*				(23-1525);                         */
/*	  [dutyCycle] - duty cycle of all leds         */
/*				(0-4095)			               */
/*    [SleepMode] - flag of sleep mode             */
/*     	        (0000 or 0001)                     */
/***************************************************/


#include <unistd.h> 
#include <termios.h>
#include <signal.h>
#include <stdlib.h>
#include <ncurses.h>
#include <fcntl.h>   
#include <sys/ioctl.h>
#include <time.h>  
#include <errno.h>  
#include <string.h>

WINDOW * menuWin;
WINDOW * inpWin;
WINDOW * ledWin;
WINDOW * genWin;
WINDOW * infoWin;

/* It doesn't matter what the symbol is. 
 * The frame in the program is made by filling the contours 
 * with symbols and filling them and the background with 
 * the same color, which gives the effect of a solid thick border. */
char symBorder = '#'; 

char read_buffer[76];   /* Buffer to store the data received*/
char lastData[76]; 		/* Temp array - copy of the read_buffer */
char write_buffer[1];   /* Buffer to store the data received */
int  bytes_read = 0;    /* Number of bytes read by the read() system call */
int tempArray[19];      /* Array with obtained and converted values */


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
void print_gen();
void print_info();
void print_time_date (struct tm* tm_info);
void menuUART();
void waitInput(char inputKey);
void parse_data();
void clearInpWin();
float myRound(float inp);

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
	init_pair(6, COLOR_BLACK, COLOR_GREEN);  // Text white / BG black (disp all info and show OFF state)
	init_pair(7, COLOR_BLACK, COLOR_RED);  // Text white / BG black (disp all info and show OFF state)

	/* Init windows(height, width, coord_Y, coord_X) */
	inpWin = newwin(10, 50, 29, 0);
	menuWin = newwin(29, 50, 0, 0);
	ledWin = newwin(16, 44, 2, 3);
	genWin = newwin(5, 44, 19, 3);
	infoWin = newwin(2, 44, 25, 3);

	/* Some settings for keyboard input*/
	signal(SIGWINCH, sig_winch);
	cbreak();
	noecho();
    nodelay(inpWin, TRUE);
    scrollok(inpWin, TRUE);
    isendwin();
	curs_set(0); //hide cursor
	refresh();
	// Request to get data
	write(fd,".",1);
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
	        if (inputKey == 'q') // quit
				return;
			if (inputKey == '1' || inputKey == '2') // set frequency/duty cycle
				waitInput(inputKey);
			if (inputKey == '!') // clear console
				clearInpWin();
		}
		/* If there is data in the buffer then read them */
		/* To run the program without 'hanging' on read() */
		int bytes;
		ioctl(fd, FIONREAD, &bytes);
		if (bytes > 0)
		{
			/* Read the data */
			bytes_read = read(fd,&read_buffer,sizeof(read_buffer));
			for (int i = 0; i < bytes_read; i++)
			lastData[i] = read_buffer[i];
		}
		parse_data();
		print_led_state();
		print_gen();
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
	SerialPortSettings.c_cc[VMIN] = 4*76; /* Read at least 76 characters */
	SerialPortSettings.c_cc[VTIME] = 10; /* Wait indefinetly */

	/* Set the attributes to the termios structure */
	if((tcsetattr(fd,TCSANOW,&SerialPortSettings)) != 0) 
	    printf("\nERROR in Setting attributes!\n");
	else
        printf("\n  BaudRate = %d \n  StopBits = 1 \n  Parity   = none", baud);		
}

void clearInpWin()
{
	/* Console cleaning function */
	for (int i = 0; i < 10; i++)
	{
		wmove(inpWin, i, 0);
		wclrtoeol(inpWin);
		wrefresh(inpWin);
	}
	wmove(inpWin, 0, 0);
	wrefresh(inpWin);
}
void waitInput(char inputKey)
{
   	/* The function changes the mode of receiving information 
 	 * from the user, waits for the entered value, sends it to 
 	 * the board and returns the push mode. */

	/* Disable one-push mode */
	/* Turn on print mode */
	nocbreak();
	echo();
	clearInpWin();
	if (inputKey == '1')
	{
		wprintw(inpWin, "Write the frequency(23-1525)Hz and press enter: \n");
		wprintw(inpWin, "The program will set the closest possible value for the frequency.\n");
	}
	else
		wprintw(inpWin, "Write the value(0-4095) and press enter: \n");
	wprintw(inpWin, "Use only numbers for input!\n");
	wrefresh(inpWin);
	curs_set(1); //show cursor
	nodelay(inpWin, FALSE);
	scrollok(inpWin, FALSE);
	keypad(inpWin, TRUE);
  	wrefresh(inpWin);
  	// Expect value from the user
  	int inputNum;
	wscanw(inpWin,"%i", &inputNum);
		
  	clearInpWin();

  	if (inputKey == '1')
  	{
  		if (inputNum < 1526 && inputNum >= 23)
		{
			char st[10];
			/* Faced with a bug: the 2nd byte received by the board simply disappears. 
			 * Therefore, I fill the first 2 bytes with unnecessary information (zeros). 
			 * If you know how to fix it - email me ;)
			 */
			sprintf(st,"00%i",inputNum);
			/* Send a symbol to the board that will make it wait for the specified frequency. */
			char buffChar = '&';
  			wprintw(inpWin, "The desired frequency is %i", inputNum);
  			write(fd,&buffChar,1);
  			write(fd,&st,strlen(st));
		}
  		else
  			wprintw(inpWin, "Invalid value, try again!");
  	}
  	else
  	{
  		if (inputNum < 4096 && inputNum >=0)
		{
			char st[10];
			sprintf(st,"00%i",inputNum);
			/* Send a symbol to the board that will make it wait for the specified duty cycle. */
			char buffChar = '$';
  			wprintw(inpWin, "Duty cycle set to %i", inputNum);
  			write(fd,&buffChar,1);
  			write(fd,&st,strlen(st));
		}
  		else
  			wprintw(inpWin, "Invalid value, try again!");
  	}
 
  	/* Return one-push mode */
	cbreak();
	noecho();
    nodelay(inpWin, TRUE);
    scrollok(inpWin, TRUE);
    isendwin();
	curs_set(0); //hide cursor
	keypad(inpWin, FALSE);
	refresh();
	wrefresh(inpWin);
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
	wprintw(menuWin, "                UART CONTROLLER16                ");
	wmove(menuWin, 28, 0);
	wattroff(menuWin, A_BOLD);
	wprintw(menuWin, "  press ! to clear console   |   press q to quit ");

	/* Initial coords for internal border */
	int startX = 1;
	int startY = 1;
	/* Internal border */
	wattron(menuWin, COLOR_PAIR(2));
	wmove(menuWin, startY + 0, startX + 0);
	whline(menuWin,symBorder, 25+23); 				// top
	wvline(menuWin,symBorder, 13+12+2); 				// lside
	wmove(menuWin, startY + 0, startX + 1);
	wvline(menuWin,symBorder, 13+12+2); 				// lside
	wmove(menuWin, startY + 0, startX + 24+22);
	wvline(menuWin,symBorder, 13+12+2); 				// rside
	wmove(menuWin, startY + 0, startX + 25+22);
	wvline(menuWin,symBorder, 13+12+2); 				// rside
	wmove(menuWin, startY + 5+12, startX + 0);
	whline(menuWin,symBorder, 25+23); 				// inside
	wmove(menuWin, startY + 23, startX + 0);
	whline(menuWin,symBorder, 25+23); 				// inside
	wmove(menuWin, startY + 26, startX + 0);
	whline(menuWin,symBorder, 25+23); 				// bottom
	/* Show changes */
	wrefresh(menuWin);
}

void printDef()
{
	/* Led internal border */
	wattron(ledWin, COLOR_PAIR(2));
	wmove(ledWin, 0, 8);
	wvline(ledWin,symBorder, 5+12);
	wmove(ledWin, 0, 12);
	wvline(ledWin,symBorder, 5+12);
	wmove(ledWin, 0, 17);
	wvline(ledWin,symBorder, 5+12);

	/* Info internal border */
	wattron(infoWin, COLOR_PAIR(2));
	wmove(infoWin, 0, 24);
	wvline(infoWin,symBorder, 2);

	/* Gen internal border */
	wattron(genWin, COLOR_PAIR(2));
	wmove(genWin, 0, 13);
	wvline(genWin,symBorder, 3);
	wmove(genWin, 0, 18);
	wvline(genWin,symBorder, 3);
	wmove(genWin, 3, 0);
	whline(genWin,symBorder, 44);
	wmove(genWin, 4, 21);
	whline(genWin,symBorder, 2);


	/* Led default text */
	wattron(ledWin, COLOR_PAIR(5));
	int pos = 13+5;
	wmove(ledWin, 0, pos);
	wprintw(ledWin, " Press: s(+25%) / S(-25%) ");
	wmove(ledWin, 1, pos);
	wprintw(ledWin, " Press: e(+25%) / E(-25%) ");
	wmove(ledWin, 2, pos);
	wprintw(ledWin, " Press: d(+25%) / D(-25%) ");
	wmove(ledWin, 3, pos);
	wprintw(ledWin, " Press: r(+25%) / R(-25%) ");
	wmove(ledWin, 4, pos);
	wprintw(ledWin, " Press: f(+25%) / F(-25%) ");
	wmove(ledWin, 5, pos);
	wprintw(ledWin, " Press: t(+25%) / T(-25%) ");
	wmove(ledWin, 6, pos);
	wprintw(ledWin, " Press: g(+25%) / G(-25%) ");
	wmove(ledWin, 7, pos);
	wprintw(ledWin, " Press: y(+25%) / Y(-25%) ");
	wmove(ledWin, 8, pos);
	wprintw(ledWin, " Press: h(+25%) / H(-25%) ");
	wmove(ledWin, 9, pos);
	wprintw(ledWin, " Press: u(+25%) / U(-25%) ");
	wmove(ledWin, 10, pos);
	wprintw(ledWin, " Press: j(+25%) / J(-25%) ");
	wmove(ledWin, 11, pos);
	wprintw(ledWin, " Press: i(+25%) / I(-25%) ");
	wmove(ledWin, 12, pos);
	wprintw(ledWin, " Press: k(+25%) / K(-25%) ");
	wmove(ledWin, 13, pos);
	wprintw(ledWin, " Press: o(+25%) / O(-25%) ");
	wmove(ledWin, 14, pos);
	wprintw(ledWin, " Press: l(+25%) / L(-25%) ");
	wmove(ledWin, 15, pos);
	wprintw(ledWin, " Press: p(+25%) / P(-25%) ");

	/* Info default text */
	wattron(infoWin, COLOR_PAIR(5));
	wmove(infoWin, 0, 0);
	wprintw(infoWin, "  Device: ");
	wmove(infoWin, 1, 0);
	wprintw(infoWin, "  Speed: ");
	wmove(infoWin, 0, 25);
	wprintw(infoWin, "  Date: ");
	wmove(infoWin, 1, 25);
	wprintw(infoWin, "  Time: ");

	/* Gen default text */
	wattron(genWin, COLOR_PAIR(5));
	wmove(genWin, 0, 0);
	wprintw(genWin, "PWM Frequency");
	wmove(genWin, 1, 0);
	wprintw(genWin, "DC all leds");
	wmove(genWin, 2, 0);
	wprintw(genWin, "Sleep mode");
	wmove(genWin, 0, 19);
	wprintw(genWin, "Press 1 : set frequency");
	wmove(genWin, 1, 19);
	wprintw(genWin, "Press 2 : set duty cycle");
	wmove(genWin, 2, 19);
	wprintw(genWin, "Press : *(enbl) / +(dsbl)");
	wmove(genWin, 4, 0);
	wprintw(genWin, "Press 5 : on/off leds");
	wmove(genWin, 4, 23);
	wprintw(genWin, "Press 0 to reset PCA");

	/* Show changes */
	wrefresh(ledWin);
	wrefresh(genWin);
	wrefresh(infoWin);
}

void parse_data()
{
	/* Every 4 bytes are converted back into an integer variable and written to an array. */
	int count = 0;
	for (int i = 0; i < sizeof(lastData);i += 4)
	{
		int mult = 1000;
		int value = 0;
		for(int j = 0; j < 4;j++)
		{
			value += (int)(lastData[i+j] - '0')*mult;
			mult /= 10;
		}
		tempArray[count++] = value;
	}
}

void print_led_state() 
{

	for (int i = 0; i < 16; i++)
	{
		wmove(ledWin, i, 9);
		if (tempArray[i] == 0)
		{
			wattron(ledWin, COLOR_PAIR(7));
			wprintw(ledWin, "%s", "OFF");
			wattron(ledWin, COLOR_PAIR(5));
		}
		else
		{
			wattron(ledWin, COLOR_PAIR(6));
			wprintw(ledWin, "%s", "ON ");
			wattron(ledWin, COLOR_PAIR(4));
			
		}
		wmove(ledWin, i, 0);
		if (i+1 < 10)
			wprintw(ledWin, " LED %i  ", i+1);
		else
			wprintw(ledWin, " LED %i ", i+1);
		wattroff(ledWin, COLOR_PAIR);
		wattron(ledWin, COLOR_PAIR(5));
		wmove(ledWin, i, 13);
		/* If the data has been read, convert the PWM value to a percentage. */
		if (bytes_read > 0)
		{
			int temp[16];
			temp[i] = myRound(tempArray[i]/(4096/100.0));
			if (temp[i] == 100)
				wprintw(ledWin, "%d%%", temp[i]);
			else if (temp[i] < 10)
				wprintw(ledWin, " %d%% ", temp[i]);
			else
				wprintw(ledWin, " %d%%", temp[i]);
		}
		wrefresh(ledWin);
	}
}

void print_gen()
{
	wattron(genWin, COLOR_PAIR(5));
	if (bytes_read > 0)
	{
		/* Frequency */
		wmove(genWin, 0, 14);
		wprintw(genWin, "%04i", tempArray[16]);
		/* Duty cycle */
		wmove(genWin, 1, 14);
		// The board will return 9999 until the same PWM is set for all LEDs at the same time.
		if (tempArray[17] == 9999)
			wprintw(genWin, "%s", "Diff");
		else
			wprintw(genWin, "%04i", tempArray[17]);

		/* Sleep Mode */
		wmove(genWin, 2, 14);
		if (tempArray[18] == 0)
		{
			wattron(genWin, COLOR_PAIR(6));
			wprintw(genWin, "%s", "OFF ");
		}
		else
		{
			wattron(genWin, COLOR_PAIR(7));
			wprintw(genWin, "%s", " ON ");
			
		}
		wrefresh(genWin);
	}
}


void print_info()
{
	/* Print info about program settings(name/speed) */
	wattron(infoWin, COLOR_PAIR(5));
	wmove(infoWin, 0, 10);
	wprintw(infoWin, "%s", config.tty);
	wmove(infoWin, 1, 13);
	wprintw(infoWin, "%li", config.baud);
	wrefresh(infoWin);
}

void print_time_date (struct tm* tm_info) 
{
	/* Print time and date from your computer. */
	wattron(infoWin, COLOR_PAIR(5));
	char buffer[12];
	strftime(buffer, 12, "%d.%m.%y", tm_info);
	wmove(infoWin, 0, 34);
	wprintw(infoWin, "%s", buffer);
	strftime(buffer, 10, "%H:%M:%S", tm_info);
	wmove(infoWin, 1, 34);
	wprintw(infoWin, "%s", buffer);
	wrefresh(infoWin);
}

float myRound(float inp)
{
	/* Number rounding function */
	int res = 0;
	res += (int)inp;
	if ((inp - res) >= 0.5)
		return res+1;
	else
		return res;
}


