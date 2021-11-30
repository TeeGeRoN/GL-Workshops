#include "main.h"
#include "lwip.h"
#include "sockets.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/select.h>
#include <unistd.h>
#include <stdbool.h>
#include "PCA9685lib.h"

#define PORTNUM 5678UL
#define PORTNUM2 1234UL
#define PORTNUMPWM 9999UL

#define CMD_BUFFER_MAX_LEN 32U

#if (USE_UDP_SERVER_PRINTF == 1)
#include <stdio.h>
#define UDP_SERVER_PRINTF(...) do { printf("[udp_server.c: %s: %d]: ",__func__, __LINE__);printf(__VA_ARGS__); } while (0)
#else
#define UDP_SERVER_PRINTF(...)
#endif

static struct sockaddr_in serv_addr, client_addr;
static int socket_fd;

static struct sockaddr_in client2_addr;
static int socket2_fd;

static struct sockaddr_in clientPWM_addr;
static int socketPWM_fd;

#define PINn 4
#define G_PIN12                         	GPIO_PIN_12
#define PIN12_GPIO_PORT                  	GPIOD

#define G_PIN13                         	GPIO_PIN_13
#define PIN13_GPIO_PORT                  	GPIOD

#define G_PIN14                        		GPIO_PIN_14
#define PIN14_GPIO_PORT                 	GPIOD

#define G_PIN15                     	    GPIO_PIN_15
#define PIN15_GPIO_PORT                  	GPIOD

typedef enum {
  PIN12 = 0,
  PIN13 = 1,
  PIN14 = 2,
  PIN15 = 3
} PIN_TypeDef;

const uint16_t G_PIN[PINn] =
{
		G_PIN12,
		G_PIN13,
		G_PIN14,
		G_PIN15
};

GPIO_TypeDef* G_PORT[PINn] =
{
		PIN12_GPIO_PORT,
		PIN13_GPIO_PORT,
		PIN14_GPIO_PORT,
		PIN15_GPIO_PORT
};
bool ReadPin(PIN_TypeDef Pin)
{
  return HAL_GPIO_ReadPin(G_PORT[Pin], G_PIN[Pin]);
}

static int udpServerInit(uint16_t portnum)
{
	uint16_t port;
	int fd = socket(AF_INET, SOCK_DGRAM, 0);
	if (fd == -1) {
		UDP_SERVER_PRINTF("socket() error\n");
		return -1;
	}

	port = htons((uint16_t)portnum);

	bzero(&serv_addr, sizeof(serv_addr));

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = INADDR_ANY;
	serv_addr.sin_port = port;

	if(bind(fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr))==-1) {
		UDP_SERVER_PRINTF("bind() error\n");
		close(fd);
		return -1;
	}

	UDP_SERVER_PRINTF("UDP Server is bound to port %d\n", portnum);

	return fd;
}

typedef enum {
	COMMAND_UNKNOWN_ERROR = -1,
	COMMAND_ERR_WRONG_FORMAT = -2,
	COMMAND_ERR_LED_NUMBER = -3,
	COMMAND_ERR_LED_CMD = -4,
	COMMAND_ERR_GPIO_NAME = -5,
	COMMAND_ERR_GPIO_PIN = -6,
	COMMAND_ERR_ARGUMENT = -7,
	COMMAND_ERR_WRONG_CMD = -8,
	COMMAND_ERR_CHANNEL_NUM =-9,
	COMMAND_ERR_VALUE_PWM = -10,
	COMMAND_OK = 0
}command_error_t;

static command_error_t led_command_handler(const uint8_t * buffer, size_t len)
{
	int num;
	char cmd[sizeof("toggle")];
	Led_TypeDef led[4] = {LED3, LED4, LED5, LED6};

	if(buffer == NULL || len == 0 || len > CMD_BUFFER_MAX_LEN)
	{
		return COMMAND_ERR_ARGUMENT;
	}

	if(sscanf((const char *)buffer, "led%d %s", &num, cmd) != 2)
	{
		return COMMAND_ERR_WRONG_FORMAT;
	}
	if (num < 3 || num > 6 )
	{
		return COMMAND_ERR_LED_NUMBER;
	}
	if (strncmp("on", cmd, sizeof(cmd)) == 0)
	{
		BSP_LED_On(led[num - 3]);
	}
	else if (strncmp("off", cmd, sizeof(cmd)) == 0)
	{
		BSP_LED_Off(led[num - 3]);
	}
	else if (strncmp("toggle", cmd, sizeof(cmd)) == 0)
	{
		BSP_LED_Toggle(led[num - 3]);
	}
	else
	{
		return COMMAND_ERR_LED_CMD;
	}

	return COMMAND_OK;
}

/*
 * Parameters:
 * buffer - a pointer to the input buffer
 * len - buffer length
 * state - a pointer to the GPIO status that should to be returned
 * pin - a pointer to the PIN number that should to be returned
 **/
static command_error_t gpio_command_handler(const uint8_t * buffer, size_t len, bool *state, int *pin)
{
		char name;
		int num;

		PIN_TypeDef pins[4] = {PIN12, PIN13, PIN14, PIN15};

		if(buffer == NULL || len == 0 || len > CMD_BUFFER_MAX_LEN)
		{
			return COMMAND_ERR_ARGUMENT;
		}
		if(sscanf((const char *)buffer, "read gpio%c %d", &name, &num) != 2)
		{
			return COMMAND_ERR_WRONG_FORMAT;
		}
		if (num < 12 || num > 15 )
		{
			return COMMAND_ERR_GPIO_PIN;
		}
		if (name != 'd')
		{
			return COMMAND_ERR_GPIO_NAME;
		}

		*state = ReadPin(pins[num - 12]);
		*pin = num;
		return COMMAND_OK;
}

/*
 * Parameters:
 * buffer - a pointer to the input buffer
 * len - buffer length
 * pwm - a pointer to the pwm value that should to be returned
 * channel - a pointer to the channel(led) number that should to be returned
 * command - a pointer to the command(r/w - read/write) that should to be returned
 **/
static command_error_t pwm_command_handler(const uint8_t * buffer, size_t len, int *pwm, int *channel, char *command)
{
		int num;
		int valPWM;
		char cmd[sizeof("write")];

		if(buffer == NULL || len == 0 || len > CMD_BUFFER_MAX_LEN)
		{
			return COMMAND_ERR_ARGUMENT;
		}
		int arg = sscanf((const char *)buffer, "/%s led%d %d", cmd, &num, &valPWM);
		if(arg < 2 || arg > 3)
			return COMMAND_ERR_WRONG_FORMAT;
		if (num < 1 || num > 16 )
			return COMMAND_ERR_CHANNEL_NUM;
		if (strncmp("read", cmd, sizeof(cmd)) == 0)
		{
			*pwm = getPWM(num);
			*channel = num;
			*command = 'r';
		}
		else if (strncmp("write", cmd, sizeof(cmd)) == 0)
		{
			if (valPWM < 0 || valPWM > 4095)
				return COMMAND_ERR_VALUE_PWM;
			setLedPWM(num, valPWM);
			*channel = num;
			*command = 'w';
		}
		else
		{
			return COMMAND_ERR_WRONG_CMD;
		}

		return COMMAND_OK;
}
void StartUdpServerTask(void const * argument)
{
	int addr_len;
	osDelay(5000);// wait 5 sec to init lwip stack

	if((socket_fd = udpServerInit(PORTNUM)) < 0)
	{
		UDP_SERVER_PRINTF("udpServerInit(PORTNUM) error\n");
		return;
	}

	int addr_len_2;
	if((socket2_fd = udpServerInit(PORTNUM2)) < 0)
	{
		UDP_SERVER_PRINTF("udpServerInit(PORTNUM2) error\n");
		return;
	}

	int addr_len_PWM;
	if((socketPWM_fd = udpServerInit(PORTNUMPWM)) < 0)
	{
		UDP_SERVER_PRINTF("udpServerInit(PORTNUMPWM) error\n");
		return;
	}
	for(;;)
	{
		bzero(&client_addr, sizeof(client_addr));
		addr_len = sizeof(client_addr);
		fd_set rfds;
		struct timeval tv;
		int retval;
		FD_ZERO(&rfds);
		FD_SET(socket_fd, &rfds);

		bzero(&client2_addr, sizeof(client2_addr));
		addr_len_2 = sizeof(client2_addr);
		fd_set rfds2;
		int retval2;
		FD_ZERO(&rfds2);
		FD_SET(socket2_fd, &rfds2);

		bzero(&clientPWM_addr, sizeof(clientPWM_addr));
		addr_len_PWM = sizeof(clientPWM_addr);
		fd_set rfdsPWM;
		int retvalPWM;
		FD_ZERO(&rfdsPWM);
		FD_SET(socketPWM_fd, &rfdsPWM);
		/* Wait up to five seconds. */

		tv.tv_sec = 1;
		tv.tv_usec = 0;

		retval = select(FD_SETSIZE, &rfds, NULL, NULL, &tv);
		/* Don't rely on the value of tv now! */

		if (retval == -1)
		{
			close(socket_fd);
			break;
		}
		else if (retval)
		{
			uint8_t buffer[CMD_BUFFER_MAX_LEN];
			const size_t buf_size = sizeof(buffer);
			command_error_t  r;
			ssize_t received;

			if (FD_ISSET(socket_fd, &rfds))
			{
				received = recvfrom(socket_fd, buffer, buf_size, MSG_DONTWAIT, (struct sockaddr *)&client_addr, (socklen_t *)&addr_len);

				if (received > 0)
				{
					if ( (r = led_command_handler(buffer, received)) != COMMAND_OK)
					{
						UDP_SERVER_PRINTF("command_handler() returned error code = %d\n", (int)r);
						if (sendto(socket_fd, "error\n", sizeof("error\n"),  MSG_DONTWAIT, (const struct sockaddr *)&client_addr, addr_len) == -1)
						{
							UDP_SERVER_PRINTF("sendto() returned -1 \n");
						}
					}
					else
					{
						UDP_SERVER_PRINTF("command was handles successfully\n");
						if (sendto(socket_fd, "OK\n", sizeof("OK\n"),  MSG_DONTWAIT, (const struct sockaddr *)&client_addr, addr_len) == -1)
						{
							UDP_SERVER_PRINTF("sendto() returned -1 \n");
						}
					}
				}
			}
		}
		else
		{
			UDP_SERVER_PRINTF("No data on the socket one within five seconds.\n");
		}

		retval2 = select(FD_SETSIZE, &rfds2, NULL, NULL, &tv);
		/* Don't rely on the value of tv now! */

		if (retval2 == -1)
		{
			close(socket2_fd);
			break;
		}
		else if (retval2)
		{
			uint8_t buffer2[CMD_BUFFER_MAX_LEN];
			const size_t buf_size = sizeof(buffer2);
			command_error_t  ret;
			ssize_t received;
			int pin;
			bool state;

			if (FD_ISSET(socket2_fd, &rfds2))
			{
				received = recvfrom(socket2_fd, buffer2, buf_size, MSG_DONTWAIT, (struct sockaddr *)&client2_addr, (socklen_t *)&addr_len_2);

				if (received > 0)
				{
					if ( (ret = gpio_command_handler(buffer2, received, &state, &pin)) != COMMAND_OK)
					{
						UDP_SERVER_PRINTF("command_handler() returned error code = %d\n", (int)ret);
						if (sendto(socket2_fd, "error\n", sizeof("error\n"),  MSG_DONTWAIT, (const struct sockaddr *)&client2_addr, addr_len_2) == -1)
						{
							UDP_SERVER_PRINTF("sendto() returned -1 \n");
						}
					}
					else
					{
						char str[15];
						sprintf(str, "GPIOD.%d=%d\n",pin, state);
						UDP_SERVER_PRINTF("command was handles successfully\n");
						if (sendto(socket2_fd, str, strlen(str),  MSG_DONTWAIT, (const struct sockaddr *)&client2_addr, addr_len_2) == -1)
						{
							UDP_SERVER_PRINTF("sendto() returned -1 \n");
						}
					}
				}
			}
		}
		else
		{
			UDP_SERVER_PRINTF("No data on the socked two within five seconds.\n");
		}

		retvalPWM = select(FD_SETSIZE, &rfdsPWM, NULL, NULL, &tv);
				/* Don't rely on the value of tv now! */

		if (retvalPWM == -1)
		{
			close(socketPWM_fd);
			break;
		}
		else if (retvalPWM)
		{
			uint8_t bufferPWM[CMD_BUFFER_MAX_LEN];
			const size_t buf_size = sizeof(bufferPWM);
			command_error_t  ret;
			ssize_t received;
			int channel;
			int pwm;
			char command;

			if (FD_ISSET(socketPWM_fd, &rfdsPWM))
			{
				received = recvfrom(socketPWM_fd, bufferPWM, buf_size, MSG_DONTWAIT, (struct sockaddr *)&clientPWM_addr, (socklen_t *)&addr_len_PWM);

				if (received > 0)
				{
					if ( (ret = pwm_command_handler(bufferPWM, buf_size, &pwm, &channel, &command)) != COMMAND_OK)
					{
						UDP_SERVER_PRINTF("pwm_command_handler() returned error code = %d\n", (int)ret);
						char str[150];
						sprintf(str, "Error\nUse: /write led[n] [val] or /read led[n]\nWhere:\n\t-[n] is number of led(1-16)\n\t-[val] is pwm value(0-4095)\n", channel, pwm);
						if (sendto(socketPWM_fd, str, sizeof(str),  MSG_DONTWAIT, (const struct sockaddr *)&clientPWM_addr, addr_len_PWM) == -1)
						{
							UDP_SERVER_PRINTF("sendto() returned -1 \n");
						}
					}
					else
					{
						char str[50];
						if (command == 'r')
							sprintf(str, "PWM on led%d = %d\n", channel, pwm);
						else
							sprintf(str, "PWM for led%d successfully installed.\n", channel);
						UDP_SERVER_PRINTF("pwm command was handles successfully\n");
						if (sendto(socketPWM_fd, str, strlen(str),  MSG_DONTWAIT, (const struct sockaddr *)&clientPWM_addr, addr_len_PWM) == -1)
						{
							UDP_SERVER_PRINTF("sendto() returned -1 \n");
						}
					}
				}
			}
		}
		else
		{
			UDP_SERVER_PRINTF("No data on the socked pwm within five seconds.\n");
		}
	}
}
