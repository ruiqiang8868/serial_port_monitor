#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <memory.h>
#include <termios.h>
#include "serial_util.h"

#define ARRAY_SIZE(a) (sizeof(a) / sizeof(a[0]))
#define pvtlen 38
#define chlen  403

static void main_loop(int fd)
{
	struct termios newstdtio, oldstdtio;
	fd_set readfds;
	unsigned char c;
	ssize_t rsize;
	
	int firstLine = 1;
	unsigned char buf[1024];
	static int count = 0; // count for pvt and channel frame length

	tcgetattr(STDIN_FILENO, &oldstdtio);
	memcpy(&newstdtio, &oldstdtio, sizeof(struct termios));
	newstdtio.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newstdtio);

	FD_ZERO(&readfds);
	FD_SET(STDIN_FILENO, &readfds);
	FD_SET(fd, &readfds);

	for (;;)
	{
		fd_set rfds = readfds;
		int sel_result;

		sel_result = select(FD_SETSIZE, &rfds, NULL, NULL, NULL);
		if (sel_result < 0)
		{
			perror("select");
			break;
		}
		else if (sel_result == 0)
		{
			/* timeout: no way */
		}
		else if (FD_ISSET(STDIN_FILENO, &rfds))
		{
			rsize = read(STDIN_FILENO, &c, 1);
			if (rsize < 0)
			{
				perror("read");
				break;
			}
			printf("got data on stdin: %c\n", c);
			if (c == 'q')
				break;
			write(fd, &c, 1);
		}
		else if (FD_ISSET(fd, &rfds))
		{
			rsize = read(fd, &c, 1);
			if (rsize < 0)
			{
				perror("read");
				break;
			}
			if ((firstLine == 1) && (count == 0))
			{
				printf("Got data on serial:\n");
				firstLine = 0;
			}

			buf[count] = c; // store char to buffer	
			count++;
			
			if (count == (pvtlen + chlen))
			{
				count = 0;

				for (int i = 0; i < (pvtlen + chlen); i++)
				{
					printf(" %02x", buf[i]);
				}

			}
		}
	}

	tcsetattr(STDIN_FILENO, TCSANOW, &oldstdtio);
} //main_loop


int main(int argc, char **argv)
{
	int fd;
	struct termios oldtio;

	char *devname;

	if (argc == 2)
	{
		devname = argv[1];
	}
	else
	{
		printf("usage: %s <serial device>\n", argv[0]);
		return 1;
	}

	if ((fd = serial_open(devname, &oldtio)) < 0) // 打开串口
	{
		perror(devname);

		return 1;
	}
	else
	{
		serial_setup_tio(fd, 115200, 0, 8, 'N', 1); // 配置串口参数
	}

	main_loop(fd);

	serial_close(fd, &oldtio); // 关闭串口

	return 0;
}
