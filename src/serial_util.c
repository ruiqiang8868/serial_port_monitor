/********************************************************************
 * 功能： 串口相关的配置和功能
 * 作者： Ruiqiang-Guo
 * 日期: 2019-05-13 22：21
 * 版本： Version V 0.0.1 
 * 最后修改日期： 19/05/14
 ********************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>     /* File control definitions */
#include <termios.h>   /* POSIX terminal control definitions */
#include "serial_util.h"

/*
#define BAUDRATE B115200

static void serial_setup_tio(int fd)
{
	struct termios newtio;

	newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
	newtio.c_iflag = 0;
	newtio.c_oflag = 0;
	newtio.c_lflag = 0;
	newtio.c_cc[VMIN] = 1;
	newtio.c_cc[VTIME] = 0;
	tcflush(fd, TCIFLUSH); //如果发生数据溢出，只接受数据，但是不进行读操作/
	tcsetattr(fd, TCSANOW, &newtio);
}
*/
    
/******************************************************************* 
* 名称：           serial_setup_tio 
* 功能：           设置串口数据位，停止位和效验位 
* 入口参数：        fd          :串口文件描述符 
*                 bauderate   :串口速度 
*                 c_flow      :数据流控制 
*                 databits    :数据位   取值为 7 或者8 
*                 stopbits    :停止位   取值为 1 或者2 
*                 parity      :效验类型 取值为N,E,O,,S 
* 出口参数：        正确返回为1，错误返回为0
* example: serial_setup_tio(fd,9600,0,8,'N',1)
*******************************************************************/  
int serial_setup_tio(int fd, int bauderate, int c_flow, 
                     int databits, char parity, int stopbits)
{
	int i;
	int speed_arr[] = {B115200, B19200, B9600, B4800, B2400, B1200, B300};
	int name_arr[] = {115200, 19200, 9600, 4800, 2400, 1200, 300};
	
    struct termios options;

    /*获取终端属性
     * tcgetattr(fd,&options)得到与fd指向对象的相关参数，并将它们保存于options,
     * 该函数还可以测试配置是否正确，该串口是否可用等。若调用成功，函数返回值为0，
     * 若调用失败，函数返回值为1.
     */
    if(tcgetattr(fd,&options) < 0)
    {
        perror("tcgetattr error");
        return -1;
    }

    /*设置输入输出波特率，两者保持一致*/
    for(i = 0; i < sizeof(speed_arr) / sizeof(int); i++)
    {
		if(bauderate == name_arr[i])
		{
			cfsetispeed(&options, speed_arr[i]); // 设置输入波特率
			cfsetospeed(&options, speed_arr[i]); // 设置输出波特率
		}
	}
		
    /*
    switch(bauderate)
    {
        case 9600:
            cfsetispeed(&options,B9600);
            cfsetospeed(&options,B9600);
            break;
        case 19200:
            cfsetispeed(&options,B19200);
            cfsetospeed(&options,B19200);
            break;
        case 38400:
            cfsetispeed(&options,B38400);
            cfsetospeed(&options,B38400);
            break;
        case 115200:
            cfsetispeed(&options,B115200);
            cfsetospeed(&options,B115200);
            break;
        default:
            fprintf(stderr,"Unkown bauderate!\n");
            return -1;
    }
	*/
	
    /*设置控制模式*/
    options.c_cflag |= CLOCAL;//保证程序不占用串口
    options.c_cflag |= CREAD; //保证程序可以从串口中读取数据

    /*设置数据流控制*/
    switch(c_flow)
    {
        case 0://不进行流控制
            options.c_cflag &= ~CRTSCTS;
            break;
        case 1://进行硬件流控制
            options.c_cflag |= CRTSCTS;
            break;
        case 2://进行软件流控制
            options.c_cflag |= IXON|IXOFF|IXANY;
            break;
        default:
            fprintf(stderr,"Unkown c_flow!\n");
            return -1;
    }

    /*设置数据位*/
    switch(databits)
    {
        case 5:
            options.c_cflag &= ~CSIZE;//屏蔽其它标志位
            options.c_cflag |= CS5;
            break;
        case 6:
            options.c_cflag &= ~CSIZE;//屏蔽其它标志位
            options.c_cflag |= CS6;
            break;
        case 7:
            options.c_cflag &= ~CSIZE;//屏蔽其它标志位
            options.c_cflag |= CS7;
            break;
        case 8:
            options.c_cflag &= ~CSIZE;//屏蔽其它标志位
            options.c_cflag |= CS8;
            break;
        default:
            fprintf(stderr,"Unkown bits!\n");
            return -1;
    }

    /*设置校验位*/
    switch(parity)
    {
        /*无奇偶校验位*/
        case 'n':
        case 'N':
            options.c_cflag &= ~PARENB;//PARENB：产生奇偶位，执行奇偶校验
            options.c_cflag &= ~INPCK;//INPCK：使奇偶校验起作用
            break;
        /*设为空格,即停止位为2位*/
        case 's':
        case 'S':
            options.c_cflag &= ~PARENB;//PARENB：产生奇偶位，执行奇偶校验
            options.c_cflag &= ~CSTOPB;//CSTOPB：使用两位停止位
            break;
        /*设置奇校验*/
        case 'o':
        case 'O':
            options.c_cflag |= PARENB;//PARENB：产生奇偶位，执行奇偶校验
            options.c_cflag |= PARODD;//PARODD：若设置则为奇校验,否则为偶校验
            options.c_cflag |= INPCK;//INPCK：使奇偶校验起作用
            options.c_cflag |= ISTRIP;//ISTRIP：若设置则有效输入数字被剥离7个字节，否则保留全部8位
            break;
        /*设置偶校验*/
        case 'e':
        case 'E':
            options.c_cflag |= PARENB;//PARENB：产生奇偶位，执行奇偶校验
            options.c_cflag &= ~PARODD;//PARODD：若设置则为奇校验,否则为偶校验
            options.c_cflag |= INPCK;//INPCK：使奇偶校验起作用
            options.c_cflag |= ISTRIP;//ISTRIP：若设置则有效输入数字被剥离7个字节，否则保留全部8位
            break;
        default:
            fprintf(stderr,"Unkown parity!\n");
            return -1;
    }

    /*设置停止位*/
    switch(stopbits)
    {
        case 1:
            options.c_cflag &= ~CSTOPB;//CSTOPB：使用一位停止位
            break;
        case 2:
            options.c_cflag |= CSTOPB;//CSTOPB：使用两位停止位
            break;
        default:
            fprintf(stderr,"Unkown stopbits!\n");
            return -1;
    }

    /*设置输出模式为原始输出*/
    options.c_oflag &= ~OPOST;//OPOST：若设置则按定义的输出处理，否则所有c_oflag失效

    /*设置本地模式为原始模式*/
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    /*
     *ICANON：允许规范模式进行输入处理
     *ECHO：允许输入字符的本地回显
     *ECHOE：在接收EPASE时执行Backspace,Space,Backspace组合
     *ISIG：允许信号
     */

    /*设置等待时间和最小接受字符*/
    options.c_cc[VTIME] = 0;//可以在select中设置
    options.c_cc[VMIN] = 1;//最少读取一个字符

    /*如果发生数据溢出，只接受数据，但是不进行读操作*/
    tcflush(fd,TCIFLUSH);

    /*激活配置(将修改后的termios数据设置到串口中）*/
    if(tcsetattr(fd,TCSANOW,&options) < 0)
    {
        perror("tcsetattr failed");
        return -1;
    }
    /* 注意tcsetattr函数中使用的标志：
     * TCSANOW：立即执行而不等待数据发送或者接受完成。
     * TCSADRAIN：等待所有数据传递完成后执行。
     * TCSAFLUSH：Flush input and output buffers and make the change
     */

    return 0;

}

/******************************************************************* 
* 名   称：        serial_open 
* 功   能：        打开串口 
* 入口参数：        fn          :串口名称，例如RS232转USB，“/dev/ttyUSB0”     
*                 tio_saved   :串口的配置参数 
* 出口参数：        正确返回为1，错误返回为0 
*******************************************************************/ 
int serial_open(const char *fn, struct termios *tio_saved)
{
	int fd;
	
	/* 因为串口和其他设备一样，在类Unix系统中都是以设备文件的形式存在的，所以，
	 * 理所当然得你可以使用open()系统调用/函数来访问它
	 */
	 
	fd = open(fn, O_RDWR | O_NOCTTY); //默认为阻塞读方式
	/* O_NOCTTY - 如果路径名指向终端设备，不要把这个设备用作控制终端.
	 * 对于串口的打开操作，必须使用O_NOCTTY参数，它表示打开的是一个终端设备，
	 * 程序不会成为该端口的控制终端。如果不使用此标志，任务的一个输入都会影响进程。
	 * 如键盘上过来的Ctrl+C中止信号等都将影响进程
	 */
	 
	if (fd < 0)
	{
		/*
		 * Could not open the port.
		 */
		perror("open_port: Unable to open /dev/ttyUSB0 -\n");
		return fd;
	}

	tcgetattr(fd, tio_saved);
	
	//serial_setup_tio(fd);
	//serial_setup_tio(fd, 115200, 0, 8, 'N', 1);
	
	return fd;
}

/******************************************************************* 
* 名   称：        serial_close 
* 功   能：        关闭串口 
* 入口参数：        fd          :文件描述符     
*                 tio_saved   :串口的配置参数 
* 出口参数：        正确返回为1，错误返回为0 
*******************************************************************/ 
void serial_close(int fd, const struct termios *tio_saved)
{
	tcsetattr(fd, TCSANOW, tio_saved);

	/*串口作为文件处理，所以一般的关闭文件函数即可*/
	close(fd);
}

/******************************************************************* 
* 名   称：        serial_receive 
* 功   能：        接收串口数据 
* 入口参数：        fd          :文件描述符     
*                 rcv_buf     :接收串口中数据存入rcv_buf缓冲区中 
*                 data_len    :一帧数据的长度 
* 出口参数：        正确返回为1，错误返回为0 
*******************************************************************/  
int serial_receive(int fd, char *rcv_buf, int data_len)  
{  
	int len,fs_sel;  
    fd_set fs_read;  
     
    struct timeval time;  
     
    FD_ZERO(&fs_read);  
    FD_SET(fd,&fs_read);  
     
    time.tv_sec = 10;  
    time.tv_usec = 0;  
     
    //使用select实现串口的多路通信  
    fs_sel = select(fd+1, &fs_read, NULL, NULL, &time);
    
    //printf("fs_sel = %d\n", fs_sel);
    
    if(fs_sel)  
	{  
		len = read(fd, rcv_buf, data_len);  
		//printf("I am right!(version1.2) len = %d fs_sel = %d\n",len,fs_sel);  
		return len;  
	}  
    else  
	{  
		//printf("Sorry,I am wrong!");  
		return -1;  
	}       
}  

/******************************************************************** 
* 名   称：        serial_send 
* 功   能：        发送数据 
* 入口参数：        fd          :文件描述符     
*                 send_buf    :存放串口发送数据 
*                 data_len    :一帧数据的个数 
* 出口参数：        正确返回为1，错误返回为0 
*******************************************************************/  
int serial_send(int fd, char *send_buf, int data_len)  
{  
    int len = 0;  
     
    len = write(fd,send_buf,data_len);  
    if (len == data_len )  
	{  
		printf("send data is %s\n",send_buf);
		return len;  
	}       
    else     
	{  
                 
		tcflush(fd,TCOFLUSH);  
		return -1;  
	}  
     
}

/*
 * c_cflag用于设置控制参数，除了波特率外还包含以下内容： 
	EXTA         External rate clock
	EXTB         External rate clock
	CSIZE        Bit mask for data bits
	CS5          5个数据位
	CS6          6个数据位
	CS7          7个数据位
	CS8          8个数据位
	CSTOPB       2个停止位（清除该标志表示1个停止位
	PARENB       允许校验位
	PARODD       使用奇校验（清除该标志表示使用偶校验）
	CREAD        Enable receiver
	HUPCL        Hangup (drop DTR) on last close
	CLOCAL       Local line – do not change “owner” of port
	LOBLK        Block job control outpu
 * c_cflag标志可以定义CLOCAL和CREAD，这将确保该程序不被其他端口控制和信号干扰，同时串口驱动将读取进入的数据。CLOCAL和CREAD通常总是被是能的。

	c_lflag用于设置本地模式，决定串口驱动如何处理输入字符，设置内容如下：
	ISIG         Enable SIGINTR, SIGSUSP, SIGDSUSP, and SIGQUIT signals 
	ICANON       Enable canonical input (else raw) 
	XCASE        Map uppercase \lowercase (obsolete) 
	ECHO         Enable echoing of input characters 
	ECHOE        Echo erase character as BS-SP-BS 
	ECHOK        Echo NL after kill character 
	ECHONL       Echo NL 
	NOFLSH       Disable flushing of input buffers after interrupt or quit characters 
	IEXTEN       Enable extended functions 
	ECHOCTL      Echo control characters as ^char and delete as ~? 
	ECHOPRT      Echo erased character as character erased 
	ECHOKE       BS-SP-BS entire line on line kill 
	FLUSHO       Output being flushed 
	PENDIN       Retype pending input at next read or input char 
	TOSTOP       Send SIGTTOU for background output

 * c_iflag用于设置如何处理串口上接收到的数据，包含如下内容：
	INPCK        Enable parity check 
	IGNPAR       Ignore parity errors 
	PARMRK       Mark parity errors 
	ISTRIP       Strip parity bits 
	IXON         Enable software flow control (outgoing) 
	IXOFF        Enable software flow control (incoming) 
	IXANY        Allow any character to start flow again 
	IGNBRK       Ignore break condition 
	BRKINT       Send a SIGINT when a break condition is detected 
	INLCR        Map NL to CR 
	IGNCR        Ignore CR 
	ICRNL        Map CR to NL 
	IUCLC        Map uppercase to lowercase 
	IMAXBEL      Echo BEL on input line too long

 * c_oflag用于设置如何处理输出数据，包含如下内容：
	OPOST        Postprocess output (not set = raw output) 
	OLCUC        Map lowercase to uppercase 
	ONLCR        Map NL to CR-NL 
	OCRNL        Map CR to NL 
	NOCR         No CR output at column 0 
	ONLRET       NL performs CR function 
	OFILL        Use fill characters for delay 
	OFDEL        Fill character is DEL 
	NLDLY        Mask for delay time needed between lines 
	NL0          No delay for NLs 
	NL1          Delay further output after newline for 100 milliseconds 
	CRDLY        Mask for delay time needed to return carriage to left column 
	CR0          No delay for CRs 
	CR1          Delay after CRs depending on current column position 
	CR2          Delay 100 milliseconds after sending CRs 
	CR3          Delay 150 milliseconds after sending CRs 
	TABDLY       Mask for delay time needed after TABs 
	TAB0         No delay for TABs 
	TAB1         Delay after TABs depending on current column position 
	TAB2         Delay 100 milliseconds after sending TABs 
	TAB3         Expand TAB characters to spaces 
	BSDLY        Mask for delay time needed after BSs 
	BS0          No delay for BSs 
	BS1          Delay 50 milliseconds after sending BSs 
	VTDLY        Mask for delay time needed after VTs 
	VT0          No delay for VTs 
	VT1          Delay 2 seconds after sending VTs 
	FFDLY        Mask for delay time needed after FFs 
	FF0          No delay for FFs 
	FF1          Delay 2 seconds after sending FFs

 * c_cc定义了控制字符，包含以下内容：
	VINTR        Interrupt   CTRL-C 
	VQUIT        Quit        CTRL-Z 
	VERASE       Erase       Backspace (BS) 
	VKILL        Kill-line   CTRL-U 
	VEOF         End-of-file CTRL-D 
	VEOL         End-of-line Carriage return (CR) 
	VEOL2        Second      end-of-line Line feed (LF) 
	VMIN         Minimum number of characters to read  
	VSTART       Start flow  CTRL-Q (XON) 
	VSTOP        Stop flow   CTRL-S (XOFF) 
	VTIME        Time to wait for data (tenths of seconds) 
	*/
