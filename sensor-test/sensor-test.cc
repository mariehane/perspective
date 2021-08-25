#include <stdio.h>      // standard input / output functions
#include <stdlib.h>
#include <string.h>     // string function definitions
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions
#include <iostream>

using namespace std;

const int BUFFER_SIZE = 1024;

int main(void) {
    /* Open File Descriptor */
    int arduino = open( "/dev/ttyACM0", O_RDWR | O_NONBLOCK | O_NDELAY );

    /* Error Handling */
    if (arduino < 0)
    {
        cout << "Error " << errno << " opening " << "/dev/ttyACM0" << ": " << strerror (errno) << endl;
        exit(errno);
    }

    /* *** Configure Port *** */
    struct termios tty;
    memset (&tty, 0, sizeof tty);

    /* Error Handling */
    if ( tcgetattr ( arduino, &tty ) != 0 )
    {
        cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << endl;
        exit(errno);
    }

    /* Set Baud Rate */
    cfsetospeed (&tty, B9600);
    cfsetispeed (&tty, B9600);

    /* Setting other Port Stuff */
    tty.c_cflag     &=  ~PARENB;   // Make 8n1
    tty.c_cflag     &=  ~CSTOPB;
    tty.c_cflag     &=  ~CSIZE;
    tty.c_cflag     |=  CS8;
    tty.c_cflag     &=  ~CRTSCTS;  // no flow control
    tty.c_lflag     =   0;         // no signaling chars, no echo, no canonical processing
    tty.c_oflag     =   0;         // no remapping, no delays
    tty.c_cc[VMIN]      =   0;     // read doesn't block
    tty.c_cc[VTIME]     =   5;     // 0.5 seconds read timeout

    tty.c_cflag     |=  CREAD | CLOCAL;          // turn on READ & ignore ctrl lines
    tty.c_iflag     &=  ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl
    tty.c_lflag     &=  ~(ICANON | ECHO | ECHOE | ISIG); // make raw
    tty.c_oflag     &=  ~OPOST;                  // make raw

    /* Flush Port, then applies attributes */
    tcflush( arduino, TCIFLUSH );

    if ( tcsetattr ( arduino, TCSANOW, &tty ) != 0)
    {
        cout << "Error " << errno << " from tcsetattr" << endl;
        exit(errno);
    }

    /* *** WRITE *** */
//unsigned char cmd[] = {'I', 'N', 'I', 'T', ' ', '\r', '\0'};
    //int n_written = write( arduino, cmd, sizeof(cmd) -1 );

    /* Get FILE* */
    FILE* arduinofp = fdopen(arduino, "r");

    /* Allocate memory for read buffer */
    char buf [BUFFER_SIZE];
    memset (&buf, '\0', sizeof(buf));

    /* *** READ *** */
	for (int i = 0; i <= 10000; i++) {
		if (fgets( buf, sizeof(buf), arduinofp ) == NULL)
		{
			cout << "Error reading: " << strerror(errno) << endl;
		}

		/* Print what I read... */
		cout << "Read: " << buf << endl;
	}

    close(arduino);
}
