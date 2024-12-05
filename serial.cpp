//
// Minimal C++ serial library
//
// MIT No Attribution
//
// Copyright (c) 2018 Roman Galperin
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this
// software and associated documentation files (the "Software"), to deal in the Software
// without restriction, including without limitation the rights to use, copy, modify,
// merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
// PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
// HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
// OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
// SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include <iostream>
#include <chrono>
#include <thread>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include "serial.h"

namespace Tools
{
    Serial::Serial(const std::string& uart, int baud) :
        _handle(-1),
        _uart(uart),
        _speed(baud)
    {
    }

    Serial::~Serial()
    {
        ClosePort();
    }

    int Serial::OpenPort(int parity)
    {
        if (_handle != -1) // port already open
        {
            std::cout << "Closing opened port first\n";
            ClosePort();
        }

        std::cout << "Opening port " << _uart << " at " << _speed << " bps\n";

        // open port
        _handle = open(_uart.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (_handle == -1)
        {
            std::cerr << "Error opening port\n";
            return -1;
        }

        if (!_ConfigurePort(_ConvertBaud(_speed), CS8, parity))
        {
            ClosePort();
            return -1;
        }
        std::cout << "Port " << _uart << " configured successfully, got port handle #" << _handle << '\n';
        return _handle;
    }


    void Serial::ClosePort(void)
    {
        if (_handle == -1) // already closed
            return;

        if ((close(_handle) < 0) && (errno != EBADF))
            std::cerr << "Error closing port\n";
        else
            std::cout << "Port " << _uart << " closed.\n";
        _handle = -1;
    }

    bool Serial::ChangeBaudrate(int baud)
    {
        return _ConfigurePort(_ConvertBaud(baud), CS8, 0);
    }

    int Serial::Read(char* buf, size_t sz, eReadType type, int sec, int usec, int retries)
    {
        int recvbytes = 0;
        char timeoutCounter = 0;

        for (timeoutCounter = 0; timeoutCounter < retries; timeoutCounter++)    // try to read the block
        {
            // sleep before receiving data (so the rs232 buffer is well filled)
            if (type == eReadFull)
                std::this_thread::sleep_for(std::chrono::milliseconds(50));

            int result = Select(sec, usec);
            if (result == -1) // not ready
                return -1;

            if (result == 0) // if there is no data to receive
            {
                //if (retries > 3) std::this_thread::sleep_for(std::chrono::milliseconds(50));
                //else if (retries == 3)
                //	return(recvbytes);
                //continue;
                if (type == eReadFull)
                {
                    if ((recvbytes != 0) && (timeoutCounter > (retries / 2)))
                        return(recvbytes);
                    else
                    {
                        std::this_thread::sleep_for(std::chrono::milliseconds(50));
                        continue;
                    }
                }
            }

            // try to read the whole block at once
            result = read(_handle, (buf + recvbytes), (sz - recvbytes));
            if (result == -1)
            {
                std::cerr << "Error reading data\n";
                return -1;
            }
            recvbytes += result;
            if (recvbytes == 0)
                continue;

            if (static_cast<size_t>(recvbytes) == sz) // block read operation complete?
                return(recvbytes);

            if (type == eReadEtx) // search for etx at the end of the string for a quicker return
            {
                for (auto i = recvbytes; i != 0; i--)
                    if (buf[i] == _etx)
                        return(recvbytes);
            }
        }
        return(recvbytes);
    }


    int Serial::ReadByte(bool smallTimeout)
    {
        char inbuf;
        int result;

        result = (smallTimeout) ? Select(0, 30000) : Select(1);
        if (result < 1)
            return(-1);

        if (read(_handle, &inbuf, 1) > 0)
            return(inbuf);

        return(-1);
    }


    int Serial::Write(char* buf, size_t sz)
    {
        size_t len = ((sz == 0) ? strlen(buf) : sz);
        int result = write(_handle, buf, len);

        if (result == -1)
            std::cerr << "Error writing data\n";

        return(static_cast<size_t>(result) == len ? result : -1);
    }


    int Serial::Write(std::string& buf)
    {
        return(Write(const_cast<char*>(buf.c_str()), buf.length()));
    }


    int Serial::WriteByte(char b)
    {
        int result;
        if ((result = write(_handle, &b, 1)) < 1)
            std::cerr << "Error writing data\n";
        return((result == 1) ? 0 : -1);
    }



    int Serial::Select(int sec, int usec)
    {
        struct timeval tv;

        FD_ZERO(&_fds);
        FD_SET(_handle, &_fds);
        tv.tv_sec = sec;
        tv.tv_usec = usec;

        int result = select(_handle + 1, &_fds, NULL, NULL, &tv);
        if ((result == -1) && (errno != EBADF))
            std::cerr << "Error reading status\n";
        return result;
    }



    bool Serial::_ConfigurePort(speed_t baud, int databits, int parity)
    {
        // port attributes
        struct termios tio;

        // read attributes
        if (tcgetattr(_handle, &tio) < 0)
        {
            std::cerr << "Error setting attributes port\n";
            return false;
        }

        tio.c_oflag = 0;  // no remapping, no delays
        tio.c_lflag = 0;  // no canonical, no echo,...
        tio.c_cflag |= CLOCAL;
        tio.c_cflag |= CREAD;
        tio.c_cc[VMIN] = 0;
        tio.c_cc[VTIME] = 0;

        // set attributes
        if (parity == 0)
        {
            tio.c_cflag |= baud | databits;
            tio.c_iflag = IGNPAR;
        }
        else
        {
            tio.c_iflag &= ~IGNBRK; // disable break processing
            tio.c_cflag |= PARENB;
            tio.c_cflag |= parity;
            tio.c_cflag &= ~CSTOPB;
            tio.c_cflag &= ~CSIZE;
            tio.c_cflag |= CS8;
        }

        // set output speed
        if (cfsetospeed(&tio, baud) < 0)
        {
            std::cerr << "Error setting port speed\n";
            return false;
        }

        // set input speed
        if (cfsetispeed(&tio, baud) < 0)
        {
            std::cerr << "Error setting port speed\n";
            return false;
        }

        // configure port
        tcsetattr(_handle, TCSANOW, &tio);
        tcflush(_handle, TCIOFLUSH);

        return true;
    }



    int Serial::_ConvertBaud(int baud)
    {
        int b;
        switch (baud)
        {
            case 1200:
                b = B1200;
                break;
            case 2400:
                b = B2400;
                break;
            case 4800:
                b = B4800;
                break;
            case 9600:
                b = B9600;
                break;
            case 19200:
                b = B19200;
                break;
            case 38400:
                b = B38400;
                break;
            case 57600:
                b = B57600;
                break;
            default:
                b = B115200;
                break;
        }
        return(b);
    }

    void Serial::_Flag(int pin, bool enable)
    {
        int status;
        ioctl(_handle, TIOCMGET, &status);
        enable ? status |= pin : status &= ~pin;
        ioctl(_handle, TIOCMSET, &status);
    }

    void Serial::RTS(bool enable)
    {
        _Flag(TIOCM_RTS, enable);
    }

    void Serial::DTR(bool enable)
    {
        _Flag(TIOCM_DTR, enable);
    }
}
