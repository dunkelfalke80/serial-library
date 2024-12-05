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

#ifndef SERIAL_H_INCLUDED
#define SERIAL_H_INCLUDED

#include <termios.h>
#include <unistd.h>
#include <string>

namespace Tools
{
    class Serial
    {
        public:
        typedef enum
        {
            eReadFull,
            eReadEtx,
        } eReadType;

        Serial(const std::string& uart, int baud);
        ~Serial();
        int OpenPort(int parity = 0);
        void ClosePort(void);
        bool ChangeBaudrate(int baud);
        int Read(char* buf, size_t sz, eReadType type = eReadEtx, int sec = 1, int usec = 0, int retries = 20);
        int ReadByte(bool smallTimeout = false);
        int Write(char* buf, size_t sz = 0);
        int Write(std::string& buf);
        int WriteByte(char b);
        int Select(int sec, int usec = 0);
        inline void Flush(void)
        {
            tcflush(_handle, TCIFLUSH);
        }
        inline int GetHandle(void) const
        {
            return _handle;
        }
        inline const std::string GetUart(void) const
        {
            return _uart;
        }
        inline int GetSpeed(void) const
        {
            return _speed;
        }
        void RTS(bool enable);
        void DTR(bool enable);

        protected:
        bool _ConfigurePort(speed_t baud, int databits, int parity);
        static int _ConvertBaud(int baud);
        void _Flag(int pin, bool enable);
        private:
        static constexpr char _stx = 2;
        static constexpr char _etx = 3;
        int _handle;
        std::string _uart;
        int _speed;
        fd_set _fds;
    };
} // namespace Tools

#endif // SERIAL_H_INCLUDED
