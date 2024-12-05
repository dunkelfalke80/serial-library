# serial-library
Tiny and simple C++ wrapper for serial port communication in Linux.

## Overview
The library provides a C++ interface for serial port communication on Linux. It is a wrapper around the termios API to simplify the process of reading from and writing to serial ports.
It was originally written for an implementation of the XMODEM protocol, hence some built-in functionality is meant to simplify the protocol implementation and is probably not needed for general use.

## Features
Open and close serial ports.
Read data from serial ports with customizable read methods.
Write data to serial ports using strings or byte buffers.
Change baud rate dynamically at runtime - doesn't work on some hardware.
Control RTS and DTR pins for advanced serial port control.
Flush input buffers to clear any unwanted data.

## Prerequisites
Linux operating systemm, the oldest kernel tested was 2.6.
C++ compiler with C++11 support or higher.

## Class Reference for Tools::Serial Class

### Constructor

    Serial(const std::string& uart, int baud)

Constructs a Serial object with the specified UART port and baud rate.

#### Destructor

    ~Serial()

Cleans up resources and closes the serial port if it's open.

#### Public Methods

    int OpenPort(int parity = 0)

Opens the serial port. Optionally, you can specify the parity.

    void ClosePort()

Closes the serial port.

    bool ChangeBaudrate(int baud)

Changes the baud rate of the serial port on the fly.

    int Read(char* buf, size_t sz, eReadType type = eReadEtx, int sec = 1, int usec = 0, int retries = 20)

Reads data from the serial port into a buffer. Supports different read types and timeouts. See the eReadType enumerator description below.

    int ReadByte(bool smallTimeout = false)

Reads a single byte from the serial port. if smallTimeout is set to true, then the timeout is 30 ms, otherwise 1 s.

    int Write(char* buf, size_t sz = 0)

Writes a buffer to the serial port.

   int Write(std::string& buf)

Writes a string to the serial port.

   int WriteByte(char b)

Writes a single byte to the serial port.

    int Select(int sec, int usec = 0)

Waits for data to be available on the serial port.

    void Flush()

Flushes the input buffer of the serial port.

    void RTS(bool enable)

Sets or clears the RTS (Request To Send) pin.

    void DTR(bool enable)

Sets or clears the DTR (Data Terminal Ready) pin.

#### Inline Methods
    
    int GetHandle() const

Returns the file descriptor of the serial port.

    const std::string GetUart() const

Returns the UART port as a string.

    int GetSpeed() const

Returns the current baud rate.

#### Enumerations

    eReadType

Specifies the read type for the Read method.

- eReadFull: Reads until the buffer is full or a timeout has occured.
- eReadEtx: Reads until an ETX (End of Text) character is encountered.
