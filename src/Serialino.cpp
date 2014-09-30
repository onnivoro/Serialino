/*
 * Serialino.cpp
 *
 *  Created on: Sep 24, 2014
 *      Author: carpediem
 */

#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <cstring>

#include "Serialino.h"

Serialino::Serialino(const char* serial_device)
{
  device_name.append(serial_device);
  serial_state =  reading;
  reader_callback = nullptr;
  cout << device_name << endl;
}

void Serialino::serial_in()
{

  cout << "Opening device: " << device_name << endl;
  /* Open the file descriptor in non-blocking mode */
  int fd = open(device_name.c_str(), O_RDWR | O_NOCTTY);
  cout <<"Opened with file descriptor: %d.\n" << fd << endl;

  /* Set up the control structure */
  struct termios toptions;

  /* get current serial port settings */
  tcgetattr(fd, &toptions);
  /* set 115200 baud both ways */
  cfsetispeed(&toptions, B115200);
  cfsetospeed(&toptions, B115200);

  /* 8 bits, no parity, no stop bits */
  toptions.c_cflag &= ~PARENB;
  toptions.c_cflag &= ~CSTOPB;
  toptions.c_cflag &= ~CSIZE;
  toptions.c_cflag |= CS8;

  /* Canonical mode */
  toptions.c_lflag |= ICANON;
  /* commit the serial port settings */
  tcsetattr(fd, TCSANOW, &toptions);

  /* Flush anything already in the serial buffer */
  tcflush(fd, TCIFLUSH);

  //  Initialize file descriptor sets
  fd_set read_fds, write_fds, except_fds;

  FD_ZERO(&except_fds);
  FD_SET(fd,&except_fds);

  // Set timeout to 1.0 seconds
  struct timeval timeout;
  timeout.tv_sec = 10;
  timeout.tv_usec = 0;

  string output;
  while(true)
  {

    state_t current_serial_state = writing;
    mtx_serial_state.lock();
    current_serial_state = serial_state;
    mtx_serial_state.unlock();

    FD_ZERO(&read_fds);
    FD_ZERO(&write_fds);

    if(current_serial_state == reading)
    {
      FD_SET(fd, &read_fds);
    }
    else
    {
      FD_SET(fd, &write_fds);
    }

    // Wait for input to become ready or until the time out; the first parameter is
    //  more than the largest file descriptor in any of the sets
    int select_status = select(fd + 1, &read_fds, &write_fds, &except_fds, &timeout);
    //cout << "aoh: "<< select_status <<endl;

    if(select_status < 0)
    {
      /* An error ocurred, just print it to stdout */
      cout << "Error on select(): " << strerror(errno) <<endl;
    }
    else
    {
      if(FD_ISSET(fd, &read_fds))
      {
        size_t chars_read = read(fd, &circular_buffer[0], 60);
        if(reader_callback != nullptr)
        {
          (*reader_callback)(circular_buffer, chars_read);
        }
      }
      else if(FD_ISSET(fd, &write_fds))
      {
        cout<<"eccome: "<< input << endl;
        if(input.compare("e")==0)
          break;
        input.clear();
        mtx_serial_state.lock();
        serial_state = reading;
        mtx_serial_state.unlock();
      }
      else if(FD_ISSET(fd,&except_fds))
      {
        //cout<<"eccome_bis"<<endl;
      }
    }
  }
  close(fd);
}

void Serialino::serial_out()
{

  while(true)
  {
    state_t current_serial_state = writing;
    mtx_serial_state.lock();
    current_serial_state = serial_state;
    mtx_serial_state.unlock();
    if(current_serial_state != reading)
      continue;
    getline(cin, input);
    cout<< "letto: " << input << endl;
    bool go_out = false;
    go_out = input.compare("e") == 0;
    mtx_serial_state.lock();
    serial_state = writing;
    mtx_serial_state.unlock();
    if(go_out)
      break;
  }
}

void Serialino::startConnection()
{
  thread t_serial_in(&Serialino::serial_in, this);
  thread t_serial_out(&Serialino::serial_out, this);
  t_serial_in.join();
}


void Serialino::closeConnection()
{

}

void Serialino::setReadingCallback(
    void (*reader_cbk)(const char* buffer, size_t chars_read))
{
  reader_callback = reader_cbk;
}

void Serialino::writeBuffer(const string &buffer)
{

}
