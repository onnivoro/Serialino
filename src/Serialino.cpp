/*
 * Serialino.cpp
 *
 *  Created on: Sep 24, 2014
 *      Author: onnivoro
 */

#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <cstring>

#include "Serialino.h"
using namespace std;

Serialino::Serialino(const char* serial_device)
{
  device_name.append(serial_device);
  serial_state =  reading;
  reader_callback = nullptr;
  input = "";
  new_input = false;
  kill_thread = false;
  t_serial_in = nullptr;
  t_serial_out = nullptr;
}

bool Serialino::get_kill_thread()
{
  bool result = false;
  mtx_kill_thread.lock();
  result = kill_thread;
  mtx_kill_thread.unlock();
  return result;
}
void Serialino::serial_in()
{
  
  cout << "Opening device: " << device_name << endl;
  /* Open the file descriptor in non-blocking mode */
  int fd = open(device_name.c_str(), O_RDWR | O_NOCTTY);
  cout <<"Opened with file descriptor " << fd << "."<< endl;
  
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
  while(!get_kill_thread())
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
        if(mtx_input.try_lock())
        {
          if(new_input)
          {
            write(fd, input.c_str(), input.size());
            input.clear();
            new_input = false;
          }
          mtx_input.unlock();
        }
        mtx_serial_state.lock();
        serial_state = reading;
        mtx_serial_state.unlock();
      }
      else if(FD_ISSET(fd,&except_fds))
      {
      }
    }
  }
  close(fd);
}

void Serialino::serial_out()
{
  
  while(!get_kill_thread())
  {
    state_t current_serial_state = writing;
    mtx_serial_state.lock();
    current_serial_state = serial_state;
    mtx_serial_state.unlock();
    if(current_serial_state != reading)
      continue;
    mtx_serial_state.lock();
    serial_state = writing;
    mtx_serial_state.unlock();
  }
}

void Serialino::startConnection()
{
  t_serial_in = new thread(&Serialino::serial_in, this);
  t_serial_out = new thread(&Serialino::serial_out, this);
}


void Serialino::closeConnection()
{
  mtx_kill_thread.lock();
  kill_thread = true;
  mtx_kill_thread.unlock();
  t_serial_in->join();
  t_serial_out->join();
  delete t_serial_in;
  delete t_serial_out;
  t_serial_in = nullptr;
  t_serial_out = nullptr;
}

void Serialino::setReadingCallback(void (*reader_cbk)(const char* buffer, size_t chars_read))
{
  reader_callback = reader_cbk;
}

void Serialino::writeBuffer(const string &buffer)
{
  mtx_input.lock();
  input = buffer;
  new_input = true;
  mtx_input.unlock();
}
