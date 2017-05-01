/*
 * Serialino.h
 *
 *  Created on: Sep 24, 2014
 *      Author: Nicolo' Boscolo
 *
 */

#ifndef SERIALINO_H
#define SERIALINO_H

#ifndef CIRCULAR_SERIAL_BUFFER_SIZE
#define CIRCULAR_SERIAL_BUFFER_SIZE 128
#endif

#include <thread>
#include <mutex>
#include <string>

enum state_t{
  reading,
  writing,
  closing
};

class Serialino
{
public:
  //constructors
  Serialino(const char* serial_device);
  
  //help methods
  
  //starting serial connection
  void startConnection();
  
  //stopping serial connection
  void closeConnection();
  
  //set the callback when new serial data is available
  void setReadingCallback(
                          void (*reader_cbk)(const char* buffer, size_t chars_read));
  
  //writing a series of chars
  void writeBuffer(const std::string &buffer);
  
private:
  std::string device_name;
  
  //serial_buffer
  char circular_buffer[CIRCULAR_SERIAL_BUFFER_SIZE];
  
  //members
  std::string input;
  
  void (*reader_callback)(const char* buffer, size_t chars_read);
  
  state_t serial_state;
  std::mutex mtx_serial_state;
  std::mutex mtx_input;
  bool new_input;
  
  std::mutex mtx_kill_thread;
  bool kill_thread;
  
  //thread functions
  void serial_in();
  void serial_out();
  bool get_kill_thread();
  std::thread* t_serial_in;
  std::thread* t_serial_out;
};


#endif /* SERIALINO_H */