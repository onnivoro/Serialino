//
//  main.cpp
//  ArduinoSerial
//
//  Created by Nicolò Boscolo on 26/08/14.
//  Copyright (c) 2014 Nicolò Boscolo. All rights reserved.
//

#include "Serialino.h"
#include <string>
#include <iostream>
using namespace std;

void test_read(const char* buffer, size_t buffer_size)
{
  string output;
  output.append(buffer,buffer_size);
  cout << output << endl;
}

int main(int argc, const char * argv[])
{
  if(argc < 2)
  {
	  cout << "bad usage, try: serialinoTest serial_port" << endl;
	  return -1;
  }
    
  Serialino ardu_serial(argv[1]);
  ardu_serial.setReadingCallback(&test_read);
  ardu_serial.startConnection();
// 	uncomment and use the following two lines to send data to arduino
//  string buffer;
//  ardu_serial.writeBuffer(buffer);
  string input;
  getline(cin, input);
  ardu_serial.closeConnection();
  return 0;
}
