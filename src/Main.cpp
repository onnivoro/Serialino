//
//  main.cpp
//  ArduinoSerial
//
//  Created by Nicolò Boscolo on 26/08/14.
//  Copyright (c) 2014 Nicolò Boscolo. All rights reserved.
//

#include "Serialino.h"
#include <string.h>
#include <iostream>
using namespace std;

void test_read(const char* buffer, size_t buffer_size)
{
  string output;
  output.append(buffer,buffer_size);
  cout << output;
}

int main(int argc, const char * argv[])
{
  if(argc < 2)
    return -1;
    
  Serialino ardu_serial(argv[1]);
  ardu_serial.setReadingCallback(&test_read);
  ardu_serial.startConnection();
  string buffer;
  ardu_serial.writeBuffer(buffer);
  ardu_serial.closeConnection();
  return 0;
}
