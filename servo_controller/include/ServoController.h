/*
 * ServoController.h
 *
 *  Created on: Dec 14, 2014
 *      Author: yuanboshe
 */

#ifndef SERVOCONTROLLER_H_
#define SERVOCONTROLLER_H_

#include "Servo32Sel.h"
#include <map>

#define RAD_RATE 500

class ServoController
{
public:
  virtual ~ServoController();
  std::map<std::string, int> nameMap;
  std::map<std::string, int> posMap;
  serial::Servo32Sel* pServo32Sel;
  double radRate; // us / rad  e.g. 1200 / (pi / 2)
  int middlePosition; // us
  ServoController(std::vector<std::string> names, std::vector<int> pins, std::string port = "/dev/ttyUSB0", uint32_t baud = 115200, uint32_t timeout = DEFAULT_TIMEOUT, double radRate = RAD_RATE);
  void initServoController(std::vector<std::string> names, std::vector<int> pins, std::string port = "/dev/ttyUSB0", uint32_t baud = 115200, uint32_t timeout = DEFAULT_TIMEOUT, double radRate = RAD_RATE);
  void run(std::vector<std::string> name, std::vector<double> position, std::vector<double> velocity = std::vector<double>());
};


#endif /* SERVOCONTROLLER_H_ */
