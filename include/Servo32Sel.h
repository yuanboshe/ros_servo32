/*
 * Servo32Sel.h
 *
 *  Created on: Dec 14, 2014
 *      Author: yuanboshe
 */

#ifndef SERVO32SEL_H_
#define SERVO32SEL_H_

#include <serial/serial.h>

#define DEFAULT_TIMEOUT 1000 //timeout in milliseconds

namespace serial
{

class Servo32Sel : public serial::Serial
{
public:
  Servo32Sel(std::string port = "/dev/ttyUSB0", uint32_t baud = 460800, uint32_t timeout = DEFAULT_TIMEOUT);
  virtual ~Servo32Sel();

  void drive(std::vector<int> pin, std::vector<int> position, std::vector<int> velocity);
  void test(std::vector<int> pin, std::vector<int> position, std::vector<int> velocity);
};

} /* namespace servo32 */

#endif /* SERVO32SEL_H_ */
