/*
 * Servo32Sel.cpp
 *
 *  Created on: Dec 14, 2014
 *      Author: Yuanboshe
 */

#include "Servo32Sel.h"
#include "boost/lexical_cast.hpp"

//using namespace serial;

namespace serial
{

Servo32Sel::Servo32Sel(std::string port, uint32_t baud, uint32_t timeout) :
    serial::Serial(port, baud, serial::Timeout::simpleTimeout(timeout))
{
  // TODO Auto-generated constructor stub

}

Servo32Sel::~Servo32Sel()
{
  // TODO Auto-generated destructor stub
}

void Servo32Sel::drive(std::vector<int> pin, std::vector<int> position, std::vector<int> velocity)
{
  int size = pin.size();
  if (size > velocity.size())
  {
    for (int i = 0; i < size; i++)
    {
      std::string str("#");
      str.append(boost::lexical_cast<std::string>(pin[i]));
      str.append("P");
      str.append(boost::lexical_cast<std::string>(position[i]));
      str.append("\r");
      write(str);
    }
  }
  else
  {
    for (int i = 0; i < size; i++)
    {
      std::string str("#");
      str.append(boost::lexical_cast<std::string>(pin[i]));
      str.append("P");
      str.append(boost::lexical_cast<std::string>(position[i]));
      str.append("S");
      str.append(boost::lexical_cast<std::string>(velocity[i]));
      str.append("\r");
      write(str);
    }
  }
}

void Servo32Sel::test(std::vector<int> pin, std::vector<int> position, std::vector<int> velocity)
{
  for (int i = 0; i < pin.size(); i++)
  {
    printf("%d %d ", pin[i], position[i]);
  }
  printf("\n");
}

} /* namespace servo32 */
