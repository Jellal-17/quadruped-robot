// The MIT License

// Copyright (c) 2022
// Robot Motion and Vision Laboratory at East China Normal University
// Contact: Xinyu Zhang   email: tophill.robotics@gmail.com

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef QR_ROBOTCONFIG_H
#define QR_ROBOTCONFIG_H

#include <iostream>
#include <yaml-cpp/yaml.h>

namespace Robot {
    class qrRobotConfig;
}


/**
 * @brief a class that will load all robot configs from YAML file
 */
class qrRobotConfig
{
public:
  /**
   * @brief Construction of qrRobotConfig
   */
  qrRobotConfig();

  /**
   * @brief Destruction of qrRobotConfig
   */
  ~qrRobotConfig();

  /**
   * @brief load parameter of the robot
   * @param path: the path to the YAML config file
   */
  void load(std::string path);

private:

};

#endif // QR_ROBOTCONFIG_H
