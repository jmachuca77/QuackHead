/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#ifndef DYNAMIXEL_PORT_HANDLER_HPP_
#define DYNAMIXEL_PORT_HANDLER_HPP_

#include <stddef.h>
#include <stdint.h>

class DXLPortHandler
{
  public:
    DXLPortHandler();
    
    virtual void begin() = 0;
    virtual void end() = 0;
    virtual int available(void) = 0;
    virtual int read() = 0;
    virtual size_t write(uint8_t) = 0;
    virtual size_t write(uint8_t *buf, size_t len) = 0;
    bool getOpenState();
    void setOpenState(bool);

  private:
    bool open_state_;
};

#endif /* DYNAMIXEL_PORT_HANDLER_HPP_ */