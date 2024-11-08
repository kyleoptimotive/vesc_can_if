/*********************************************************************
 * Copyright (c) 2019, SoftBank Corp.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Softbank Corp. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************/

/** NOTE *************************************************************
 * This program had been developed by Michael T. Boulet at MIT under
 * the BSD 3-clause License until Dec. 2016. Since Nov. 2019, Softbank
 * Corp. takes over development as new packages.
 ********************************************************************/

#include "vesc_driver/vesc_interface.hpp"
#include <serial_driver/serial_driver.hpp>
#include <rclcpp/rclcpp.hpp>

namespace vesc_driver
{
class VescInterface::Impl
{
public:
  Impl() : owned_ctx(new IoContext(2)), serial_driver_(new drivers::serial_driver::SerialDriver(*owned_ctx))
  {
    data_updated_ = false;
  }

  void* rxThread(void);

  static void* rxThreadHelper(void* context)
  {
    return ((VescInterface::Impl*)context)->rxThread();
  }

  static void packetHandler(int cansocket) {
    struct can_frame rxmsg;
    while (1) {
      read(cansocket, &rxmsg, sizeof(rxmsg));
      printf("message received\n");
    }
  }

  pthread_t rx_thread_;
  bool rx_thread_run_;
  PacketHandlerFunction packet_handler_;
  ErrorHandlerFunction error_handler_;
  std::unique_ptr<IoContext> owned_ctx{};
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  VescFrame::CRC send_crc_;
  bool data_updated_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
};

void* VescInterface::Impl::rxThread(void)
{
  Buffer buffer;
  buffer.reserve(4096);
  auto temp_buffer = Buffer(4096);

  while (rx_thread_run_)
  {
    int bytes_needed = VescFrame::VESC_MIN_FRAME_SIZE;
    // attempt to read at least bytes_needed bytes from the serial port
    const auto bytes_read = serial_driver_->port()->receive(temp_buffer);
    buffer.reserve(buffer.size() + bytes_read);
    buffer.insert(buffer.end(), temp_buffer.begin(), temp_buffer.begin() + bytes_read);
    // RCLCPP_INFO(rclcpp::get_logger("VescDriver"), "Read packets: %d", bytes_read);
    if (bytes_needed > 0 && 0 == bytes_read && !buffer.empty())
    {
      error_handler_("Possibly out-of-sync with VESC, read timout in the middle of a frame.");
    }
    if (!buffer.empty())
    {
      // search buffer for valid packet(s)
      Buffer::iterator iter(buffer.begin());
      Buffer::iterator iter_begin(buffer.begin());
      while (iter != buffer.end())
      {
        // check if valid start-of-frame character
        if (VescFrame::VESC_SOF_VAL_SMALL_FRAME == *iter || VescFrame::VESC_SOF_VAL_LARGE_FRAME == *iter)
        {
          // good start, now attempt to create packet
          std::string error;
          VescPacketConstPtr packet = VescPacketFactory::createPacket(iter, buffer.end(), &bytes_needed, &error);
          if (packet)
          {
            // Packet received;
            data_updated_ = true;
            // good packet, check if we skipped any data
            if (std::distance(iter_begin, iter) > 0)
            {
              std::ostringstream ss;
              ss << "Out-of-sync with VESC, unknown data leading valid frame. Discarding "
                 << std::distance(iter_begin, iter) << " bytes.";
              error_handler_(ss.str());
            }
            // call packet handler
            packet_handler_(packet);
            // update state
            iter = iter + packet->getFrame().size();
            iter_begin = iter;
            // continue to look for another frame in buffer
            continue;
          }
          else if (bytes_needed > 0)
          {
            // need more data, break out of while loop
            break;  // for (iter_sof...
          }
          else
          {
            // else, this was not a packet, move on to next byte
            error_handler_(error);
          }
        }

        iter++;
      }

      // if iter is at the end of the buffer, more bytes are needed
      if (iter == buffer.end())
        bytes_needed = VescFrame::VESC_MIN_FRAME_SIZE;

      // erase "used" buffer
      if (std::distance(iter_begin, iter) > 0)
      {
        std::ostringstream ss;
        ss << "Out-of-sync with VESC, discarding " << std::distance(iter_begin, iter) << " bytes.";
        error_handler_(ss.str());
      }
      buffer.erase(buffer.begin(), iter);
    }
  }
}

VescInterface::VescInterface(const std::string& port, const PacketHandlerFunction& packet_handler,
                             const ErrorHandlerFunction& error_handler)
  : impl_(new Impl())
{
  setPacketHandler(packet_handler);
  setErrorHandler(error_handler);
  // attempt to conect if the port is specified
  if (!port.empty())
    connect(port);
}

VescInterface::~VescInterface()
{
  // stops the motor
  setDutyCycle(0.0);

  disconnect();
}

void VescInterface::setPacketHandler(const PacketHandlerFunction& handler)
{
  // todo - definately need mutex
  impl_->packet_handler_ = handler;
}

void VescInterface::setErrorHandler(const ErrorHandlerFunction& handler)
{
  // todo - definately need mutex
  impl_->error_handler_ = handler;
}

void VescInterface::connect(const std::string& port)
{
  // todo - mutex?

  std::string usb_port = "/dev/tty";
  std::string can_port = "can";
  if (std::equal(usb_port.begin(), usb_port.end(), port.begin())) {
    if (isConnected()) {
      throw SerialException("Already connected to serial port.");
    }
    // connect to serial port
    try {
      const uint32_t baud_rate = 115200;
      const auto fc = drivers::serial_driver::FlowControl::NONE;
      const auto pt = drivers::serial_driver::Parity::NONE;
      const auto sb = drivers::serial_driver::StopBits::ONE;
      impl_->device_config_ =
          std::make_unique<drivers::serial_driver::SerialPortConfig>(
              baud_rate, fc, pt, sb);
      impl_->serial_driver_->init_port(port, *impl_->device_config_);
      if (!impl_->serial_driver_->port()->is_open()) {
        impl_->serial_driver_->port()->open();
      }
    } catch (const std::exception& e) {
      std::stringstream ss;
      ss << "Failed to open the serial port to the VESC. " << e.what();
      throw SerialException(ss.str().c_str());
    }

    // start up a monitoring thread
    impl_->rx_thread_run_ = true;
    int result =
        pthread_create(&impl_->rx_thread_, NULL,
                       &VescInterface::Impl::rxThreadHelper, impl_.get());
    assert(0 == result);

  } else if (std::equal(can_port.begin(), can_port.end(), port.begin())) {
    // connect to can port
    try {
      socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
      strncpy(ifr_.ifr_name, port.c_str(), IFNAMSIZ);
      ioctl(socket_, SIOCGIFINDEX, &ifr_);
      if (socket_ < 0) {
        throw std::exception();
      }

      recv_addr_.can_family = AF_CAN;
      recv_addr_.can_ifindex = ifr_.ifr_ifindex;
      if (bind(socket_, (struct sockaddr*)&recv_addr_, sizeof(recv_addr_)) <
          0) {
        throw std::exception();
      }

      memset(&send_addr_, 0, sizeof(send_addr_));

      send_addr_.can_family = AF_CAN;
      send_addr_.can_ifindex = ifr_.ifr_ifindex;

    } catch (const std::exception& e) {
      std::stringstream ss;
      ss << "Failed to open the can port to the VESC. " << e.what();
      throw SerialException(ss.str().c_str());
    }
  } else {
    throw SerialException("Invalid port name.");
  }
}

void VescInterface::disconnect()
{
  // todo - mutex?

  if (isConnected())
  {
    // bring down read thread
    impl_->rx_thread_run_ = false;
    int result = pthread_join(impl_->rx_thread_, NULL);
    assert(0 == result);

    impl_->serial_driver_->port()->close();
  }
}

bool VescInterface::isConnected() const
{
  auto port = impl_->serial_driver_->port();

  if (port)
  {
    return port->is_open();
  }
  else
  {
    return false;
  }
}

bool VescInterface::isRxDataUpdated() const
{
  bool output = impl_->data_updated_;
  impl_->data_updated_ = false;
  return output;
}

void VescInterface::send(const VescPacket& packet)
{
  RCLCPP_DEBUG(rclcpp::get_logger("VescDriver"), "send data");
  std::size_t written = impl_->serial_driver_->port()->send(packet.getFrame());
  if (written != packet.getFrame().size())
  {
    std::stringstream ss;
    ss << "Wrote " << written << " bytes, expected " << packet.getFrame().size() << ".";
    throw SerialException(ss.str().c_str());
  }
}

void VescInterface::requestFWVersion()
{
  send(VescPacketRequestFWVersion());
}

void VescInterface::requestState()
{
  send(VescPacketRequestValues());
}

void VescInterface::setDutyCycle(double duty_cycle)
{
  RCLCPP_INFO(rclcpp::get_logger("VescDriver"), "Set duty: %f", duty_cycle);
  send(VescPacketSetDuty(duty_cycle));
}

void VescInterface::setCurrent(double current)
{
  send(VescPacketSetCurrent(current));
}

void VescInterface::setBrake(double brake)
{
  send(VescPacketSetCurrentBrake(brake));
}

void VescInterface::setSpeed(double speed)
{
  send(VescPacketSetVelocityERPM(speed));
}

void VescInterface::setPosition(double position)
{
  RCLCPP_DEBUG(rclcpp::get_logger("VescDriver"), "Set position: %f", position);
  send(VescPacketSetPos(position));
}

void VescInterface::setServo(double servo)
{
  RCLCPP_DEBUG(rclcpp::get_logger("VescDriver"), "Set servoPosition: %f", servo);
  send(VescPacketSetServoPos(servo));
}

}  // namespace vesc_driver
