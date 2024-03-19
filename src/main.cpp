#include <communication/interface.hpp>
#include <communication/interfaces/uart.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <yaml-cpp/yaml.h>
#include <string>
#include <iomanip>
#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <mutex>
#include <shared_mutex>
#include <cstdint>
#include <cstdlib>

#include "fdcl_common.hpp"

using namespace std;
using namespace communication;

std::vector<int> motion_to_velocity(int _m, double _d)
{
  if (_m==0) // stop
  {
    return std::vector<int>{0, 0, 0, 0, 0, 0};
  }
  if (_m==1) // forward
  {
    return std::vector<int>{1, 100, 200, 1, 100, 200};
  }
  if (_m==2) // backward
  {
    return std::vector<int>{0, 100, 200, 0, 100, 200};
  }
  if (_m==3) // rotate clockwise 90 degree
  {
   return std::vector<int>{1, 100, 200, 0, 100, 200}; 
  }
  if (_m==4) // rotate counter clockwise 90 degree
  {
    return std::vector<int>{0, 100, 200, 1, 100, 200};
  }
  if (_m==5) // rotate clockwise 180 degree
  {
    return std::vector<int>{1, 100, 200, 1, 100, 200};
  }
  if (_m==6) // rotate counter clockwise 180 degree
  {
    return std::vector<int>{0, 100, 200, 0, 100, 200};
  }
  return std::vector<int>();
}

void printData(const std::vector<uint8_t>& data) 
{
  std::cout << "Data: ";
  for (auto byte : data) 
  {
    std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<unsigned>(byte) << " ";
  }
  std::cout << std::endl;
}

uint8_t checksum(const std::vector<uint8_t>& data) {
  uint8_t _d = data[0];
  for (size_t i = 1; i < data.size(); ++i) 
  {
    _d = _d ^ data[i];
  }
  _d = _d + 1;
  if (_d == 256) 
  {
    _d = 0;
  }
  return _d;
}

std::vector<uint8_t> extractMessage(const char* buffer, ssize_t start, ssize_t length) 
{
  return std::vector<uint8_t>(buffer + start, buffer + start + length);
}

uint16_t readLittleEndian16(const std::vector<uint8_t>& data, size_t index) 
{
  return static_cast<uint16_t>(data[index]) |
          (static_cast<uint16_t>(data[index + 1]) << 8);
}

std::vector<uint16_t> translateData(const std::vector<uint8_t>& data) 
{
  std::vector<uint16_t> result;
  std::vector<uint8_t> _data(data.begin(), data.end()-1);
  auto _c = checksum(_data);
  if (_c == data.back())
  {
    // std::cout << "Checksum is correct" << std::endl;
    if (_data.size() >= 8)
    {
      result.push_back(readLittleEndian16(_data, 0));
      result.push_back(readLittleEndian16(_data, 2));
      result.push_back(readLittleEndian16(_data, 4));
      result.push_back(readLittleEndian16(_data, 6));
    }
    return result;
  }
  else
  {
    cerr << "Checksum is not correct" << std::endl;
    return result;
  }
  
}

std::vector<uint8_t> motor_push(int mode, int l_dir=0, int l_rpm=0, int l_acc=0, int r_dir=0, int r_rpm=0, int r_acc=0)
{
  std::vector<uint8_t> data;

  data.push_back(0xFF);
  data.push_back(0xFF);

  std::vector<uint8_t> mode_byte;
  std::vector<uint8_t> byte_l;
  std::vector<uint8_t> byte_r;

  mode_byte.push_back(static_cast<uint8_t>(mode));

  int l_rpm_combine = l_dir << 15;
  l_rpm_combine |= l_rpm&0xFFF;
  byte_l.push_back(static_cast<uint8_t>((l_rpm_combine >> 8) & 0xFF));
  byte_l.push_back(static_cast<uint8_t>(l_rpm_combine & 0xFF));
  byte_l.push_back(static_cast<uint8_t>(l_acc & 0xFF));

  int r_rpm_combine = r_dir << 15;
  r_rpm_combine |= r_rpm&0xFFF;
  byte_r.push_back(static_cast<uint8_t>((r_rpm_combine >> 8) & 0xFF));
  byte_r.push_back(static_cast<uint8_t>(r_rpm_combine & 0xFF));
  byte_r.push_back(static_cast<uint8_t>(r_acc & 0xFF));

  std::vector<uint8_t> _check;
  _check.insert(_check.end(), mode_byte.begin(), mode_byte.end());
  _check.insert(_check.end(), byte_l.begin(), byte_l.end());
  _check.insert(_check.end(), byte_r.begin(), byte_r.end());

  uint8_t _checksum = checksum(_check);

  data.insert(data.end(), _check.begin(), _check.end());
  data.push_back(_checksum);
  return data;
}

void writeUart(UART& uart, std::vector<std::pair<int, double>>& motion) 
{
  bool isCallback = true;
  const size_t BUFFER_SIZE = 128;
  char buffer[BUFFER_SIZE];

  unsigned int it = 0;
  unsigned int system_it = 0;
  double _passTime = 0.0;
  double duration = 0.0;
  auto start = chrono::steady_clock::now();
  
  while (true) 
  {
    auto loop_start = chrono::steady_clock::now();
    auto _m = get<0>(motion[it]);
    auto _d = get<1>(motion[it]);

    if (it > motion.size())
    {
      std::cout << "end sequence" << std::endl;
      continue;
    }

    if (_d < duration)
    {
      it++;
      duration = 0.0;
    }
    auto _v = motion_to_velocity(_m, duration);
    if (_v.size() != 6)
    {
      cerr << "motion not found" << endl;
    }
    else
    {
      auto _data = motor_push(1, _v[0], _v[1], _v[2], _v[3], _v[4], _v[5]);
      ssize_t result = uart.write(reinterpret_cast<const char*>(_data.data()), _data.size());
      this_thread::sleep_for(chrono::milliseconds(3));
      if (result < 0) 
      {
        cerr << "not \n";    
      }
    }

    if (system_it%2 == 0 && isCallback)
    {
      auto _data = motor_push(3); // motor data callback
      ssize_t check_return = uart.write(reinterpret_cast<const char*>(_data.data()), _data.size());

      this_thread::sleep_for(chrono::milliseconds(3));
      if (check_return < 0)
        cerr << "not system callback send\n";
      else
      {
        ssize_t bytesRead = uart.read(buffer, BUFFER_SIZE);
        if (bytesRead < 0) 
        {
          cerr << "Read error" << endl;
        }
        else 
        {
          int messageLength = 9;
          for (ssize_t i = 0; i < bytesRead; ++i) 
          {
            if (i + 1 < bytesRead && static_cast<unsigned char>(buffer[i])==0xFF && static_cast<unsigned char>(buffer[i+1])==0xFF)
            {
              if (i + messageLength + 1 <= bytesRead) 
              {
                auto message = extractMessage(buffer, i + 2, messageLength);
                auto result = translateData(message);
                i += messageLength + 1;
              }
            }
          }
        }
      } 
    }
    this_thread::sleep_for(chrono::milliseconds(10));

    auto loop_end = chrono::steady_clock::now(); 
    auto loop_duration = chrono::duration_cast<chrono::milliseconds>(loop_end - loop_start);

    std::cout << "Loop duration: " << loop_duration.count()/1000.0 << " s" << endl;
    duration += double(loop_duration.count()/1000.0);
    std::cout << "Loop past duration : " << duration << std::endl;

    system_it++;
  }
}

void readAruco(cv::CommandLineParser& parser) 
{
  
  const char* about = "Detect ArUco marker images";
  auto success = parse_inputs(parser, about);
  if (!success) {
      return;
  }

  cv::VideoCapture in_video;
  success = parse_video_in(in_video, parser);
  if (!success) {
      return;
  }

  int dictionary_id = parser.get<int>("d");
  int wait_time = 10;
  cv::Ptr<cv::aruco::Dictionary> dictionary =
      cv::aruco::getPredefinedDictionary( \
      cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionary_id));

  while (in_video.grab()) 
  {
    cv::Mat image, image_copy;
    in_video.retrieve(image);
    image.copyTo(image_copy);
    
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(image, dictionary, corners, ids);
    
    if (ids.size() > 0) {
        cv::aruco::drawDetectedMarkers(image_copy, corners, ids);
    }

    imshow("Detected markers", image_copy);
    char key = (char)cv::waitKey(wait_time);
    if (key == 27) {
        break;
    }
    this_thread::sleep_for(chrono::milliseconds(10));
  }
  in_video.release();
}

int main(int argc, char const *argv[]) 
{
  cv::CommandLineParser parser(argc, argv, fdcl::keys);

  std::vector<std::pair<int, double>> motion;
  YAML::Node config = YAML::LoadFile("../data/motion.yml");

  if (!config["root"]) 
  {
    std::cerr << "Error: root not found" << std::endl;
    return -1;
  }
  else
  {
    auto _agent = config["root"]["agent"];
    int number = _agent["number"].as<int>();
    std::cout << "Agent number: " << number << std::endl;
    auto _path = _agent["path"];
    double path_duration = _path["duration"].as<double>();
    std::cout << "Path duration: " << path_duration << std::endl;
    for (const auto& _motion: _path["sections"])
    {
      int _m = _motion["motion"].as<int>();
      double _d = _motion["duration"].as<double>();
      std::cout << "motion : " << _m << " duration : " << _d << std::endl;
      motion.push_back(std::make_pair(_m, _d));
    }
  }

  Logger::set_level(Logger::level::debug);
  InterfaceInitParam param({
      {UART::USE_SYS_POLLING, false},
      {UART::PORT, "/dev/ttyUSB0"},
      {UART::BAUDRATE, 115200}
  });

  UART uart;
  uart.init(param);



  thread writeUartThread(writeUart, std::ref(uart), std::ref(motion));
  thread readUartThread(readAruco, std::ref(parser));
  writeUartThread.join();
  readUartThread.join();

  return 0;
}


////////////////************////////////////
// motion 0 : stop
// motion 1 : forward
// motion 2 : backward
// motion 3 : rotate clockwise 90 degree
// motion 4 : rotate counter clockwise 90 degree
// motion 5 : rotate clockwise 180 degree
// motion 6 : rotate counter clockwise 180 degree