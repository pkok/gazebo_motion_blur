/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
 *
*/
#include "gazebo/gazebo.hh"
#include "gazebo/plugins/CameraPlugin.hh"

#include <string>
#include <deque>
#include <vector>

#include <iostream>

//#include <chrono>
//#include <regex>
//
///** Parse a string to its corresponding std::chrono::duration representation.
// *
// * Strings can contain numbers followed by:
// * - h for hours
// * - m or min for minutes
// * - s for seconds
// * - ms for milliseconds
// * - us for microseconds
// * - ns for nanoseconds
// * The order of magnitudes should be h, m, s, ms, us, ns.
// * 
// * If an invalid string is passed, a duration of 0 ns is returned.
// */
//std::chrono::nanoseconds parse_duration(const std::string &txt_duration) {
//  std::chrono::nanoseconds duration;
//  const std::string time_pattern = " *"
//    "((\\d+) *h)? *"
//    "((\\d+) *m(in)?)? *"
//    "((\\d+) *s)? *"
//    "((\\d+) *ms)? *"
//    "((\\d+) *([^m]|[^n])s)? *"
//    "((\\d+) *ns)? *";
//  const std::regex time_regex(time_pattern, std::regex::icase);
//
//  std::smatch match;
//  std::regex_search(txt_duration, match, time_regex);
//
//  if (match[2].length())
//    duration += std::chrono::hours(std::stoi(match[2]));
//
//  if (match[4].length())
//    duration += std::chrono::minutes(std::stoi(match[4]));
//
//  if (match[7].length())
//    duration += std::chrono::seconds(std::stoi(match[7]));
//
//  if (match[9].length())
//    duration += std::chrono::milliseconds(std::stoi(match[9]));
//
//  if (match[11].length())
//    duration += std::chrono::microseconds(std::stoi(match[11]));
//
//  if (match[14].length())
//    duration += std::chrono::nanoseconds(std::stoi(match[14]));
//
//  return duration;
//}


namespace gazebo
{
  class CameraBlur : public CameraPlugin
  {
    private: typedef std::deque<std::vector<unsigned char>> image_history_t;

    public: CameraBlur() 
            : CameraPlugin(), 
              history_size(1),
              previousFrames(2)
    {}

    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
    {
      gzwarn << "Loading CameraBlur\n";
      // Don't forget to load the camera plugin
      CameraPlugin::Load(_parent, _sdf);

      if (_sdf->HasElement("history_size")) {
        _sdf->GetElement("history_size")->GetValue()->Get<int>(this->history_size);
        this->previousFrames = image_history_t(this->history_size + 1);
      }
    }

    public: void OnNewFrame(const unsigned char *_image,
        unsigned int _width, unsigned int _height, unsigned int _depth,
        const std::string &_format)
    {
      const unsigned int image_size = _width * _height * _depth;

      // new image size does not agree with size of previous frames?
      // then reset the blurring buffer
      if (this->previousFrames.size() < this->history_size
          && image_size != this->previousFrames.front().size()) {
        gzwarn << "Camera image has been resized, restarting blur procedure.\n";
        this->previousFrames.clear();
      }

      std::vector<unsigned int> summedFrame(image_size);
      unsigned char* blurredImage = const_cast<unsigned char*>(_image);

      this->previousFrames.emplace_back(image_size);

      for (unsigned int i = 0; i < image_size; ++i) {
        this->previousFrames.back()[i] = _image[i];
      }

      for (auto frame : this->previousFrames) {
        for (unsigned int i = 0; i < image_size; ++i) {
          summedFrame[i] += frame[i];
        }
      }

      for (unsigned int i = 0; i < image_size; ++i) {
        blurredImage[i] = summedFrame[i] / this->previousFrames.size();
      }

      if (this->previousFrames.size() > this->history_size) {
        this->previousFrames.pop_front();
      }
      else {
        gzmsg << "Building image history for motion blur\n";
      }
    }

    private: image_history_t previousFrames;
    private: int history_size;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_SENSOR_PLUGIN(CameraBlur)
}
