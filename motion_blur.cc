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

#include <chrono>
#include <string>
#include <regex>
#include <vector>
#include <cmath>
#include <iostream>

/** Parse a string to its corresponding std::chrono::duration representation.
 *
 * Strings can contain numbers followed by:
 * - h for hours
 * - m or min for minutes
 * - s for seconds
 * - ms for milliseconds
 * - us for microseconds
 * - ns for nanoseconds
 * The order of magnitudes should be h, m, s, ms, us, ns.
 * 
 * If an invalid string is passed, a duration of 0 ns is returned.
 */
std::chrono::nanoseconds parse_duration(const std::string &txt_duration) {
  std::chrono::nanoseconds duration;
  const std::string time_pattern = " *"
    "((\\d+) *h)? *"
    "((\\d+) *m(in)?)? *"
    "((\\d+) *s)? *"
    "((\\d+) *ms)? *"
    "((\\d+) *([^m]|[^n])s)? *"
    "((\\d+) *ns)? *";
  const std::regex time_regex(time_pattern, std::regex::icase);

  std::smatch match;
  std::regex_search(txt_duration, match, time_regex);

  if (match[2].length())
    duration += std::chrono::hours(std::stoi(match[2]));

  if (match[4].length())
    duration += std::chrono::minutes(std::stoi(match[4]));

  if (match[7].length())
    duration += std::chrono::seconds(std::stoi(match[7]));

  if (match[9].length())
    duration += std::chrono::milliseconds(std::stoi(match[9]));

  if (match[11].length())
    duration += std::chrono::microseconds(std::stoi(match[11]));

  if (match[14].length())
    duration += std::chrono::nanoseconds(std::stoi(match[14]));

  return duration;
}


namespace gazebo
{
  class CameraBlur : public CameraPlugin
  {
    public: CameraBlur() 
            : CameraPlugin(), 
              firstFrame(true),
              exposure_time(std::chrono::nanoseconds::zero()),
              sensor_frequency(std::chrono::nanoseconds::zero()),
              is_duration_set(false)
    {}

    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
    {
      gzwarn << "Loading CameraBlur\n";
      // Don't forget to load the camera plugin
      CameraPlugin::Load(_parent, _sdf);

      is_duration_set = true;

      if (_parent->GetUpdateRate()) {
        sensor_frequency = std::chrono::nanoseconds((long int)(std::nano::den / _parent->GetUpdateRate()));
      }
      else {
        is_duration_set = false;
      }

      if (_sdf->HasElement("exposure_time")) {
        sdf::ElementPtr sdf_exposure_time = _sdf->GetElement("exposure_time");
        std::string exposure_time_str = sdf_exposure_time->GetValue()->GetAsString();
        exposure_time = parse_duration(exposure_time_str);
      }

      if (exposure_time == std::chrono::nanoseconds::zero()) {
        is_duration_set = false;
      }
    }

    // Update the controller
    public: void OnNewFrame(const unsigned char *_image,
        unsigned int _width, unsigned int _height, unsigned int _depth,
        const std::string &_format)
    {
      const unsigned int image_size = _width * _height * _depth;
      if (!this->firstFrame && image_size != this->previousFrame.size()) {
        gzwarn << "Camera image has been resized, restarting blur procedure.\n";
        this->firstFrame = false;
      }

      if (!this->firstFrame) {
        // do stuff!
        unsigned char *image = const_cast<unsigned char*>(_image);
        std::vector<unsigned char> thisFrame(image_size);
        for (unsigned int i = 0; i < image_size; ++i) {
          thisFrame[i] = _image[i];
          image[i] = ((image[i] + previousFrame[i]) / 2);
        }
        this->previousFrame = thisFrame;
        gzmsg << "Blurred a frame\n";
      }
      else {
        this->firstFrame = false;
        this->previousFrame.resize(image_size);
        for (unsigned int i = 0; i < image_size; ++i) {
          this->previousFrame[i] = _image[i];
        }
        gzmsg << "Ignoring the first frame\n";
      }
    }

    private: bool firstFrame;
    private: std::vector<unsigned char> previousFrame;
    private: std::chrono::nanoseconds sensor_frequency;
    private: std::chrono::nanoseconds exposure_time;
    private: bool is_duration_set;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_SENSOR_PLUGIN(CameraBlur)
}
