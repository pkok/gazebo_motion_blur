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

namespace gazebo
{
  class CameraBlur : public CameraPlugin
  {
    private: typedef std::deque<std::vector<unsigned char>> image_history_t;

    public: CameraBlur() 
            : CameraPlugin(), 
              history_size(1)
    {}

    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
    {
      gzwarn << "Loading CameraBlur\n";
      // Don't forget to load the camera plugin
      CameraPlugin::Load(_parent, _sdf);

      if (_sdf->HasElement("history_size")) {
        this->history_size = std::stoi(_sdf->GetElement("history_size")->GetValue()->GetAsString());
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

      for (const std::vector<unsigned char>& frame : this->previousFrames) {
        for (unsigned int i = 0; i < image_size; ++i) {
          summedFrame[i] += (unsigned int) frame[i];
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
