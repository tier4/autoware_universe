// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/ml_planner/preprocessing/scene_input_builder.hpp"

#include <t4_scene/protobuf/maps.hpp>
#include <t4_scene/protobuf/messages.hpp>

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::ml_planner::preprocess
{
namespace py = pybind11;
namespace pb = t4scene::protobuf;
namespace
{
class SceneInputBuilder
{
public:
  explicit SceneInputBuilder(const py::bytes & map_resource)
  {
    const auto bytes = std::string(map_resource);
    const py::gil_scoped_release release;
    const auto resource = pb::parse<pb::MapResource>(bytes);
    map_context_ = build_map_context(pb::decode_map(resource));
    map_sha256_ = resource.sha256();
  }

  py::dict build(
    const py::bytes & scene, TrafficLightFuture traffic_light_future,
    double traffic_light_timeout_s, bool normalize) const
  {
    const auto bytes = std::string(scene);
    const auto output = [&] {
      const py::gil_scoped_release release;
      const auto message = pb::parse<pb::T4Scene>(bytes);
      if (!message.has_map() || message.map().sha256() != map_sha256_) {
        throw std::invalid_argument("Scene does not reference the builder's map resource");
      }
      auto result = build_scene_input(
        pb::from_proto(message), *map_context_, traffic_light_future, traffic_light_timeout_s);
      if (result && normalize) {
        normalize_input_data(result->tensors);
      }
      return result;
    }();
    if (!output) {
      throw std::invalid_argument(output.error());
    }

    py::dict tensors;
    for (const auto & [name, tensor] : output->tensors) {
      const std::vector<py::ssize_t> shape(tensor.shape().begin(), tensor.shape().end());
      py::array_t<float> array(shape);
      std::copy_n(tensor.data(), tensor.size(), array.mutable_data());
      tensors[py::str(name)] = std::move(array);
    }
    py::array_t<double> ego_to_map({py::ssize_t{4}, py::ssize_t{4}});
    auto matrix = ego_to_map.mutable_unchecked<2>();
    for (Eigen::Index row = 0; row < 4; ++row) {
      for (Eigen::Index column = 0; column < 4; ++column) {
        matrix(row, column) = output->ego_to_map(row, column);
      }
    }

    py::dict result;
    result["tensors"] = std::move(tensors);
    result["selected_track_ids"] = py::cast(output->selected_track_ids);
    result["ego_to_map"] = std::move(ego_to_map);
    return result;
  }

private:
  std::string map_sha256_;
  std::unique_ptr<LaneSegmentContext> map_context_;
};
}  // namespace

PYBIND11_MODULE(_native, module)
{
  module.doc() = "Build Autoware ML-Planner features directly from T4Scene protobufs";
  py::enum_<TrafficLightFuture>(module, "TrafficLightFuture")
    .value("INFER", TrafficLightFuture::Infer)
    .value("RECORDED", TrafficLightFuture::Recorded);
  py::class_<SceneInputBuilder>(module, "SceneInputBuilder")
    .def(py::init<const py::bytes &>(), py::arg("map_resource"))
    .def(
      "build", &SceneInputBuilder::build, py::arg("scene"),
      py::arg("traffic_light_future") = TrafficLightFuture::Infer,
      py::arg("traffic_light_timeout_s") = 0.2, py::arg("normalize") = false);
}
}  // namespace autoware::ml_planner::preprocess
