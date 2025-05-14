// Copyright 2023 TIER IV, Inc.
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

#include "yabloc_common/static_tf_subscriber.hpp"

#include <memory>
#include <string>

namespace yabloc::common
{
StaticTfSubscriber::StaticTfSubscriber()
{
  managed_tf_buffer_ = std::make_unique<managed_transform_buffer::ManagedTransformBuffer>();
}

std::optional<Sophus::SE3f> StaticTfSubscriber::se3f(
  const std::string & frame_id, const std::string & parent_frame_id)
{
  std::optional<Eigen::Affine3f> opt_aff = (*this)(frame_id, parent_frame_id);
  if (!opt_aff.has_value()) return std::nullopt;

  Sophus::SE3f se3f(opt_aff->rotation(), opt_aff->translation());
  return se3f;
}

std::optional<Eigen::Affine3f> StaticTfSubscriber::operator()(
  const std::string & frame_id, const std::string & parent_frame_id)
{
  auto extrinsic_opt =
    managed_tf_buffer_->getLatestTransform<Eigen::Matrix4f>(parent_frame_id, frame_id);
  if (!extrinsic_opt) return {};

  return Eigen::Affine3f(extrinsic_opt->matrix());
}

}  // namespace yabloc::common
