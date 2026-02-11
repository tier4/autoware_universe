// Copyright 2020 Tier IV, Inc.
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

/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: filter.h 35876 2011-02-09 01:04:36Z rusu $
 *
 */

#ifndef AUTOWARE__POINTCLOUD_PREPROCESSOR__FILTER_HPP_
#define AUTOWARE__POINTCLOUD_PREPROCESSOR__FILTER_HPP_

#include "autoware/pointcloud_preprocessor/transform_info.hpp"
#include "autoware/pointcloud_preprocessor/utility/memory.hpp"

#include <agnocast/agnocast.hpp>
#include <agnocast/message_filters/subscriber.hpp>
#include <agnocast/message_filters/sync_policies/approximate_time.hpp>
#include <agnocast/message_filters/sync_policies/exact_time.hpp>
#include <agnocast/message_filters/synchronizer.hpp>
#include <autoware_utils/ros/debug_publisher.hpp>
#include <autoware_utils/ros/diagnostics_interface.hpp>
#include <autoware_utils/ros/published_time_publisher.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <managed_transform_buffer/agnocast_managed_transform_buffer.hpp>
#include <managed_transform_buffer/managed_transform_buffer.hpp>

#include <boost/thread/mutex.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/filters/filter.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_msgs/msg/model_coefficients.h>
#include <pcl_msgs/msg/point_indices.h>
#include <sensor_msgs/msg/point_cloud2.h>

#include <memory>
#include <set>
#include <string>
#include <type_traits>
#include <vector>

namespace autoware::pointcloud_preprocessor
{
namespace sync_policies = message_filters::sync_policies;

/** \brief For parameter service callback */
template <typename T>
bool get_param(const std::vector<rclcpp::Parameter> & p, const std::string & name, T & value)
{
  auto it = std::find_if(p.cbegin(), p.cend(), [&name](const rclcpp::Parameter & parameter) {
    return parameter.get_name() == name;
  });
  if (it != p.cend()) {
    value = it->template get_value<T>();
    return true;
  }
  return false;
}

// Type trait to check if NodeT is rclcpp::Node
template <typename NodeT>
struct is_rclcpp_node : std::is_same<NodeT, rclcpp::Node>
{
};

template <typename NodeT>
inline constexpr bool is_rclcpp_node_v = is_rclcpp_node<NodeT>::value;

// Type trait to check if NodeT is agnocast::Node
template <typename NodeT>
struct is_agnocast_node : std::is_same<NodeT, agnocast::Node>
{
};

template <typename NodeT>
inline constexpr bool is_agnocast_node_v = is_agnocast_node<NodeT>::value;

/** \brief @b FilterBase represents the base filter class template.
 * Supports both rclcpp::Node and agnocast::Node.
 * \author Radu Bogdan Rusu (original), extended for agnocast support
 */
template <typename NodeT>
class FilterBase : public NodeT
{
public:
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using PointCloud2ConstPtr = sensor_msgs::msg::PointCloud2::ConstSharedPtr;

  using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
  using PointCloudPtr = PointCloud::Ptr;
  using PointCloudConstPtr = PointCloud::ConstPtr;

  using PointIndices = pcl_msgs::msg::PointIndices;
  using PointIndicesPtr = PointIndices::SharedPtr;
  using PointIndicesConstPtr = PointIndices::ConstSharedPtr;

  using ModelCoefficients = pcl_msgs::msg::ModelCoefficients;
  using ModelCoefficientsPtr = ModelCoefficients::SharedPtr;
  using ModelCoefficientsConstPtr = ModelCoefficients::ConstSharedPtr;

  using IndicesPtr = pcl::IndicesPtr;
  using IndicesConstPtr = pcl::IndicesConstPtr;

  using ExactTimeSyncPolicy =
    message_filters::Synchronizer<sync_policies::ExactTime<PointCloud2, PointIndices>>;
  using ApproximateTimeSyncPolicy =
    message_filters::Synchronizer<sync_policies::ApproximateTime<PointCloud2, PointIndices>>;

  using AgnocastExactTimeSyncPolicy = agnocast::message_filters::Synchronizer<
    agnocast::message_filters::sync_policies::ExactTime<PointCloud2, PointIndices>>;
  using AgnocastApproximateTimeSyncPolicy = agnocast::message_filters::Synchronizer<
    agnocast::message_filters::sync_policies::ApproximateTime<PointCloud2, PointIndices>>;

  PCL_MAKE_ALIGNED_OPERATOR_NEW
  explicit FilterBase(
    const std::string & filter_name = "pointcloud_preprocessor_filter",
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  /**
   * \brief Subscription type depends on NodeT:
   * For rclcpp::Node: rclcpp::Subscription
   * For agnocast::Node: agnocast::Subscription
   */
  std::conditional_t<
    is_rclcpp_node_v<NodeT>,
    typename rclcpp::Subscription<PointCloud2>::SharedPtr,
    typename agnocast::Subscription<PointCloud2>::SharedPtr>
    sub_input_;

  /**
   * \brief Publisher type depends on NodeT:
   * For rclcpp::Node: rclcpp::Publisher
   * For agnocast::Node: agnocast::Publisher
   */
  std::conditional_t<
    is_rclcpp_node_v<NodeT>,
    typename rclcpp::Publisher<PointCloud2>::SharedPtr,
    typename agnocast::Publisher<PointCloud2>::SharedPtr>
    pub_output_;

  /** \brief The message filter subscriber for PointCloud2. */
  std::conditional_t<
    is_rclcpp_node_v<NodeT>, message_filters::Subscriber<PointCloud2>,
    agnocast::message_filters::Subscriber<PointCloud2, agnocast::Node>>
    sub_input_filter_;

  /** \brief The message filter subscriber for PointIndices. */
  std::conditional_t<
    is_rclcpp_node_v<NodeT>, message_filters::Subscriber<PointIndices>,
    agnocast::message_filters::Subscriber<PointIndices, agnocast::Node>>
    sub_indices_filter_;

  /** \brief The desired user filter field name. */
  std::string filter_field_name_;

  /** \brief The minimum allowed filter value a point will be considered from. */
  double filter_limit_min_;

  /** \brief The maximum allowed filter value a point will be considered from. */
  double filter_limit_max_;

  /** \brief Set to true if we want to return the data outside (\a filter_limit_min_;\a
   * filter_limit_max_). Default: false. */
  bool filter_limit_negative_;

  /** \brief The input TF frame the data should be transformed into,
   * if input.header.frame_id is different. */
  std::string tf_input_frame_;

  /** \brief The original data input TF frame. */
  std::string tf_input_orig_frame_;

  /** \brief The output TF frame the data should be transformed into,
   * if input.header.frame_id is different. */
  std::string tf_output_frame_;

  /** \brief Internal mutex. */
  std::mutex mutex_;

  /** \brief The diagnostic message */
  std::unique_ptr<autoware_utils::BasicDiagnosticsInterface<NodeT>> diagnostics_interface_;

  /** \brief processing time publisher. **/
  std::unique_ptr<autoware_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_;
  std::unique_ptr<autoware_utils_debug::BasicDebugPublisher<NodeT>> debug_publisher_;
  std::unique_ptr<autoware_utils::BasicPublishedTimePublisher<NodeT>> published_time_publisher_;

  /** \brief Virtual abstract filter method. To be implemented by every child.
   * \param input the input point cloud dataset.
   * \param indices a pointer to the vector of point indices to use.
   * \param output the resultant filtered PointCloud2
   */
  virtual void filter(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output) = 0;

  // TODO(sykwer): Temporary Implementation: Remove this interface when all the filter nodes conform
  // to new API. It's not pure virtual function so that a child class does not have to implement
  // this function.
  virtual void faster_filter(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output,
    const TransformInfo & transform_info);  // != 0

  /** \brief Lazy transport subscribe routine. */
  virtual void subscribe(const std::string & filter_name);
  virtual void subscribe();

  /** \brief Lazy transport unsubscribe routine. */
  virtual void unsubscribe();

  /** \brief Call the child filter () method, optionally transform the result, and publish it.
   * \param input the input point cloud dataset.
   * \param indices a pointer to the vector of point indices to use.
   */
  virtual void compute_publish(const PointCloud2ConstPtr & input, const IndicesPtr & indices);
  /** \brief PointCloud2 + Indices data callback. */
  virtual void input_indices_callback(
    const PointCloud2ConstPtr cloud, const PointIndicesConstPtr indices);
  virtual bool convert_output_costly(std::unique_ptr<PointCloud2> & output);

  //////////////////////
  // from PCLNodelet //
  //////////////////////
  /** \brief Set to true if point indices are used.
   *
   * When receiving a point cloud, if use_indices_ is false, the entire
   * point cloud is processed for the given operation. If use_indices_ is
   * true, then the ~indices topic is read to get the vector of point
   * indices specifying the subset of the point cloud that will be used for
   * the operation. In the case where use_indices_ is true, the ~input and
   * ~indices topics must be synchronised in time, either exact or within a
   * specified jitter. See also @ref latched_indices_ and approximate_sync.
   **/
  bool use_indices_ = false;
  /** \brief Set to true if the indices topic is latched.
   *
   * If use_indices_ is true, the ~input and ~indices topics generally must
   * be synchronised in time. By setting this flag to true, the most recent
   * value from ~indices can be used instead of requiring a synchronised
   * message.
   **/
  bool latched_indices_ = false;

  /** \brief The maximum queue size (default: 3). */
  size_t max_queue_size_ = 3;

  /** \brief True if we use an approximate time synchronizer
   * versus an exact one (false by default). */
  bool approximate_sync_ = false;

  using ManagedTFBufferType = std::conditional_t<
    is_agnocast_node_v<NodeT>,
    managed_transform_buffer::AgnocastManagedTransformBuffer,
    managed_transform_buffer::ManagedTransformBuffer>;
  std::unique_ptr<ManagedTFBufferType> managed_tf_buffer_{nullptr};

  inline bool is_valid(
    const PointCloud2ConstPtr & cloud, const std::string & /*topic_name*/ = "input")
  {
    if (cloud->width * cloud->height * cloud->point_step != cloud->data.size()) {
      RCLCPP_WARN(
        this->get_logger(),
        "Invalid PointCloud (data = %zu, width = %d, height = %d, step = %d) with stamp %f, "
        "and frame %s received!",
        cloud->data.size(), cloud->width, cloud->height, cloud->point_step,
        rclcpp::Time(cloud->header.stamp).seconds(), cloud->header.frame_id.c_str());
      return false;
    }
    return true;
  }

  static inline bool is_valid(
    const PointIndicesConstPtr & /*indices*/, const std::string & /*topic_name*/ = "indices")
  {
    return true;
  }

  static inline bool is_valid(
    const ModelCoefficientsConstPtr & /*model*/, const std::string & /*topic_name*/ = "model")
  {
    return true;
  }

private:
  /** \brief Parameter service callback result : needed to be hold */
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr set_param_res_filter_;

  /** \brief Parameter service callback */
  rcl_interfaces::msg::SetParametersResult filter_param_callback(
    const std::vector<rclcpp::Parameter> & p);

  /** \brief Synchronized input, and indices. */
  std::conditional_t<
    is_rclcpp_node_v<NodeT>, std::shared_ptr<ExactTimeSyncPolicy>,
    std::shared_ptr<AgnocastExactTimeSyncPolicy>>
    sync_input_indices_e_;
  std::conditional_t<
    is_rclcpp_node_v<NodeT>, std::shared_ptr<ApproximateTimeSyncPolicy>,
    std::shared_ptr<AgnocastApproximateTimeSyncPolicy>>
    sync_input_indices_a_;

  /** \brief Get a matrix for conversion from the original frame to the target frame */
  bool calculate_transform_matrix(
    const std::string & target_frame, const sensor_msgs::msg::PointCloud2 & from,
    TransformInfo & transform_info /*output*/);

  // TODO(sykwer): Temporary Implementation: Remove this interface when all the filter nodes conform
  // to new API.
  void faster_input_indices_callback(
    const PointCloud2ConstPtr cloud, const PointIndicesConstPtr indices);

  void setup_tf();
};

// Backward compatibility: Filter class inherits from FilterBase<rclcpp::Node>
class Filter : public FilterBase<rclcpp::Node>
{
public:
  explicit Filter(
    const std::string & filter_name = "pointcloud_preprocessor_filter",
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : FilterBase<rclcpp::Node>(filter_name, options)
  {
  }
};

// ============================================================================
// Template Implementation
// ============================================================================

template <typename NodeT>
FilterBase<NodeT>::FilterBase(const std::string & filter_name, const rclcpp::NodeOptions & options)
: NodeT(filter_name, options), filter_field_name_(filter_name)
{
  // Debug log to confirm node type
  if constexpr (is_rclcpp_node_v<NodeT>) {
    RCLCPP_INFO(
      this->get_logger(),
      "\n=============================="
      "\n[FilterBase] NodeT = rclcpp::Node"
      "\n  filter_name: %s"
      "\n==============================",
      filter_name.c_str());
  } else if constexpr (is_agnocast_node_v<NodeT>) {
    RCLCPP_INFO(
      this->get_logger(),
      "\n=============================="
      "\n[FilterBase] NodeT = agnocast::Node"
      "\n  filter_name: %s"
      "\n==============================",
      filter_name.c_str());
  }

  // Set parameters (moved from NodeletLazy onInit)
  {
    tf_input_frame_ = this->template declare_parameter<std::string>("input_frame", std::string(""));
    tf_output_frame_ = this->template declare_parameter<std::string>("output_frame", std::string(""));
    max_queue_size_ = static_cast<std::size_t>(this->template declare_parameter<int>("max_queue_size", 5));

    // ---[ Optional parameters
    use_indices_ = this->template declare_parameter<bool>("use_indices", false);
    latched_indices_ = this->template declare_parameter<bool>("latched_indices", false);
    approximate_sync_ = this->template declare_parameter<bool>("approximate_sync", false);

    RCLCPP_DEBUG_STREAM(
      this->get_logger(),
      "Filter (as Component) successfully created with the following parameters:"
        << std::endl
        << " - approximate_sync : " << (approximate_sync_ ? "true" : "false") << std::endl
        << " - use_indices      : " << (use_indices_ ? "true" : "false") << std::endl
        << " - latched_indices  : " << (latched_indices_ ? "true" : "false") << std::endl
        << " - max_queue_size   : " << max_queue_size_);
  }

  // Set publisher
  if constexpr (is_rclcpp_node_v<NodeT>) {
    rclcpp::PublisherOptions pub_options;
    pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
    pub_output_ = this->template create_publisher<PointCloud2>(
      "output", rclcpp::SensorDataQoS().keep_last(max_queue_size_), pub_options);
  } else if constexpr (is_agnocast_node_v<NodeT>) {
    pub_output_ = this->template create_publisher<PointCloud2>(
      "output", rclcpp::SensorDataQoS().keep_last(max_queue_size_));
  }

  subscribe(filter_name);

  // Set tf_listener, tf_buffer.
  setup_tf();

  // Set parameter service callback
  set_param_res_filter_ = this->add_on_set_parameters_callback(
    std::bind(&FilterBase::filter_param_callback, this, std::placeholders::_1));

  published_time_publisher_ =
    std::make_unique<autoware_utils::BasicPublishedTimePublisher<NodeT>>(this);
  RCLCPP_DEBUG(this->get_logger(), "[Filter Constructor] successfully created.");
}

template <typename NodeT>
void FilterBase<NodeT>::setup_tf()
{
  if constexpr (is_rclcpp_node_v<NodeT>) {
    managed_tf_buffer_ = std::make_unique<managed_transform_buffer::ManagedTransformBuffer>();
  } else if constexpr (is_agnocast_node_v<NodeT>) {
    managed_tf_buffer_ =
      std::make_unique<managed_transform_buffer::AgnocastManagedTransformBuffer>();
  }
}

template <typename NodeT>
void FilterBase<NodeT>::subscribe()
{
  std::string filter_name = "";
  subscribe(filter_name);
}

template <typename NodeT>
void FilterBase<NodeT>::subscribe(const std::string & filter_name)
{
  // TODO(sykwer): Change the corresponding node to subscribe to `faster_input_indices_callback`
  // each time a child class supports the faster version.
  // When all the child classes support the faster version, this workaround is deleted.
  std::set<std::string> supported_nodes = {
    "CropBoxFilter", "RingOutlierFilter", "VoxelGridDownsampleFilter", "ScanGroundFilter",
    "PointCloudDensifier"};

  if constexpr (is_rclcpp_node_v<NodeT>) {
    auto callback = supported_nodes.find(filter_name) != supported_nodes.end()
                      ? &FilterBase::faster_input_indices_callback
                      : &FilterBase::input_indices_callback;

    if (use_indices_) {
      // Subscribe to the input using a filter (only available for rclcpp::Node)
      sub_input_filter_.subscribe(
        this, "input", rclcpp::SensorDataQoS().keep_last(max_queue_size_).get_rmw_qos_profile());
      sub_indices_filter_.subscribe(
        this, "indices", rclcpp::SensorDataQoS().keep_last(max_queue_size_).get_rmw_qos_profile());

      if (approximate_sync_) {
        sync_input_indices_a_ = std::make_shared<ApproximateTimeSyncPolicy>(max_queue_size_);
        sync_input_indices_a_->connectInput(sub_input_filter_, sub_indices_filter_);
        sync_input_indices_a_->registerCallback(
          std::bind(callback, this, std::placeholders::_1, std::placeholders::_2));
      } else {
        sync_input_indices_e_ = std::make_shared<ExactTimeSyncPolicy>(max_queue_size_);
        sync_input_indices_e_->connectInput(sub_input_filter_, sub_indices_filter_);
        sync_input_indices_e_->registerCallback(
          std::bind(callback, this, std::placeholders::_1, std::placeholders::_2));
      }
    } else {
      // Subscribe in an old fashion to input only (no filters)
      std::function<void(const PointCloud2ConstPtr msg)> cb =
        std::bind(callback, this, std::placeholders::_1, PointIndicesConstPtr());
      sub_input_ = this->template create_subscription<PointCloud2>(
        "input", rclcpp::SensorDataQoS().keep_last(max_queue_size_), cb);
    }
  } else if constexpr (is_agnocast_node_v<NodeT>) {
    auto callback = supported_nodes.find(filter_name) != supported_nodes.end()
                      ? &FilterBase::faster_input_indices_callback
                      : &FilterBase::input_indices_callback;

    if (use_indices_) {
      // Subscribe using agnocast message_filters
      sub_input_filter_.subscribe(
        static_cast<agnocast::Node *>(this), "input",
        rclcpp::SensorDataQoS().keep_last(max_queue_size_).get_rmw_qos_profile());
      sub_indices_filter_.subscribe(
        static_cast<agnocast::Node *>(this), "indices",
        rclcpp::SensorDataQoS().keep_last(max_queue_size_).get_rmw_qos_profile());

      // NullP type for unused synchronizer slots
      using NullP = const std::shared_ptr<::message_filters::NullType const> &;
      using SyncCb = std::function<void(
        const agnocast::ipc_shared_ptr<PointCloud2 const> &,
        const agnocast::ipc_shared_ptr<PointIndices const> &, NullP, NullP, NullP, NullP, NullP,
        NullP, NullP)>;

      const SyncCb sync_cb = [this, callback](
                          const agnocast::ipc_shared_ptr<PointCloud2 const> & cloud,
                          const agnocast::ipc_shared_ptr<PointIndices const> & indices, NullP,
                          NullP, NullP, NullP, NullP, NullP, NullP) {
        auto cloud_ptr =
          std::shared_ptr<const PointCloud2>(cloud.get(), [](const PointCloud2 *) {});
        auto indices_ptr =
          std::shared_ptr<const PointIndices>(indices.get(), [](const PointIndices *) {});
        (this->*callback)(cloud_ptr, indices_ptr);
      };

      if (approximate_sync_) {
        agnocast::message_filters::sync_policies::ApproximateTime<PointCloud2, PointIndices>
          approx_policy(max_queue_size_);
        sync_input_indices_a_ =
          std::make_shared<AgnocastApproximateTimeSyncPolicy>(approx_policy);
        sync_input_indices_a_->connectInput(sub_input_filter_, sub_indices_filter_);
        sync_input_indices_a_->registerCallback(sync_cb);
      } else {
        agnocast::message_filters::sync_policies::ExactTime<PointCloud2, PointIndices> exact_policy(
          max_queue_size_);
        sync_input_indices_e_ = std::make_shared<AgnocastExactTimeSyncPolicy>(exact_policy);
        sync_input_indices_e_->connectInput(sub_input_filter_, sub_indices_filter_);
        sync_input_indices_e_->registerCallback(sync_cb);
      }
    } else {
      sub_input_ = this->template create_subscription<PointCloud2>(
        "input", rclcpp::SensorDataQoS().keep_last(max_queue_size_),
        [this, callback](const agnocast::ipc_shared_ptr<PointCloud2> & msg) {
          auto input_ptr =
            std::shared_ptr<const PointCloud2>(msg.get(), [](const PointCloud2 *) {});
          (this->*callback)(input_ptr, PointIndicesConstPtr());
        });
    }
  }
}

template <typename NodeT>
void FilterBase<NodeT>::unsubscribe()
{
  if (use_indices_) {
    sub_input_filter_.unsubscribe();
    sub_indices_filter_.unsubscribe();
    if (approximate_sync_) {
      sync_input_indices_a_.reset();
    } else {
      sync_input_indices_e_.reset();
    }
  } else {
    sub_input_.reset();
  }
}

template <typename NodeT>
void FilterBase<NodeT>::compute_publish(
  const PointCloud2ConstPtr & input, const IndicesPtr & indices)
{
  if constexpr (is_rclcpp_node_v<NodeT>) {
    auto output = std::make_unique<PointCloud2>();

    // Call the virtual method in the child
    filter(input, indices, *output);

    if (!convert_output_costly(output)) return;

    // Copy timestamp to keep it
    output->header.stamp = input->header.stamp;

    // Publish a boost shared ptr
    pub_output_->publish(std::move(output));
    published_time_publisher_->publish_if_subscribed(pub_output_, input->header.stamp);
  } else if constexpr (is_agnocast_node_v<NodeT>) {
    // First process with unique_ptr to allow convert_output_costly to work
    auto output = std::make_unique<PointCloud2>();

    // Call the virtual method in the child
    filter(input, indices, *output);

    if (!convert_output_costly(output)) return;

    // Copy timestamp to keep it
    output->header.stamp = input->header.stamp;

    // Copy to loaned message and publish
    auto loaned_output = pub_output_->borrow_loaned_message();
    *loaned_output = *output;
    pub_output_->publish(std::move(loaned_output));
    published_time_publisher_->template publish_if_subscribed<PointCloud2>(
      pub_output_, input->header.stamp);
  }
}

template <typename NodeT>
rcl_interfaces::msg::SetParametersResult FilterBase<NodeT>::filter_param_callback(
  const std::vector<rclcpp::Parameter> & p)
{
  std::scoped_lock lock(mutex_);

  if (get_param(p, "input_frame", tf_input_frame_)) {
    RCLCPP_DEBUG(this->get_logger(), "Setting the input TF frame to: %s.", tf_input_frame_.c_str());
  }
  if (get_param(p, "output_frame", tf_output_frame_)) {
    RCLCPP_DEBUG(this->get_logger(), "Setting the output TF frame to: %s.", tf_output_frame_.c_str());
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}

template <typename NodeT>
void FilterBase<NodeT>::input_indices_callback(
  const PointCloud2ConstPtr cloud, const PointIndicesConstPtr indices)
{
  // If cloud is given, check if it's valid
  if (!is_valid(cloud)) {
    RCLCPP_ERROR(this->get_logger(), "[input_indices_callback] Invalid input!");
    return;
  }
  // If indices are given, check if they are valid
  if (indices && !is_valid(indices)) {
    RCLCPP_ERROR(this->get_logger(), "[input_indices_callback] Invalid indices!");
    return;
  }

  /// DEBUG
  if (indices) {
    RCLCPP_DEBUG(
      this->get_logger(),
      "[input_indices_callback]\n"
      "   - PointCloud with %d data points (%s), stamp %f, and frame %s on input topic received.\n"
      "   - PointIndices with %zu values, stamp %f, and frame %s on indices topic received.",
      cloud->width * cloud->height, pcl::getFieldsList(*cloud).c_str(),
      rclcpp::Time(cloud->header.stamp).seconds(), cloud->header.frame_id.c_str(),
      indices->indices.size(), rclcpp::Time(indices->header.stamp).seconds(),
      indices->header.frame_id.c_str());
  } else {
    RCLCPP_DEBUG(
      this->get_logger(),
      "[input_indices_callback] PointCloud with %d data points and frame %s on input topic "
      "received.",
      cloud->width * cloud->height, cloud->header.frame_id.c_str());
  }
  ///

  // Check whether the user has given a different input TF frame
  tf_input_orig_frame_ = cloud->header.frame_id;
  PointCloud2ConstPtr cloud_tf;
  if (!tf_input_frame_.empty() && cloud->header.frame_id != tf_input_frame_) {
    RCLCPP_DEBUG(
      this->get_logger(), "[input_indices_callback] Transforming input dataset from %s to %s.",
      cloud->header.frame_id.c_str(), tf_input_frame_.c_str());
    // Save the original frame ID
    // Convert the cloud into the different frame
    PointCloud2 cloud_transformed;

    if (!managed_tf_buffer_->transformPointcloud(
          tf_input_frame_, *cloud, cloud_transformed, cloud->header.stamp,
          rclcpp::Duration::from_seconds(1.0), this->get_logger())) {
      return;
    }
    cloud_tf = std::make_shared<PointCloud2>(cloud_transformed);
  } else {
    cloud_tf = cloud;
  }
  // Need setInputCloud () here because we have to extract x/y/z
  IndicesPtr vindices;
  if (indices) {
    vindices.reset(new std::vector<int>(indices->indices));
  }

  compute_publish(cloud_tf, vindices);
}

template <typename NodeT>
bool FilterBase<NodeT>::calculate_transform_matrix(
  const std::string & target_frame, const sensor_msgs::msg::PointCloud2 & from,
  TransformInfo & transform_info /*output*/)
{
  transform_info.need_transform = false;

  if (target_frame.empty() || from.header.frame_id == target_frame) return true;

  RCLCPP_DEBUG(
    this->get_logger(), "[get_transform_matrix] Transforming input dataset from %s to %s.",
    from.header.frame_id.c_str(), target_frame.c_str());

  auto eigen_transform_opt = managed_tf_buffer_->template getTransform<Eigen::Matrix4f>(
    target_frame, from.header.frame_id, from.header.stamp, rclcpp::Duration::from_seconds(1.0),
    this->get_logger());
  if (!eigen_transform_opt) {
    return false;
  }

  transform_info.eigen_transform = *eigen_transform_opt;
  transform_info.need_transform = true;
  return true;
}

template <typename NodeT>
bool FilterBase<NodeT>::convert_output_costly(std::unique_ptr<PointCloud2> & output)
{
  // In terms of performance, we should avoid using pcl_ros library function,
  // but this code path isn't reached in the main use case of Autoware, so it's left as is for now.
  if (!tf_output_frame_.empty() && output->header.frame_id != tf_output_frame_) {
    RCLCPP_DEBUG(
      this->get_logger(), "[convert_output_costly] Transforming output dataset from %s to %s.",
      output->header.frame_id.c_str(), tf_output_frame_.c_str());

    // Convert the cloud into the different frame
    auto cloud_transformed = std::make_unique<PointCloud2>();

    if (!managed_tf_buffer_->transformPointcloud(
          tf_output_frame_, *output, *cloud_transformed, output->header.stamp,
          rclcpp::Duration::from_seconds(1.0), this->get_logger())) {
      RCLCPP_ERROR(
        this->get_logger(),
        "[convert_output_costly] Error converting output dataset from %s to %s.",
        output->header.frame_id.c_str(), tf_output_frame_.c_str());
      return false;
    }

    output = std::move(cloud_transformed);
  }

  // Same as the comment above
  if (tf_output_frame_.empty() && output->header.frame_id != tf_input_orig_frame_) {
    // No tf_output_frame given, transform the dataset to its original frame
    RCLCPP_DEBUG(
      this->get_logger(), "[convert_output_costly] Transforming output dataset from %s back to %s.",
      output->header.frame_id.c_str(), tf_input_orig_frame_.c_str());

    auto cloud_transformed = std::make_unique<PointCloud2>();

    if (!managed_tf_buffer_->transformPointcloud(
          tf_input_orig_frame_, *output, *cloud_transformed, output->header.stamp,
          rclcpp::Duration::from_seconds(1.0), this->get_logger())) {
      return false;
    }

    output = std::move(cloud_transformed);
  }

  return true;
}

template <typename NodeT>
void FilterBase<NodeT>::faster_input_indices_callback(
  const PointCloud2ConstPtr cloud, const PointIndicesConstPtr indices)
{
  if (
    !utils::is_data_layout_compatible_with_point_xyzircaedt(*cloud) &&
    !utils::is_data_layout_compatible_with_point_xyzirc(*cloud)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "The pointcloud layout is not compatible with PointXYZIRCAEDT or PointXYZIRC. Aborting");

    if (utils::is_data_layout_compatible_with_point_xyziradrt(*cloud)) {
      RCLCPP_ERROR(
        this->get_logger(),
        "The pointcloud layout is compatible with PointXYZIRADRT. You may be using legacy "
        "code/data");
    }

    if (utils::is_data_layout_compatible_with_point_xyzi(*cloud)) {
      RCLCPP_ERROR(
        this->get_logger(),
        "The pointcloud layout is compatible with PointXYZI. You may be using legacy "
        "code/data");
    }

    return;
  }

  if (!is_valid(cloud)) {
    RCLCPP_ERROR(this->get_logger(), "[input_indices_callback] Invalid input!");
    return;
  }

  if (indices && !is_valid(indices)) {
    RCLCPP_ERROR(this->get_logger(), "[input_indices_callback] Invalid indices!");
    return;
  }

  // Add RCLCPP_INFO_THROTTLE for rclcpp::Node
  if constexpr (is_rclcpp_node_v<NodeT>) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "\n=============================="
      "\n[FilterBase] rclcpp::Node CALLBACK INVOKED"
      "\n  frame_id: %s"
      "\n  points: %u"
      "\n==============================",
      cloud->header.frame_id.c_str(), cloud->width * cloud->height);
  } else if constexpr (is_agnocast_node_v<NodeT>) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "\n=============================="
      "\n[FilterBase] agnocast::Node CALLBACK INVOKED"
      "\n  frame_id: %s"
      "\n  points: %u"
      "\n==============================",
      cloud->header.frame_id.c_str(), cloud->width * cloud->height);
  }

  if (indices) {
    RCLCPP_DEBUG(
      this->get_logger(),
      "[input_indices_callback]\n"
      "   - PointCloud with %d data points (%s), stamp %f, and frame %s on input topic received.\n"
      "   - PointIndices with %zu values, stamp %f, and frame %s on indices topic received.",
      cloud->width * cloud->height, pcl::getFieldsList(*cloud).c_str(),
      rclcpp::Time(cloud->header.stamp).seconds(), cloud->header.frame_id.c_str(),
      indices->indices.size(), rclcpp::Time(indices->header.stamp).seconds(),
      indices->header.frame_id.c_str());
  } else {
    RCLCPP_DEBUG(
      this->get_logger(),
      "[input_indices_callback] PointCloud with %d data points and frame %s on input topic "
      "received.",
      cloud->width * cloud->height, cloud->header.frame_id.c_str());
  }

  tf_input_orig_frame_ = cloud->header.frame_id;

  // For performance reason, defer the transform computation.
  // Do not use pcl_ros::transformPointCloud(). It's too slow due to the unnecessary copy.
  TransformInfo transform_info;
  if (!calculate_transform_matrix(tf_input_frame_, *cloud, transform_info)) return;

  // Need setInputCloud() here because we have to extract x/y/z
  IndicesPtr vindices;
  if (indices) {
    vindices.reset(new std::vector<int>(indices->indices));
  }

  if constexpr (is_rclcpp_node_v<NodeT>) {
    auto output = std::make_unique<PointCloud2>();

    // TODO(sykwer): Change to `filter()` call after when the filter nodes conform to new API.
    faster_filter(cloud, vindices, *output, transform_info);

    if (!convert_output_costly(output)) return;

    output->header.stamp = cloud->header.stamp;
    pub_output_->publish(std::move(output));
    published_time_publisher_->publish_if_subscribed(pub_output_, cloud->header.stamp);
  } else if constexpr (is_agnocast_node_v<NodeT>) {
    // Use a local buffer for faster_filter so that no loaned message is outstanding
    // while faster_filter publishes diagnostics/debug messages.
    PointCloud2 output_buffer;

    // TODO(sykwer): Change to `filter()` call after when the filter nodes conform to new API.
    faster_filter(cloud, vindices, output_buffer, transform_info);

    // For agnocast, we skip the costly transform for now
    output_buffer.header.stamp = cloud->header.stamp;
    auto output = pub_output_->borrow_loaned_message();
    *output = output_buffer;
    pub_output_->publish(std::move(output));
    published_time_publisher_->template publish_if_subscribed<PointCloud2>(
      pub_output_, cloud->header.stamp);
  }
}

template <typename NodeT>
void FilterBase<NodeT>::faster_filter(
  const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output,
  const TransformInfo & transform_info)
{
  (void)input;
  (void)indices;
  (void)output;
  (void)transform_info;
}

}  // namespace autoware::pointcloud_preprocessor

#endif  // AUTOWARE__POINTCLOUD_PREPROCESSOR__FILTER_HPP_
