// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef AUTOWARE__ML_PLANNER__PREPROCESSING__MESSAGE_VIEW_HPP_
#define AUTOWARE__ML_PLANNER__PREPROCESSING__MESSAGE_VIEW_HPP_

#include <boost/iterator/indirect_iterator.hpp>

#include <cstddef>
#include <deque>
#include <iterator>
#include <utility>
#include <vector>

namespace autoware::ml_planner::preprocess
{

template <typename MessageT>
class MessageView
{
public:
  using Storage = std::vector<const MessageT *>;
  using const_iterator = boost::indirect_iterator<typename Storage::const_iterator>;
  using const_reverse_iterator = std::reverse_iterator<const_iterator>;

  MessageView() = default;

  explicit MessageView(const std::deque<MessageT> & messages)
  {
    messages_.reserve(messages.size());
    for (const auto & message : messages) {
      messages_.push_back(&message);
    }
  }

  explicit MessageView(Storage messages) : messages_(std::move(messages)) {}

  bool empty() const { return messages_.empty(); }
  size_t size() const { return messages_.size(); }
  const MessageT & front() const { return *messages_.front(); }
  const MessageT & back() const { return *messages_.back(); }
  const MessageT & operator[](const size_t index) const { return *messages_[index]; }

  const_iterator begin() const { return const_iterator(messages_.begin()); }
  const_iterator end() const { return const_iterator(messages_.end()); }
  const_reverse_iterator rbegin() const { return const_reverse_iterator(end()); }
  const_reverse_iterator rend() const { return const_reverse_iterator(begin()); }

private:
  Storage messages_;
};

}  // namespace autoware::ml_planner::preprocess

#endif  // AUTOWARE__ML_PLANNER__PREPROCESSING__MESSAGE_VIEW_HPP_
