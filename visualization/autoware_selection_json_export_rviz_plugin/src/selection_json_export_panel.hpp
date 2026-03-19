// Copyright 2025
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

#ifndef AUTOWARE_SELECTION_JSON_EXPORT_RVIZ_PLUGIN__SELECTION_JSON_EXPORT_PANEL_HPP_
#define AUTOWARE_SELECTION_JSON_EXPORT_RVIZ_PLUGIN__SELECTION_JSON_EXPORT_PANEL_HPP_

#include <rviz_common/panel.hpp>

class QTextEdit;

namespace autoware_selection_json_export_rviz_plugin
{

class SelectionJsonExportPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit SelectionJsonExportPanel(QWidget * parent = nullptr);

  void onInitialize() override;

private Q_SLOTS:
  void onRefresh();
  void onCopyClipboard();
  void onSaveFile();

private:
  QTextEdit * json_view_{nullptr};
};

}  // namespace autoware_selection_json_export_rviz_plugin

#endif  // AUTOWARE_SELECTION_JSON_EXPORT_RVIZ_PLUGIN__SELECTION_JSON_EXPORT_PANEL_HPP_
