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

#include "selection_json_export_panel.hpp"

#include <rviz_common/display_context.hpp>
#include <rviz_common/interaction/selection_manager_iface.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/properties/property_tree_model.hpp>

#include <QApplication>
#include <QClipboard>
#include <QColor>
#include <QFile>
#include <QFileDialog>
#include <QIODevice>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonValue>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QTextEdit>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>

namespace autoware_selection_json_export_rviz_plugin
{

namespace
{

QJsonValue variantToJsonValue(const QVariant & v)
{
  if (!v.isValid()) {
    return QJsonValue::Null;
  }

  switch (v.type()) {
    case QVariant::Int:
      return v.toInt();
    case QVariant::UInt:
      return static_cast<qint64>(v.toUInt());
    case QVariant::LongLong:
      return QString::number(v.toLongLong());
    case QVariant::ULongLong:
      return QString::number(v.toULongLong());
    case QVariant::Double:
      return v.toDouble();
    case QVariant::Bool:
      return v.toBool();
    case QVariant::String:
      return v.toString();
    case QVariant::Color: {
      const QColor c = v.value<QColor>();
      return QStringLiteral("rgba(%1,%2,%3,%4)")
        .arg(c.red())
        .arg(c.green())
        .arg(c.blue())
        .arg(c.alpha());
    }
    default:
      break;
  }

  if (v.canConvert<QString>()) {
    return v.toString();
  }
  return QJsonValue(QStringLiteral("<unserializable:%1>").arg(static_cast<int>(v.type())));
}

QJsonObject propertyToJson(rviz_common::properties::Property * prop, int depth_limit)
{
  QJsonObject o;
  if (!prop || depth_limit <= 0) {
    o[QStringLiteral("error")] = QStringLiteral("null_or_depth_exceeded");
    return o;
  }

  o[QStringLiteral("name")] = prop->getName();

  const QString desc = prop->getDescription();
  if (!desc.isEmpty()) {
    o[QStringLiteral("description")] = desc;
  }

  const int n = prop->numChildren();
  if (n > 0) {
    QJsonArray children;
    for (int i = 0; i < n; ++i) {
      rviz_common::properties::Property * ch = prop->childAt(i);
      if (ch) {
        children.append(propertyToJson(ch, depth_limit - 1));
      }
    }
    o[QStringLiteral("children")] = children;
  }

  const QVariant val = prop->getValue();
  if (val.isValid()) {
    o[QStringLiteral("value")] = variantToJsonValue(val);
  }

  return o;
}

QJsonArray pickedToJsonArray(const rviz_common::interaction::M_Picked & picked)
{
  QJsonArray arr;
  for (const auto & entry : picked) {
    QJsonObject o;
    o[QStringLiteral("handle")] = static_cast<qint64>(entry.first);
    o[QStringLiteral("pixel_count")] = entry.second.pixel_count;
    QJsonArray extra;
    for (const uint64_t h : entry.second.extra_handles) {
      extra.append(QString::number(h));
    }
    if (!extra.isEmpty()) {
      o[QStringLiteral("extra_handles")] = extra;
    }
    arr.append(o);
  }
  return arr;
}

}  // namespace

SelectionJsonExportPanel::SelectionJsonExportPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  auto * refresh_btn = new QPushButton(QStringLiteral("Refresh JSON"));
  auto * copy_btn = new QPushButton(QStringLiteral("Copy"));
  auto * save_btn = new QPushButton(QStringLiteral("Save to file..."));

  json_view_ = new QTextEdit;
  json_view_->setReadOnly(true);
  json_view_->setFontFamily(QStringLiteral("monospace"));
  json_view_->setPlaceholderText(
    QStringLiteral("Use RViz \"Select\" tool on an object, then press \"Refresh JSON\"."));

  auto * btn_row = new QHBoxLayout;
  btn_row->addWidget(refresh_btn);
  btn_row->addWidget(copy_btn);
  btn_row->addWidget(save_btn);
  btn_row->addStretch(1);

  auto * layout = new QVBoxLayout(this);
  layout->addWidget(new QLabel(QStringLiteral("Selection → JSON export")));
  layout->addLayout(btn_row);
  layout->addWidget(json_view_, 1);
  setLayout(layout);

  connect(refresh_btn, &QPushButton::clicked, this, &SelectionJsonExportPanel::onRefresh);
  connect(copy_btn, &QPushButton::clicked, this, &SelectionJsonExportPanel::onCopyClipboard);
  connect(save_btn, &QPushButton::clicked, this, &SelectionJsonExportPanel::onSaveFile);
}

void SelectionJsonExportPanel::onInitialize()
{
  // Panel is ready after RViz wires DisplayContext.
}

void SelectionJsonExportPanel::onRefresh()
{
  auto ctx = getDisplayContext();
  if (!ctx) {
    json_view_->setPlainText(QStringLiteral("{\"error\":\"no_display_context\"}"));
    return;
  }

  auto sel_mgr = ctx->getSelectionManager();
  if (!sel_mgr) {
    json_view_->setPlainText(QStringLiteral("{\"error\":\"no_selection_manager\"}"));
    return;
  }

  QJsonObject root;
  root[QStringLiteral("picked")] = pickedToJsonArray(sel_mgr->getSelection());

  rviz_common::properties::PropertyTreeModel * model = sel_mgr->getPropertyModel();
  if (model && model->getRoot()) {
    root[QStringLiteral("property_tree")] = propertyToJson(model->getRoot(), 256);
  } else {
    root[QStringLiteral("property_tree")] = QJsonValue::Null;
  }

  const QJsonDocument doc(root);
  json_view_->setPlainText(doc.toJson(QJsonDocument::Indented));
}

void SelectionJsonExportPanel::onCopyClipboard()
{
  const QString t = json_view_->toPlainText();
  if (t.isEmpty()) {
    onRefresh();
  }
  QApplication::clipboard()->setText(json_view_->toPlainText());
}

void SelectionJsonExportPanel::onSaveFile()
{
  if (json_view_->toPlainText().trimmed().isEmpty()) {
    onRefresh();
  }
  const QString path = QFileDialog::getSaveFileName(
    this, QStringLiteral("Save selection JSON"), QStringLiteral("rviz_selection.json"),
    QStringLiteral("JSON (*.json);;All (*)"));
  if (path.isEmpty()) {
    return;
  }
  QFile f(path);
  if (!f.open(QIODevice::WriteOnly | QIODevice::Text)) {
    QMessageBox::warning(
      this, QStringLiteral("Save failed"),
      QStringLiteral("Could not open file for writing: %1").arg(path));
    return;
  }
  f.write(json_view_->toPlainText().toUtf8());
}

}  // namespace autoware_selection_json_export_rviz_plugin

PLUGINLIB_EXPORT_CLASS(
  autoware_selection_json_export_rviz_plugin::SelectionJsonExportPanel, rviz_common::Panel)
