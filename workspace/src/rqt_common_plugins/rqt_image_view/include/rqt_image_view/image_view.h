/*
 * Copyright (c) 2011, Dirk Thomas, TU Darmstadt
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the TU Darmstadt nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef rqt_image_view__ImageView_H
#define rqt_image_view__ImageView_H

#include <rqt_gui_cpp/plugin.h>

#include <ui_image_view.h>

#include <image_transport/image_transport.h>
#include <ros/macros.h>
#include <sensor_msgs/Image.h>

#include <opencv2/core/core.hpp>

#include <QList>
#include <QString>
#include <QSize>
#include <QWidget>

#include <vector>

namespace rqt_image_view {

class ImageView
  : public rqt_gui_cpp::Plugin
{

  Q_OBJECT

public:

  ImageView();

  virtual void initPlugin(qt_gui_cpp::PluginContext& context);

  virtual void shutdownPlugin();

  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;

  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

protected slots:

  virtual void updateTopicList();

protected:

  // deprecated function for backward compatibility only, use getTopics() instead
  ROS_DEPRECATED virtual QList<QString> getTopicList(const QSet<QString>& message_types, const QList<QString>& transports);

  virtual QSet<QString> getTopics(const QSet<QString>& message_types, const QSet<QString>& message_sub_types, const QList<QString>& transports);

  virtual void selectTopic(const QString& topic);

protected slots:

  virtual void onTopicChanged(int index);

  virtual void onZoom1(bool checked);

  virtual void onDynamicRange(bool checked);

  virtual void saveImage();

protected:

  virtual void callbackImage(const sensor_msgs::Image::ConstPtr& msg);

  Ui::ImageViewWidget ui_;

  QWidget* widget_;

  image_transport::Subscriber subscriber_;

  cv::Mat conversion_mat_;

private:

  QString arg_topic_name;

};

}

#endif // rqt_image_view__ImageView_H
