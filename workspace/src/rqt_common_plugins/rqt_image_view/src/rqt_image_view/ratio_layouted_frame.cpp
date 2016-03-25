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

#include <rqt_image_view/ratio_layouted_frame.h>

#include <assert.h>

namespace rqt_image_view {

RatioLayoutedFrame::RatioLayoutedFrame(QWidget* parent, Qt::WFlags flags)
  : QFrame()
  , aspect_ratio_(4, 3)
{
  connect(this, SIGNAL(delayed_update()), this, SLOT(update()), Qt::QueuedConnection);
}

RatioLayoutedFrame::~RatioLayoutedFrame()
{
}

const QImage& RatioLayoutedFrame::getImage() const
{
  return qimage_;
}

QImage RatioLayoutedFrame::getImageCopy() const
{
  QImage img;
  qimage_mutex_.lock();
  img = qimage_.copy();
  qimage_mutex_.unlock();
  return img;
}

void RatioLayoutedFrame::setImage(const QImage& image)//, QMutex* image_mutex)
{
  qimage_mutex_.lock();
  qimage_ = image.copy();
  qimage_mutex_.unlock();
  setAspectRatio(qimage_.width(), qimage_.height());
  emit delayed_update();
}

void RatioLayoutedFrame::resizeToFitAspectRatio()
{
  QRect rect = contentsRect();

  // reduce longer edge to aspect ration
  double width = double(rect.width());
  double height = double(rect.height());
  if (width * aspect_ratio_.height() / height > aspect_ratio_.width())
  {
    // too large width
    width = height * aspect_ratio_.width() / aspect_ratio_.height();
    rect.setWidth(int(width + 0.5));
  }
  else
  {
    // too large height
    height = width * aspect_ratio_.height() / aspect_ratio_.width();
    rect.setHeight(int(height + 0.5));
  }

  // resize taking the border line into account
  int border = lineWidth();
  resize(rect.width() + 2 * border, rect.height() + 2 * border);
}

void RatioLayoutedFrame::setInnerFrameMinimumSize(const QSize& size)
{
  int border = lineWidth();
  QSize new_size = size;
  new_size += QSize(2 * border, 2 * border);
  setMinimumSize(new_size);
  emit delayed_update();
}

void RatioLayoutedFrame::setInnerFrameMaximumSize(const QSize& size)
{
  int border = lineWidth();
  QSize new_size = size;
  new_size += QSize(2 * border, 2 * border);
  setMaximumSize(new_size);
  emit delayed_update();
}

void RatioLayoutedFrame::setInnerFrameFixedSize(const QSize& size)
{
  setInnerFrameMinimumSize(size);
  setInnerFrameMaximumSize(size);
}

void RatioLayoutedFrame::setAspectRatio(unsigned short width, unsigned short height)
{
  int divisor = greatestCommonDivisor(width, height);
  if (divisor != 0) {
    aspect_ratio_.setWidth(width / divisor);
    aspect_ratio_.setHeight(height / divisor);
  }
}

void RatioLayoutedFrame::paintEvent(QPaintEvent* event)
{
  QPainter painter(this);
  qimage_mutex_.lock();
  if (!qimage_.isNull())
  {
    resizeToFitAspectRatio();
    // TODO: check if full draw is really necessary
    //QPaintEvent* paint_event = dynamic_cast<QPaintEvent*>(event);
    //painter.drawImage(paint_event->rect(), qimage_);
    painter.drawImage(contentsRect(), qimage_);
  } else {
    // default image with gradient
    QLinearGradient gradient(0, 0, frameRect().width(), frameRect().height());
    gradient.setColorAt(0, Qt::white);
    gradient.setColorAt(1, Qt::black);
    painter.setBrush(gradient);
    painter.drawRect(0, 0, frameRect().width() + 1, frameRect().height() + 1);
  }
  qimage_mutex_.unlock();
}

int RatioLayoutedFrame::greatestCommonDivisor(int a, int b)
{
  if (b==0)
  {
    return a;
  }
  return greatestCommonDivisor(b, a % b);
}

}
