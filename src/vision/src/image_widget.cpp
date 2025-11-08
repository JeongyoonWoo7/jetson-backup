#include "vision/image_widget.hpp"

ImageWidget::ImageWidget(QWidget *parent)
    : QWidget(parent)
{
    label_ = new QLabel(this);
    label_->setAlignment(Qt::AlignCenter);

    auto layout = new QVBoxLayout(this);
    layout->addWidget(label_);
    setLayout(layout);
}

void ImageWidget::updateImage(const QImage &img)
{
    if (!img.isNull()) {
        label_->setPixmap(QPixmap::fromImage(img).scaled(
            label_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    }
}
