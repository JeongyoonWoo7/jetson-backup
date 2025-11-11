#pragma once

#include <QWidget>
#include <QLabel>
#include <QImage>
#include <QVBoxLayout>
#include <QPixmap>

class ImageWidget : public QWidget
{
    Q_OBJECT
public:
    ImageWidget(QWidget *parent = nullptr);

public slots:
    void updateImage(const QImage &img);

private:
    QLabel *label_;
};



/* 
<class>auto_frame</class>
 <widget class="QMainWindow" name="move_ui">
 */