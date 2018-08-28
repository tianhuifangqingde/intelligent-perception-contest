#-------------------------------------------------
#
# Project created by QtCreator 2018-08-08T20:04:52
#
#-------------------------------------------------

QT       += core gui

QMAKE_CXXFLAGS += -std=c++11

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = intelligent
TEMPLATE = app


SOURCES += main.cpp

HEADERS  += \
    process.hpp

FORMS    +=

INCLUDEPATH+=/usr/local/include \
/usr/local/include/opencv \
/usr/local/include/opencv2 \
/usr/local/include/tesseract \


LIBS    +=  /usr/local/lib/libopencv_highgui.so.3.1.0 \
            /usr/local/lib/libopencv_photo.so.3.1.0 \
            /usr/local/lib/libopencv_calib3d.so.3.1.0 \
            /usr/local/lib/libopencv_imgproc.so.3.1.0 \
            /usr/local/lib/libopencv_stitching.so.3.1.0 \
            /usr/local/lib/libopencv_superres.so.3.1.0 \
            /usr/local/lib/libopencv_core.so.3.1.0 \
            /usr/local/lib/libopencv_ml.so.3.1.0 \
            /usr/local/lib/libopencv_video.so.3.1.0 \
            /usr/local/lib/libopencv_features2d.so.3.1.0 \
            /usr/local/lib/libopencv_videostab.so.3.1.0 \
            /usr/local/lib/libopencv_flann.so.3.1.0 \
            /usr/local/lib/libopencv_objdetect.so.3.1.0 \
            /usr/local/lib/libopencv_imgcodecs.so.3.1.0 \
            /usr/local/lib/libopencv_videoio.so.3.1.0 \
            /usr/local/lib/libtesseract.so \
            -lserial

