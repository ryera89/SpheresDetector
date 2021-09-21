TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
        main.cpp

CONFIG += link_pkgconfig
PKGCONFIG += eigen3

INCLUDEPATH += /usr/include/pcl-1.10/
INCLUDEPATH += /usr/include/vtk-7.1/
LIBS += -L/usr/lib/x86_64-linux-gnu -lpcl_common -lpcl_filters -lpcl_io -lpcl_visualization -lpcl_search -lpcl_features -lpcl_segmentation \
-lpcl_sample_consensus -lvtkCommonCore-7.1 #-lvtksys-7.1   -lvtkCommonDataModel-7.1 -lvtkRenderingCore-7.1 -lvtkCommonMath-7.1 -lvtkRenderingLOD-7.1


HEADERS += \
    SpheresDetectionUtils.h
