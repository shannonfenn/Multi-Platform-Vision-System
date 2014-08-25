#include <iostream>

#ifdef TARGET_IS_RPI
    #include "VisionWrapper/visioncontrolwrapperrpi.h"
#elif TARGET_IS_PC || TARGET_IS_MAC || TARGET_IS_WINDOWS
    //#include "VisionWrapper/visioncontrolwrapperpc.h"
    #include "VisionWrapper/visioncontrolwrapperqt.h"
#elif TARGET_IS_NUVIEW
    #include "Vision/VisionWrapper/visioncontrolwrappernuview.h"
#elif TARGET_IS_TRAINING
    #include "Vision/VisionWrapper/visioncontrolwrappertraining.h"
#else
    #include "Vision/VisionWrapper/visioncontrolwrapperdarwin.h"
#endif

#ifndef TARGET_IS_RPI
#include <QApplication>
#include <QFileDialog>
#include <QMessageBox>

//for catching exceptions
class MyApplication : public QApplication {
public:
    MyApplication(int& argc, char ** argv) : QApplication(argc, argv) { }
    //MyApplication(Display* dpy, Qt::HANDLE visual = 0, Qt::HANDLE cmap = 0, int flags = ApplicationFlags) : QApplication(dpy, visual, cmap, flags) { }
    virtual ~MyApplication() { }

    // reimplemented from QApplication so we can throw exceptions in slots
    virtual bool notify(QObject * receiver, QEvent * event) {
        try {
        return QApplication::notify(receiver, event);
        }
        catch(const std::exception& e) {
            QMessageBox::warning(NULL, "Exception", QString("Exception thrown: ") + e.what());
        }
        catch (const std::string& ex) {
            QMessageBox::warning(NULL, "Exception", QString(("Manual exception thrown: " + ex).c_str()));
        }
        catch (const char* str) {
            QMessageBox::warning(NULL, "Exception", QString("Manual exception thrown: ") + str);
        }
        catch (...) {
            QMessageBox::warning(NULL, "Exception", "Unknown exception thrown.");
        }
        return false;
    }
};

int qt()
{
    int argc = 0;
    char** argv = NULL;
    MyApplication app(argc, argv);
    VisionControlWrapper* vision = VisionControlWrapper::getInstance();

    int error = vision->run();
    if(error != 0)
        std::cout << "Error: " << error << std::endl;
    return 0;
}
#else

int rpi()
{
    VisionControlWrapper* vision = VisionControlWrapper::getInstance();

    int frame_no = 0;
    int error=0;
    while(error==0) {
        error = vision->runFrame();
        frame_no += 1;
    }
    std::cout << "Done " << frame_no << " frames processed." << std::endl;
    return 0;
}
#endif

int main(int argc, char** argv)
{
    #ifdef TARGET_IS_RPI
    return rpi();
    #elif TARGET_IS_PC || TARGET_IS_MAC || TARGET_IS_WINDOWS
    return qt();
    #else
    std::cout << "Error not a valid define! Must be TARGET_IS_RPI or TARGET_IS_PC" << std::endl;
    return 0;
    #endif
}
