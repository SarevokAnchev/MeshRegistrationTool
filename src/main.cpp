#include <filesystem>
#include <iostream>

#include <QApplication>

#include "gui/MainWindow.h"

int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    QApplication::setQuitOnLastWindowClosed(true);
    MainWindow window;
    window.show();
    return QApplication::exec();
}
