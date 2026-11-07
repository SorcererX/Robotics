#include <iostream>
#include <QApplication>
#include "movementwidget.h"
#include "settingswidget.h"
#include <QMainWindow>
#include <QTabWidget>
#include <sepia/comm2/qtmessagereceiver.h>
#include <sepia/comm2/scopedmessagesender.h>

int main( int argc, char** argv )
{
    QApplication app( argc, argv );

    sepia::comm2::ScopedMessageSender sender;

    // A main window keeps the window from becoming smaller than the tabs need.
    QMainWindow window;
    window.setWindowTitle( "RoboClaw" );
    auto* tabs = new QTabWidget( &window );
    MovementWidget* movementWidget = new MovementWidget( tabs );
    SettingsWidget* settingsWidget = new SettingsWidget( tabs );
    tabs->addTab( movementWidget, "Movement" );
    tabs->addTab( settingsWidget, "Settings" );
    window.setCentralWidget( tabs );
    window.show();

    sepia::comm2::QtMessageReceiver receiver( movementWidget, movementWidget );
    sepia::comm2::QtMessageReceiver settingsReceiver( settingsWidget, settingsWidget );

    return app.exec();
}
