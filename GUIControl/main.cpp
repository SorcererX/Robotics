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
    auto* movementCm = new MovementWidget( tabs, MovementWidget::Commands::Cm );
    auto* movementRaw = new MovementWidget( tabs, MovementWidget::Commands::Raw );
    SettingsWidget* settingsWidget = new SettingsWidget( tabs );
    tabs->addTab( movementCm, "Movement (cm)" );
    tabs->addTab( movementRaw, "Movement (raw)" );
    tabs->addTab( settingsWidget, "Settings" );
    window.setCentralWidget( tabs );
    window.show();

    sepia::comm2::QtMessageReceiver receiverCm( movementCm, movementCm );
    sepia::comm2::QtMessageReceiver receiverRaw( movementRaw, movementRaw );
    sepia::comm2::QtMessageReceiver settingsReceiver( settingsWidget, settingsWidget );

    return app.exec();
}
