#include <iostream>
#include <QApplication>
#include "movementwidget.h"
#include <sepia/comm2/scopedmessagesender.h>

int main( int argc, char** argv )
{
    QApplication app( argc, argv );

    sepia::comm2::ScopedMessageSender sender;

    MovementWidget* movementWidget = new MovementWidget( nullptr );
    movementWidget->show();
    return app.exec();
}