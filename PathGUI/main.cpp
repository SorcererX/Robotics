#include <QApplication>
#include "pathwindow.h"
#include <sepia/comm2/qtmessagereceiver.h>
#include <sepia/comm2/scopedmessagesender.h>

int main( int argc, char** argv )
{
    QApplication app( argc, argv );

    sepia::comm2::ScopedMessageSender sender;

    PathWindow window;
    window.setWindowTitle( "Paths" );
    window.resize( 1100, 750 );
    window.show();

    sepia::comm2::QtMessageReceiver receiver( &window, &window );

    return app.exec();
}
