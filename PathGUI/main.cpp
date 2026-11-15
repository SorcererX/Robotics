#include <QApplication>
#include "followersettingswidget.h"
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
    sepia::comm2::QtMessageReceiver settingsReceiver( window.followerSettings(), window.followerSettings() );

    return app.exec();
}
