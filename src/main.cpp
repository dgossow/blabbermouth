/*******************************************************************************
 *  TalkingHead - A talking head for robots
 *  Copyright (C) 2012 AG Aktives Sehen <agas@uni-koblenz.de>
 *                     Universitaet Koblenz-Landau
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Library General Public License for more details.
 *
 *  You should have received a copy of the GNU Library General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 *  MA 02110-1301  USA or see <http://www.gnu.org/licenses/>.
 *******************************************************************************/

#include <ros/ros.h>

#include "MainWindow.h"
#include "QtRosNode.h"

int main( int argc, char *argv[] )
{
    festival_initialize ( true, FESTIVAL_HEAP_SIZE );

    // Create application object
    QApplication app( argc, argv );

    app.setApplicationName( QString( "RobotFace" ) );

    MainWindow window;

    QIcon icon;
    icon.addFile(QString::fromUtf8("../logo.png"), QSize(), QIcon::Normal, QIcon::Off);
    window.setWindowIcon(icon);

    window.show();

    // Check for and move to second screen
    QRect screen_res = qApp->desktop()->screenGeometry(1/*screenNumber*/);
    window.move( screen_res.x(), screen_res.y() );

    // Initialize ROS
    QtRosNode qt_ros_node( argc, argv, "RobotFace", &window, &app );
    qt_ros_node.start();

    window.setNodeHandle( qt_ros_node.getNodeHandle() );

    app.exec();

    return 0;
}
