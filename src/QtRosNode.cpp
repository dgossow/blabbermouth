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

#include "MainWindow.h"
#include "QtRosNode.h"

QtRosNode::QtRosNode(int argc, char *argv[], const char* node_name, MainWindow* main_window, QApplication* app)
{
  ros::init(argc, argv, node_name);

  node_handle_ = new ros::NodeHandle;
  main_window_ = main_window;
  application_ = app;

  quit_from_gui = false;
}

void QtRosNode::quitNow()
{
  quit_from_gui = true;
}

void QtRosNode::run()
{
    //ros::Rate loop_rate(10);

    while (ros::ok() && !quit_from_gui)
    {
      //main_window_->updateGeometry();
      ros::spinOnce();
      //loop_rate.sleep();
    }

  if (!ros::ok())
  {
    application_->exit();
    emit rosQuits();
    ROS_INFO("ROS-Node Terminated\n");
  }
}

