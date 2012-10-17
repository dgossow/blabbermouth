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

#ifndef TALKING_HEAD_INCLUDE_QTROSNODE_H_
#define TALKING_HEAD_INCLUDE_QTROSNODE_H_

#include <QApplication>
#include <QObject>
#include <QThread>

#include <ros/ros.h>

class MainWindow;

/**
 * @class  QtRosNode
 * @brief  Handles the ROS connection
 * @author Julian Giesen (R16)(R18)
 */

class QtRosNode : public QThread {

  Q_OBJECT

  public:
    /// Note: The constructor will block until connected with roscore
    /// Instead of ros::spin(), start this thread with the start() method
    /// to run the event loop of ros
    QtRosNode(int argc, char *argv[], const char* node_name, MainWindow* main_window, QApplication* app);

    ros::NodeHandle* getNodeHandle(){ return node_handle_; }

    /** @brief This method contains the ROS event loop. Feel free to modify */
    void run();

  public slots:
    /// Connect to aboutToQuit signals, to stop the thread
    void quitNow();

  signals:
    /// Triggered if ros::ok() != true
    void rosQuits();

  private:

    bool                quit_from_gui;
    ros::NodeHandle*    node_handle_;

    MainWindow*         main_window_;
    QApplication*       application_;
};

#endif // TALKING_HEAD_INCLUDE_QTROSNODE_H_
