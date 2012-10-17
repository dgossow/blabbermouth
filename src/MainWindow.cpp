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

#include <string>
#include <vector>

#include "Config.h"
#include "MainWindow.h"

MainWindow::MainWindow( QWidget* parent ) : QWidget( parent, Qt::FramelessWindowHint ), talking_head_( 0 )
{
    setContentsMargins ( 0, 0, 0, 0 );

    QPalette custom_palette = palette();
    custom_palette.setColor ( QPalette::Window, Qt::black );
    custom_palette.setColor ( QPalette::WindowText, Qt::white );
    setPalette( custom_palette );
    setAutoFillBackground ( true );

    // Center Layout (fixed size to fit small screen)
    QWidget* center_widget = new QWidget();
    center_widget->setContentsMargins ( 0, 0, 0, 0 );

    QVBoxLayout* center_layout = new QVBoxLayout( );
    center_layout->setSpacing( 0 );
    center_layout->setContentsMargins ( 0, 0, 0, 0 );

//    //If we have more than one screen, open full-screen on 2nd screen
//    ostringstream command;
//    command << "Number of screens: " << qApp->desktop()->numScreens() << endl
//            << "Screennr.: " << qApp->desktop()->screen(0) << endl
//            << "Screennr.: " << qApp->desktop()->screen(1) << endl
//            << "Primary screen: " << qApp->desktop()->primaryScreen() << endl
//            << "Is virtual desktop: " << qApp->desktop()->isVirtualDesktop() << endl
//            <<  "screen 0: " << qApp->desktop()->availableGeometry(0).left() << ","
//            << qApp->desktop()->availableGeometry(0).top() << ","
//            << qApp->desktop()->availableGeometry(0).right() << ","
//            << qApp->desktop()->availableGeometry(0).bottom() << endl
//            <<  "screen 1: " << qApp->desktop()->availableGeometry(1).left() << ","
//            << qApp->desktop()->availableGeometry(1).top() << ","
//            << qApp->desktop()->availableGeometry(1).right() << ","
//            << qApp->desktop()->availableGeometry(1).bottom() << ",";

//    ROS_INFO( command.str().c_str() );

    int width = 480;
    int height = 640;

    try
    {
        std::vector< std::vector<float> > material_colors;
        const char* cfgFilename =  "../config.cfg";

        Config cfg(cfgFilename);
        width = cfg.get("Window Width");
        height = cfg.get("Window Height");
        std::string mesh_filename = cfg.get("Mesh Filename");
        material_colors.push_back(cfg.get("Head Color"));
        material_colors.push_back(cfg.get("Iris Color"));
        material_colors.push_back(cfg.get("Outline Color"));
        talking_head_ = new TalkingHead( this, mesh_filename, material_colors );

        std::cout << "Mesh Filename : " << mesh_filename << std::endl;
    }
    catch( const std::exception& e )
    {
        std::cerr << e.what() << std::endl;
    }

    setMinimumSize( width, height );

    festival_generator_ = new FestivalGenerator();

    // TextOutputDisplay
    text_out_display_= new TextOutDisplay( 0, 27, false, this );
    text_out_display_->setMaximumSize( width, (height)/4 );

    // TextRecognitionDisplay
    text_rec_display_= new TextOutDisplay( 0, 20, true, this );
    custom_palette.setColor ( QPalette::WindowText, QColor( 200, 200, 200 ) );
    text_rec_display_->setPalette( custom_palette );
    text_rec_display_->setMaximumSize( width, (height)/4 );

    center_layout->addWidget( text_out_display_ );
    center_layout->addStretch();
    center_layout->addWidget( text_rec_display_ );

    center_layout->setStretchFactor( text_out_display_, 1 );

    center_widget->setFixedSize( width, height/4 );
    center_widget->setLayout( center_layout );

    // Main Layout
    QVBoxLayout* main_layout = new QVBoxLayout( );
    main_layout->setSpacing( 0 );
    main_layout->setContentsMargins ( 0, 0, 0, 0 );

    main_layout->addWidget( talking_head_ );
    talking_head_->setMinimumSize( width, height/4 );
    talking_head_->setMaximumSize( width, (height*3)/4 );

    main_layout->setStretchFactor( talking_head_, 1 );

    QHBoxLayout* h_layout = new QHBoxLayout( );
    h_layout->setSpacing( 0 );
    h_layout->setContentsMargins ( 0, 0, 0, 0 );

    h_layout->addStretch();
    h_layout->addWidget( center_widget );
    h_layout->addStretch();

    main_layout->addStretch();
    main_layout->addLayout( h_layout );
    main_layout->addStretch();

    setLayout( main_layout );

    ostringstream next_command;
    next_command << "Info: " << qApp->desktop()->screenNumber( talking_head_ ) << endl;
    ROS_INFO( next_command.str().c_str() );
}

MainWindow::~MainWindow() {}

void MainWindow::setNodeHandle(ros::NodeHandle* node_handle)
{
    talking_head_->subscribeWithNodeHandle( *node_handle );

    text_out_display_->subscribeWithNodeHandle( *node_handle );

    festival_generator_->subscribeWithNodeHandle( *node_handle );

    text_rec_display_->subscribeWithNodeHandle( *node_handle );
}
