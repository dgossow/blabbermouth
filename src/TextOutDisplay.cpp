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

#include "TextOutDisplay.h"

#include <QLabel>
#include <QTimer>
#include <QVBoxLayout>

#include <string>

TextOutDisplay::TextOutDisplay(int min_height, int font_size, bool user_input, QWidget* parent) :
    QWidget( parent )
{
  user_input_ = user_input;

  QVBoxLayout* layout = new QVBoxLayout( this );

  smileys_.push_back( ">:" );
  smileys_.push_back( ":)" );
  smileys_.push_back( ":(" );
  smileys_.push_back( ":O" );
  smileys_.push_back( ":o" );
  smileys_.push_back( ":!" );
  smileys_.push_back( ":&" );

  // Text output
  font_.setPixelSize(font_size);
  font_.setBold(true);

  text_out_label_ = new QLabel( this );
  text_out_label_->setFont( font_ );
  text_out_label_->setMinimumSize( 100, min_height );
  text_out_label_->setAlignment ( Qt::AlignCenter );
  text_out_label_->setWordWrap ( true );

  layout->addWidget( text_out_label_ );

  setLayout( layout );

  reset_timer_ = new QTimer ( this );  // create internal timer
  connect ( reset_timer_, SIGNAL ( timeout() ), SLOT ( clearText() ) );
  reset_timer_->start ( 1000 / 25  );

  setVisible( false );

  text_ = "";
}


TextOutDisplay::~TextOutDisplay()
{
}

void TextOutDisplay::clearText()
{
  if( text_ != "" )
  {
    setText( text_ );
    text_ = "";
  }
  else
  {
    setText( "" );
    text_out_label_->setText( "" );
    setVisible( false );
  }
}

void TextOutDisplay::setText( std::string text )
{
  if ( text == "" )
  {
    reset_timer_->start ( 1000 / 25  );
  }
  else
  {
      if( user_input_ )
      {
          font_.setPixelSize( 20 );
          text_out_label_->setFont(font_);
          setVisible( true );
          text_out_label_->setText( text.c_str() );
          reset_timer_->start( 10000 );
      }
      else
      {
          if( text.length() >= 130 )
          {
              font_.setPixelSize( 18 );
              text_out_label_->setFont(font_);
          }
          else
          {
              font_.setPixelSize( 27 );
              text_out_label_->setFont(font_);
          }
          setVisible( true );
          text_out_label_->setText( text.c_str() );
          reset_timer_->start ( 100000  );
      }
  }
}

void TextOutDisplay::subscribeWithNodeHandle( ros::NodeHandle node_handle )
{
    if( user_input_ )
    {
        user_input_subscriber_ = node_handle.subscribe( "robot_face/user_input", 1, &TextOutDisplay::callbackText, this );
    }
    else
    {
        text_out_subscriber_ = node_handle.subscribe( "robot_face/text_out", 1, &TextOutDisplay::callbackText, this );
        talking_finished_subscriber_ = node_handle.subscribe( "robot_face/talking_finished", 1, &TextOutDisplay::callbackTalkingFinished, this );
    }
}

void TextOutDisplay::callbackText( const std_msgs::String::ConstPtr& msg )
{
    text_ = prepareText( msg->data );
    if( user_input_ )
    {
       reset_timer_->start( text_.length() * 10  );
    }
}

void TextOutDisplay::callbackTalkingFinished( const std_msgs::String::ConstPtr& msg )
{
    reset_timer_->start( text_.length() * 10  );
}

std::string TextOutDisplay::prepareText( std::string text )
{
    std::string tmp_text = text;

    tmp_text = trimSpaces(tmp_text);
    tmp_text = clearSmileys(tmp_text);

    tmp_text = trimSpaces( tmp_text );

    if( tmp_text == ".")
    {
        tmp_text = "";
    }

    return tmp_text;
}

std::string TextOutDisplay::clearSmileys( std::string text )
{
    std::string tmp_text = text;

    size_t i_smiley = std::string::npos;

    for( unsigned int j = 0; j < smileys_.size(); j++ )
    {
        for( unsigned int i = 0; i <= tmp_text.length(); i++ )
        {
            i_smiley = tmp_text.find( smileys_.at( j ), 0 );
            if( i_smiley != std::string::npos )
            {
                tmp_text.erase(i_smiley, smileys_.at( j ).length());
            }
        }
    }

    return tmp_text;
}

std::string TextOutDisplay::trimSpaces( std::string text )
{
    std::string tmp_text = text;

    size_t startpos = tmp_text.find_first_not_of(" \t");
    size_t endpos = tmp_text.find_last_not_of(" \t");

    if(( std::string::npos == startpos ) || ( std::string::npos == endpos))
    {
        tmp_text = "";
    }
    else
    {
        tmp_text = tmp_text.substr( startpos, endpos-startpos+1 );
    }

    return tmp_text;
}
