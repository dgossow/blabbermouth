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

#include "FestivalGenerator.h"

FestivalGenerator::FestivalGenerator ( QObject* parent ) :
    QObject( parent ),
    synth_phonemes_( false ),
    synth_words_( false ),
    publish_smiley_( false ),
    punctuation_characters_( 0 )
{
    timer_ = new QTimer ( this );  // create internal timer
    connect ( timer_, SIGNAL ( timeout() ), SLOT ( run() ) );
    timer_->start( 1000.0 / 25 );

    smileys_.push_back( ">:" );
    smileys_.push_back( ":)" );
    smileys_.push_back( ":(" );
    smileys_.push_back(":O");
    smileys_.push_back(":o");
    smileys_.push_back(":!");
    smileys_.push_back(":&");

    punctuation_characters_.push_back( ":" );
    punctuation_characters_.push_back( ")" );
    punctuation_characters_.push_back( "(" );
    punctuation_characters_.push_back( "'" );
    punctuation_characters_.push_back( "\"" );
    punctuation_characters_.push_back( "}" );
    punctuation_characters_.push_back( "{" );
    punctuation_characters_.push_back( "§" );
    punctuation_characters_.push_back( "!" );
    punctuation_characters_.push_back( "?" );
    punctuation_characters_.push_back( "`" );
    punctuation_characters_.push_back( "´" );
    punctuation_characters_.push_back( "-" );
    punctuation_characters_.push_back( ";" );
    punctuation_characters_.push_back( "€" );
    punctuation_characters_.push_back( "°" );
    punctuation_characters_.push_back( "<" );
    punctuation_characters_.push_back( ">" );
    punctuation_characters_.push_back( "." );
    punctuation_characters_.push_back( "," );
}

FestivalGenerator::~FestivalGenerator ()
{
    festival_tidy_up();
}

void FestivalGenerator::run()
{
    if( (publish_smiley_ && synth_phonemes_ && synth_words_) || (!publish_smiley_ && synth_phonemes_ && synth_words_) )
    {
        festival_wait_for_spooler();

        synthPhonemes( text_for_synth_ );
        synthWords( text_for_synth_ );

        std_msgs::Empty empty;

        phonemes_publisher_.publish( empty );

        synth_phonemes_ = false;
        synth_words_ = false;

        publish_smiley_ = false;
    }
    else
    {
        std_msgs::Empty empty;

        phonemes_publisher_.publish( empty );
        publish_smiley_ = false;
    }
}

void FestivalGenerator::callbackSynth( const std_msgs::String::ConstPtr& msg )
{
    text_for_synth_ = prepareText( msg->data );
    if( text_for_synth_.length() > 0 )
    {
        synth_phonemes_ = true;
        synth_words_ = true;
    }
}

void FestivalGenerator::callbackTalkingFinished( const std_msgs::String::ConstPtr& msg )
{
     timer_->start( 1000.0 / 25 );
}

void FestivalGenerator::synthPhonemes( std::string text )
{
    ostringstream command;
    command << "(set! utt1 (SynthText \"" << text << "\"))";
    festival_eval_command( command.str().c_str() );
    festival_eval_command( "(utt.save.segs utt1 \"phonemes.txt\"))" );
}

void FestivalGenerator::synthWords( std::string text )
{
    ostringstream command;
    command << "(set! utt2 (SynthText \"" << text << "\"))";
    festival_eval_command( command.str().c_str() );
    festival_eval_command( "(utt.save.words utt2 \"words.txt\"))" );
    timer_->start( 100000 );
}

void FestivalGenerator::subscribeWithNodeHandle( ros::NodeHandle node_handle )
{
    subscriber_ = node_handle.subscribe( "robot_face/text_out", 1, &FestivalGenerator::callbackSynth, this );
    talking_finished_subscriber_ = node_handle.subscribe( "robot_face/talking_finished", 1, &FestivalGenerator::callbackTalkingFinished, this );

    // Subscribe to ROS message
    phonemes_publisher_ = node_handle.advertise<std_msgs::Empty>( "robot_face/festival_phonemes", 1 );
}

std::string FestivalGenerator::prepareText( std::string text )
{
    std::string tmp_text = text;

    tmp_text = trimSpaces( tmp_text );

    for( unsigned int j = 0; j < smileys_.size(); j++ )
    {
        if( text_for_synth_.find( smileys_.at( j ) ) )
        {
            publish_smiley_ = true;
        }
    }
    tmp_text = clearSmileys( tmp_text );

    size_t i_symbol = std::string::npos;

    for( unsigned int j = 0; j < punctuation_characters_.size(); j++ )
    {
        for( unsigned int i = 0; i < tmp_text.length(); i++ )
        {
            i_symbol = tmp_text.find( punctuation_characters_.at( j ), 0 );
            if( i_symbol != std::string::npos )
            {
                tmp_text.erase(i_symbol, punctuation_characters_.at( j ).length());
            }
        }
    }
    tmp_text = trimSpaces( tmp_text );

    return tmp_text;
}

std::string FestivalGenerator::clearSmileys( std::string text )
{
    std::string tmp_text = text;

    size_t i_smiley = std::string::npos;

    for( unsigned int j = 0; j < smileys_.size(); j++ )
    {
        for( unsigned int i = 0; i < tmp_text.length(); i++ )
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

std::string FestivalGenerator::trimSpaces( std::string text )
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
