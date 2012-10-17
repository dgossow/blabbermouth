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

#ifndef TALKING_HEAD_INCLUDE_FESTIVALGENERATOR_H_
#define TALKING_HEAD_INCLUDE_FESTIVALGENERATOR_H_

#include <festival.h>

#include <QObject>
#include <QTimer>

#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

#include <fstream>
#include <ostream>
#include <string>
#include <vector>

/**
 * @class FestivalGenerator
 * @brief Provides text-to-phonemes/words sythesis via Festival
 * @author Julian Giesen (R18)
 */

class FestivalGenerator: public QObject
{
   Q_OBJECT
    
	public:
                /** Constructor */
                explicit FestivalGenerator( QObject *parent = 0 );

                /** Destructor */
                ~FestivalGenerator();

                void callbackSynth( const std_msgs::String::ConstPtr& msg );
                void callbackTalkingFinished( const std_msgs::String::ConstPtr& msg );

                /** @brief subscribe to topics here */
                void subscribeWithNodeHandle( ros::NodeHandle node_handle );

        public slots:
                void run();

        private:
                bool                        synth_phonemes_;
                bool                        synth_words_;
                bool                        publish_smiley_;
                std::string                 text_for_synth_;

                std::vector<std::string>    punctuation_characters_;
                std::vector<std::string>    smileys_;

                QTimer*                     timer_;

                /// Define your custom ROS subscribers, callbacks and publishers here
                ros::Publisher              phonemes_publisher_;
                ros::Subscriber             subscriber_;
                ros::Subscriber             talking_finished_subscriber_;


                /// @brief Synthesize phonemes for given text
                void synthPhonemes( std::string text );

                /// @brief Synthesize words for given text
                void synthWords( std::string text );

                /// @return trims leading and trailing spaces and returns the prepared String
                std::string trimSpaces( std::string text );

                /// @return Deletes smilies for better Synthesis and returns the prepared String
                std::string prepareText( std::string text );

                /// @return Deletes smileys for better Synthesis and returns the prepared String
                std::string clearSmileys( std::string text );
	
};

#endif // TALKING_HEAD_INCLUDE_FESTIVALGENERATOR_H_
