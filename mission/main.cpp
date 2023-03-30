/*  
 
 Copyright © 2022 DTU, Christian Andersen jcan@dtu.dk
 
 The MIT License (MIT)  https://mit-license.org/
 
 Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 and associated documentation files (the “Software”), to deal in the Software without restriction, 
 including without limitation the rights to use, copy, modify, merge, publish, distribute, 
 sublicense, and/or sell copies of the Software, and to permit persons to whom the Software 
 is furnished to do so, subject to the following conditions:
 
 The above copyright notice and this permission notice shall be included in all copies 
 or substantial portions of the Software.
 
 THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
 INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
 PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE 
 FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, 
 ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN 
 THE SOFTWARE. 
 
Flashing guide for the robot, (How to transfer mission code)
1. acces the robot through ssh at the ip on the robort lcd
    ssh local@10.197.218.45 
2. pull the latest updates 
 
 
 
 */


#include <iostream>
#include "src/ubridge.h"
#include "src/uvision.h"
#include "src/upose.h"
#include "src/ucomment.h"
#include "src/ustate.h"
#include "src/uplay.h"
#include "src/uevent.h"
#include "src/ujoy.h"

// to avoid writing std:: 
using namespace std;


bool setup(int argc, char **argv)
{ // check for command line parameters
  for (int i = 1; i < argc; i++)
  { // check for command line parameters
    // for process debug
    if (strcmp(argv[i], "help") == 0)
    { 
      printf("-----\n# User mission command line help\n");
      printf("# usage:\n#   ./user_mission [help] [ball] [show] [aruco] [videoX]\n-----\n");
      return false;
    }
  }
  // connect to robot hardware using bridge
  bridge.setup("127.0.0.1", "24001", argc, argv);
  if (true or bridge.connected)
  {  /// call setup for data structures
    pose.setup();
    comment.setup();
    state.setup();
    vision.setup(argc, argv);
    event.setup();
    joy.setup();
    printf("# Setup finished OK\n");
  }
  else
    printf("# Setup failed\n");

  return true;
}

// Follow the line to the right until the ramp objective
void gillutineChallenge()
{
  sound.say("seventeen thirtyeight. Yah.", 1);
  // remove old mission
  bridge.tx("regbot mclear\n");
  // clear events received from last mission
  event.clearEvents();
  // add mission lines

  bridge.tx("regbot madd vel=0.5, edger=0:dist=2.5\n"); // follow the line to the right until the gillutine challenge
  bridge.tx("regbot madd vel=0:time=1\n"); // wait under the challenge to flex
  bridge.tx("regbot madd vel=0.25,edger=-1:time=1\n"); 
  bridge.tx("regbot madd vel=0.25,edger=0:time=1\n"); 
  
  // start this mission
  bridge.tx("regbot start\n");
  // wait until finished
  cout << "Waiting for step 1 to finish (event 0 is send, when mission is finished)\n";
  event.waitForEvent(0);
//   sound.say(". Step one finished.");
}
void seesawChallenge()
{
  sound.say("And I can ride with my baby. I just left the mall, I'm getting fly with my baby, yeah.", 1);
  // remove old mission
  bridge.tx("regbot mclear\n");
  event.clearEvents();
  // clear events received from last mission
  bridge.tx("regbot madd servo=1, pservo=2000, vservo=0:time=1\n");
  bridge.tx("regbot madd vel=0.25,edgel=2:xl > 16\n");
  bridge.tx("regbot madd vel=0.1:dist=0.1\n");
  bridge.tx("regbot madd vel=0.1,tr=0:turn=90\n");

  bridge.tx("regbot madd vel=0.25,edger=1:dist=0.2\n"); // catch the line
  bridge.tx("regbot madd vel=0.25,edger=2:dist=0.8\n"); // drive to the ball 
  bridge.tx("regbot madd servo=1, pservo=-700, vservo=0:time=1\n"); //grab the ball
  bridge.tx("regbot madd vel=-0.1:dist=0.165\n");
  bridge.tx("regbot madd servo=1, pservo=-650, vservo=0:time=1\n");
  bridge.tx("regbot madd vel=0.01,edger=0:lv<4\n"); // slowly drive down the ramp
  bridge.tx("servo=1, pservo=-700, vservo=0:time=1\n");

  // Get to the goal
  bridge.tx("regbot madd vel=0.25:xl>16\n");
  bridge.tx("regbot madd vel=0.1:dist=0.1\n");
  bridge.tx("regbot madd vel=0.1,tr=0:turn=-90\n");
  bridge.tx("regbot madd vel=0.25,edger=0:lv<4\n");
  bridge.tx("regbot madd vel=0.25:lv>4 \n");
  // zero the distance on the goal
  bridge.tx("regbot madd vel=0.25, edger=0:ir2 < 0.35\n");
  bridge.tx("regbot madd vel=0.1,tr=0:turn=180\n");
  bridge.tx("regbot madd vel=0.25,edgel=2:time=10\n");
  // drive up the ramp to the post
  bridge.tx("regbot madd vel=0.1,edgel=2:ir1 < 0.30\n");
  bridge.tx("regbot madd servo=1, pservo=-750, vservo=0:time=1\n");
  bridge.tx("regbot madd vel=0.25:dist=0.45\n");

  bridge.tx("regbot madd servo=1, pservo=-650, vservo=0:time=1\n");
  bridge.tx("regbot madd label=1,vel=0.05, tr=0: turn=-50\n");
  bridge.tx("regbot madd vel=0.1, tr=0: turn=50\n");
  bridge.tx("regbot madd vel=0.1, tr=0: turn=50\n");
  bridge.tx("regbot madd vel=0.1, tr=0: turn=-50\n");
  bridge.tx("regbot madd vel=0.1:dist=0.05\n");
  bridge.tx("regbot madd goto=1 : count = 2\n");

  bridge.tx("regbot madd servo=1, pservo=2000, vservo=0:time=1\n");
  bridge.tx("regbot madd vel=0.1,tr=0:turn=180\n");
  bridge.tx("regbot madd vel=0.1:lv>4\n");

/*
servo=1, pservo=2000, vservo=0:time=1
vel=0.25,edgel=2:xl > 16
vel=0.1:dist=0.1
vel=0.1,tr=0:turn=90

vel=0.25,edger=1:dist=0.2
vel=0.25,edger=2:dist=0.8
servo=1, pservo=-700, vservo=0:time=1
vel=-0.1:dist=0.165
servo=1, pservo=-650, vservo=0:time=1
vel=0.01,edger=0:lv<4
servo=1, pservo=-700, vservo=0:time=1

vel=0.25:xl>16
vel=0.1:dist=0.1
vel=0.1,tr=0:turn=-90
vel=0.25,edger=0:lv<4
vel=0.25:lv>4

vel=0.25, edger=0:ir2 < 0.35
vel=0.1,tr=0:turn=180
vel=0.25,edgel=2:time=10
vel=0.1,edgel=2:ir1 < 0.30
servo=1, pservo=-750, vservo=0:time=1
vel=0.25:dist=0.45

servo=1, pservo=-650, vservo=0:time=1
label=1,vel=0.05, tr=0: turn=-50
vel=0.1, tr=0: turn=50
vel=0.1, tr=0: turn=50
vel=0.1, tr=0: turn=-50
vel=0.1:dist=0.05
goto=1 : count = 2

servo=1, pservo=2000, vservo=0:time=1
vel=0.1,tr=0:turn=180
vel=0.1:lv>4

*/
   // start this mission
  bridge.tx("regbot start\n");
  event.waitForEvent(0);

}

void intermissionRotaryChallenge()
{
  bridge.tx("regbot mclear\n");
  // clear events received from last mission
  event.clearEvents();

  bridge.tx("regbot madd vel=0.25, edger=2:ir2 < 0.35\n"); // continue until just before the goal post
  bridge.tx("regbot madd tr=0,vel=0.25:turn=180\n"); // turn the robot to face along the line
  bridge.tx("regbot madd vel=0:time=1\n"); // wait to flex
  bridge.tx("regbot madd vel=0.25,edger=-1:dist=1.25\n"); // countinue to the line going towards the rotating challenge
  bridge.tx("regbot madd vel=0.25,edger=-2:xl>6\n"); // countinue to the line going towards the rotating challenge
  bridge.tx("vel=0.1:dist=0.1\n"); // drive a little past the line so that the robot turns onto the track
  bridge.tx("regbot madd tr=0,vel=0.25:turn=-90\n"); // turn onto the path of the rotating challenge

  bridge.tx("regbot start\n");
  event.waitForEvent(0);

  /*
  code for GUI test
  vel=0.25, edger=0:ir2 < 0.35
  tr=0,vel=0.25:turn=180
  vel=0:time=1
  vel=0.25,edger=-1:dist=1.25
  vel=0.25,edger=-2:xl>6
  vel=0.1:dist=0.1
  tr=0.,vel=0.25:turn=-90
  */
}
// Rotary challenge
void rotaryChallenge()
{
  sound.say(". Trap queen.", 0.3);
  // remove old mission
  bridge.tx("regbot mclear\n");
  // clear events received from last mission
  event.clearEvents();

  bridge.tx("regbot madd vel=0.1,edger=-2:lv<4\n"); // Follow the line until the discontinuety in the line
  bridge.tx("regbot madd tr=0.1,vel=0.25:turn=-90\n");  // Turn onto the other line
  bridge.tx("regbot madd vel=0.05,edgel=0:ir2 < 0.2\n");// Drive until the robot is 25cm from the spining disk
  bridge.tx("regbot madd vel=0: ir2 > 0.5\n");  // wait until the disk opening is regisered
  bridge.tx("regbot madd vel=0: time=0.5\n"); // delay for disc to spin
  bridge.tx("regbot madd vel=0.5,edger=0 : lv<4\n"); //quickly drive thrugh the gate when its open
  bridge.tx("regbot madd vel=0.35: xl>16\n");//continue at a lower speed until a crossing line is registered
  bridge.tx("regbot madd tr=0,vel=0.2:turn=-90\n");//turn the robot onto the line for the speedchallenge
  bridge.tx("regbot start\n");// start this mission
  
  /*
  vel=0.1,edger=-2:lv<4
  tr=0.1,vel=0.25:turn=-90
  vel=0.05,edgel=0:ir2 < 0.2
  vel=0: ir2 > 0.5
  vel=0: time=0.5
  vel=0.5,edger=0 : lv<4
  vel=0.35: xl>16
  tr=0,vel=0.2:turn=-90
  */
}

// Speed challenge
void speedChallenge()
{
  bridge.tx("regbot mclear\n");
  event.clearEvents();

    bridge.tx("regbot madd vel=0.5, edgel=0.0: dist=0.9\n"); // Drive to the start of the race track
    bridge.tx("regbot madd vel=1.0, edgel=-1.0: dist=2.5\n"); // Speed up towards the first corner
    bridge.tx("regbot madd servo=1, pservo=-550, vservo=0.2\n"); // Reset the speed to 1 on the long straight strech 
    bridge.tx("regbot madd vel=1.0, edger=0.0: dist=2.8\n"); // drive through the goal
    bridge.tx("regbot madd vel=1.0, edger=1.0: lv<6\n"); // drive through the goal
    bridge.tx("regbot madd servo=1, pservo=3000, vservo=0\n"); // Reset the speed to 1 on the long straight strech 
  // start this mission
  bridge.tx("regbot start\n");

  /*
  vel=0.5, edgel=0.0: dist=0.9
  vel=1.0, edgel=-1.0: dist=2.5
  servo=1, pservo=-550, vservo=0
  vel=1.0, edger=0.0: dist=2.8
  vel=1.0, edger=-1.0: lv<6
  servo=1, pservo=3000, vservo=0
  */
}
    // recalibraation
void intermissionTunelChallenge()
{
  bridge.tx("regbot mclear\n");
  event.clearEvents();

    bridge.tx("regbot madd vel=0.1, tr=0: turn=-90\n"); // Turn the robot towards the tunnel challenge
    bridge.tx("regbot madd vel=0.25:xl>16\n");  // Drive the robot forward to the first crossing line
    bridge.tx("regbot madd vel=0.1, tr=0: turn=-90\n"); // turn the robot towards the goal
    bridge.tx("regbot madd vel=0.1, edger=0:ir2 < 0.20\n"); // drive the robot 20cm from the goal post to obtain system percision
    bridge.tx("regbot madd tr=0,vel=0.25:turn=180\n");  // turn the robot around
    bridge.tx("regbot madd vel=0.25: dist=0.2\n"); // drive the robot 20cm further away from the goal post
    bridge.tx("regbot madd vel=0.1, tr=0: turn=-90\n"); // turn the robot towards the tunnel challenge
  // start this mission
  bridge.tx("regbot start\n");

  /*
  vel=0.1, tr=0: turn=-90
  vel=0.25:xl>16
  vel=0.1, tr=0: turn=-90
  vel=0.1, edger=0:ir2 < 0.20
  tr=0,vel=0.25:turn=180
  vel=0.25: dist=0.2
  vel=0.1, tr=0: turn=-90
  */
}

// Tunnel challenge
void tunnelChallenge()
{
  bridge.tx("regbot mclear\n");
  event.clearEvents();

  bridge.tx("regbot madd vel=0.25: ir2 < 0.10 \n"); // drive until the side of the tunnel challenge 
  bridge.tx("regbot madd vel=0.0: time=1 \n"); // wait for one second
  bridge.tx("regbot madd vel=0.1,tr=0:turn=-90 \n"); // turn towards the gate opening
  bridge.tx("regbot madd vel=0.125: ir2 > 0.10 \n"); //
  bridge.tx("regbot madd vel=0.75:dist=0.2\n"); // 
  bridge.tx("regbot madd vel=0.25:dist=0.5 \n"); //
  bridge.tx("regbot madd vel=0.25,tr=0.0:turn=90\n"); //
  bridge.tx("regbot madd vel=0.25:dist=0.5\n"); //
  bridge.tx("regbot madd vel=0.25,tr=0.0:turn=90\n"); //
  bridge.tx("regbot madd vel=0.25: ir2 < 0.1\n"); //
  bridge.tx("regbot madd vel=0.0: time=1\n"); //

  bridge.tx("regbot madd vel=0.25:dist=0.5\n"); //
  bridge.tx("regbot madd vel=0.25,tr=0.0:turn=-90\n"); //
  bridge.tx("regbot madd vel=0.25: xl > 6\n"); //
  bridge.tx("regbot madd vel=0.25,tr=0.0:turn=-90\n"); //
  bridge.tx("refbot madd vel=0.25,edger=0: dist=0.6\n"); //
  bridge.tx("regbot madd vel=0.25,tr=0.0:turn=-90\n"); //

  bridge.tx("regbot madd vel=0.25: ir2 < 0.1\n"); //
  bridge.tx("regbot madd vel=0.25,tr=0.0:turn=90\n"); //
  bridge.tx("regbot madd vel=0.25:dist=0.5\n"); //
  bridge.tx("regbot madd vel=0.25,tr=0.0:turn=-90\n"); //
  bridge.tx("refbot madd vel=0.5: dist=0.2\n"); //
  bridge.tx("regbot madd vel=0.25: dist=1\n"); //

  bridge.tx("regbot madd vel=0.25,tr=0.0:turn=-90\n"); //
  bridge.tx("regbot madd vel=0.25: dist=0.5\n"); //
  bridge.tx("regbot madd vel=0.25,tr=0.0:turn=90\n"); //
  bridge.tx("regbot madd vel=0.25: xl > 6\n"); //
  bridge.tx("regbot madd vel=0.25,tr=0.0:turn=90\n"); //
  bridge.tx("regbot madd vel=0.1, edger=0: ir2 < 0.1\n"); //
  bridge.tx("regbot madd vel=0.25,tr=0.0:turn=180\n"); //
  bridge.tx("regbot madd vel=0.25: dist=0.75\n");
  bridge.tx("regbot madd vel=0.25,tr=0.0:turn=-90\n");
  bridge.tx("regbot madd vel=0.25: xl > 6\n");
  bridge.tx("regbot madd vel=0.25:dist=0.1\n");
  bridge.tx("regbot madd vel=0.25,tr=0.0:turn=-90\n");

  /*
fixed code
// Drive into box
vel=0.25: ir2 < 0.1
vel=0.0: time=1
vel=0.1,tr=0:turn=-90
vel=-0.25:ir1 > 0.5
vel=0.1,tr=0:turn=10
vel=0.2:dist=0.5
vel=0.1,tr=0:turn=-10
vel=0.125: ir2 < 0.10

vel=0.5:dist=0.2
vel=0.25:dist=0.5
vel=0.25,tr=0.0:turn=90
vel=0.25:dist=0.45
vel=0.25,tr=0.0:turn=95
vel=0.25: ir2 < 0.1
vel=0.0: time=1

vel=0.25:dist=0.6
vel=0.25,tr=0.0:turn=-95
vel=0.25: xl > 6
vel=0.25:dist=0.1
vel=0.25,tr=0.0:turn=-90
vel=0.25,edger=0: dist=0.6
vel=0.25,tr=0.0:turn=-90

vel=0.25: ir2 < 0.1
vel=0.25,tr=0.0:turn=90
vel=0.25:dist=0.5
vel=0.25,tr=0.0:turn=-90
vel=0.5: dist=0.2
vel=0.25: dist=1

vel=0.25,tr=0.0:turn=-90
vel=0.25: dist=1
vel=0.25,tr=0.0:turn=90
vel=0.25: xl > 6
vel=0.25,tr=0.0:turn=90
vel=0.1, edger=0: ir2 < 0.1
vel=0.25,tr=0.0:turn=180
vel=0.25: dist=0.75
vel=0.25,tr=0.0:turn=-90
vel=0.25: xl > 6
vel=0.25:dist=0.1
vel=0.25,tr=0.0:turn=-90
  
  */

  // start this mission
  bridge.tx("regbot start\n");
}

void goalChallenge()
{
  bridge.tx("regbot mclear\n");
  event.clearEvents();

  bridge.tx("regbot madd servo=1, pservo=3000, vservo=0:time=1\n");
  bridge.tx("regbot madd vel=0.5,edger=0:lv<4\n");
  bridge.tx("regbot madd vel=0.5,tr=0.25:turn=-90\n");
  bridge.tx("regbot madd vel=0.5:xl>16\n");
  bridge.tx("regbot madd vel=0.5,tr=0.25:turn=-90\n");
  bridge.tx("regbot madd servo=1, pservo=-650, vservo=0:time=1\n");
  bridge.tx("regbot madd vel=0.5,edger=0:ir<20\n");

  bridge.tx("regbot start\n");
  /*
  servo=1, pservo=3000, vservo=0:time=1
vel=0.5,edger=0:lv<4
vel=0.5,tr=0.25:turn=-90
vel=0.5:xl>16
vel=0.5,tr=0.25:turn=-90
servo=1, pservo=-650, vservo=0:time=1
vel=0.5,edger=0:ir<20
  */


}

int main(int argc, char **argv) 
{
  if (setup(argc, argv))
  { // start mission
    std::cout << "# Robobot mission starting ...\n";
    //
    gillutineChallenge(); // gillutine
    seesawChallenge(); // complete the sesaw challenge
    intermissionRotaryChallenge(); // Intermission to the rotarychallenge with odometry calibration reset
    rotaryChallenge(); // rotary challenge
    speedChallenge();  // Racing challenge
    intermissionTunelChallenge(); // // Intermission from racetrack to tunnel challenge with odometry calibration reset
    tunnelChallenge(); //tunnel challenge
    goalChallenge(); // goto goal (final)
     

    //
    std::cout << "# Robobot mission finished ...\n";
    // remember to close camera
    vision.stop();
    sound.say("17 38", 0.2);
    while (sound.isSaying())
      sleep(1);
    bridge.tx("regbot mute 1\n");
  }
  return 0;
}
