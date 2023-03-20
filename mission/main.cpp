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
 THE SOFTWARE. */


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

// Follow the line to the right until the ramp objective has been completed 
void step1()
{
  sound.say(". 17 38. Yah.", 1);
  // remove old mission
  bridge.tx("regbot mclear\n");
  // clear events received from last mission
  event.clearEvents();
  // add mission lines
  bridge.tx("regbot madd vel=0.5, edger=0:dist=2.5\n"); // follow the line to the right until the first challenge
  bridge.tx("regbot madd vel=0:time=1\n"); // wait under the challenge to flex
  bridge.tx("regbot madd vel=0.5, edger=1:time=5\n");  // follow the line to the right until the first turn is complete
  bridge.tx("regbot madd vel=0.5, edger=2:ir2 < 0.1\n");  // continue until just before the goal post
  bridge.tx("regbot madd tr=0,vel=0.2:turn=180\n"); // turn the robot to face along the line
  bridge.tx("regbot madd vel=0.25,edger=2:xl>10\n"); // countinue to the line going towards the rotating challenge
  bridge.tx("regbot madd tr=0,vel=0.2:turn=-90\n"); // turn onto the path of the rotating challenge
  
  // start this mission
  bridge.tx("regbot start\n");
  // wait until finished
  cout << "Waiting for step 1 to finish (event 0 is send, when mission is finished)\n";
  event.waitForEvent(0);
//   sound.say(". Step one finished.");
}

// Rotary challenge
void step2()
{

  sound.say(". Trap queen.", 0.3);
  // remove old mission
  bridge.tx("regbot mclear\n");
  // clear events received from last mission
  event.clearEvents();

  // Follow the line until the discontinuety in the line
  bridge.tx("regbot madd vel=0.25,edger=0:dist=0.30\n");
  //drive 0.6m to overcome the discontinuety 
  bridge.tx("regbot madd vel=0.25:dist=0.60\n");
  // Drive until the robot is 25cm from the spining disk
  bridge.tx("regbot madd vel=0.35, edger=0 : ir2 < 0.25\n");
  // wait until the disk opening is regisered
  bridge.tx("regbot madd vel=0: ir2 > 0.3\n");
  //quickly drive thrugh the gate when its open
  bridge.tx("regbot madd vel=0.5,edger=0 : time=1\n");
  //continue at a lower speed until a crossing line is registered
  bridge.tx("regbot madd vel=0.35: xl>16\n");
  //turn the robot onto the line
  bridge.tx("regbot madd tr=0,vel=0.2:turn=90\n");
  // start this mission
  bridge.tx("regbot start\n");
  
}

// Speed challenge
void step3()
{
  bridge.tx("regbot mclear\n");
  event.clearEvents();

    bridge.tx("regbot madd vel=0.5, edger=0.0: dist=0.9\n"); // Drive to the start of the race track
    bridge.tx("regbot madd vel=1.0, edger=0.0: dist=2.5\n"); // Speed up towards the first corner
    bridge.tx("regbot madd servo=1, pservo=-550, vservo=0\n"); // Reset the speed to 1 on the long straight strech 
    bridge.tx("regbot madd vel=1.0, edgel=0.0: dist=2.8\n"); // drive through the goal
    bridge.tx("regbot madd vel=1.0, edgel=0.0: lv<4\n"); // drive through the goal
    bridge.tx("regbot madd servo=1, pservo=3000, vservo=0\n"); // Reset the speed to 1 on the long straight strech 
    bridge.tx("regbot madd vel=0.1, tr=0,: turn=-90\n");
    bridge.tx("regbot madd vel=0.25:xl>16\n");
  // start this mission
  bridge.tx("regbot start\n");
}

// Tunnel challenge
void step4()
{
  bridge.tx("regbot mclear\n");
  event.clearEvents();

  bridge.tx("regbot madd vel=0.25: ir2 < 0.10 \n"); // drive until the side of the tunnel challenge 
  bridge.tx("regbot madd vel=0.0: time=1 \n"); // wait for one second
  bridge.tx("regbot madd vel=0.1,tr=0:turn=-90 \n"); // turn towards the gate opening
  bridge.tx("regbot madd vel=0.125: ir1 > 0.10 \n"); // check the side ir sensor on the side to check when the box ends
  bridge.tx("regbot madd vel=0.25,tr=0.5:turn=-180 \n"); // turn into the tunnel ( mind the turning radius has to be tested)
  bridge.tx("regbot madd vel=0.25:ir1 > 0.10 \n"); // Drive through the tunnel and stop when the ir sensor no longer can see the tunnel wall


  // start this mission
  bridge.tx("regbot start\n");
}

int main(int argc, char **argv) 
{
  if (setup(argc, argv))
  { // start mission
    std::cout << "# Robobot mission starting ...\n";
    //
    step1(); // Ramp
    step2(); // Rotating disk
    step3(); // Racetrack
    step4(); // Tunnel challenge
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
