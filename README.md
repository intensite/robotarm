# robotarm

My contribution to the Ftobler robot arm community.

Derived from 20sffactory's firmware version of the robot arm which was originally designed by Ftobler at:<br>
  * https://github.com/20sffactory/robotarm<br>
  * https://github.com/ftobler/robotArm/<br>
  * https://www.thingiverse.com/thing:1718984<br>

### Major changes:
  * New G-codes support:
    * Added support for G90 & G91: Set to Absolute / Relative Positioning coordinate modes
    * Added support for M114: Get Current Position.
    * Added support for partial coordinate movements commands
  * Refactor code to ease the use of VSCode IDE
  * Disable gripper stepper when not in use to prevent overheat (in 12 volt mode)
  * Added debug messages and a few bug fixes.


support group at facebook for discussions and learning materials: <br>
https://www.facebook.com/groups/828749637471209/


Future improvements ideas:
  * Save maximum limits of each axis of the arm to prevent damages to the gears.
  * Support for game controller movement. See: https://github.com/intensite/node_robot_ctrl
  * Wifi communication using esp8266/esp32 board.

