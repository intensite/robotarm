
#include "command.h"

void cmdGetPosition();
void ZRot(boolean DIR, int STEP);
void GoHome();
void SetDirection();
void stepper();
void handleAsErr(Cmd(&cmd));
void cmdMove(Cmd(&cmd));
void cmdDwell(Cmd(&cmd));
void cmdGripperOn(Cmd(&cmd));
void cmdGripperOff(Cmd(&cmd));
void executeCommand(Cmd cmd);




