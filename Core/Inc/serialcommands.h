#ifndef SERIALCOMMANDS_H
#define SERIALCOMMANDS_H

enum Cmd{
    CMD_PinMode,
    CMD_DigitalWrite,
    CMD_AnalogWrite,
    CMD_DigitalRead,
    CMD_AnalogRead,
    CMD_AnalogConfig,
    CMD_Sync,
    CMD_DigitalReadouts
};

enum PinMode {
    PinMode_Input,
    PinMode_Output,
    PinMode_InPullUp
//    PullUp_False,
//    PullDown_True,
//    PullDown_False,
};

enum Output{
	Output_Low,
	Output_High
};

#endif // SERIALCOMMANDS_H
