#include "SPCMMonochromator.h"

const uint8_t SPCMMonochromator::CompeletedByte = 24;
const uint32_t SPCMMonochromator::ResetCompleteTime = 1000000;
const uint32_t SPCMMonochromator::CommandReplyTimeMax = 250000;
const uint32_t SPCMMonochromator::TimeToCompleteDefault = 100000;
const uint8_t SPCMMonochromator::RetryCountMax = 8;
const uint32_t SPCMMonochromator::WipeInputEvery = 100000;
const SPCMMonochromator::CommandStruct SPCMMonochromator::CommandLibrary[] =
{
	{CommandType::None,0,CommandParameterType::None,false,false,false,0},
	{CommandType::Calibrate,18,CommandParameterType::uInt16,false,true,false,0},
	{CommandType::Decrement,1,CommandParameterType::None,false,false,false,1000000},
	{CommandType::Echo,27,CommandParameterType::None,true,false,false,0},
	{CommandType::Goto,16,CommandParameterType::uInt16,false,true,false,10000000},
	{CommandType::Increment,7,CommandParameterType::None,false,false,false,1000000},
	{CommandType::Order,51,CommandParameterType::uInt8,false,true,false,0},
	{CommandType::Query,56,CommandParameterType::uInt8,true,true,false,0},
	{CommandType::Reset,255,CommandParameterType::uInt8,true,false,false,10000000},
	{CommandType::Scan,12,CommandParameterType::uInt32,false,false,false,0},
	{CommandType::Grating,26,CommandParameterType::uInt8,false,false,true,0},
	{CommandType::StepSize,55,CommandParameterType::uInt8,false,false,true,0},
	{CommandType::Speed,13,CommandParameterType::uInt16,false,false,false,0},
	{CommandType::Step,54,CommandParameterType::None,false,false,true,0},
	{CommandType::Units,50,CommandParameterType::uInt8,false,false,true,0},
	{CommandType::Zero,52,CommandParameterType::uInt8,false,false,false,0},
};
SPCMMonochromator::SPCMMonochromator(HardwareSerial *serial)
{
	SerialPort = serial;
	SerialPort->begin(9600, SERIAL_8N1);
	RecievedCallback = NULL;
	CurrentCommand = NULL;
	CurrentCommandParameter = 0;
	CurrentCommandTimeToComplete = 0;
	ReplyByteCount = 0;
	for (uint8_t Index = 0; Index < SPCMMonochromaotrReplyDataMaxCount; ++Index)
	{
		ReplyData[Index] = 0;
	}
	ReplyByteCountMax = 6;
	ClearCommandQueue();
	StatusByte = 0;
	Busy = false;
	Wavelength = 0.0;
	GratingCurrent = 0;
	LastCommandSentTime = 0;
	CommandReplyTime = 0;
	StatusByteReceivedTime = 0;
	ResetTime = 0;
	ResetWaitCount = 0;
	CommandRetryCount = 0;
	SerialNumber = 0;
	ScanSpeed = 0;
	GratingsInstalled = 0;
	GratingsCurrent = 0;
	GratingGrooves = 0;
	GratingBlaze = 0;
	UseCTSPin = false;
	UseRTSPin = false;
	CTSPin = 0;
	RTSPin = 0;
	IgnoreCTS = true;
	RTSWrite = NULL;
	CTSRead = NULL;
	Verbose = false;
	ExpectReply = false;
	ExpectStatus = false;
	Mode = ModeType::Inactive;
}
void SPCMMonochromator::SetCTSPin(uint8_t CTSPinToSet)
{
	IgnoreCTS = false;
	UseCTSPin = true;
	CTSPin = CTSPinToSet;
	CTSRead = NULL;
}
void SPCMMonochromator::SetRTSPin(uint8_t RTSPinToSet)
{
	UseRTSPin = true;
	RTSPin = RTSPinToSet;
	RTSWrite = NULL;
}
void SPCMMonochromator::SetReadCTSFunction(PinReader CTSReadFunction)
{
	if (CTSReadFunction == NULL)
	{
		IgnoreCTS = true;
		UseCTSPin = false;
		CTSPin = 0;
		CTSRead = NULL;
	}
	else
	{
		IgnoreCTS = false;
		UseCTSPin = false;
		CTSPin = 0;
		CTSRead = NULL;
	}
}
void SPCMMonochromator::SetVerbose(bool VerboseToSet)
{
	Verbose = VerboseToSet;
}
void SPCMMonochromator::SetWriteRTSFunction(PinWriter RTSSetFunction)
{
	UseRTSPin = false;
	RTSPin = 0;
	RTSWrite = RTSSetFunction;
}
void SPCMMonochromator::SendGetWavelength()
{
	Enqueue(CommandType::Query,static_cast<uint8_t>(QueryType::Position));
}
void SPCMMonochromator::SendGetGrooves()
{
	Enqueue(CommandType::Query,static_cast<uint8_t>(QueryType::Grooves));
}
void SPCMMonochromator::SendGetBlaze()
{
	Enqueue(CommandType::Query,static_cast<uint8_t>(QueryType::Blaze));
}
void SPCMMonochromator::SendGetGrating()
{
	Enqueue(CommandType::Query,static_cast<uint8_t>(QueryType::Grating));
}
void SPCMMonochromator::SendGetSpeed()
{
	Enqueue(CommandType::Query,static_cast<uint8_t>(QueryType::Speed));
}
void SPCMMonochromator::SendGetSize()
{
	Enqueue(CommandType::Query,static_cast<uint8_t>(QueryType::SizeByte));
}
void SPCMMonochromator::SendGetGratingCount()
{
	Enqueue(CommandType::Query,static_cast<uint8_t>(QueryType::GratingCount));
}
void SPCMMonochromator::SendGetUnits()
{
	Enqueue(CommandType::Query,static_cast<uint8_t>(QueryType::Units));
}
void SPCMMonochromator::SendGetSerial()
{
	Enqueue(CommandType::Query,static_cast<uint8_t>(QueryType::Serial));
}
void SPCMMonochromator::Reset()
{
	if (Mode == ModeType::Inactive)
	{
		Mode = ModeType::Idle;
	}
	Enqueue(CommandType::Reset);
}
void SPCMMonochromator::Begin()
{
	ClearCommandQueue();
	Enqueue(CommandType::Echo);
	SendGetWavelength();
	SendGetGrooves();
	SendGetBlaze();
	SendGetGrating();
	SendGetSpeed();
	SendGetGratingCount();
	SendGetUnits();
	SendGetSerial();
	if (Mode == ModeType::Inactive)
	{
		Mode = ModeType::Idle;
	}
}
void SPCMMonochromator::SetRecievedCallback(FinishedListener Finished)
{
	RecievedCallback = Finished;
}
void SPCMMonochromator::SendSetUnits(UnitsType UnitsToSet)
{
	Enqueue(CommandType::Units,static_cast<uint8_t>(UnitsToSet));
}
void SPCMMonochromator::SendSetWavelength(int32_t Wavelength)
{
	uint32_t WavelengthToSend = (uint32_t)(Wavelength);
	Enqueue(CommandType::Goto,WavelengthToSend);
}
void SPCMMonochromator::SendSetGrating(uint8_t GratingToSet)
{
	Enqueue(CommandType::Grating,GratingToSet);
}
uint16_t SPCMMonochromator::GetSerial()
{
	return SerialNumber;
}
uint16_t SPCMMonochromator::GetWavelength()
{
	return Wavelength;
}
uint16_t SPCMMonochromator::GetScanSpeed()
{
	return ScanSpeed;
}
uint8_t SPCMMonochromator::GetGratingCount()
{
	return GratingsInstalled;
}
uint16_t SPCMMonochromator::GetGratingGrooves()
{
	return GratingGrooves;
}
uint16_t SPCMMonochromator::GetGratingBlaze()
{
	return GratingBlaze;
}
SPCMMonochromator::UnitsType SPCMMonochromator::GetUnits()
{
	return Units;
}
uint8_t SPCMMonochromator::GetGrating()
{
	return GratingCurrent;
}
void SPCMMonochromator::Check()
{
	switch (Mode)
	{
		case ModeType::Idle:
			CheckCommandQueue();
			break;
		case ModeType::WaitForParameterReply:
			CheckForParameterReply();
			break;
		case ModeType::WaitForStatus:
			CheckForStatus();
			break;
		case ModeType::WaitForCompleted:
			CheckForCompleted();
			break;
		case ModeType::WaitToSendEcho:
			WaitToSendEcho();
			break;
		default:
			break;
	}
}
void SPCMMonochromator::CheckCommandQueue()
{
	bool NewCommandPulled = CommandQueuePullToCurrentCommand();
	if (NewCommandPulled)
	{
		if (CurrentCommand != NULL)
		{
			Busy = true;
			UpdateCurrentCommandVariables();
			bool SendStatus = SendCommand(CurrentCommand);
			if (SendStatus)
			{
				ChangeModeForCurrentCommand();
			}
		}
		else
		{
			Serial.print("<MONOERROR>(Command in queue is null.)\n");
		}
	}
	if ( (micros() - LastWipeTime) > WipeInputEvery )
	{
		LastWipeTime = micros();
		if (SerialPort->available())
		{
			if (Verbose)
			{
				uint8_t ByteRead = SerialPort->read();
				Serial.print("<MONOJ>(");
				Serial.print(ByteRead);
				Serial.print(")\n");
			}
			else
			{
				SerialPort->read();
			}
		}
	}
}
void SPCMMonochromator::ChangeModeForCurrentCommand()
{
	if (ExpectReply)
	{
		Mode = ModeType::WaitForParameterReply;
	}
	else if (ExpectStatus)
	{
		Mode = ModeType::WaitForStatus;
	}
	else
	{
		Mode = ModeType::WaitForCompleted;
	}
}
void SPCMMonochromator::Enqueue(CommandType Command)
{
	SPCMMonochromator::Enqueue(Command, 0);
}
void SPCMMonochromator::Enqueue(CommandType Command, uint32_t Parameter)
{
	CommandStruct* CommandPointer = const_cast<CommandStruct*>(&CommandLibrary[static_cast<uint8_t>(Command)]);
	SPCMMonochromator::CommandQueuePut(CommandPointer, Parameter);
}
void SPCMMonochromator::ClearCommandQueue()
{
	for (int Index = 0; Index < SPCMMonochromatorQueueCount; ++Index)
	{
		CommandQueue[Index].Command = NULL;
		CommandQueue[Index].Parameter = 0;
	}
	CommandQueueHead = 0;
	CommandQueueTail = 0;
	CommandQueueFullFlag = false;
}
bool SPCMMonochromator::CommandQueueFull()
{
	return CommandQueueFullFlag;
}
bool SPCMMonochromator::CommandQueueEmpty()
{
	return ( !CommandQueueFullFlag && (CommandQueueHead == CommandQueueTail) );
}
uint8_t SPCMMonochromator::CommandQueueCount()
{
	uint8_t Count = SPCMMonochromatorQueueCount;
	if(!CommandQueueFullFlag)
	{
		if(CommandQueueHead >= CommandQueueTail)
		{
			Count = (CommandQueueHead - CommandQueueTail);
		}
		else
		{
			Count = (SPCMMonochromatorQueueCount + CommandQueueHead - CommandQueueTail);
		}
	}
	return Count;
}
void SPCMMonochromator::CommandQueueAdvance()
{
	if(CommandQueueFullFlag)
	{
		CommandQueueTail = (CommandQueueTail + 1) % SPCMMonochromatorQueueCount;
	}
	CommandQueueHead = (CommandQueueHead + 1) % SPCMMonochromatorQueueCount;
	CommandQueueFullFlag = (CommandQueueHead == CommandQueueTail);
}
void SPCMMonochromator::CommandQueueRetreat()
{
	CommandQueueFullFlag = false;
	CommandQueueTail = (CommandQueueTail + 1) % SPCMMonochromatorQueueCount;
}
void SPCMMonochromator::CommandQueuePut(CommandStruct* CommandPointer, uint32_t Parameter)
{
	CommandQueue[CommandQueueHead].Command = CommandPointer;
	CommandQueue[CommandQueueHead].Parameter = Parameter;
	CommandQueueAdvance();
}
bool SPCMMonochromator::CommandQueuePullToCurrentCommand()
{
	bool Status = false;
	if (!CommandQueueEmpty())
	{
		CurrentCommand = CommandQueue[CommandQueueTail].Command;
		CurrentCommandParameter = CommandQueue[CommandQueueTail].Parameter;
		CommandQueueRetreat();
		Status = true;
	}
	return Status;
}
void SPCMMonochromator::UpdateCurrentCommandVariables()
{
	CurrentCommandTimeToComplete = CurrentCommand->TimeToComplete;
	if (CurrentCommandTimeToComplete == 0)
	{
		CurrentCommandTimeToComplete = TimeToCompleteDefault;
	}
	if (CurrentCommand->HasAlternateReply)
	{
		ReplyByteCount = 0;
		switch (CurrentCommand->Command)
		{
			case CommandType::Query:
				ExpectReply = true;
				ExpectStatus = true;
				ReplyByteCountMax = 2;
				break;
			case CommandType::Reset:
				ExpectReply = false;
				ExpectStatus = false;
				ReplyByteCountMax = 0;
				break;
			default:
				ExpectReply = false;
				ExpectStatus = true;
				ReplyByteCountMax = 0;
				break;
		}
	}
}
void SPCMMonochromator::CheckForParameterReply()
{
	if (SerialPort->available())
	{
		uint8_t ByteRead = SerialPort->read();
		if (Verbose)
		{
			Serial.print("[MONOPR](");
			Serial.print(ReplyByteCount);
			Serial.print(",");
			Serial.print(ReplyByteCountMax);
			Serial.print(",");
			Serial.print(ByteRead);
			Serial.print(")\n");
		}
		ReplyByteCount++;
		ReplyData[ReplyByteCountMax - ReplyByteCount] = ByteRead;
		if (ReplyByteCount == ReplyByteCountMax)
		{
			ParseReplyData();
			Mode = ModeType::WaitForStatus;
		}
		CommandReplyTime = micros();
	}
	else if ( (micros() - CommandReplyTime) > CommandReplyTimeMax)
	{
		Serial.print("<MONOERROR>(No command reply.)\n");
		ModeTransitionToIdle();
	}
}
void SPCMMonochromator::ParseReplyData()
{
	ParameterConverter Converter;
	Converter.uInt32 = 0;
	Converter.uInt8Array[0] = ReplyData[0];
	Converter.uInt8Array[1] = ReplyData[1];
	switch (CurrentCommandParameter)
	{
		case static_cast<uint8_t>(QueryType::Position):
			Wavelength = Converter.uInt16Array[0];
			break;
		case static_cast<uint8_t>(QueryType::Grooves):
			GratingGrooves = Converter.uInt16Array[0];
			break;
		case static_cast<uint8_t>(QueryType::Blaze):
			GratingBlaze = Converter.uInt16Array[0];
			break;
		case static_cast<uint8_t>(QueryType::Grating):
			GratingCurrent = Converter.uInt8Array[0];
			break;
		case static_cast<uint8_t>(QueryType::Speed):
			ScanSpeed = Converter.uInt16Array[0];
			break;
		case static_cast<uint8_t>(QueryType::GratingCount):
			GratingsInstalled = Converter.uInt8Array[0];
			break;
		case static_cast<uint8_t>(QueryType::Units):
			Units = static_cast<UnitsType>(Converter.uInt8Array[0]);
			break;
		case static_cast<uint8_t>(QueryType::Serial):
			SerialNumber = Converter.uInt16Array[0];
			break;
		default:
			break;
	}
}
bool SPCMMonochromator::RTSHandshake(bool StartEndHandshake)
{
	if (RTSWrite == NULL)
	{
		digitalWrite(RTSPin, StartEndHandshake);
	}
	else
	{
		RTSWrite(StartEndHandshake);
	}
	uint32_t RTSStart = micros();
	bool KeepWaiting = true;
	bool CTSStatus = false;
	while (KeepWaiting)
	{
		delayMicroseconds(10);
		if (IgnoreCTS)
		{
			delayMicroseconds(500);
			KeepWaiting = false;
		}
		else
		{
			if (CTSRead == NULL)
			{
				CTSStatus = digitalRead(CTSPin);
			}
			else
			{
				CTSStatus = CTSRead();
			}
			if (CTSStatus == StartEndHandshake)
			{
				KeepWaiting = false;
			}
			else if ( (RTSStart - micros()) > 100000)
			{
				Serial.print("<MONOERROR>(CTS timeout.)\n");
				KeepWaiting = false;
				return false;
			}
		}
	}
	return true;
}
void SPCMMonochromator::ModeTransitionToIdle()
{
	if (CommandQueueEmpty())
	{
		RTSHandshake(false);
		Busy = false;
		if (RecievedCallback != NULL)
		{
			RecievedCallback(CurrentCommand->Command);
		}
	}
	Mode = ModeType::Idle;
}
void SPCMMonochromator::CheckForStatus()
{
	if (SerialPort->available())
	{
		StatusByte = SerialPort->read();
		if (Verbose)
		{
			Serial.print("[MONOS](");
			Serial.print(StatusByte);
			Serial.print(")\n");
		}
		if (CurrentCommand->Command == CommandType::Echo)
		{
			if (StatusByte != 27)
			{
				Serial.print("<MONOERROR>(Echo return unexpected reply ");
				Serial.print(StatusByte); 
				Serial.print(".)\n");
			}
		}
		else
		{
			bool ParameterNotAccepted = bitRead(StatusByte,static_cast<uint8_t>(StatusByteMeaning::AcceptedNot));
			bool ParameterIsAlreadyEqual = bitRead(StatusByte,static_cast<uint8_t>(StatusByteMeaning::ActionRequiredNot));
			Units = static_cast<UnitsType>( (uint8_t)(StatusByte & 0x03) );
			if (ParameterNotAccepted && !ParameterIsAlreadyEqual)
			{
				Serial.print("<MONOERROR>(Status indicated parameter not accepted.)\n");
			}
		}
		if (CurrentCommand->Command == CommandType::Echo)
		{
			ModeTransitionToIdle();
		}
		else if (CurrentCommand->Command == CommandType::Reset)
		{
			ResetTime = micros();
			Mode = ModeType::WaitToSendEcho;
		}
		else
		{
			Mode = ModeType::WaitForCompleted;
			StatusByteReceivedTime = micros();
		}
	}
	else if ( (micros() - CommandReplyTime) > CommandReplyTimeMax)
	{
		Serial.print("<MONOERROR>(No status byte received.)\n");
		ModeTransitionToIdle();
	}
}
void SPCMMonochromator::CheckForCompleted()
{
	if (SerialPort->available())
	{
		uint8_t ByteRead = SerialPort->read();
		if (Verbose)
		{
			Serial.print("[MONOC](");
			Serial.print(ByteRead);
			Serial.print(")\n");
		}
		bool ByteMatchesCompleted = (ByteRead == CompeletedByte);
		if (!ByteMatchesCompleted)
		{
			Serial.print("<MONOERROR>(Unexpected byte received waiting for complete.)\n");
		}
		ModeTransitionToIdle();
	}
	else if ( (micros() - StatusByteReceivedTime) > CurrentCommandTimeToComplete)
	{
		Serial.print("<MONOERROR>(Timeout waiting for command to complete.)\n");
		ModeTransitionToIdle();
	}
}
void SPCMMonochromator::WaitToSendEcho()
{
	if ( (micros() - ResetTime) > ResetCompleteTime)
	{
		CommandStruct* EchoCommand = const_cast<CommandStruct*>(&CommandLibrary[static_cast<uint8_t>(CommandType::Echo)]);
		SendCommand(EchoCommand);
		Mode = ModeType::WaitForStatus;
	}
}
bool SPCMMonochromator::IsBusy()
{
	return Busy;
}
bool SPCMMonochromator::ReadCTSPin()
{
	if (UseCTSPin)
	{
		return digitalRead(CTSPin);
	}
	else
	{
		return true;
	}
}
void SPCMMonochromator::WriteRTSPin(bool Setting)
{
	if (UseRTSPin)
	{
		digitalWrite(RTSPin,Setting);
	}
}
bool SPCMMonochromator::SendCommand(CommandStruct* CommandToSend)
{
	bool Status = true;
	Status = RTSHandshake(true);
	if (CommandToSend->Command == CommandType::None)
	{
		Serial.print("<MONOERROR>(Invalid command in buffer.)\n");
		Status = false;
	}
	else if (CommandToSend->Command == CommandType::Reset)
	{
		SerialPort->write(CurrentCommand->CommandInt);
		SerialPort->write(CurrentCommand->CommandInt);
		SerialPort->write(CurrentCommand->CommandInt);
		LastCommandSentTime = micros();
	}
	else
	{
		SerialPort->write(CurrentCommand->CommandInt);
		if (Verbose)
		{
			Serial.print("[MONOS](");
			Serial.print(CurrentCommand->CommandInt);
			Serial.print(")\n");
		}
		LastCommandSentTime = micros();
	}
	return Status;
}
void SPCMMonochromator::SendCommandParameter()
{
	ParameterConverter Converter;
	Converter.uInt32 = CurrentCommandParameter;
	if (Verbose)
	{
		Serial.print("[MONOPA](");
		Serial.print(CurrentCommandParameter);
		Serial.print(")\n");
	}
	switch (CurrentCommand->SendType)
	{
		case(CommandParameterType::uInt32):
			SerialPort->write(Converter.uInt8Array[3]);
		case(CommandParameterType::uInt24):
			SerialPort->write(Converter.uInt8Array[2]);
		case(CommandParameterType::uInt16):
			SerialPort->write(Converter.uInt8Array[1]);
		case(CommandParameterType::uInt8):
			SerialPort->write(Converter.uInt8Array[0]);
			break;
		default:
			Serial.print("<MONOERROR>(Attempt to send invalid parameter.)\n");
			break;
	}
}
