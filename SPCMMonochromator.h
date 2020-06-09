#ifndef SPCMMonochromator_h	//check for multiple inclusions
#define SPCMMonochromator_h

#include "Arduino.h"

#define SPCMMonochromatorQueueCount 8

class SPCMMonochromator
{
	public:
		enum class CommandType : uint8_t
		{
			None,
			Calibrate,
			Decrement,
			Echo,
			Goto,
			Increment,
			Order,
			Query,
			Reset,
			Scan,
			Select,
			StepSize,
			Speed,
			Step,
			Units,
			Zero
		};
		enum class CommandParameterType : uint8_t
		{
			None,
			uInt8,
			uInt16,
			uInt32,
		};
		enum class UnitsType : uint8_t
		{
			Microns = 0,
			Nanometers = 1,
			Angstroms = 2,
		};
		enum class StatusByteMeaning : uint8_t
		{
			NegPosOrder = 3,
			NegPosScan = 4,
			TooSmallLarge = 5,
			ActionRequiredNot = 6,
			AcceptedNot = 7
		};
		enum class QueryType : uint8_t
		{
			Position = 0,
			Type = 1,
			Grooves = 2,
			Blaze = 3,
			Grating = 4,
			Speed = 5,
			SizeByte = 6,
			GratingCount = 13,
			Units = 14,
			Serial = 19
		}
		enum class ModeType : uint8_t
		{
			Inactive,
			Idle,
			WaitForParameterReply,
			WaitForStatus,
			WaitForCompleted,
			WaitToSendEcho
		};
		union ParameterConverter
		{
			uint32_t uInt32;
			uint8_t uInt8Array[4];
			uint16_t uInt16Array[2];
		};
		struct CommandStruct
		{
			CommandType Command;
			uint8_t CommandInt;
			CommandParameterType SendType;
			bool HasAlternateReply;
			bool ResetAfter;
			bool StartContinuousMovement;
			uint32_t TimeToComplete;
		};
		struct CommandQueueEntry
		{
			CommandStruct* Command;
			uint32_t Parameter;
		};
		typedef void ( *FinishedListener )(SPCMMonochromator::CommandType Command);
		typedef void ( *PinWriter )(bool OutputHighLow);
		typedef bool ( *PinReader )(void);
		SPCMMonochromator(HardwareSerial* serial); //Invoke with SPCMMonochromator(&SerialN);
		void SetCTSPin(uint8_t CTSPin);
		void SetRTSPin(uint8_t RTSPin);
		void SetReadCTSFunction(PinReader CTSReadFunction);
		void SetWriteRTSFunction(PinWriter RTSSetFunction);
		void SendSetWavelength(float Wavelength);
		void SendGetWavelength();
		void SendGetGrooves();
		void SendGetBlaze();
		void SendGetGrating();
		void SendGetSpeed();
		void SendGetSize();
		void SendGetGratingCount();
		void SendGetUnits();
		void SendGetSerial();
		void SetRecievedCallback(FinishedListener Finished);
		bool IsBusy();
		void Check();
		void Begin();
		float GetCurrentWavelength();
	private:
		void CheckCommandQueue();
		void Enqueue(CommandType Command);
		void Enqueue(CommandType Command, uint32_t Parameter);
		void ClearCommandQueue();
		bool SendCommand(CommandStruct* CommandToSend);
		bool CommandQueueFull();
		bool CommandQueueEmpty();
		uint8_t CommandQueueCount();
		void CommandQueueAdvance();
		void CommandQueueRetreat();
		void CommandQueuePut(CommandStruct* CommandPointer, uint32_t Parameter);
		bool CommandQueuePullToCurrentCommand();
		void UpdateCurrentCommandVariables();
		void CheckForCommandReply();
		void CheckForParameterReply();
		void ParseReplyData();
		void UpdateInternalVariables(uint32_t NewValue, CommandType PropertyToUpdate);
		void ModeTransitionToIdle();
		void CheckForStatus();
		void CheckForCompleted();
		void WaitToSendEcho();
		void SendCommandParameter();
		static const CommandStruct CommandLibrary[];
		static const uint8_t CompeletedByte;
		static const uint32_t ResetCompleteTime;
		static const uint32_t CommandReplyTimeMax;
		static const uint32_t TimeToCompleteDefault;
		static const uint8_t RetryCountMax;
		static const uint32_t WipeInputEvery;
		HardwareSerial* SerialPort;
		FinishedListener RecievedCallback;
		CommandStruct* CurrentCommand;
		PinWriter RTSWrite;
		PinReader CTSRead;
		bool UseCTSPin;
		bool UseRTSPin;
		uint8_t CTSPin;
		uint8_t RTSPin;
		uint32_t CurrentCommandParameter;
		uint32_t CurrentCommandTimeToComplete;
		uint32_t LastWipeTime = 0;
		bool ExpectStatus;
		uint8_t ReplyByteCount;
		uint8_t ReplyData[4];
		uint8_t ReplyByteCountMax;
		CommandQueueEntry CommandQueue[SPCMMonochromatorQueueCount];
		uint8_t CommandQueueHead;
		uint8_t CommandQueueTail;
		bool CommandQueueFullFlag;
		uint8_t StatusByte;
		ModeType Mode;
		bool Busy;
		uint8_t GratingCurrent;
		uint32_t LastCommandSentTime;
		uint32_t CommandReplyTime;
		uint32_t StatusByteReceivedTime;
		uint32_t ResetTime;
		uint8_t ResetWaitCount;
		uint32_t CommandRetryCount;
		uint16_t SerialNumber;
		uint16_t Wavelength;
		uint16_t ScanSpeed;
		uint8_t GratingsInstalled;
		uint8_t GratingsCurrent;
		uint16_t GratingGrooves;
		uint16_t GratingBlaze;
};
#endif
