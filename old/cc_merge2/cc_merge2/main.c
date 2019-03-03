#define F_CPU 16000000 
#include "main.h"

uint16_t ignitionStateDebounceCount = 0; 
uint8_t ignitionStateLock = 0;
uint8_t ignitionState = 0;

uint8_t armedState = 0;

uint8_t shutdownState = 0;

uint8_t RecievedMsgInv = 1;

volatile uint8_t testError = 0;

uint8_t inverterArray[8] = {0,0,0,0,0,0,0,0};
uint8_t PDMarray[8] = {0,0,0,0,0,0,0,0};
uint8_t WheelArray[8] = {0,0,10,10,0,0,40,200};

volatile uint8_t out = 0;

int main(void) {    

    // Set Up
    firmware_init();
    timer_init();

    // Grab the state of the shutdown circuity
    shutdownState = 0;
	_delay_ms(500);
    // Enable Interupts
    sei();		
    // Main Poll
    // ------------------------------------------------------------------------
    while(1) {
		
		if(isCharAvailable() == 1)uart_process_byte(receiveChar());
		uart1_puts("Hello World!\n");
	}
	
	return 0;
}

/**
 * @brief Core system's timer, 1ms, core of the pseudo RTOS system
 */
void oneKHzTimer(void)
{
	static int test_counter = 0;

    static uint16_t CANheartbeatCountInverters = 0;		// Number of iterations for the inverter heartbeat trigger
    static uint16_t CANheartbeatCountWheel = 1;			// Number of iterations for the data heartbeat trigger
    static uint16_t CANheartbeatCountPDM = 2;			// Number of iterations for the power heartbeat trigger
	static uint16_t CANheartbeatCountShutdown = 3;			// Number of iterations for the power heartbeat trigger
	static uint16_t CANheartbeatCountAMU = 4;			// Number of iterations for the power heartbeat trigger

    // static uint8_t CanHeartbeatErrorInverters = 100;		// Time without successfull heartbeat for inverters
    // static uint8_t CanHeartbeatErrorData = 101;			// Time without successfull heartbeat for data
    // static uint8_t CanHeartbeatErrorPower = 102;			// Time without successfull heartbeat for power

    static uint8_t InputPedalThrottleCount = 3;				// Number of iterations for the pot heartbeat trigger
    static uint8_t InputPedalBrakeCount = 4;				// Number of iterations for the pot heartbeat trigger
	static uint8_t InputSteeringCount = 5;			// Number of iterations for the pot heartbeat trigger
    // static uint8_t InputTempCount = 5;					// Number of iterations for the temp heartbeat trigger

    // static uint8_t CANInputSendTime = 0;					// Number of iterations for the input send trigger

	// flash the LED to show the system is running
	if(test_counter++ > 100)
	{
		//PORTK ^= 32;
		test_counter = 0;
		//UART_formTestPacket();
	}
	
	
	
    // Check the ignition button state
    // 1s debounce, IE hold for 50ms and if held, change state
    // ------------------------------------------------------------------------
	if(!(PINJ & (1<<PJ6))) // Checking Pin J6 (69)
    {
        ignitionState = 1; // Tracks that the ignition button is on
        if(ignitionStateDebounceCount++ > 2000) // If 1s has been counted
		{
            //If this is the first time though from a previous press
            if(ignitionStateLock == 0) {
                ignitionStateLock = 1;		// Disabled first run though after press
                armedState ^= 1;
            }
        }
    }
    else {
        ignitionState = 0; // Tracks that the ignition button is off
        ignitionStateLock = 0; // Re-enables the first run though after the timer has been reached
        ignitionStateDebounceCount = 0; // Resets the counter for time the button is pressed
    }
	
     //Send CAN heartbeats -> Inverters: 100Hz, Data: 100Hz, Power: 20Hz
     //100Hz = 1 / 100 = 0.01s = 10ms, 20Hz = 1 / 20 = 0.05s = 50ms
     //------------------------------------------------------------------------
    
	
	 inverterArray[0] = INPUT_accelerationPedal;//CAN_HEARTBEAT_TIME_INVERTERS
	 if(CANheartbeatCountInverters >= CAN_HEARTBEAT_TIME_INVERTERS)
	 {
		 // Reset inverter heartbeat counter
		 CANheartbeatCountInverters = 0;
		 // To wait for Inv message
		 RecievedMsgInv = 0;
		 // Send inverter system heartbeat 0b0100100000000000000000000011110
		 CAN_send(TRACTIVE_CAN, 8, inverterArray, 0x4666666);
	 }else if(CANheartbeatCountInverters > CAN_HEARTBEAT_TIME_INVERTERS * 2){
		// Inverter dead, shutdown
	 }
	 
	 if(CANheartbeatCountWheel > CAN_HEARTBEAT_TIME_WHEEL)
	 {
		 // Reset data heartbeat counter
		 CANheartbeatCountWheel = 0;
		 // Send data system heartbeat
		 CAN_send(DATA_CAN, 8, WheelArray, HEARTBEAT_WHEEL_ID | 1);
		 
	 }
	 
	 if(CANheartbeatCountPDM > CAN_HEARTBEAT_TIME_PDM)
	 {
		 // Reset power heartbeat counter
		 CANheartbeatCountPDM = 0;
		 // Send power system heartbeat
		 //if(armedState == 1)PDMarray[0] |= 255; //192
		 PDMarray[0] |= 255; // testing CAN
		 PDMarray[1] |= 255; // testing CAN
		 PDMarray[2] |= 255; // testing CAN
		 PDMarray[3] |= 255; // testing CAN
		 PDMarray[4] |= 0; // testing CAN
		 PDMarray[5] |= 0; // testing CAN
		 PDMarray[6] |= 0; // testing CAN
		 PDMarray[7] |= 0; // testing CAN
		 //else PDMarray[0] &= ~255;
		 CAN_send(POWER_CAN, 8, PDMarray, HEARTBEAT_PDM_ID | 1);
	 }
	 
	 if(CANheartbeatCountShutdown > CAN_HEARTBEAT_TIME_SHUTDOWN)
	 {
		 // Reset power heartbeat counter
		 CANheartbeatCountShutdown = 0;
		 // Send shutdown heartbeat (dont care what for now)
		 //CAN_send(POWER_CAN, 8, PDMarray, HEARTBEAT_SHUTDOWN_ID | 1);
	 }
	 
	 if(CANheartbeatCountAMU > CAN_HEARTBEAT_TIME_AMU)
	 {
		 // Reset power heartbeat counter
		 CANheartbeatCountAMU = 0;
		 // Send shutdown heartbeat (dont care what for now)
		 //CAN_send(POWER_CAN, 8, PDMarray, HEARTBEAT_AMU_ID | 1);
	 }
	 
	 // the adding commented to test a counting system in the inverters if statement
	 CANheartbeatCountInverters++;
	 CANheartbeatCountWheel++;
	 CANheartbeatCountPDM++;
	 CANheartbeatCountShutdown++;
	 CANheartbeatCountAMU++;
    // CAN Error counts -> Missing Receives
    // ------------------------------------------------------------------------
    // if(CanHeartbeatErrorInverters > CAN_HEARTBEAT_ERROR_DELAY)
    // {
    //     throw_error_code(ERROR_LEVEL_WARN, ERROR_CANBUS_1_NO_RESPONSE);
    // }
    // if(CanHeartbeatErrorData > CAN_HEARTBEAT_ERROR_DELAY)
    // {
    //     throw_error_code(ERROR_LEVEL_WARN, ERROR_CANBUS_2_NO_RESPONSE);
    // }
    // if(CanHeartbeatErrorPower > CAN_HEARTBEAT_ERROR_DELAY)
    // {
    //     throw_error_code(ERROR_LEVEL_WARN, ERROR_CANBUS_3_NO_RESPONSE);
    // }
    // CanHeartbeatErrorInverters++;
    // CanHeartbeatErrorData++;
    // CanHeartbeatErrorPower++;


    // Send CAN input
    uint8_t tmpInputVal;
    if(InputPedalThrottleCount > INPUT_TIME_PEDAL_THROTTLE)
    {
		//INPUT_accelerationPedal = a2d_10bitCh(5);

        if(INPUT_get_accelPedal(&tmpInputVal) == 0) {
             INPUT_accelerationPedal = tmpInputVal;
        }
        InputPedalThrottleCount = 0;
    }

    if(InputPedalBrakeCount > INPUT_TIME_PEDAL_BRAKE)
    {
        //INPUT_brakePedal = (uint8_t)(a2d_10bitCh(8)/4);
        if(INPUT_get_brakePedal(&tmpInputVal) == 0) {
             INPUT_brakePedal = tmpInputVal;
		}
		
        InputPedalBrakeCount = 0;
    }
	
	if(InputSteeringCount > INPUT_TIME_STEERING)
	{
		INPUT_steeringAngle = (uint8_t)(a2d_8bitCh(INPUT_STEERING_ANGLE_CH));
		//if(INPUT_get_steeringWheel(&tmpInputVal) == 0) {
			//INPUT_steeringAngle = tmpInputVal;
		//}
		
		InputSteeringCount = 0;
	}
	
    InputPedalThrottleCount++;
    InputPedalBrakeCount++;
	InputSteeringCount++;
    
	
	inverterArray[0] = INPUT_accelerationPedal;
	WheelArray[1] = INPUT_accelerationPedal;

    // if(INPUT_get_brakePressureBack(&tmpInputVal) == 0) {
    //     INPUT_brakePressureBack = tmpInputVal;
    // }
    // if(INPUT_get_brakePressureFront(&tmpInputVal) == 0) {
    //     INPUT_brakePressureFront = tmpInputVal;
    // }
	
}

// -------------------------------------------------- Interrupt Service Routines --------------------------------------------------

/**
 * @brief Called whenever the 1Khz timer triggers
 */
ISR(TIMER0_COMPA_vect)
{
    oneKHzTimer();
	//uart_puts("HelloWorld!");
	//char msg[12];
	//sprintf(msg, "r: %d", out);
	//uart_puts(msg);
}

ISR(TIMER1_COMPA_vect)
{

}

/**
 * @brief Called whenever CANBUS 1 interupt is triggered
 *        * When ever there is data waiting in CAN 1
 */
ISR(INT1_vect) {
	// Details about the message we're attempting to pull from the CAN bus
	uint8_t data[8];
	uint32_t ID;
	uint8_t numBytes;
	led_toggle();
	RecievedMsgInv = 1;
	// Get the data from the CAN bus and process it
	CAN_pull_packet(TRACTIVE_CAN, &numBytes, data, &ID);

    // If the data packet is crap
    // throw_error_code(ERROR_LEVEL_WARN, ERROR_CANBUS_1_RESPONSE_MALFORMED);
	//out++;
}

/**
 * @brief Called whenever CANBUS 2 interupt is triggered
 *        * When ever there is data waiting in CAN 2
 */
ISR(INT0_vect)	{
	// Details about the message we're attempting to pull from the CAN bus
	uint8_t data[8];
	uint32_t ID;
	uint8_t numBytes;
	//led_toggle();
	// Get the data from the CAN bus and process it
	CAN_pull_packet(POWER_CAN, &numBytes, data, &ID);

    // If the data packet is crap
    // throw_error_code(ERROR_LEVEL_WARN, ERROR_CANBUS_2_RESPONSE_MALFORMED);
		
}

/**
 * @brief Called whenever CANBUS 3 interupt is triggered
 *        * When ever there is data waiting in CAN 3
 */
ISR(PCINT0_vect) {
	// Details about the message we're attempting to pull from the CAN bus
	uint8_t data[8];
	uint32_t ID;
	uint8_t numBytes;
	//led_toggle();
	// Get the data from the CAN bus and process it
	CAN_pull_packet(DATA_CAN, &numBytes, data, &ID);

    // If the data packet is crap
    // throw_error_code(ERROR_LEVEL_WARN, ERROR_CANBUS_3_RESPONSE_MALFORMED);

	

}