/* USER CODE BEGIN Header */
/* Copyright (C) 2023 Brad Kauffman <brad@prydwen.info>

   This Program is free software; you can redistribute it and/or
   modify it under the terms of the GNU General Public
   License as published by the Free Software Foundation; either
   version 3 of the License, or (at your option) any later version.

   This is based on the original work by:
   Sean D'Epagnier <seandepagnier@gmail.com>
   modified by:
   Timo Birnschein (timo.birnschein@googlemail.com)

   The previous versions were for Arduino Nano boards.

   This version has been rebuilt into an STM32CubeIDE project to run on an
   STM32F103C8 such as found on the popular "blue pill". All Nano specific code
   has been removed. The original communication with Pypilot from Sean has been retained.
   Timo followed on with work to use an off the shelf IBT-2 motor controller. This code
   has been simplified to ONLY apply to the IBT-2 type controller.
   I have also retained Timo's work on using exponential filters on analog inputs.

   This is currently aimed at a boat with hydraulic steering so no clutch has been
   implemented. It would be easy to add.

   Please see config.h to set important configuration parameters

   The pinout for the STM32 is as follows:

   Motor controller---STM32
        RPWM          PB3
        LPWM          PA15
        R_EN          PB12
        L_EN          PB13
        R_IS          See notes on current sensing
        L_IS          See notes on current sensing
        VCC           3.3
        Gnd           G

        RPi-----------STM32
        Tx            PA10
        Rx            PA9
        Gnd           G
        3.3V          3.3

Analog Sensors- All are 0.0-3.3V analog signals

        Voltage                  PA2
        Current                  PA1
        Rudder Position          PA0
        Motor Temperature        PA3
        Controller Temperature   PA4

In the current version The analog inputs are all read and filtered but only Rudder
Position is used for control. Other inputs will be implemented later

Notes on current sensing:
The IS pins on the IBT-2 boards are a current source based on a linear ratio to the load current.
This current source needs to be converted to voltage by a resistor to ground. Furthermore each IS
pin is only active for current in one direction. The IBT-2 board supplies a resistor to ground
for each IS pin but it will not give a 0-3.3V signal. By wiring the two IS pins together and
supplying an additional resister to ground the signal can be conditioned. The resistor to ground
is dependant on the motor voltage being used and which driver chip is on the IBT-2. I have
seen two different chips used. I will provide more on this when I implement it in this code.

*/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "usb.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "config.h"
#include "crc.h"
//#include "adc_filtering.h"
#include "stdlib.h"
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t inbyte=0, SerialInReady=0;
uint8_t in_bytes[3];
uint8_t TxPending=0;
uint8_t ADCcomplete=0;
uint8_t sync_b=0;
uint8_t in_sync_count;
uint16_t flags = 0;
uint8_t serialin;
uint16_t count=0;
uint8_t timeout = 0;
uint16_t command_value = 1000;
uint16_t lastpos = 1000;

uint16_t max_current = CURRENT_MOTOR_MAX; // 20 Amps
uint16_t max_controller_temp= TEMPERATURE_CONTROLLER_MAX; // 70C
uint16_t max_motor_temp = TEMPERATURE_MOTOR_MAX; // 70C
uint8_t max_slew_speed = SPEEDUP_SLEW_RATE;
uint8_t max_slew_slow = SLOWDOWN_SLEW_RATE; // 200 is full power in 1/10th of a second
uint16_t rudder_min = RUDDER_MIN;
uint16_t rudder_max = RUDDER_MAX; // Analog rudder value between -100 and 100 full scale.

uint16_t RudderRaw,RudderFiltered,RudderHistory;
uint16_t VoltRaw,VoltFiltered,VoltHistory;
uint16_t AmpRaw,AmpFiltered,AmpHistory;
uint16_t TempMotorRaw,TempMotorFiltered,TempMotorHistory;
uint16_t TempControllerRaw,TempControllerFiltered,TempControllerHistory;

uint32_t last_loop_cycle_millis = 0;
uint32_t last_loop_voltage_millis=0;
uint32_t last_loop_rudder_millis = 0;
uint32_t last_loop_current_millis =0;
uint32_t last_loop_temperature_millis=0;
uint8_t  out_sync_pos = 0;
uint8_t OutputPacket[4];
uint16_t AnalogRaw[5];

uint8_t currentByte;
// Test variables
float LoopTime;
uint32_t LastLoopTick;
uint32_t LoopTotal;
uint16_t Loopi;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void process_packet(void);
void stop();
void position(uint16_t value);
void engage();
void disengage();
void update_command();
void detach();
void SendByte(void);
void stop_port(void);
void stop_starboard(void);
void SerialIn(void);
void ADC_updateAndFilter(void);
void SetFlags(void);

//test stuff to be removed
void GetLoopTime(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();         //turn on I/O pins
  MX_DMA_Init();          // Setup DMA for ADC
  MX_USART1_UART_Init();  // Setup UART for serial communication
  MX_TIM2_Init();         // Setup PWM channels
  MX_ADC1_Init();         // Turn on ADC
  MX_USB_PCD_Init();      // Start USB
#ifndef NO_WATCH_DOG
  MX_IWDG_Init();         // Start watchdog
#endif
  /* USER CODE BEGIN 2 */
// Initialize Motor driver outputs
  RPWMset = 0;  // PWM set to zero duty cycle to both h-bridges
  LPWMset = 0;
  EnableL_High; // Enable both half bridges
  EnableR_High;
//Start peripherals
  HAL_UART_Receive_IT(&huart1,&inbyte,1);
  HAL_ADC_Start_DMA(&hadc1,(uint32_t *) AnalogRaw,5);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
  // wait for first analog scan complete
  while (ADCcomplete==0)
  {
  }
  // initialize all analog variables
  RudderFiltered = AnalogRaw[0]; //AnalogRaw array is filled by ADC->DMA
  RudderHistory =  AnalogRaw[0];

  VoltFiltered = AnalogRaw[1];
  VoltHistory  = AnalogRaw[1];

  AmpFiltered = AnalogRaw[2];
  AmpHistory  = AnalogRaw[2];

  TempMotorFiltered = AnalogRaw[3];
  TempMotorHistory  = AnalogRaw[3];

  TempControllerFiltered = AnalogRaw[4];
  TempControllerHistory  = AnalogRaw[4];

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /****************************************Main Control Loop ************************************/
  while (1)
  {
#ifndef NO_WATCH_DOG
	  HAL_IWDG_Refresh(&hiwdg) ; //reset watchdog timer
#endif
	  GetLoopTime(); //debug for finding loop time in milliseconds
	  update_command();      // this updates the motor command at appropriate slew rate
	  ADC_updateAndFilter(); //Update all ADC values
	  if(timeout == 120) disengage();  // disengage if nothing is going on, timeout is reset when command is processed
	  if(timeout >= 128) detach();     // detach 160 ms later
	  SerialIn(); // get serial input from pypilot, process command if valid command received
	  SetFlags(); // check all inputs and set flags to send back to pypilot, also shut down for error conditions
	  SendByte(); // Send data back to Pypilot

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
/**************************************End Main Control Loop **********************************/


  }
  // This is outside infinite loop. We can't get here.
  /* USER CODE END 3 */
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}
/* USER CODE BEGIN 4 */
// Takes action on the received packet from serial_in()
void process_packet()
{
    flags |= SYNC;
    uint16_t value = in_bytes[1] | in_bytes[2]<<8;

    switch(in_bytes[0])
    {
    case REPROGRAM_CODE:
    	{
    		// Not implemented for STM32
    	} break;
    case RESET_CODE:
        // reset overcurrent flag
        flags &= ~OVERCURRENT_FAULT;
        break;
    case COMMAND_CODE:


        timeout = 0; // Reset timeout to make sure we're not resetting anything else
        if(serialin < 12)
            serialin+=4; // output at input rate
        if(value > 2000); // out of range (can only be positive because it's a uint16_t)
            // unused range, invalid!!!
            // ignored
        else if(flags & (OVERTEMP_FAULT | OVERCURRENT_FAULT | BADVOLTAGE_FAULT));
            // no command because of overtemp or overcurrent or badvoltage
        else if((flags & (PORT_PIN_FAULT | MAX_RUDDER_FAULT)) && value > 1000)
            stop();
            // no forward command if port fault
        else if((flags & (STARBOARD_PIN_FAULT | MIN_RUDDER_FAULT)) && value < 1000)
            stop();
            // no starboard command if port fault
        else {

            command_value = value;

            engage();

        }
        break;
    case MAX_CURRENT_CODE: { // current in units of 10mA
      /*
       * Todo: Needs reimplpementation.
       */

        unsigned int max_max_current = 2000;

        if(value > max_max_current)
            value = max_max_current;
        max_current = value;
    } break;
    case MAX_CONTROLLER_TEMP_CODE:
        if(value > 10000) // maximum is 100C
            value = 10000;
        max_controller_temp = value;
        break;
    case MAX_MOTOR_TEMP_CODE:
        if(value > 10000) // maximum is 100C
            value = 10000;
        max_motor_temp = value;
        break;

    case RUDDER_MIN_CODE:
        rudder_min = value;
        break;
    case RUDDER_MAX_CODE:
        rudder_max = value;
        break;
    case DISENGAGE_CODE:
        if(serialin < 12)
            serialin+=4; // output at input rate
        disengage();
        break;
    case MAX_SLEW_CODE: {
        max_slew_speed = in_bytes[1];
        max_slew_slow =  in_bytes[2];

        // if set at the end of range (up to 255)  no slew limit
        if(max_slew_speed > 250)
            max_slew_speed = 250;
        if(max_slew_slow > 250)
            max_slew_slow = 250;

        // must have some slew
        if(max_slew_speed < 1)
            max_slew_speed = 1;
        if(max_slew_slow < 1)
            max_slew_slow = 1;
    } break;

    case EEPROM_READ_CODE:{
        // Not implemented for STM32
        }
    break;

    case EEPROM_WRITE_CODE:
        //Not implemented for STM32
    break;
    }
}
// Stops motor by sending a neutral command
void stop()
{
    position(1000);
    command_value = 1000;
}
void stop_port()
{
    if(lastpos > 1000)
       stop();
}
void stop_starboard()
{
    if(lastpos < 1000)
       stop();
}
// Sets PWM and direction based on current command value
void position(uint16_t value)
{
  // store the new value as the new last position, used by update_command() to determine next command
  lastpos = value;
  int16_t newValue = abs((int)value - 1000);

  // Determine if value is bigger or smaller than 1000 plus deadzone
  if(value > 1000 + PWM_DEADBAND)
  {
    // Turn PWM for CCW operation on and the other off
    RPWMset = (uint16_t)(5*newValue);  //timer count is 5000 and command max is 1000 so 5*1000 gives maximum command
    LPWMset =0;
  } else if(value < 1000 - PWM_DEADBAND)
  {
    // Turn PWM for CW operation on and the other off
	    LPWMset = (uint16_t)(5*newValue);
	    RPWMset =0;
  } else
  {
      // Nothing to do. We got about 1000 and need to turn  PWM off
    LPWMset = 0;
    RPWMset = 0;
  }


}
// Sets PWM to zero and turns on driver enables
void engage()
{
    if(flags & ENGAGED)
    { // Already engaged
        return;
    }

    RPWMset = 0; // Disengage PWM to both h-bridges
    LPWMset = 0; // Disengage PWM to both h-bridges
    EnableL_High; // Enable both half bridges
    EnableR_High;
    timeout = 0;
    flags |= ENGAGED;

}
void disengage() // Will not be changed
{
    stop();
    flags &= ~ENGAGED;
    timeout = 60; // detach in about 62ms

}
// Ramps the motor speed up to the commanded rate based on the slew speed set in Pypilot
void update_command()
{
    if(HAL_GetTick() - last_loop_cycle_millis > 20)
    {
        if(flags & ENGAGED)
        {
        	int16_t speed_rate  = max_slew_speed;
        	int16_t slow_rate   = max_slew_slow; // value of 20 is 1 second full range at 50hz
        	uint16_t cur_value  =  lastpos;
        	int16_t diff        = (int)command_value - (int)cur_value;

        	// limit motor speed change to stay within speed and slow slew rates
        	if(diff > 0)
        	{
        		if(cur_value < 1000)
        		{
        			if(diff > slow_rate)
        				diff = slow_rate;
        		} else
        			if(diff > speed_rate)
        				diff = speed_rate;
        	} else
        	{
        		if(cur_value > 1000)
        		{
        			if(diff < -slow_rate)
        				diff = -slow_rate;
        		} else
        			if(diff < -speed_rate)
        					diff = -speed_rate;
        	}


    // Push the new value over to the position function
    position(cur_value + diff);
        }

	timeout++; // timeout resets when valid command processed
	last_loop_cycle_millis = HAL_GetTick(); // Store the time from here to next iteration
 }
}
// Turns off the driver enable pins and sets the PWMs to zero duty cycle
void detach()
{
    RPWMset = 0; // Disengage PWM to both h-bridges
    LPWMset = 0; // Disengage PWM to both h-bridges
    EnableL_Low; // Disable both H-Bridges. This will also disable the motor brake
    EnableR_Low;
    timeout = 80; // avoid overflow
}
//Receives serial data from Pypilot, syncs to input stream, Checks for valid command and calls process_packet()
void SerialIn(void)
{
    /*
     * **************************** SERIAL INPUT ***********************************************
     * 1. Read 3 bytes
     * 2. Check if the fourth byte is a valid CRC of the first three.
     * 3. If yes, check if at least three packets in a row were valid, if so process packet.
     * 4. If no, input isn't synced, advance one byte and try again until sync is achieved
     */
if (SerialInReady)
	{
		uint8_t c=currentByte;
		SerialInReady=0;
		HAL_UART_Receive_IT(&huart1,&inbyte,1);
		//count++;
		if(sync_b < 3) // get a string of 3 bytes
		{
			in_bytes[sync_b] = c;
			sync_b++;
		} else
		{

			if(c == crc8(in_bytes, 3))// check CRC of string and act if match
			{
				if(in_sync_count >= 2) // Wait for 3 valid commands before acting just to be safe
				{ // if crc matches, we have a valid packet
					process_packet(); //act on current received input
				} else
					in_sync_count++;

				sync_b = 0;
				flags &= ~INVALID;
			} else
			{
	              // invalid packet or not synced to input
				  // flag not sync and stop until valid command stream
	              flags &= ~SYNC;
	              stop();
	              in_sync_count = 0; //reset counts
	              // rotate in_byte string by one and get another to get into sync
	              in_bytes[0] = in_bytes[1];
	              in_bytes[1] = in_bytes[2];
	              in_bytes[2] = c;
	              flags |= INVALID;
			}

		}
	}

}
// Sends packets of data back to pypilot, cycles through data based on out_sync_pos which incremented after each packet is sent
void SendByte(void)
{
if (TxPending) return; //Skip if UART is busy

	        // match output rate to input rate
	        if(serialin < 4)
	            return;

	        uint16_t v;
	        uint8_t code;

	        //  flags C R V C R ct C R mt flags  C  R  V  C  R EE  C  R mct flags  C  R  V  C  R  EE  C  R rr flags  C  R  V  C  R EE  C  R cc  C  R vc
	        //  0     1 2 3 4 5  6 7 8  9    10 11 12 13 14 15 16 17 18  19    20 21 22 23 24 25  26 27 28 29    30 31 32 33 34 35 36 37 38 39 40 41 42
	        switch(out_sync_pos++)
	        {
	        	case 0: case 10: case 20: case 30:
	                flags |= CURRENT_RANGE;
	                v = flags;
	                 code = FLAGS_CODE;
	                 break;
	             case 1: case 4: case 7: case 11: case 14: case 17: case 21: case 24: case 27: case 31: case 34: case 37: case 40:
	                 v = AmpFiltered;
	                 code = CURRENT_CODE;
	                 serialin-=4; // fix current output rate to input rate
	                 break;
	             case 2: case 5: case 8: case 12: case 15: case 18: case 22: case 25: case 28: case 32: case 35: case 38: case 41:
	                 v = RudderFiltered;
	                 code = RUDDER_SENSE_CODE;
	                 break;
	             case 3: case 13: case 23: case 33:
	                 v = VoltFiltered;
	                 code = VOLTAGE_CODE;
	                 break;
	             case 6:
	                 v = TempControllerFiltered;
	                 code = CONTROLLER_TEMP_CODE;
	                 break;
	             case 9:
	                 v = TempMotorFiltered;
	                 code = MOTOR_TEMP_CODE;
	                 break;
	             case 16: case 26: case 36: /* eeprom reads */
	                 // Not implemented on STM32
	                 return;
	             default:
	                 return;
	        }
	        // Assemble output packet
	        OutputPacket[0] = code;
	        OutputPacket[1] = v;
	        OutputPacket[2] = v>>8;
	        OutputPacket[3]=crc8(OutputPacket, 3);
	    	HAL_UART_Transmit_IT(&huart1, OutputPacket,4); //send it out
	        TxPending=1;

}
// Add current analog values based on alpha values for exponential filtering
// This is a simple low pass single pole recursive filter of the form
//   Y[n]=a*X[n]+b*Y[n-1] where a=1-x, b=x, x<1 ref:The Scientist and Engineer's Guide to
//   Digital Signal Processing by Steven W. Smith, Phd.
// For some reason the coefficients are defined here in terms of a or ALPHA rather than x
void ADC_updateAndFilter(void)
{
	if(ADCcomplete) //Set in ADC ISR HAL_ADC_ConvCpltCallback
	{

      RudderRaw = AnalogRaw[0];  //AnalogRaw is the DMA location of ADC output, 5 channels 0-4
      RudderFiltered = (uint16_t)(ALPHA_RUDDER * (float)RudderRaw + (1 - ALPHA_RUDDER) * (float)RudderHistory);
      RudderHistory = RudderFiltered;

      VoltRaw = AnalogRaw[1];
      VoltFiltered = (uint16_t)(ALPHA_VOLTAGE * (float)VoltRaw + (1 - ALPHA_VOLTAGE) * (float)VoltHistory);
      VoltHistory = VoltFiltered;

      AmpRaw = AnalogRaw[2];
      AmpFiltered = (uint16_t)(ALPHA_CURRENT * (float)AmpRaw + (1 - ALPHA_CURRENT) * (float)AmpHistory);
      AmpHistory = AmpFiltered;

      TempMotorRaw = AnalogRaw[3];
      TempMotorFiltered = (uint16_t)(ALPHA_TEMPMOTOR * (float)TempMotorRaw + (1 - ALPHA_TEMPMOTOR) * (float)TempMotorHistory);
      TempMotorHistory = TempMotorFiltered;

      TempControllerRaw = AnalogRaw[4];
      TempControllerFiltered = (uint16_t)(ALPHA_TEMPCNTRL * (float)TempControllerRaw + (1 - ALPHA_TEMPCNTRL) * (float)TempControllerHistory);
      TempControllerHistory = TempControllerFiltered;

      ADCcomplete=0; // will be set again in ADC ISR
	}

}
void SetFlags(void)
{
#ifndef DISABLE_RUDDER_SENSE

    if (HAL_GetTick() - last_loop_rudder_millis > 100)
    {
      uint16_t v = RudderFiltered;
      // if not positive, then rudder feedback has negative gain (reversed)
      uint8_t pos = rudder_min < rudder_max;

      if((pos && v < rudder_min) || (!pos && v > rudder_min)) {
          stop_starboard();
          flags |= MIN_RUDDER_FAULT;
      } else
          flags &= ~MIN_RUDDER_FAULT;
      if((pos && v > rudder_max) || (!pos && v < rudder_max)) {
          stop_port();
          flags |= MAX_RUDDER_FAULT;
      } else
          flags &= ~MAX_RUDDER_FAULT;

      last_loop_rudder_millis = HAL_GetTick();
    }
#endif

#ifndef DISABLE_CURRENT_SENSE
    if (HAL_GetTick() - last_loop_current_millis > 500)
    {
      uint16_t amps = AmpFiltered;
      if(amps >= max_current) {
          stop();
          flags |= OVERCURRENT_FAULT;
      } else {
          flags &= ~OVERCURRENT_FAULT;
      }
      last_loop_current_millis = HAL_GetTick();
    }
#endif

#ifndef DISABLE_VOLTAGE_SENSE

    if (HAL_GetTick() - last_loop_voltage_millis > 2000)
    {
      uint16_t volts = VoltFiltered;
      // voltage must be between min and max voltage
      if(volts <= (uint16_t)(VIN_MIN) || volts >= (uint16_t)(VIN_MAX)) {
          stop();
          flags |= BADVOLTAGE_FAULT;
      } else
          flags &= ~BADVOLTAGE_FAULT;

      last_loop_voltage_millis = HAL_GetTick();
    }
#endif

#ifndef DISABLE_TEMP_SENSE

    if (HAL_GetTick() - last_loop_temperature_millis > 4000)
    {
      uint16_t controller_temp = TempControllerFiltered;
      uint16_t motor_temp = TempMotorFiltered;
      if(controller_temp >= max_controller_temp || motor_temp >= max_motor_temp) {
          stop();
          flags |= OVERTEMP_FAULT;
      } else
          flags &= ~OVERTEMP_FAULT;

      last_loop_temperature_millis = HAL_GetTick();
    }
// todo: need to fix this for shutdown
//    if(controller_temp > 11000) {
//        stop();
//        asm volatile ("ijmp" ::"z" (0x0000)); // attempt soft reset
//    }
#endif
}

void GetLoopTime(void)
{
	uint32_t t=HAL_GetTick();

	if(t>=LastLoopTick)
	{
		uint32_t delta =t-LastLoopTick;
		LoopTotal=LoopTotal+delta;
		Loopi++;
		LastLoopTick=t;
		if (Loopi>1000)
		{
			LoopTime=(float)LoopTotal/1000;
			LoopTotal=0;
			Loopi=0;
		}
	}

}
/*********************Interrupt callback routines********************************************************/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef * hadc)
{
ADCcomplete=1;
}
void HAL_UART_RxCpltCallback  ( UART_HandleTypeDef *  huart )
{
	SerialInReady=1;
	currentByte=inbyte;
}
void HAL_UART_TxCpltCallback  ( UART_HandleTypeDef *  huart )
{
	TxPending=0;
}
/* USER CODE END 4 */
/**
 * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}
/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
