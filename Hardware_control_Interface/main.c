#include "StdTypes.h"
#include "MemMap.h"
#include "Utlis.h"
# define F_CPU 8000000UL
#include <util/delay.h>
#include "DIO_Interface.h"
#include "DIO_Private.h"
#include "UART.h"
#include "UART_Service.h"
#include "Timers.h"
#include "Timers_Services.h"
#include "EX_Interrupt.h"
#include <avr/interrupt.h>
#include <stdlib.h>
#include <string.h>

/*=================== DEFINITIONS ================ */

#define CONTROL_INTERVAL_MS 100
#define PULSES_PER_REVOLUTION 3828.0  // Total encoder counts per wheel revolution
#define MAXIMUM_RPM 97.0  // Free-run speed at 12V
#define RATED_RPM 80.0    // Rated speed at 12V

// Convert RPM to rad/sec
#define MAX_RAD_PER_SEC ((MAXIMUM_RPM * 2.0 * 3.141592653589793) / 60.0)
#define RAD_PER_SEC ((2.0 * 3.141592653589793) / PULSES_PER_REVOLUTION)
#define CMD_BUFFER_SIZE 64
/*=================== PIN DEFINITIONS ================ */

#define MOTOR_A_ENABLE_PIN PIND5  // --> OC1A - TIMER1
#define MOTOR_B_ENABLE_PIN PIND4  // --> OC1B - TIMER1
#define MOTOR_C_ENABLE_PIN PINB3  // --> OC0 - TIMER0
#define MOTOR_D_ENABLE_PIN PIND7  // --> OC2 - TIMER2

#define MOTOR_A_DIR1 PINA1
#define MOTOR_A_DIR2 PINA0
#define MOTOR_B_DIR1 PINA3
#define MOTOR_B_DIR2 PINA2
#define MOTOR_C_DIR1 PINA4
#define MOTOR_C_DIR2 PINA5
#define MOTOR_D_DIR1 PINA6
#define MOTOR_D_DIR2 PINA7

#define ENCODER_A_PHASE_A  PIND2 // INT0
#define ENCODER_A_PHASE_B  PINC1
#define ENCODER_B_PHASE_A  PIND3 // INT1
#define ENCODER_B_PHASE_B  PINC3
#define ENCODER_C_PHASE_A PINB2  // INT2
#define ENCODER_C_PHASE_B PINC5
#define ENCODER_D_PHASE_A PINC6
#define ENCODER_D_PHASE_B PINC7

#define MIN_MOTOR_PWM 35
/*=================== GLOBAL VARIABLES ================ */

volatile s32 encoder_counts[4] = {0,0,0,0};
volatile Bool_t motor_directions[4] = {TRUE,TRUE,TRUE,TRUE};
double target_velocities[4] = {0.0,0.0,0.0,0.0}; // from ros2 controller
double measured_velocities[4] = {0.0,0.0,0.0,0.0}; // Actual velocities
u8 motor_commands[4] = {0,0,0,0}; // PWM Commands (0-255)


// Software timer variables
volatile u32 system_ticks = 0;
volatile u32 last_control_update = 0;


typedef struct
{
	double kp;
	double ki;
	double kd;
	double integral;
	double prev_error;
}PIDController;

PIDController pid_controllers[4] = {
	{8.0, 1.0, 0.2, 0.0, 0.0},  // Motor A 
	{8.0, 1.0, 0.2, 0.0, 0.0},  // Motor B
	{8.0, 1.0, 0.2, 0.0, 0.0},  // Motor C
	{8.0, 1.0, 0.2, 0.0, 0.0}   // Motor D
};

char cmd_buffer[32];
u8 cmd_index = 0;

/*=================== FUNCTIONS PROTOTYPES ================ */

void init_system(void);
void update_motors(void);
void set_motor_direction(u8 motor_index, Bool_t forward);
void set_motor_speed(u8 motor_index, u8 speed);
void process_command(void);
void report_velocities(void);
double calculate_pid(u8 motor_index, double target, double measured);
double absolute(double value);
void UpdateEncoder_Motor_a(void);
void UpdateEncoder_Motor_b(void);
void UpdateEncoder_Motor_c(void);
void UpdateEncoder_Motor_d(void);
void system_tick_handler(void);

// =================================================//

void UART_Diagnostic(void)
{
	// Send a diagnostic message
	UART_SendString("\r\nUART Diagnostic Starting...\r\n");
	
	// Wait a bit to ensure the message is sent
	for(volatile u16 i = 0; i < 10000; i++);
	
	// Direct test of RXC flag
	UART_SendString("Testing RXC flag directly: ");
	if(UCSRA & (1 << RXC))
	{
		UART_SendString("SET\r\n");
		u8 data = UDR; // Read the data to clear the flag
		UART_SendString("Received byte: ");
		UART_Send(data);
		UART_SendString("\r\n");
	}
	else
	{
		UART_SendString("NOT SET\r\n");
	}
	
	// Send instructions for testing
	UART_SendString("Send a character to test reception...\r\n");
	
	// Wait for any character
	_delay_ms(5000);
		if(UCSRA & (1 << RXC))
		{
			u8 data = UDR;
			UART_SendString("Character received: ");
			UART_Send(data);
			UART_SendString("\r\n");
			UART_SendString("UART reception working!\r\n");
			return;
		}
	

}
//=============================================================//

int main(void) {
	_delay_ms(2000);
	
	init_system();
	sei();  // Enable global interrupts
	u8 c = 0;
	char buffer[10];
	UART_SendString("ACSAR Robot Ready\r\n");
	
	
	//communication_timeout = 0;
	
	while (1) {

		if (UART_ReceivePeriodic(&c) != 0) {

			if (c == '\n' || c == '\r') {
				if (cmd_index > 0) {
					
					cmd_buffer[cmd_index] = '\0';
					process_command();
					cmd_index = 0;
				}
				
			}
			else if (cmd_index < sizeof(cmd_buffer) - 1)
			{
				cmd_buffer[cmd_index++] = c;
			}
		}


		// Software timer for motor control updates
		if ((system_ticks - last_control_update >= 50))
		{
			update_motors();
			last_control_update = system_ticks;
		}

		
	}
	
	
	return 0;
}
	
// System tick interrupt handler (1ms interval)
void update_system_tick(void) {
	static u8 ms_counter = 0;
	if (++ms_counter >= 4) {
		system_ticks++;
		ms_counter = 0;
	}
}

void init_system(void)
{
	DIO_Init();
	UART_Init(); // 9600 BAUD RATE
	
	// Setup Timer0 for both PWM (Motor C) and system tick (overflow interrupt)
	TIMER0_Init(TIMER0_FASTPWM_MODE, TIMER0_SCALER_8);
	TIMER0_OC0Mode(OC0_NON_INVERTING);
	TIMER0_OV_InterruptEnable();
	TIMER0_OV_SetCallBack(update_system_tick);
	
	Timer1_Init(TIMER1_FASTPWM_8_BIT_MODE, TIMER1_SCALER_8);
	Timer1_OCRA1Mode(OCRA_NON_INVERTING);
	Timer1_OCRB1Mode(OCRB_NON_INVERTING);
	
	TIMER2_Init(TIMER2_FASTPWM_MODE, TIMER2_SCALER_8);
	TIMER2_OC2Mode(OC2_NON_INVERTING);
	
	set_motor_direction(0, TRUE);
	set_motor_direction(1, TRUE);
	set_motor_direction(2, TRUE);
	set_motor_direction(3, TRUE);
	
	set_motor_speed(0,0); // MOTOR A
	set_motor_speed(1,0);// MOTOR B
	set_motor_speed(2,0); // MOTOR C
	set_motor_speed(3,0); // MOTOR D
	
	
	 EXI_TriggerEdge(EX_INT0,RISING_EDGE);  
	 EXI_TriggerEdge(EX_INT1, RISING_EDGE);
	 EXI_TriggerEdge(EX_INT2, RISING_EDGE);
	 
	 EXI_Enable(EX_INT0);
	 EXI_Enable(EX_INT1);
	 EXI_Enable(EX_INT2);
	 
	 EXI_SetCallBack(EX_INT0, UpdateEncoder_Motor_a);
	 EXI_SetCallBack(EX_INT1, UpdateEncoder_Motor_b);
	 EXI_SetCallBack(EX_INT2, UpdateEncoder_Motor_c);
	
}

void set_motor_direction(u8 motor_index, Bool_t forward)
{
	u8 dir1_pin, dir2_pin;
	
	switch(motor_index)
	{
		case 0: // MOTOR A
		dir1_pin = MOTOR_A_DIR1;
		dir2_pin = MOTOR_A_DIR2;
		break;
		case 1:  // Motor B
		dir1_pin = MOTOR_B_DIR1;
		dir2_pin = MOTOR_B_DIR2;
		break;
		case 2:  // Motor C
		dir1_pin = MOTOR_C_DIR1;
		dir2_pin = MOTOR_C_DIR2;
		break;
		case 3:  // Motor D
		dir1_pin = MOTOR_D_DIR1;
		dir2_pin = MOTOR_D_DIR2;
		break;
		default:
		return;
	}
	
	// Debug message
	//char buf[20];
	//itoa(motor_index, buf, 10);
	//UART_SendString("Setting motor ");
	//UART_SendString("\r\n");
	
	if (!forward)
	{
		Dio_WritePin(dir1_pin, HIGH);
		Dio_WritePin(dir2_pin, LOW);
	}
	else
	{
		Dio_WritePin(dir1_pin, LOW);
		Dio_WritePin(dir2_pin, HIGH);
	}
	
	motor_directions[motor_index] = forward;
}

void set_motor_speed(u8 motor_index, u8 speed)
{
	/*
	// Debug message
	char buf[10];
	itoa(motor_index, buf, 10);
	UART_SendString("Setting motor ");
	UART_SendString(buf);
	UART_SendString(" speed: ");
	itoa(speed, buf, 10);
	UART_SendString(buf);
	UART_SendString("\r\n");
	*/
	switch(motor_index)
	{
		case 0: // MOTOR A
		OCR1A = speed;
		break;
		case 1: // MOTOR B
		OCR1B = speed;
		break;
		case 2: // MOTOR C
		OCR0 = speed;
		break;
		case 3: // MOTOR D
		OCR2 = speed;
		break;
	}
}

double calculate_pid(u8 motor_index, double target, double measured)
{
	PIDController *pid = &pid_controllers[motor_index];
	
	double error = target - measured;
	
	// P TERM
	double p_term = pid->kp * error;
	
	// I TERM
	pid->integral += error * (CONTROL_INTERVAL_MS/1000.0);
	if (pid->integral > 5.0)  
	{
		pid->integral = 5.0;
	}
	if (pid->integral < -5.0)  
	{
		pid->integral = -5.0;
	}
	double i_term = pid->ki * pid->integral;
	
	// D TERM
	double d_term = pid->kd * (error - pid->prev_error) * (1000.0 / CONTROL_INTERVAL_MS);
	pid->prev_error = error;
	
	double output = p_term + i_term + d_term;
	
	// Clamp output
	if (output > 255.0)
	{
		output = 255.0;
	}
	if (output < 0.0)
	{
		output = 0.0;
	}
	
	return output;
}
void update_motors(void)
{
	
	for (u8 i = 0; i < 4; i++)
	{
		
		double new_velocity = (encoder_counts[i] * (1000.0 / CONTROL_INTERVAL_MS)) * RAD_PER_SEC;
		
		 if (encoder_counts[i] == 0 || motor_commands[i] == 0) 
		 {
			 measured_velocities[i] = 0.0;  // Force to zero when no movement
			 }
			  else
			 { 
			 double alpha = 0.7;
			 measured_velocities[i] = alpha * new_velocity + (1.0 - alpha) * measured_velocities[i];
		 }
		
	}
	
	for (u8 i = 0; i < 4; i++)
	{
		if (absolute(target_velocities[i]) > 0.01)
		{
			set_motor_direction(i, target_velocities[i] >= 0);
			motor_commands[i] = (u8)calculate_pid(i, absolute(target_velocities[i]), absolute(measured_velocities[i]));
			
			// Apply minimum PWM threshold to overcome static friction
			if (motor_commands[i] > 0 && motor_commands[i] < MIN_MOTOR_PWM)
			{
				motor_commands[i] = MIN_MOTOR_PWM;
			}
		}
		else
		{
			// Reset PID integral when stopped to prevent windup
			motor_commands[i] = 0;
			pid_controllers[i].integral = 0.0;
			pid_controllers[i].prev_error = 0.0;
		}
		
		set_motor_speed(i, motor_commands[i]);
	}
	
	// Reset encoder counts after velocity calculation
	for (u8 i = 0; i < 4; i++)
	{
		encoder_counts[i] = 0;
	}
	
	report_velocities();
}

void process_command(void)
{
	// Format: a[p/n]XX.XX,b[p/n]XX.XX,c[p/n]XX.XX,d[p/n]XX.XX,
	
	char *ptr = cmd_buffer;
	
	
	while (*ptr)
	{
		char motor_id = *ptr++;
		
		if (motor_id == '\0')
		{
			break;
		}
		if (motor_id >= 'a' && motor_id <= 'd')
		{
			u8 motor_index = motor_id - 'a'; // from a,b,c,d to 0,1,2,3
			
			char direction = *ptr++;
			if(direction == '\0')
			break;
			
			Bool_t is_positive = (direction == 'p');
			
			char value[10];
			u8 value_index = 0;
			
			while(*ptr && *ptr != ','  && value_index < 9)
			{
				value[value_index++] = *ptr++;
			}
			value[value_index] = '\0';
			/*
			// Debug parsing
			UART_SendString("Parsed motor ");
			UART_Send(motor_id);
			//UART_SendString(" dir ");
			//UART_Send(direction);
			UART_SendString(" value ");
			UART_SendString(value);
			UART_SendString("\r\n");
			*/
			double velocity = atof(value);
			if (!is_positive) {
				velocity = -velocity;
			}
			
			if (motor_index < 4) {
				target_velocities[motor_index] = velocity;
			}
			
			if (*ptr == ',')
			{ptr++;}
		}
		
	}
}

void report_velocities(void)
{
	char buffer[10];
	
	for (u8 i = 0; i < 4 ; i++)
	{
		UART_Send('a' + i);
		UART_Send(measured_velocities[i] >= 0 ? 'p' : 'n');
		if (absolute(measured_velocities[i]) < 10.0)
		{
			UART_Send('0');
		}
		double abs_vel = absolute(measured_velocities[i]);
		int decimal = (int)abs_vel;
		int fraction = (int) ((abs_vel-decimal) * 100);
		itoa(decimal,buffer,10);
		UART_SendString(buffer);
		UART_Send('.');
		
		if (fraction < 10)
		{
			UART_Send('0');
		}
		itoa(fraction,buffer,10);
		UART_SendString(buffer);
		
		UART_Send(',');
		
	}
	
	UART_SendString("\r\n");
}

double absolute(double value) {
	return (value < 0) ? -value : value;
}


void UpdateEncoder_Motor_a(void)
{
	Bool_t phase_B = Dio_ReadPin(ENCODER_A_PHASE_B);
	
	if (phase_B) {
		encoder_counts[0]--;  
		} else {
		encoder_counts[0]++;  
	}
}

void UpdateEncoder_Motor_b(void)
{
	 // Handle Motor B encoder
	 Bool_t phase_B_motor_b = Dio_ReadPin(ENCODER_B_PHASE_B);
	 
	 if (phase_B_motor_b) {
		 encoder_counts[1]--;  
		 } else {
		 encoder_counts[1]++;  
	 }
	 
	 // Handle Motor B encoder
	 Bool_t phase_B_motor_d = Dio_ReadPin(ENCODER_D_PHASE_B);
	 
	 if (!phase_B_motor_d) {
		 encoder_counts[3]--;  
		 } else {
		 encoder_counts[3]++;  
	 }
}

void UpdateEncoder_Motor_c(void)
{
	 Bool_t phase_B = Dio_ReadPin(ENCODER_C_PHASE_B);
	 
	 if (!phase_B) {
		 encoder_counts[2]--;  
		 } else {
		 encoder_counts[2]++;  
	 }
}


/*
void UpdateEncoder_Motor_a(void)
{
	Bool_t direction = (Dio_ReadPin(ENCODER_A_PHASE_A) == Dio_ReadPin(ENCODER_A_PHASE_B));
	
	if (!direction)
	{
		encoder_counts[0]++;  // Forward
	}
	else
	{
		encoder_counts[0]--;  // Reverse
	}
	
	motor_directions[0] = direction;
}
void UpdateEncoder_Motor_b(void)
{
	Bool_t direction_b = (Dio_ReadPin(ENCODER_B_PHASE_A) == Dio_ReadPin(ENCODER_B_PHASE_B));
	
	if (direction_b) {
		encoder_counts[1]++;
		} else {
		encoder_counts[1]--;
	}
	
	motor_directions[1] = direction_b;
	
	Bool_t direction_d = (Dio_ReadPin(ENCODER_D_PHASE_A) == Dio_ReadPin(ENCODER_D_PHASE_B));
	
	if (!direction_d) {
		encoder_counts[3]++;
		} else {
		encoder_counts[3]--;
	}
	motor_directions[3] = direction_d;
	
}

void UpdateEncoder_Motor_c(void) {
	
	Bool_t direction = (Dio_ReadPin(ENCODER_C_PHASE_A) == Dio_ReadPin(ENCODER_C_PHASE_B));
	
	if (direction) {
		encoder_counts[2]++;
		} else {
		encoder_counts[2]--;
	}
	
	motor_directions[2] = direction;
}

void UpdateEncoder_Motor_d(void)
{
	static Bool_t last_A = FALSE;
	
	Bool_t current_A = Dio_ReadPin(ENCODER_D_PHASE_A);
	Bool_t current_B = Dio_ReadPin(ENCODER_D_PHASE_B);
	
	// Only count on rising edge of Phase A
	if (current_A && !last_A) {
		if (current_A != current_B) {
			encoder_counts[3]++;  // Forward
			} else {
			encoder_counts[3]--;  // Reverse
		}
	}
	
	last_A = current_A;
}
*/