/*
 * Main.c
 *
 * Main file containing the main state machine.
 */
/* Standard Includes */


#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

#include "Library/Clock.h"
#include "Library/Bump.h"
#include "Library/Motor.h"
#include "Library/Encoder.h"
#include "Library/Button.h"

#include "Library/HAL_I2C.h"
#include "Library/HAL_OPT3001.h"





// Old and custom variables

#define TURN_TARGET_TICKS 77            // Slight, approximately 45-degree turn
#define TURN_TARGET_TICKS90 164         // 90 degree turn
#define DRIVE_TARGET_TICKS 470          // This is an unnecessary variable; replaced with the variable squareDistance that changes below.
#define BACKWARD_DRIVE_TARGET_TICKS 75  // How far backward should the robot drive? No longer used.


void Initialize_System();

uint8_t bump_data;
uint8_t bump_data0; // Slight turn made - maybe adjust to be even lighter
uint8_t bump_data1; // Slight turn made
uint8_t bump_data2; // 90 degree turn made
uint8_t bump_data3; // 90 degree turn made
uint8_t bump_data4; // Slight turn made
uint8_t bump_data5; // Slight turn made - maybe adjust to be even lighter

int tick=0;

int mytime[20];
int i=0;
int mouse;                  // Disabled.

float robotSpeed = 0.2;     // Speed of the robot can be updated here; changing this affects distance values and tick rates.




// Custom queue move system (robot does not drive forward infinitely, but must be instructed how to drive and runs each step one at a time)
// Before each maze, decide which distances need to be traveled and which moves need to be made and fill in the numbers for each move.
// To end the robot's instructions and stop (such as on the final square of the maze), set all final moves to the value of 0.

// Moves List:
// 0    Stop and wait
// 1    Drive forward 1 square
// 2    Drive forward 2 squares
// 3    Drive forward 3 squares
// 4    Drive forward 4 squares
// 5    Drive forward a half a square
// 10   Turn right 90 degrees
// 11   Turn left 90 degrees
// 20   Turn right 45 degrees
// 21   Turn left 45 degrees


bool canContinue = false;   // Changes to true when the S1 button is pressed. Allows the robot to continue to the next step; while running a step, this is false.
int currentMove = 1;        // Start at move 1. After each move is completed, the move advances 1 to the next step and canContinue is set to true to load the next step.
int whichTurn = 1;          // Changes depending on the move. 1 makes left turns, 2 makes right turns.

int moveToMake = 0;         // Depending on the current move (see each move variable below), set this to a value. The value corresponds with the moves list above.

int squareDistance = 452;   // This changes depending on the number of squares. One square is 452 at a robotSpeed of 0.2, two is 904, etc.

// Instructions for the current maze are filled in here. Before each maze, replace each move's number with a value from the moves list.
// The robot will perform each move step-by-step in order from 1 to 20, skipping moves or stopping at the value of 0.

int move1 = 0;
int move2 = 0;
int move3 = 0;
int move4 = 0;
int move5 = 0;
int move6 = 0;
int move7 = 0;
int move8 = 0;
int move9 = 0;
int move10 = 0;
int move11 = 0;
int move12 = 0;
int move13 = 0;
int move14 = 0;
int move15 = 0;
int move16 = 0;
int move17 = 0;
int move18 = 0;
int move19 = 0;     // More moves can be technically added, but that is not required. Also, yes, an array is more efficient here but I don't know how to use them in C.






/* Variable for storing lux value returned from OPT3001 */
float lux;  // This is not used anymore.

typedef enum
{
    START = 0,
    WAIT,
    SETUP_DRIVEFORWARD,
    DRIVEFORWARD,
    STOP_DRIVE1,
    SETUP_TURNLEFT,
    TURNLEFT,
    SETUP_TURN1,
    TURN1,
    SETUP_TURN90,
    TURN90,
    SETUP_TURN45,
    TURN45,
    STOP_TURN1,
    SETUP_BUMPEDOUTSIDE,
    BUMPEDOUTSIDE,
    SETUP_BUMPEDOUTSIDE2,
    BUMPEDOUTSIDE2,
    SETUP_BUMPEDINSIDE,
    BUMPEDINSIDE,
    SETUP_BUMPEDMIDDLE,
    BUMPEDMIDDLE,
    SETUP_BACKWARDS,
    BACKWARDS,
    SETUP_BACKWARDS2,
    BACKWARDS2,
    SETUP_BACKWARDS3,
    BACKWARDS3,
    ALL_DONE
} my_state_t;

my_state_t state = START;

int main(void)

{

    bool left_done, right_done;

    int left_encoder_zero_pos, right_encoder_zero_pos;

    Initialize_System();

    set_left_motor_pwm(0);
    set_right_motor_pwm(0);

    while (1)
    {


        // Read Bump data into a byte
        // Lower six bits correspond to the six bump sensors
        // put into individual variables so we can view it in GC
        bump_data = Bump_Read();
        bump_data0 = BUMP_SWITCH(bump_data,0);
        bump_data1 = BUMP_SWITCH(bump_data,1);
        bump_data2 = BUMP_SWITCH(bump_data,2);
        bump_data3 = BUMP_SWITCH(bump_data,3);
        bump_data4 = BUMP_SWITCH(bump_data,4);
        bump_data5 = BUMP_SWITCH(bump_data,5);

        /* Obtain lux value from the OPT3001 light sensor */
        lux = OPT3001_getLux(); // THIS IS A BRAND NEW LINE GETS LIGHT FROM SENSOR

        // Emergency stop switch S2
        // Switch to state "STOP" if pressed
        if (button_S2_pressed())
        {
            currentMove = 1;        // Reset the robot to the first move and prevent it from loading each step; resets the robot for a re-run of its current instructions.
            canContinue = false;    // Prevents the robot from immediately continuing and restarting after engaging the emergency stop.
            state = ALL_DONE;
        }

        //-----------------------------------
        //        Main State Machine
        //-----------------------------------
        switch (state) 
        {

        case START:
            state = WAIT;
        break;

        case WAIT:
            if (button_S1_pressed())
            {
                canContinue = true;     // Start loading the robot's instructions for each move in order.
                currentMove = 1;        // Start from move 1.
            }


            if (canContinue == true) // If the robot is not currently making a move or has finished its last step and can move on...
            {
                canContinue = false; // Prevent the robot from immediately skipping a step during the next run of the while loop; can continue only when the current move
                                     // is completed.

                // Get the current move to make depending on which step of the instructions we are on.
                if (currentMove == 1) // We have not begun calculating moves. Start the first move.
                {
                    moveToMake = move1;
                }
                else if (currentMove == 2)
                {
                    moveToMake = move2;
                }
                else if (currentMove == 3)
                {
                    moveToMake = move3;
                }
                else if (currentMove == 4)
                {
                    moveToMake = move4;
                }
                else if (currentMove == 5)
                {
                    moveToMake = move5;
                }
                else if (currentMove == 6)
                {
                    moveToMake = move6;
                }
                else if (currentMove == 7)
                {
                    moveToMake = move7;
                }
                else if (currentMove == 8)
                {
                    moveToMake = move8;
                }
                else if (currentMove == 9)
                {
                    moveToMake = move9;
                }
                else if (currentMove == 10)
                {
                    moveToMake = move10;
                }
                else if (currentMove == 11)
                {
                    moveToMake = move11;
                }
                else if (currentMove == 12)
                {
                    moveToMake = move12;
                }
                else if (currentMove == 13)
                {
                    moveToMake = move13;
                }
                else if (currentMove == 14)
                {
                    moveToMake = move14;
                }
                else if (currentMove == 15)
                {
                    moveToMake = move15;
                }
                else if (currentMove == 16)
                {
                    moveToMake = move16;
                }
                else if (currentMove == 17)
                {
                    moveToMake = move17;
                }
                else if (currentMove == 18)
                {
                    moveToMake = move18;
                }
                else if (currentMove == 19)
                {
                    moveToMake = move19;
                }
                else if (currentMove == 20)
                {
                    moveToMake = 0;
                }

                currentMove++; // Advance the current move for the next time around, when we're ready later to continue to the next step.

                // Based on the moveToMake, load the next move into the robot and run it. When the move finishes, the movement's code will ONLY set canContinue to true.

                if (moveToMake == 0) // Stop the robot for whatever reason (more than likely, the end has been reached; do nothing).
                {
                    left_done = true;
                    right_done = true;
                    set_left_motor_pwm(0);
                    set_right_motor_pwm(0);

                    if (currentMove == 21) // currentMove, when set to 21, is impossible and means the robot is completely finished with its steps. Stop and reset.
                    {
                        currentMove = 1;
                        canContinue = false;
                        state = ALL_DONE;
                    }
                    else // The robot's not done yet, so just let it continue to the next step (we already increased the currentMove).
                    {
                        canContinue = true; // If there are no more steps and the rest are set to 0, this will loop until currentMove reaches 21 and resets the robot.
                    }
                }
                else if (moveToMake == 1) // Instruction to complete: Drive forward 1 square
                {
                    squareDistance = 452;
                    state = SETUP_DRIVEFORWARD;
                }
                else if (moveToMake == 2) // Instruction to complete: Drive forward 2 squares
                {
                    squareDistance = 904;
                    state = SETUP_DRIVEFORWARD;
                }
                else if (moveToMake == 3) // Instruction to complete: Drive forward 3 squares
                {
                    squareDistance = 1356;
                    state = SETUP_DRIVEFORWARD;
                }
                else if (moveToMake == 4) // Instruction to complete: Drive forward 4 squares
                {
                    squareDistance = 1808;
                    state = SETUP_DRIVEFORWARD;
                }
                else if (moveToMake == 5) // Instruction to complete: Drive forward a half of a square
                {
                    squareDistance = 276;
                    state = SETUP_DRIVEFORWARD;
                }
                else if (moveToMake == 10) // Instruction to complete: Turn right 90 degrees
                {
                    whichTurn = 2; // Right turn
                    state = SETUP_TURN90; // 90 degrees
                }
                else if (moveToMake == 11) // Instruction to complete: Turn left 90 degrees
                {
                    whichTurn = 1; // Left turn
                    state = SETUP_TURN90; // 90 degrees
                }
                else if (moveToMake == 20) // Instruction to complete: Turn right 45 degrees
                {
                    whichTurn = 2; // Right turn
                    state = SETUP_TURN45; // 45 degrees
                }
                else if (moveToMake == 21) // Instruction to complete: Turn left 45 degrees
                {
                    whichTurn = 1; // Left turn
                    state = SETUP_TURN45; // 45 degrees
                }

                // Removed code for bump sensors and light sensor; unnecessary.
            }
        break;



        // Prepare the motors to drive in the forward direction.
        case SETUP_DRIVEFORWARD:
            left_encoder_zero_pos = get_left_motor_count();
            right_encoder_zero_pos = get_right_motor_count();

            set_left_motor_direction(true);
            set_right_motor_direction(true);

            left_done = false;
            right_done = false;

            state = DRIVEFORWARD;
        break;

        // Drive forward a set amount of squares. This does not continuously drive forward forever; only drives until squareDistance is reached (set in the moves above).
        case DRIVEFORWARD:
            canContinue = false; // Just in case, prevent the robot from skipping a step and finishing the move early.

            if (!left_done)
            {
                set_left_motor_pwm(robotSpeed);
                left_done = (get_left_motor_count() - left_encoder_zero_pos) > squareDistance ; // This variable will change based on the squares to travel.
            }
            else
            {
                set_left_motor_pwm(0); // Stop the left wheel, it's done.
            }

            if(!right_done)
            {
                set_right_motor_pwm(robotSpeed + 0.01); // Accounts for drifting of the wheel; one motor was more powerful than the other, it seemed.
                right_done = (get_right_motor_count() - right_encoder_zero_pos) > squareDistance; // This variable will change based on the squares to travel.
            }
            else
            {
                set_right_motor_pwm(0); // Stop the right wheel, it's done.
            }

            // The correct amount of squares has been traveled. Go to the next move and enter the wait state so that the while loop will advance to the next step.
            if (left_done && right_done)
            {
                set_left_motor_pwm(0);          // Stop all motors
                set_right_motor_pwm(0);
                left_done = false;              // Reset values.
                right_done = false;
                canContinue = true;             // Make it so the robot can continue to the next step.
                state = WAIT;                   // Change to the WAIT state where the while loop will run and, because canContinue is true, advance to the next step.
            }
        break;


        // Set up 90 degree left or right turns depending on whichTurn's value.
        case SETUP_TURN90:
            if (whichTurn == 1) // LEFT turns only.
            {
                left_encoder_zero_pos = get_left_motor_count();
                right_encoder_zero_pos = get_right_motor_count();

                set_left_motor_direction(false);
                set_right_motor_direction(true);

                left_done = false;
                right_done = false;

                // Start the motors here so we only have to start them once
                set_left_motor_pwm(robotSpeed);
                set_right_motor_pwm(robotSpeed);

                state = TURN90;
            }
            else if (whichTurn == 2) // RIGHT turns only.
            {
                left_encoder_zero_pos = get_left_motor_count();
                right_encoder_zero_pos = get_right_motor_count();

                set_left_motor_direction(true);
                set_right_motor_direction(false);

                left_done = false;
                right_done = false;

                // Start the motors here so we only have to start them once
                set_left_motor_pwm(robotSpeed);
                set_right_motor_pwm(robotSpeed);

                state = TURN90; // Complete the turn.
            }
                break;

        // Complete the 90 degree left or right turn, depending on whichTurn's value.
        case TURN90:
            if (whichTurn == 1) // LEFT turns only.
            {
                left_done = (get_left_motor_count() - left_encoder_zero_pos) < -TURN_TARGET_TICKS90;
                right_done = (get_right_motor_count() - right_encoder_zero_pos) > TURN_TARGET_TICKS90;
                if (left_done)
                {
                    set_left_motor_pwm(0);
                }
                if (right_done)
                {
                    set_right_motor_pwm(0);
                }
                if (left_done && right_done) // The turn is finished, make it so the robot can continue to the next step and return to the WAIT state so it runs the code.
                {
                    canContinue = true;
                    state = WAIT;
                }
            }
            else if (whichTurn == 2) // RIGHT turns only.
            {
                left_done = (get_left_motor_count() - left_encoder_zero_pos) > TURN_TARGET_TICKS90;
                right_done = (get_right_motor_count() - right_encoder_zero_pos) < -TURN_TARGET_TICKS90;
                if (left_done)
                {
                    set_left_motor_pwm(0);
                }
                if (right_done)
                {
                    set_right_motor_pwm(0);
                }
                if (left_done && right_done)
                {
                    canContinue = true; // The turn is finished, make it so the robot can continue to the next step and return to the WAIT state so it runs the code.
                    state = WAIT;
                }
            }

        break;



        // Set up 45 degree left or right turns depending on whichTurn's value.
        case SETUP_TURN45:
            if (whichTurn == 1) // LEFT turns only.
            {
                left_encoder_zero_pos = get_left_motor_count();
                right_encoder_zero_pos = get_right_motor_count();

                set_left_motor_direction(false);
                set_right_motor_direction(true);

                left_done = false;
                right_done = false;

                // Start the motors here so we only have to start them once
                set_left_motor_pwm(robotSpeed);
                set_right_motor_pwm(robotSpeed);

                state = TURN45;
            }
            else if (whichTurn == 2) // RIGHT turns only.
            {
                left_encoder_zero_pos = get_left_motor_count();
                right_encoder_zero_pos = get_right_motor_count();

                set_left_motor_direction(true);
                set_right_motor_direction(false);

                left_done = false;
                right_done = false;

                // Start the motors here so we only have to start them once
                set_left_motor_pwm(robotSpeed);
                set_right_motor_pwm(robotSpeed);

                state = TURN45;
            }
            break;

        // Complete the 45 degree left or right turn, depending on whichTurn's value.
        case TURN45:
            if (whichTurn == 1) // LEFT turns only.
            {
                left_done = (get_left_motor_count() - left_encoder_zero_pos) < -TURN_TARGET_TICKS;
                right_done = (get_right_motor_count() - right_encoder_zero_pos) > TURN_TARGET_TICKS;
                if (left_done)
                {
                    set_left_motor_pwm(0);
                }
                if (right_done)
                {
                    set_right_motor_pwm(0);
                }
                if (left_done && right_done)
                {
                    canContinue = true; // The turn is finished, make it so the robot can continue to the next step and return to the WAIT state so it runs the code.
                    state = WAIT;
                }
            }
            else if (whichTurn == 2) // RIGHT turns only.
            {
                left_done = (get_left_motor_count() - left_encoder_zero_pos) > TURN_TARGET_TICKS;
                right_done = (get_right_motor_count() - right_encoder_zero_pos) < -TURN_TARGET_TICKS;
                if (left_done)
                {
                    set_left_motor_pwm(0);
                }
                if (right_done)
                {
                    set_right_motor_pwm(0);
                }
                if (left_done && right_done)
                {
                    canContinue = true; // The turn is finished, make it so the robot can continue to the next step and return to the WAIT state so it runs the code.
                    state = WAIT;
                }
            }

            break;













        // All of the code below is completely unused as it was originally part of the bump sensor code that was removed.

        case SETUP_TURN1:
                    left_encoder_zero_pos = get_left_motor_count();
                    right_encoder_zero_pos = get_right_motor_count();

                    set_left_motor_direction(true);
                    set_right_motor_direction(false);

                    left_done = false;
                    right_done = false;

                    // Start the motors here so we only have to start them once
                    set_left_motor_pwm(robotSpeed);
                    set_right_motor_pwm(robotSpeed);

                    state = TURN1;
                break;


                case TURN1:
                    left_done = (get_left_motor_count() - left_encoder_zero_pos) > TURN_TARGET_TICKS;
                    right_done = (get_right_motor_count() - right_encoder_zero_pos) < -TURN_TARGET_TICKS;

                    if (left_done)
                    {
                        set_left_motor_pwm(0);
                    }

                    if (right_done)
                    {
                        set_right_motor_pwm(0);
                    }

                    if (left_done && right_done) {
                        canContinue = true;
                        state = WAIT;
                    }
                break;

        case SETUP_TURNLEFT:
                    left_encoder_zero_pos = get_left_motor_count();
                    right_encoder_zero_pos = get_right_motor_count();

                    set_left_motor_direction(false);
                    set_right_motor_direction(true);

                    // Start the motors here so we only have to start them once
                    set_left_motor_pwm(robotSpeed);
                    set_right_motor_pwm(robotSpeed);

                    left_done = false;
                    right_done = false;

                    state = TURNLEFT;
                 break;


                case TURNLEFT:
                    left_done = (left_encoder_zero_pos - get_left_motor_count()) > TURN_TARGET_TICKS;
                    right_done = (right_encoder_zero_pos - get_right_motor_count()) < -TURN_TARGET_TICKS;

                    if (left_done)
                    {
                        set_left_motor_pwm(0);
                    }

                    if(right_done)
                    {
                        set_right_motor_pwm(0);
                    }

                    if (left_done && right_done) {
                        canContinue = true;
                        state = WAIT;
                    }
        break;

        case ALL_DONE:
            state = WAIT;
        break;

        case SETUP_BUMPEDOUTSIDE:
            state  = WAIT;
        break;

        case BUMPEDOUTSIDE:
            state = SETUP_TURNLEFT;
        break;

        case SETUP_BUMPEDOUTSIDE2:
             state  = WAIT;
        break;

        case BUMPEDOUTSIDE2:
             state = SETUP_TURN1;
        break;

        case SETUP_BUMPEDINSIDE:
            state  = WAIT;
        break;

        case BUMPEDINSIDE:
            state  = SETUP_BACKWARDS;
        break;

        case SETUP_BUMPEDMIDDLE:
             state = WAIT;
        break;

        case BUMPEDMIDDLE:
             state = WAIT;
        break;

        case SETUP_BACKWARDS:

                 left_encoder_zero_pos = get_left_motor_count();
                 right_encoder_zero_pos = get_right_motor_count();

                 set_left_motor_direction(false);
                 set_right_motor_direction(false);

                 // Start the motors here so we only have to start them once
                 set_left_motor_pwm(robotSpeed);
                 set_right_motor_pwm(robotSpeed);

                 left_done = false;
                 right_done = false;

                 state = BACKWARDS;
        break;

        case BACKWARDS:

            left_done = (left_encoder_zero_pos - get_left_motor_count()) > BACKWARD_DRIVE_TARGET_TICKS;
            right_done = (right_encoder_zero_pos - get_right_motor_count()) > BACKWARD_DRIVE_TARGET_TICKS;

            if (left_done)
            {
                set_left_motor_pwm(0);
            }

            if (right_done)
            {
                set_right_motor_pwm(0);
            }

            if (left_done && right_done) {
                state = SETUP_TURNLEFT;
            }
        break;

        case SETUP_BACKWARDS2:

                    left_encoder_zero_pos = get_left_motor_count();
                    right_encoder_zero_pos = get_right_motor_count();

                    set_left_motor_direction(false);
                    set_right_motor_direction(false);

                    // Start the motors here so we only have to start them once
                    set_left_motor_pwm(robotSpeed);
                    set_right_motor_pwm(robotSpeed);

                    left_done = false;
                    right_done = false;

                    state = BACKWARDS2;
                break;

                case BACKWARDS2:

                    left_done = (left_encoder_zero_pos - get_left_motor_count()) > BACKWARD_DRIVE_TARGET_TICKS;
                    right_done = (right_encoder_zero_pos - get_right_motor_count()) > BACKWARD_DRIVE_TARGET_TICKS;

                    if (left_done)
                    {
                        set_left_motor_pwm(0);
                    }

                    if (right_done)
                    {
                        set_right_motor_pwm(0);
                    }

                    if (left_done && right_done)
                    {
                        state = SETUP_TURN1;
                    }
                    break;

                case SETUP_BACKWARDS3:

                    left_encoder_zero_pos = get_left_motor_count();
                    right_encoder_zero_pos = get_right_motor_count();

                    set_left_motor_direction(false);
                    set_right_motor_direction(false);

                    // Start the motors here so we only have to start them once
                    set_left_motor_pwm(robotSpeed);
                    set_right_motor_pwm(robotSpeed);

                    left_done = false;
                    right_done = false;

                    state = BACKWARDS3;
                break;

                case BACKWARDS3:

                    left_done = (left_encoder_zero_pos - get_left_motor_count()) > BACKWARD_DRIVE_TARGET_TICKS;
                    right_done = (right_encoder_zero_pos - get_right_motor_count()) > BACKWARD_DRIVE_TARGET_TICKS;

                    if (left_done)
                    {
                        set_left_motor_pwm(0);
                    }

                    if (right_done)
                    {
                        set_right_motor_pwm(0);
                    }

                    if (left_done && right_done)
                    {
                        state = SETUP_TURN90;
                    }
                    break;
        } // end of case

        Clock_Delay1ms(10);
    }
}


void Initialize_System()
{
    /*
     * Initialize main clock
     *
     * SMCLK = 12Mhz
     */
    Clock_Init48MHz();

    /* Halting the Watchdog */
    MAP_WDT_A_holdTimer();

    /* Configuring GPIO LED1 as an output */
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);

    /* Configure GPIO LED Red, LED Green, LED Blue */
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN1);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN2);

    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);

    Bump_Init();

    motor_init();

    encoder_init();

    button_init();

    MAP_SysCtl_enableSRAMBankRetention(SYSCTL_SRAM_BANK1);

    /*
     * Configuring SysTick to trigger at .001 sec (MCLK is 48Mhz)
     */
    MAP_SysTick_enableModule();
    MAP_SysTick_setPeriod(48000);
    MAP_SysTick_enableInterrupt();

    MAP_Interrupt_enableMaster();

    /* Initialize I2C communication */      // ALL OF THIS BELOW IS FOR LIGHT SENSOR, NEW
    Init_I2C_GPIO();
    I2C_init();

    /* Initialize OPT3001 digital ambient light sensor */
    OPT3001_init();

    //__delay_cycles(100000);
}


/*
 * Handle the SysTick Interrupt.  Currently interrupting at 1/10 second.
 *
 * Increment the tick counter "tick"
 * Blink the red led
 */
void SysTick_Handler(void)
{
    tick++;
    // if ((tick%1000)==0) MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);        // Toggle RED LED each time through loop
}



