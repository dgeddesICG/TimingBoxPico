/*
Pico Pianola V8
This version will hopefully acheive the lofty goal of fully communicating with the spim-gui.
Some commands listed in the command protocol.rtf file differ from the implementation in xcode project (TimingBoxXMOS.cpp)
Mainly just the fact that RUN_Pianola returns the time we ran at as well as getFirmware being mapped to the command 0xFD instead of 0xD
This is also the debut of my experimental custom clock function using a state machine. Oooh Exciting. 

Main changes frm V7 will be actually reading and writing data in the correct order
In the Spim-gui commands are received in the format of a byte array where:
The zeroth entry is the command byte
The next nth entries are arranged in a pattern of (1st byte) MSB to LSB (nth byte)
Similarily data is written back in the order of MSB being the zeroth char (first thing we write out)
and the LSB being the nth char (last thing we write out)

This revelation should've been figured out much earlier by the fact I kept having to reverse input and outputs but hey ho
we got there in the end
*/

#include <stdio.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/pio.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"
#include <triggerControl.pio.h>
#include <pianola24bitClock.pio.h>

// ===================================================================================
// Defines for UART Control and GPIOs
// ===================================================================================

#define picoCPUclkRateMHz 125



// UART defines
// bog standard 8n1 @ 115200
#define UART_ID uart0
#define DATA_BITS 8
#define STOP_BITS 1
#define BAUDRATE 115200
#define PARITY UART_PARITY_NONE //no parity bits
#define UART_INTERRUPT UART0_IRQ


#define UART_TX_PIN 0
#define UART_RX_PIN 1
#define STATUSLED_PIN 25 // on board LED
// defining all data pins but we only need to refer to the 1st data pin
#define DATA1_PIN 18
#define DATA2_PIN 19
#define DATA3_PIN 20
#define DATA4_PIN 21
#define NUM_DATAPIN 4
#define IRQ_PIANOLACORE 46


// ===============================================================================
// Set up some pianola attributes
//================================================================================

// for now the comand buffers are 32ytes wide and 1 command deep
#define RX_BUFFER_SIZE_BYTES 32
#define TX_BUFFER_SIZE_BYTES 32

unsigned char responseBuffer[TX_BUFFER_SIZE_BYTES];
unsigned char commandBuffer[RX_BUFFER_SIZE_BYTES];

queue_t responseQ;
queue_t commandQ;
queue_t dumpDebugQ;

unsigned int numBytesToReceive = 0;
unsigned int numBytesToTransmit = 0;


#define pianolaTimeBaseUS 2.56 // set to match the xmos board

enum STATUS
{
    INIT,
    READY,
    RUNNING,
    REPEATING,
    STOPPED,
    RESET,
    PANIC
};


uint32_t pianola[256];
uint32_t pianolaCameraClock[8];
int pianolaFinalPos = 0;
uint32_t pianolaPos = 0;
uint32_t pianolaNumInstr = 1;
uint32_t pianolaTime;
signed int delT;
const uint32_t pianolaFireTimeThresholdUS = 1;
int pianolaFireTimeThresholdTicks = pianolaFireTimeThresholdUS / pianolaTimeBaseUS;
uint8_t pianolaRepeatAddr;
volatile uint32_t pianolaFireTime = 0;


const float pinDriverClockDivisorFLOAT = picoCPUclkRateMHz * pianolaTimeBaseUS;
volatile uint32_t pinDriverClockDivisorFP = (uint32_t)((pinDriverClockDivisorFLOAT) * (1<<8));

uint32_t pianolaClockDivisorFP = (uint32_t)((pinDriverClockDivisorFLOAT / 10) * (1<<8));

// it takes 4 cycles to initialise the each pianola instruction so we'll subtract it later
const int pianolaPinTriggerOffset = 4;

enum STATUS pianolaStatus = INIT;

// Some values to keep the current version of spim-interface happy

// firmware version for type 3 box
int pianolaFirmwareVersion = 6;
int pianolaFirmwareEarliestVersion = 6;

// pin sources
// relating to pin source
uint8_t pianolaPinIdx;
uint8_t pianolaPinBitMap;
uint8_t pianolaPinInvert;


// piv params
uint8_t pivIdx;
uint32_t pivIntervalLaserPulses;
uint32_t pivDuratonPulsePair;
uint32_t pivIntervalPulseToExp;
uint32_t pivCameraExposure;
uint8_t pivCameraMonitorIdx;
// ===============================================================================
// Set some values for setting the pio block
//================================================================================
PIO pio = pio0;
uint smPinDriver = 0;
uint offsetPinDriver;


uint smPianolaClock = 1;
uint offsetPianolaClock;

uint lsbPin = DATA1_PIN;




// ===============================================================================
// Command Set
//================================================================================

enum CMD_SET 
{
    SET_Pianola             = 0x1,
    SET_PianolaFinalPos     = 0x2,
    SET_PianolaRepeatFrom   = 0x3,
    SET_PianolaRepeating    = 0x4, 
    RUN_Pianola             = 0x5,
    SET_PianolaFireTime     = 0x6,
    IRQ_StopAndReset        = 0x7,
    GET_CurrentPianolaTime  = 0x8,
    SET_PinSource           = 0x9,
    GET_PinSource           = 0xA,
    SET_CameraClk           = 0xB,
    SET_PIVParams           = 0xC,
    SET_ClockDivisor        = 0xAB,
    GET_FirmwareVersion     = 0xFD, // this is technically different from whats in the communicatons protocol.rtf (0xD) but the TimingBoxXMOS.cpp file wants 0xFD
    IRQ_DumpLog             = 0xFE,
    IRQ_HARDRESET           = 0xFF,


};

struct CMD_CONFIG 
{
    int numBytesReceive;
    int numBytesResponse;
};

struct CMD_CONFIG commandConfig = {0};
// ===============================================================================
// Helper Functions That need to be decalred before the parsers
//================================================================================

uint32_t getPianolaTime()
{
    uint32_t t = 0;
    pio_sm_put_blocking(pio, smPianolaClock, 0xB);
    t = 0xFFFFFF - pio_sm_get_blocking(pio, smPianolaClock);
    pio_sm_drain_tx_fifo(pio, smPianolaClock);
    return t;
}



bool isTimeInFuture(uint32_t currentTime, uint32_t proposedTime)
{
    // sanitise inputs to 3 bytes
    currentTime = currentTime & 0xFFFFFF;
    proposedTime = proposedTime & 0xFFFFFF;

    // determine the range for a time being in the past the same way as the xmos board does
    signed int timeInPastRange[2] = { currentTime - (1<<23) , currentTime - 1};

    // check if time is in the past range
    bool timeIsInPast = (proposedTime >= timeInPastRange[0] & (proposedTime <= timeInPastRange[1]));
    // invert for checking if time is in the future
    bool timeIsInFuture = !timeIsInPast;
    return timeIsInFuture;
}


// ===============================================================================
// Parsers for Handling Commands and Deciding Input / Output Lengths
//================================================================================



void commandDataConfig(enum CMD_SET command)
{
    switch(command)
    {
        case SET_Pianola:
            commandConfig.numBytesReceive = 5;
            commandConfig.numBytesResponse = 0; 
            break;

        case SET_PianolaFinalPos:
            commandConfig.numBytesReceive = 1;
            commandConfig.numBytesResponse = 0; 
            break;

        case SET_PianolaRepeatFrom:
            commandConfig.numBytesReceive = 1;
            commandConfig.numBytesResponse = 0; 
            break;

        case SET_PianolaRepeating:
            commandConfig.numBytesReceive = 1;
            commandConfig.numBytesResponse = 0; 
            break;

        case RUN_Pianola:
            commandConfig.numBytesReceive = 0;
            commandConfig.numBytesResponse = 3;
            break;

        case SET_PianolaFireTime:
            commandConfig.numBytesReceive = 3;
            commandConfig.numBytesResponse = 4;
            break;

        case IRQ_StopAndReset:
            commandConfig.numBytesReceive = 0;
            commandConfig.numBytesResponse = 0; 
            break;

        case GET_CurrentPianolaTime:
            commandConfig.numBytesReceive = 0;
            commandConfig.numBytesResponse = 3;
            break;

        case SET_PinSource:
            commandConfig.numBytesReceive = 0;
            commandConfig.numBytesResponse = 3; 
            break;

        case GET_PinSource:
            commandConfig.numBytesReceive = 4;
            commandConfig.numBytesResponse = 0; 
            break;

        case SET_CameraClk:
            commandConfig.numBytesReceive = 2;
            commandConfig.numBytesResponse = 0; 
            break;

        case SET_PIVParams:
            commandConfig.numBytesReceive = 18;
            commandConfig.numBytesResponse = 0; 
            break;

        case GET_FirmwareVersion:
            commandConfig.numBytesReceive = 0;
            commandConfig.numBytesResponse = 2;
            break;


        case SET_ClockDivisor:
            commandConfig.numBytesReceive = 3;
            commandConfig.numBytesResponse = 0;
            break;

        case IRQ_DumpLog:
            commandConfig.numBytesReceive = 0;
            commandConfig.numBytesResponse = 12;
            break;

        case IRQ_HARDRESET:
            commandConfig.numBytesReceive = 0;
            commandConfig.numBytesResponse = 0;
            break;

        default:
            break;

    };

}


// the big yin
void CommandParser(unsigned char dataRecBuffer[RX_BUFFER_SIZE_BYTES], unsigned char dataRespBuffer[TX_BUFFER_SIZE_BYTES])
{
    enum CMD_SET opcode = dataRecBuffer[0] & 0xFF;


    switch(opcode)
    {
        case SET_Pianola:
            uint32_t pianolaSetAddr =  dataRecBuffer[1];
            uint32_t pianolaPinMask = dataRecBuffer[2];
            uint32_t pianolaDuration = dataRecBuffer[3] << 16 |dataRecBuffer[4] << 8| dataRecBuffer[5] & 0xFF;
            uint32_t pianolaInstr = pianolaDuration << 8 | pianolaPinMask & 0xFF;
            pianola[pianolaSetAddr] = pianolaInstr;
            // if (pianolaSetAddr <= pianolaFinalPos)
            // {
            //     pianola[pianolaSetAddr] = pianolaInstr;
            // }
            break;

        case SET_PianolaFinalPos:
            pianolaFinalPos = dataRecBuffer[1] & 0xFF;
            pianolaNumInstr = pianolaNumInstr + 1;
            break;

        case SET_PianolaRepeatFrom:
            uint8_t tempAddr = dataRecBuffer[1] & 0xFF;
            pianolaRepeatAddr = tempAddr < pianolaFinalPos ? tempAddr : pianolaFinalPos - 1;
            break;

        case SET_PianolaRepeating:
            pianolaStatus = (dataRecBuffer[1] == 1) ? REPEATING : READY;
            pianolaRepeatAddr = 0;
            break;

        case RUN_Pianola:
            pianolaStatus = RUNNING;
            pianolaPos = 0;
            dataRespBuffer[0] = (pianolaTime >> 16) & 0xFF;
            dataRespBuffer[1] = (pianolaTime >> 8) & 0xFF;
            dataRespBuffer[2] = pianolaTime & 0xFF;
            break;

        case SET_PianolaFireTime:
            uint32_t proposedFireTime = (dataRecBuffer[1] << 16) | (dataRecBuffer[2] << 8)  | (dataRecBuffer[3] & 0xFF);
            pianolaTime = getPianolaTime();
            int timeInFuture = isTimeInFuture(pianolaTime, proposedFireTime);

            // queue_try_add(&dumpDebugQ, &tmpDebugEntry);
            if (timeInFuture)
            {
                pianolaFireTime = proposedFireTime;
            }    
            dataRespBuffer[0] = (unsigned char) (timeInFuture);
            dataRespBuffer[1] = (pianolaTime >> 16) & 0xFF;
            dataRespBuffer[2] = (pianolaTime >> 8) & 0xFF;
            dataRespBuffer[3] = pianolaTime & 0xFF;
            break;

        case IRQ_StopAndReset:
            pianolaStatus = READY;
            pianolaPos = pianolaRepeatAddr;  
            break;

        case GET_CurrentPianolaTime:
            pianolaTime = getPianolaTime();
            dataRespBuffer[0] = (pianolaTime >> 16) & 0xFF;
            dataRespBuffer[1] = (pianolaTime >> 8) & 0xFF;
            dataRespBuffer[2] = pianolaTime & 0xFF;
            break;

        case SET_PinSource:

            pianolaPinIdx = dataRecBuffer[1];
            pianolaPinBitMap = dataRecBuffer[2];
            pianolaPinInvert = dataRecBuffer[3];
            break;

        case GET_PinSource:

            dataRespBuffer[0] = pianolaPinBitMap;
            dataRespBuffer[1] = pianolaPinInvert;
            break;

        case SET_CameraClk:
            int clockIdx = dataRecBuffer[1];
            pianolaCameraClock[clockIdx] = dataRecBuffer[2] << 16 | dataRecBuffer[3] << 8 | dataRecBuffer[4] & 0xFF;
            break;

        case SET_PIVParams:
        // double check order with xmos code - likely not a problem until farther down the line
            pivIdx = dataRecBuffer[1] & 0xFF;
            
            pivIntervalLaserPulses = dataRecBuffer[5] << 24 | dataRecBuffer[4] << 16 | dataRecBuffer[3] << 8| dataRecBuffer[2] & 0xFF;
            pivDuratonPulsePair  = dataRecBuffer[9] << 24 | dataRecBuffer[8] << 16 | dataRecBuffer[7] << 8| dataRecBuffer[6] & 0xFF;
            pivIntervalPulseToExp  = dataRecBuffer[13] << 24 | dataRecBuffer[12] << 16 | dataRecBuffer[11] << 8| dataRecBuffer[10] & 0xFF;
            pivCameraExposure =  dataRecBuffer[17] << 24 | dataRecBuffer[16] << 16 | dataRecBuffer[15] << 8| dataRecBuffer[14] & 0xFF;
            pivCameraMonitorIdx = dataRecBuffer[18] & 0xFF;
            break;

        case GET_FirmwareVersion:
            dataRespBuffer[0] = pianolaFirmwareVersion & 0xFF;
            dataRespBuffer[1] = pianolaFirmwareEarliestVersion & 0xFF;
            //printf("0x%02X : 0x%02X", cmdResponse[0], cmdResponse[1]);
            break;


        case SET_ClockDivisor:
            pianolaClockDivisorFP = (dataRecBuffer[1] << 16) |(dataRecBuffer[2] << 8) |(dataRecBuffer[3] & 0xFF);
            uint16_t pianolaClockDivisorInt = (pianolaClockDivisorFP << 8) & 0xFFFF;
            uint8_t pianolaClockDivisorFrac = pianolaClockDivisorFP & 0xFF;

            pio_sm_set_clkdiv_int_frac8(pio, smPinDriver, pianolaClockDivisorInt, pianolaClockDivisorFrac);
            break;

        case IRQ_HARDRESET:
            pianolaStatus = INIT;
            // do nothing for now;
            break;

        case IRQ_DumpLog:


            // int numEntries = 2;
            // for (int i = 0; i < numEntries; i ++)
            // {
            //     unsigned char tmpQEntry[6] ={0};
            //     queue_try_remove(&dumpDebugQ, &tmpDebugEntry);
            //     uint32_t tmpProposedTime = (tmpDebugEntry[0] << 16) | (tmpDebugEntry[1] << 8) | (tmpDebugEntry[2] & 0xFF);

            //     uint32_t tmpPianolaTime = (tmpDebugEntry[3] << 16) | (tmpDebugEntry[4] << 8) | (tmpDebugEntry[5] & 0xFF);
            //     //printf("%d Entry | Proposed Time = %d | Pianola Time = %d\n", i, tmpProposedTime, tmpPianolaTime);
            //     for (int k = 0; k < 6; k++)
            //     {
            //         dataRespBuffer[6*i + k] = tmpDebugEntry[k];
            //     }

            // }

            break;
        default:
            //printf("Not a valid response\n");
            break;
    };
}

// ===============================================================================
// Other Helpful functions
//================================================================================

void clearCommandBuffer(void)
{
    for (int i = 0; i < RX_BUFFER_SIZE_BYTES; i++)
    {
        commandBuffer[i] = 0x00;
    }

}

void clearResponseBuffer(void)
{
    for (int i = 0; i < TX_BUFFER_SIZE_BYTES; i++)
    {
        responseBuffer[i] = 0x00;
    }
}

void clearBuffers(void)
{
    for (int i = 0; i < RX_BUFFER_SIZE_BYTES; i++)
    {
        commandBuffer[i] = 0x00;
        responseBuffer[i] = 0x00;
    }
}


// ===============================================================================
// Pianola Manager and UART handler
//================================================================================



void uartHandlerCore0(void)
{
    // check if UART is readabler
    if (uart_is_readable(UART_ID))
    {
        // grab the opcode and pass to the parser to decide how many more bytes to read
        clearCommandBuffer();
        uint8_t opcode = uart_getc(UART_ID);

        commandDataConfig(opcode);
        commandBuffer[0] = opcode & 0xFF;
        if (commandConfig.numBytesReceive > 0)
        {

            unsigned char tempBuffer[commandConfig.numBytesReceive];
            uart_read_blocking(UART_ID, tempBuffer, commandConfig.numBytesReceive);
            
            for (int i = 0; i < commandConfig.numBytesReceive;  i ++)
            {
                commandBuffer[i+1] = tempBuffer[i];
            }

        }
        // command buffer to the queue because the pinaola manager will decide what to do with it
        queue_try_add(&commandQ, commandBuffer);
        //queue_try_add(&dumpDebugQ, commandBuffer);

    }
    // check if theres any response waiting in the response queue
    if (queue_is_full(&responseQ))
    {
        queue_try_remove(&responseQ, &responseBuffer);
        uart_write_blocking(UART_ID, responseBuffer, commandConfig.numBytesResponse);
        // clear buffer and reset the command config ready for the next interrupt

        commandConfig.numBytesReceive  = 0;
        commandConfig.numBytesResponse = 0;

        clearBuffers();

    }


}


void pianolaManagerCore1(void)
{

    printf("Launching Pianola Manager\n");
    while(1)
    {
        pianolaTime = getPianolaTime();
        delT = pianolaFireTime - pianolaTime;
        // if were near the fire time then we should start running 
        if ((delT > 0) && (delT < pianolaFireTimeThresholdUS) && (pianolaStatus != RUNNING))
        {
            //start running from the first pianola entry
            pianolaStatus = RUNNING;
            pianolaPos = 0;

            // reset pianolaFireTime so we don't fire again when the clock wraps around
            pianolaFireTime = 0;
            
        }
        // if theres a command waiting to be processed then process it
        else if (queue_is_full(&commandQ))
        {

            queue_try_remove(&commandQ, &commandBuffer);
            // pass to the parser 
            CommandParser(commandBuffer, responseBuffer);
            //sort out response
            if (commandConfig.numBytesResponse > 0)
            {
                queue_try_add(&responseQ, &responseBuffer);
                uartHandlerCore0();
            }

        }
        else if ((pianolaStatus == RUNNING | pianolaStatus == REPEATING) && pio_sm_is_tx_fifo_empty(pio, smPinDriver))
        {


            if (pianolaPos <= pianolaFinalPos)
            {
                pio_sm_put(pio, smPinDriver, pianola[pianolaPos] - (pianolaPinTriggerOffset << 8));
                pianolaPos++;
            }
            // if we get to the end of the pianola
            // stop running if we're not repeating
            else if (pianolaStatus == RUNNING)
            {   
                pianolaPos = 0;
                pianolaStatus = READY;
            }
            // if we want to repeat then set to the repeat from address
            // this defaults to 0 so its the same as going to the start
            // else if we've set it from a command, repeat from that address till heat death
            else if (pianolaStatus == REPEATING)
            {
                pianolaPos = pianolaRepeatAddr;
            }

        }
    }
}


void stabilisePianolaClock(void)
{
    // the clock statemachine is a bit weird for the first 30-100ms / first few gets
    // just going to gettime a few times and then discard
    for (int i = 0; i < 10; i++)
    {
        uint32_t t = getPianolaTime();
        sleep_us(10000);

    }
}
void main(void)
{


    // This is Core0 which handles basic setup of hardware and then runs an interrupt 
    // to handle incoming commands and responses
    // Core 1 handles the running of the pianola and some command handling as well.

    stdio_init_all();
    gpio_init(STATUSLED_PIN);
    gpio_set_dir(STATUSLED_PIN, true);
    gpio_put(STATUSLED_PIN, true);

    // create queues to store commands we want to shuffle between cores
    // commandQ stores a single char array of 32 bytes containing data received over UART
    // responseQ stores a single 32byte char array which will contain the reponse to any command.
    // dumpDebugQ is 6 entries deep and 64 bytes wide. This is configurable but this seems sufficient for now.
    queue_init(&commandQ, RX_BUFFER_SIZE_BYTES, 1);
    queue_init(&responseQ, TX_BUFFER_SIZE_BYTES, 1);
    queue_init(&dumpDebugQ, 6, 64);

    // init the uart and gpios
    uart_init(UART_ID, BAUDRATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_hw_flow(UART_ID, false, false);
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);

    irq_set_exclusive_handler(UART_INTERRUPT, uartHandlerCore0);
    irq_set_enabled(UART_INTERRUPT, true);
    uart_set_irq_enables(UART_ID, true, false);

    // init the pio blocks
    //pinDriver
    bool pioTriggerStart = pio_claim_free_sm_and_add_program_for_gpio_range(&triggerControl_program, &pio, &smPinDriver,&offsetPinDriver, lsbPin, NUM_DATAPIN, true);
    triggerControl_program_init(pio, smPinDriver, offsetPinDriver, lsbPin, pinDriverClockDivisorFP);
    //pio_sm_set_enabled(pio, smPinDriver, true);
    //pio_sm_set_enabled(pioPinDriver, smPinDriver, true);
    // pianolaClock

    bool pioClockStart = pio_claim_free_sm_and_add_program(&pianolaClock_program, &pio, &smPianolaClock, &offsetPianolaClock);
    pianolaClock_program_init(pio, smPianolaClock, offsetPianolaClock, pianolaClockDivisorFP);
    //pio_sm_set_enabled(pio, smPianolaClock, true);
    pio_enable_sm_mask_in_sync(pio, (1ULL << smPinDriver)|(1ULL << smPianolaClock));
    if (pioTriggerStart & pioClockStart)
    {
        printf("Pio Init\n");
        printf("Offsets: Clock (%d) | Trigger (%d)\n", offsetPianolaClock, offsetPinDriver);
    }
    else
    {
        printf("Pio Clock %d | Pio Trigger %d\n", pioClockStart, pioTriggerStart);
    }




    //pio_set_sm_mask_enabled(pio, (1LL << smPinDriver) | (1ULL << smPianolaClock), true);
    // stabilise the pianolaClock
    stabilisePianolaClock();
    // launch the other core to handle running the pianola
    pianolaTime = getPianolaTime();
    printf("Init Complete\n");


    multicore_launch_core1(&pianolaManagerCore1);


    // enter a tight loop while were not in a reset or panic mode

    while(pianolaStatus < 5) // is pianolaStatus not RESET or PANIC
    {
        tight_loop_contents();
    }

    // after we reset, everything else will be cleared to just auto reboot everything
    main();



}

