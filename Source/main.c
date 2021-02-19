#include "project.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"

#include "sound.h"
#include "gps.h"
#include "lpc_synth.h"
#include "custom_synth.h"
#include "lsm303d.h"
#include "battery_level.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#define TABLE_LENGTH 90

#define ON();       { AMux_1_Start(); AMux_2_Start(); }
#define OFF();      { AMux_1_DisconnectAll(); AMux_2_DisconnectAll();}
#define SOUND();    { AMux_1_FastSelect(0); AMux_2_FastSelect(0); } // Navigation sound output - SOUND for short
#define SPEECH();   { AMux_1_FastSelect(1); AMux_2_FastSelect(1); } // Speech sythesiser output - SPEECH for short

#define BlinkLED(); { Pin_LED_Write(1); CyDelayUs(30); Pin_LED_Write(0); } // blink LED indicator - for debugging/testing

/* Task Priority Level */
#define TASK_GPS_PRIO           (configMAX_PRIORITIES - 1)
#define TASK_PATH_START_PRIO    (configMAX_PRIORITIES - 2)
#define TASK_PATH_PRIO          (configMAX_PRIORITIES - 2)
#define TASK_DIRECTION_PRIO     (configMAX_PRIORITIES - 3)
#define TASK_SPEECH_PRIO        (configMAX_PRIORITIES - 4)
#define TASK_BATTERY_LEVEL_PRIO (configMAX_PRIORITIES - 5)

/*-----------------------------------------------------------*/
/* Global Varaibles */
/* PC UART variables - for testing/debugging*/
char tempStr[100];

/* Path planning variables */
/* Checkpoint latitudes */
double checkpointLat[15] = {
    -37.911547, -37.911685, -37.911286, -37.911307, -37.911286, 
    -37.910313, -37.910275, -37.910235, -37.909966, -37.910235, 
    -37.910215, -37.910688, -37.910862, -37.911202, -37.911295
};

/* Checkpoint longitudes */
double checkpointLon[15] = {
    145.13335, 145.13398, 145.13415, 145.13442, 145.13415, 
    145.13432, 145.13333, 145.13211, 145.13219, 145.13211, 
    145.13167, 145.13158, 145.13277, 145.13298, 145.13339 
};

/* Checkpoint names - for debugging/testing */
int checkpointName[15] = { 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14 };
//{"H0","H1","H2","H3","H4","H5","H6","H7","H8","H9","H10","H11","H12","H13"};

int checkpointOperation = 0; // 0 - addition, 1- substraction.
int checkpointCurrent = 0; // {0,1,2,3,4,5,6,7,8,9,10,11,12,13}
int checkpointDest = 0; // 0,3, 8
char checkpointDestName; // for debugging/testing
BaseType_t checkpointDestSelected = pdFALSE; // True if user has selected a valid destination
BaseType_t atDestination = pdFALSE; // True if user is within the proximity of the selected destination

/* GPS variables */
long double latitudeInDec, longitudeInDec;
int firstFix = 0;

/* Button variables */
int buttonCount = -1; // button count mod 3 = ans, if 0-Campus centre, 1-Campbell Hall, 2- Hargrave
int initialPowerUp = 0; // solve INITIAL POWER UP that causes positive edge on the button (no need to reset)


/*-----------------------------------------------------------*/
/* Task Handlers */
TaskHandle_t vTaskGPSHandle = NULL;
TaskHandle_t vTaskPathStartHandle = NULL;
TaskHandle_t vTaskPathHandle = NULL;
TaskHandle_t vTaskSpeechHandle = NULL;
TaskHandle_t vTaskDirectionHandle = NULL;
TaskHandle_t vTaskBatteryLevelHandle = NULL;

/* Semaphore Handlers */
SemaphoreHandle_t xGPSSemaphore;
SemaphoreHandle_t xUARTMutex;

/* Queue Handlers */


/*-----------------------------------------------------------*/
/* Function declarations */
extern void RTOS_Start( void );

/*-----------------------------------------------------------*/
/* Task declarations */
static void vTaskGPS ( void *pvParameter );
static void vTaskPathStart (void *pvParameter);
static void vTaskPath ( void *pvParameter );
static void vTaskSpeech ( void *pvParameter );
static void vTaskDirection ( void *pvParameter );
static void vTaskBatteryLevel ( void *pvParameter );

/*-----------------------------------------------------------*/
/* Interrupt Service Routines */
CY_ISR( ISR_GPS_Received )
{
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR( xGPSSemaphore, &xHigherPriorityTaskWoken );
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

/*-----------------------------------------------------------*/
/* Interrupt Service Routines */
CY_ISR( ISR_Button_Press )
{
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    
    if (initialPowerUp == 0)
    {
        initialPowerUp = 1;
    }
    else {
        buttonCount++;
        switch( buttonCount % 3 ) {
            case 0:
            xTaskNotifyFromISR(vTaskSpeechHandle, (uint32_t)(1<<0), (eNotifyAction)eSetValueWithOverwrite, &xHigherPriorityTaskWoken );
            break;
            case 1:
            xTaskNotifyFromISR(vTaskSpeechHandle, (uint32_t)(1<<1), (eNotifyAction)eSetValueWithOverwrite, &xHigherPriorityTaskWoken );
            break;
            case 2:
            xTaskNotifyFromISR(vTaskSpeechHandle, (uint32_t)(1<<1)|(1<<0), (eNotifyAction)eSetValueWithOverwrite, &xHigherPriorityTaskWoken );
            break;
            default: 
            //error
            sprintf(tempStr, "Error in button presses\n");
            UART_PutString(tempStr);
            break;
        }
    }
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

/*-----------------------------------------------------------*/
CY_ISR( ISR_Button_Hold )
{
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    
    /* stop button press isr once destination verified */
    /* ENSURE this interrupt has a higher priority level than isr_nutton_press */
    isr_button_press_Stop();
    
    switch( buttonCount % 3 ) {
        case 0:
        checkpointDestName = 'C';
        sprintf(tempStr, "Campus Center Destination Selected\n");
        UART_PutString(tempStr);
        break;
        case 1:
        checkpointDestName = 'H';
        sprintf(tempStr, "Campbell Hall Destination Selected\n");
        UART_PutString(tempStr);
        break;
        case 2:
        checkpointDestName = 'L';
        sprintf(tempStr, "Hargrave Library Destination Selected\n");
        UART_PutString(tempStr);
        break;
        default: 
        //error
        sprintf(tempStr, "Error in button hold\n");
        UART_PutString(tempStr);
        break;
    }
    checkpointDestSelected = pdTRUE;
    
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );  
}

/*-----------------------------------------------------------*/
void PSOC_Start( void )
{
    /* Start GPS and PC uart - for debugging/testing */
    UART_Start();
    
    /* Start Button */
    Timer_1_Start();
    
    /* Start and Initialize Sound */
    startSoundComponents();
    dmaConfiguration();
    int freq = 100;       // put as define constant for system frequency once tested
    sineWaveInitialize( freq );
    
    /* Start and Initialize Speech */
    synthInitialize();
    
    /* Start and Initialize Compass */
    compassStart();
    compassInitialize();
    
    /* Start Battery Level Monitor */
    batteryLevelMonitorStart();
    
    /* Turn on System Output */
    ON();
    
}

/*-----------------------------------------------------------*/
int main( void )
{
    CyGlobalIntEnable; /* Enable global interrupts. */
    
    /* System Initialization */
    RTOS_Start();
    PSOC_Start();
    
    SPEECH();
    // Vocalize Welcome to SoundMap
    sayWelocome(); sayPause();
    
    OFF();
    
    /* Interrupts */
    isr_GPS_Received_ClearPending();    /* Cancel any pending isr_RxSignal interrupts */
    isr_GPS_Received_StartEx( ISR_GPS_Received ); /* Enable the interrupt service routine */
    
    isr_button_press_ClearPending();    /* Cancel any pending isr_RxSignal interrupts */
    isr_button_press_StartEx( ISR_Button_Press ); /* Enable the interrupt service routine */
    
    isr_button_hold_ClearPending();    /* Cancel any pending isr_RxSignal interrupts */
    isr_button_hold_StartEx( ISR_Button_Hold ); /* Enable the interrupt service routine */

    
    /* Creating Semaphores and Mutxes */
    xGPSSemaphore = xSemaphoreCreateCounting( 10, 0 );
    xUARTMutex = xSemaphoreCreateMutex();
    
    if ( xGPSSemaphore != NULL || xUARTMutex != NULL )
    {
        BaseType_t err;
        
        err = xTaskCreate( vTaskGPS, "task gps", 200, (void*) 0, TASK_GPS_PRIO, &vTaskGPSHandle );
        if ( err != pdPASS ){
            sprintf( tempStr, "Failed to Create Task GPS\n" );
            UART_PutString( tempStr );
            while(1){};
        }
        
        err = xTaskCreate ( vTaskSpeech, "task speech", 200, (void*) 0, TASK_SPEECH_PRIO, &vTaskSpeechHandle );
        if ( err != pdPASS ){
            sprintf( tempStr, "Failed to Create Task Speech\n" );
            UART_PutString( tempStr );
            while(1){};
        }
        
        err = xTaskCreate ( vTaskBatteryLevel, "task battery level", 200, (void*) 0, TASK_BATTERY_LEVEL_PRIO, &vTaskBatteryLevelHandle );
        if ( err != pdPASS ){
            sprintf( tempStr, "Failed to Create Task Battery Level\n" );
            UART_PutString( tempStr );
            while(1){};
        }
        
        sprintf( tempStr, "Main: Start\n\n" );
        UART_PutString( tempStr );
        
        vTaskStartScheduler();  /* Should never return, add reset after */
    }
    else 
    {
        /* failed to create semaphore or mutex */
        sprintf( tempStr, "failed to create semaphore or mutex" );
        UART_PutString( tempStr );
        while(1){};
    }
    
    for(;;);
}

/*-----------------------------------------------------------*/
/* Tasks */

static void vTaskGPS ( void *pvParameter )
{
    (void) pvParameter;
    const TickType_t xDelay40ms = pdMS_TO_TICKS(40UL);
    char PT_term_buffer_GPS[120], GGA_buffer[66], ns, ew;
    float latitude, longitude, gps_time;
    int n_char;
   
    while(1)
    {   
        static char ch;
        n_char = 0;
        
        while( n_char < 100 )
        {
            xSemaphoreTake( xGPSSemaphore, portMAX_DELAY );
            xSemaphoreTake( xUARTMutex, portMAX_DELAY );
            {
                ch = UART_GetChar();
            }
            xSemaphoreGive( xUARTMutex );
            if(ch == '\r') {
                PT_term_buffer_GPS[n_char] = 0; // zero terminate the string
                break;
            }
            else if (ch == '$') {
                    n_char = 0;
                    PT_term_buffer_GPS[n_char++] = ch ;
            }
            else  {PT_term_buffer_GPS[n_char++] = ch ;}
        } 
        
        if ( PT_term_buffer_GPS[4] == 'G' )
        { 
            // only extract GGA string
            strncpy( GGA_buffer, PT_term_buffer_GPS, 66 );
            
            char *p = strchr(GGA_buffer, ',');
            gps_time = atof(p+1); // if field is empty this encounters a comma and returns 0.0

            p = strchr(p+1, ',');
            latitude = atof(p+1);

            p = strchr(p+1, ',');
            ns = p[1] == ',' ? '?' : p[1];

            p = strchr(p+1, ',');
            longitude = atof(p+1);

            p = strchr(p+1, ',');
            ew = p[1] == ',' ? '?' : p[1];
        }
        
        longitudeInDec = min2dec( longitude );
        latitudeInDec = min2dec( latitude ) * -1;
        
        
        if ( longitudeInDec == 0 && latitudeInDec == 0 )
        {
            SPEECH();
            /* Prevent the RTOS kernel swapping out the task.*/
            vTaskSuspendAll();
            /* Interrupts will still operate and the tick count will be maintained. */
            sayFix();
            /* Restart the RTOS kernel.  We want to force a context switch, 
            but there is no point if resuming the scheduler caused a context switch already. */
            if( !xTaskResumeAll () )
            {
                taskYIELD ();
            }
            OFF();
        }
        
        if ( firstFix == 0 && longitudeInDec != 0 && latitudeInDec != 0 && checkpointDestSelected == pdTRUE )
        {
            firstFix = 1; // run one time
            //checkpointDestSelected = pdFAIL; /*can selct mutiple times until fix happens */
            BaseType_t err = xTaskCreate( vTaskPathStart, "task path start", 200, (void*) 0, TASK_PATH_START_PRIO, &vTaskPathStartHandle );
            if ( err != pdPASS ){
                sprintf(tempStr, "Failed to Create Task Path Start\n");
                UART_PutString( tempStr );
                while(1){};
            }
            err = xTaskCreate( vTaskDirection, "task direction", 200, (void*) 0, TASK_DIRECTION_PRIO, &vTaskDirectionHandle );
            if ( err != pdPASS ){
                sprintf(tempStr, "Failed to Create Task Path Start\n");
                UART_PutString( tempStr );
                while(1){};
            }
        }
        vTaskDelay(xDelay40ms);
    }
}

/*-----------------------------------------------------------*/
static void vTaskPathStart ( void *pvParameter )
{
    (void) pvParameter;
    double diffDistanceToStart[4];/* Starting points H0,H3,H8 */
    int startArrayLocation;
    
    while (1)
    {
        /* Distance from current location to each starting point */
        diffDistanceToStart[0] = distance( latitudeInDec, longitudeInDec, checkpointLat[0],checkpointLon[0] );
        diffDistanceToStart[1] = distance( latitudeInDec, longitudeInDec, checkpointLat[3],checkpointLon[3] );
        diffDistanceToStart[2] = distance( latitudeInDec, longitudeInDec, checkpointLat[8],checkpointLon[8] );
        
        if( (diffDistanceToStart[0] < diffDistanceToStart[1]) && (diffDistanceToStart[0] < diffDistanceToStart[2]) )
        {
           startArrayLocation = 0;
        }
        else if( diffDistanceToStart[1] < diffDistanceToStart[2] )
        {
           startArrayLocation = 1;
        }
        else
        {
           startArrayLocation = 2;
        }

        /* Determine starting location, chechpoint opertion and destination (for now)*/
        switch ( startArrayLocation )
        {
            case 0: /* H0 is the starting point*/
                checkpointCurrent = 0;
                switch ( checkpointDestName )
                {
                    case 'C':
                        checkpointDest = 0;
                        atDestination = pdTRUE; /* at destnation */
                        break;
                    case 'H':
                        checkpointDest = 3;
                        checkpointOperation = 0; /* addition */
                        break;
                    case 'L':
                        checkpointDest = 8;
                        checkpointOperation = 1; /* substraction */
                        break;
                    default:
                        /* error */
                        break;
                }
                sprintf( tempStr, "Starting Point: H%d \nDestination is: H%d\n", checkpointName[checkpointCurrent], checkpointName[checkpointDest] );
                UART_PutString( tempStr );
                break;
            case 1: /* H3 is the starting point */
                checkpointCurrent = 3;
                switch ( checkpointDestName )
                {
                    case 'C':
                        checkpointDest = 0;
                        checkpointOperation = 1; /* substraction */
                        break;
                    case 'H':
                        checkpointDest = 3;
                        atDestination = pdTRUE; /* at destnation */
                        break;
                    case 'L':
                        checkpointDest = 8;
                        checkpointOperation = 0; /* addition */
                        break;
                    default:
                        /* error */
                        break;
                }
                sprintf( tempStr, "Starting Point: H%d \nDestination is: H%d\n", checkpointName[checkpointCurrent], checkpointName[checkpointDest] );
                UART_PutString( tempStr );
                break;
            case 2: /* H8 is the starting point */
                checkpointCurrent = 8;
                switch ( checkpointDestName )
                {
                    case 'C':
                        checkpointDest = 0;
                        checkpointOperation = 0; /* addition */
                        break;
                    case 'H':
                        checkpointDest = 3;
                        checkpointOperation = 1; /* substraction */
                        break;
                    case 'L':
                        checkpointDest = 8;
                        atDestination = pdTRUE; /* at destnation */
                        break;
                    default:
                        /* error */
                        break;
                }
                sprintf( tempStr, "Starting Point: H%d \nDestination is: H%d\n", checkpointName[checkpointCurrent], checkpointName[checkpointDest] );
                UART_PutString( tempStr );
                break;
            default:
                /* error */
                break;
        }
        
        BaseType_t err;
        err = xTaskCreate( vTaskPath, "task path", 200, (void*) 0, TASK_PATH_PRIO, &vTaskPathHandle );
        if ( err != pdPASS ){
            sprintf( tempStr, "Failed to Create Task Path\n" );
            UART_PutString( tempStr );
            while(1){};
        }
        vTaskDelete( NULL );
    }
}

/*-----------------------------------------------------------*/
static void vTaskPath( void *pvParameter )
{
    (void) pvParameter;
    double diffDistance;
    const TickType_t xDelay2000ms = pdMS_TO_TICKS(2000UL);
    int nextCheckpoint;
    SOUND(); // sound always on as long as task path is running - ***place this in sound task when written***
    while (1)
    {
        if ( checkpointOperation == 0 )
        {
            nextCheckpoint = checkpointCurrent+1;
            if ( nextCheckpoint == 15 ) { nextCheckpoint = 0; }
            diffDistance = distance( latitudeInDec, longitudeInDec, checkpointLat[nextCheckpoint],checkpointLon[nextCheckpoint] );
        }
        if ( checkpointOperation == 1 )
        {
            nextCheckpoint = checkpointCurrent-1;
            if ( nextCheckpoint == -1 ) { nextCheckpoint = 14; }
            diffDistance = distance(latitudeInDec, longitudeInDec, checkpointLat[nextCheckpoint],checkpointLon[nextCheckpoint] );
        }
        
        if ( diffDistance < 15 || atDestination == pdTRUE )
        {
            if ( checkpointOperation == 0 && atDestination == pdFALSE )
            { 
                checkpointCurrent = checkpointCurrent + 1;
                if ( checkpointCurrent == 15 ) { checkpointCurrent = 0; }
                
            }
            if ( checkpointOperation == 1 && atDestination == pdFALSE ) 
            {
                checkpointCurrent = checkpointCurrent - 1;
                if ( checkpointCurrent == -1 ) { checkpointCurrent = 14; }
            }
            else { /* error in checkPointOperation value*/}
            if ( checkpointCurrent == checkpointDest )/* if atDestination is pdTRUE, this condition must be satisfied */
            {
                SPEECH();
                /* Prevent the RTOS kernel swapping out the task.*/
                vTaskSuspendAll();
                /* Interrupts will still operate and the tick count will be maintained. */
                sayArrived();
                /* Restart the RTOS kernel.  We want to force a context switch, 
                but there is no point if resuming the scheduler caused a context switch already. */
                if( !xTaskResumeAll () )
                {
                    taskYIELD ();
                }
                OFF();
                // arrived at destination
                sprintf( tempStr, "Arrived at destination \n" );
                UART_PutString( tempStr );
                
                /* RESTART PROGRAM - works :) */
                isr_button_press_ClearPending(); 
                isr_button_press_StartEx( ISR_Button_Press );  // enable button press inerrupt
                checkpointDestSelected = pdFALSE; // no destination selected
                firstFix = 0; // to create vTaskPathStart once in vTaskGPS
                buttonCount = -1; // reset button count
                vTaskDelete(vTaskDirectionHandle);
                vTaskDelete(NULL); // delete current task - and all others
            }
        }
        sprintf( tempStr, "Current Checkpoint: H%d \nNext Checkpoint:    H%d \n", checkpointName[checkpointCurrent], checkpointName[nextCheckpoint] );
        UART_PutString( tempStr );
        
        sprintf( tempStr, "Distance to next checkpoint: %.2f \n", diffDistance );
        UART_PutString( tempStr);
        
        vTaskDelay( xDelay2000ms );
    }
}

/*-----------------------------------------------------------*/
static void vTaskSpeech ( void *pvParameter )
{
    (void) pvParameter;
    uint32_t ulNotificationValue;
    while (1)
    {
        if ( xTaskNotifyWait((uint32_t)0, (uint32_t)0, &ulNotificationValue, portMAX_DELAY ) == pdTRUE )
        {
            sprintf( tempStr, "Notification received: %d", ulNotificationValue );
            UART_PutString( tempStr );
        }

        SPEECH();
                
        //Prevent the RTOS kernel swapping out the task.
        vTaskSuspendAll();
        //Interrupts will still operate and the tick count will be maintained.
        switch (ulNotificationValue)
        {
            case 1:
            sprintf( tempStr, "     Campus centre\n" );
            UART_PutString( tempStr );
            sayCampusCentre();
            break;
            case 2:
            sprintf( tempStr, "     Campbell Hall\n" );
            UART_PutString( tempStr );
            sayCampbellHall();
            break;
            case 3:
            sprintf( tempStr, "     Hargrave Library\n" );
            UART_PutString( tempStr );
            sayHargraveLibrary();
            break;
            default:
            sprintf( tempStr, "Error in notification value received \n" );
            UART_PutString( tempStr );
            break;
        }
        // Restart the RTOS kernel.  We want to force a context switch, 
        //but there is no point if resuming the scheduler caused a context switch already.
        if( !xTaskResumeAll () )
        {
            taskYIELD ();
        }
    }

}

/*-----------------------------------------------------------*/
static void vTaskDirection ( void *pvParameter )
{
    (void) pvParameter;
    double bearing, difference, direction;
    const TickType_t xDelay1000ms = pdMS_TO_TICKS(1000UL);
    
    while(1)
    {
        /* UNCOMMENT when working on the compass */
        /*bearing = heading();
        if (bearing > M_PI) bearing = bearing - 2*M_PI;
        
        difference = GPSbearing(latitudeInDec, longitudeInDec ,checkpointLat[checkpointDest], checkpointLon[checkpointDest]);
		
        direction = difference-bearing; // calculate the direction need to walk in to get to destination        
       
        sprintf(tempStr, "Direction: %lf \n", difference);
        UART_PutString( tempStr );*/
        vTaskDelay(xDelay1000ms);
    }
}

/*-----------------------------------------------------------*/
static void vTaskBatteryLevel ( void *pvParameter )
{
    (void) pvParameter;
    int batteryLevelValue = 0;
    const TickType_t xDelay5000ms = pdMS_TO_TICKS(5000UL);
    
    while(1)
    {
        batteryLevelValue = readBatteryLevel();
        
        sprintf(tempStr, "Battery Level: %d%%\n", batteryLevelValue);
        UART_PutString(tempStr);
        
        SPEECH();
        
        //Prevent the RTOS kernel swapping out the task.
        vTaskSuspendAll();
        //Interrupts will still operate and the tick count will be maintained.
        
        sayBatteryPercent(batteryLevelValue);
        
        // Restart the RTOS kernel.  We want to force a context switch, 
        //but there is no point if resuming the scheduler caused a context switch already.
        if( !xTaskResumeAll () )
        {
            taskYIELD ();
        }
        OFF();
        vTaskDelay(xDelay5000ms);
    }
    
}
/* [] END OF FILE */
