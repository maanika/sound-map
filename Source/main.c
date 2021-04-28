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
#include "mode.h"
#include "path.h"
#if OBJ_DETECT_MODE == 1
    #include "distance.h"
    #include "motor.h"
#endif

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#define TABLE_LENGTH 90
#define PATH_PROXIMITY 5 // update checkpoint when user is within PATH_PROMIMITY meters

#define ON();       { AMux_1_Start(); AMux_2_Start(); }
#define OFF();      { AMux_1_DisconnectAll(); AMux_2_DisconnectAll();}
#define SOUND();    { AMux_1_FastSelect(0); AMux_2_FastSelect(0); } // Navigation sound output - SOUND for short
#define SPEECH();   { AMux_1_FastSelect(1); AMux_2_FastSelect(1); } // Speech sythesiser output - SPEECH for short

#define BlinkLED(); { Pin_LED_Write(1); CyDelayUs(30); Pin_LED_Write(0); } // blink LED indicator - for debugging/testing

/* Task Priority Level */
#define TASK_GPS_PRIO           (configMAX_PRIORITIES - 2)
#define TASK_PATH_PRIO          (configMAX_PRIORITIES - 3)
#define TASK_DIRECTION_PRIO     (configMAX_PRIORITIES - 4)
#define TASK_SPEECH_PRIO        (configMAX_PRIORITIES - 5)
#define TASK_BATTERY_LEVEL_PRIO (configMAX_PRIORITIES - 6)
#define TASK_BUTTON_PRIO        (configMAX_PRIORITIES - 1)
#if OBJ_DETECT_MODE == 1
    #define TASK_MOTOR_PRIO     (configMAX_PRIORITIES - 7)
    #define TASK_DIS_PRIO       (configMAX_PRIORITIES - 8)
#endif

/* Task Stack Size */
#define TASK_GPS_STK_SIZE           300
#define TASK_SPEECH_STK_SIZE        300
#define TASK_PATH_STK_SIZE          300
#define TASK_DIRECTION_STK_SIZE     100
#define TASK_BATTERY_LEVEL_STK_SIZE 300
#define TASK_BUTTON_STK_SIZE        300
#if OBJ_DETECT_MODE == 1
    #define TASK_MOTOR_STK_SIZE     200
    #define TASK_DIS_STK_SIZE       200
#endif

/*-----------------------------------------------------------*/
/* Global Varaibles */
/* Path planning variables */
struct Path path = {
    /* Checkpoint latitudes */      /* Checkpoint longitudes */
    .checkpointLat[0] = -37.911340, .checkpointLon[0] = 145.133382,
    .checkpointLat[1] = -37.911751, .checkpointLon[1] = 145.133984,
    .checkpointLat[2] = -37.911251, .checkpointLon[2] = 145.134131,
    .checkpointLat[3] = -37.911283, .checkpointLon[3] = 145.134456,
    .checkpointLat[4] = -37.911251, .checkpointLon[4] = 145.134131,
    .checkpointLat[5] = -37.910270, .checkpointLon[5] = 145.134342,
    .checkpointLat[6] = -37.910278, .checkpointLon[6] = 145.133317,
    .checkpointLat[7] = -37.910235, .checkpointLon[7] = 145.13211,
    .checkpointLat[8] = -37.909957, .checkpointLon[8] = 145.132161,
    .checkpointLat[9] = -37.910235, .checkpointLon[9] = 145.13211,
    .checkpointLat[10] = -37.910128, .checkpointLon[10] = 145.131608,
    .checkpointLat[11] = -37.910693, .checkpointLon[11] = 145.131576,
    .checkpointLat[12] = -37.910889, .checkpointLon[12] = 145.132747,
    .checkpointLat[13] = -37.911165, .checkpointLon[13] = 145.132943,
    .checkpointLat[14] = -37.911292, .checkpointLon[14] = 145.133382,
    .checkpointOperation = 0,   /* addition by default */
    .checkpointDestSelected = pdFALSE,
    .checkpointDestName = ' ',
    .atDestination = pdFALSE,
};

/* GPS variables */
long double latitudeInDec, longitudeInDec;
int firstFix = 0;

/* Battery Level variables */
int batteryLevelValue = 0;

/* Variables for debugging/testing */
#if DEBUG_PRINT_MODE == 1
    /* Checkpoint name printing */
    int checkpointName[15] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14 };
    
    /* PC UART printing */
    char tempStr[100]; 
#endif

/* Variables for object detection */
#if OBJ_DETECT_MODE == 1
    ultrasonicSensor ultrasonicReadings;
#endif

/*-----------------------------------------------------------*/
/* Task Handlers */
TaskHandle_t vTaskGPSHandle           = NULL;
TaskHandle_t vTaskPathHandle          = NULL;
TaskHandle_t vTaskSpeechHandle        = NULL;
TaskHandle_t vTaskDirectionHandle     = NULL;
TaskHandle_t vTaskBatteryLevelHandle  = NULL;
TaskHandle_t vTaskButtonHandle        = NULL;
#if OBJ_DETECT_MODE == 1
    TaskHandle_t xTaskMotorHandle     = NULL;
    TaskHandle_t xTaskDistanceHandle  = NULL;
#endif

/* Semaphore Handlers */
SemaphoreHandle_t xGPSSemaphore;
SemaphoreHandle_t xUARTMutex;

/* Queue Handlers */
QueueHandle_t xButtonTimeQueue;

/*-----------------------------------------------------------*/
/* Function declarations */
extern void RTOS_Start( void );

/*-----------------------------------------------------------*/
/* Task declarations */
static void vTaskGPS            ( void *pvParameter );
static void vTaskPathStart      ( void *pvParameter );
static void vTaskPath           ( void *pvParameter );
static void vTaskSpeech         ( void *pvParameter );
static void vTaskDirection      ( void *pvParameter );
static void vTaskBatteryLevel   ( void *pvParameter );
static void vTaskButton         ( void *pvParameter );
#if OBJ_DETECT_MODE == 1
static void vTaskDistance       ( void *pvParameter );
static void vTaskMotor          ( void *pvParameter );
#endif
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

CY_ISR( ISR_Button )
{
    BaseType_t xHigherPriorityTaskWoken;
    BaseType_t xStatus;
    xHigherPriorityTaskWoken = pdFALSE;
    
    /* Find how long the button was pressed for */
    uint16_t counter = Timer_1_ReadCapture();
    float buttonHoldTime;
    buttonHoldTime = (float) ((uint32_t)(65536 - counter)) * 0.001;
    
    /* Reset Timer_1 */
    Control_Reg_1_Write(1); 
    CyDelayUs(1000);
    Control_Reg_1_Write(0);
    
    /* Send  button hold time */
    xQueueSendToFrontFromISR(xButtonTimeQueue, (void*)&buttonHoldTime, (TickType_t)portMAX_DELAY );
    
    xTaskResumeFromISR(vTaskButtonHandle);
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );  
}

/*-----------------------------------------------------------*/
void PSOC_Start( void )
{
    /* Start UART for GPS and debug printing */
    UART_Start();
    
    /* Start Button */
    Timer_1_Start();
    
    /* Start and Initialize Sound */
    startSoundComponents();
    dmaConfiguration();
    int freq = 100;       // put as a constant for system frequency once tested
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
    
    isr_button_ClearPending();    /* Cancel any pending isr_RxSignal interrupts */
    isr_button_StartEx( ISR_Button ); /* Enable the interrupt service routine */
    
    /* Creating Semaphores and Mutxes */
    xGPSSemaphore = xSemaphoreCreateCounting( 10, 0 );
    xUARTMutex = xSemaphoreCreateMutex();
    xButtonTimeQueue = xQueueCreate( 1, sizeof(portFLOAT) );
    
    if ( xGPSSemaphore != NULL || xUARTMutex != NULL || xButtonTimeQueue != NULL )
    {
        BaseType_t err;
        
        err = xTaskCreate( vTaskGPS, "task gps", TASK_GPS_STK_SIZE, (void*) 0, TASK_GPS_PRIO, &vTaskGPSHandle );
        if ( err != pdPASS ){
            #if DEBUG_PRINT_MODE == 1
                sprintf( tempStr, "Failed to Create Task GPS\n" );
                UART_PutString( tempStr );
            #endif
            while(1){};
        }
      
        err = xTaskCreate ( vTaskSpeech, "task speech", TASK_SPEECH_STK_SIZE, (void*) 0, TASK_SPEECH_PRIO, &vTaskSpeechHandle );
        if ( err != pdPASS ){
            #if DEBUG_PRINT_MODE == 1
                sprintf( tempStr, "Failed to Create Task Speech\n" );
                UART_PutString( tempStr );
            #endif
            while(1){};
        }

        err = xTaskCreate ( vTaskBatteryLevel, "task battery level", TASK_BATTERY_LEVEL_STK_SIZE, (void*) 0, TASK_BATTERY_LEVEL_PRIO, &vTaskBatteryLevelHandle );
        if ( err != pdPASS ){
            #if DEBUG_PRINT_MODE == 1
                sprintf( tempStr, "Failed to Create Task Battery Level\n" );
                UART_PutString( tempStr );
            #endif
            while(1){};
        }
        
        err = xTaskCreate ( vTaskButton, "task button ", TASK_BUTTON_STK_SIZE, (void*) 0, TASK_BUTTON_PRIO, &vTaskButtonHandle );
        if ( err != pdPASS ){
            #if DEBUG_PRINT_MODE == 1
                sprintf( tempStr, "Failed to Create Task Button\n" );
                UART_PutString( tempStr );
            #endif
            while(1){};
        }
        
        #if OBJ_DETECT_MODE == 1
            err = xTaskCreate(vTaskDistance, "task distance", TASK_DIS_STK_SIZE, (void*) 0, TASK_DIS_PRIO, &xTaskDistanceHandle);
            if (err != pdPASS){
                sprintf(tempStr, "Failed to Create Task Distance\n");
                UART_WriteTxData(0x0d);
                UART_PutString(tempStr);
                while(1){};
            }
            
            err = xTaskCreate(vTaskMotor, "task motor", TASK_MOTOR_STK_SIZE, (void*) 0, TASK_MOTOR_PRIO, &xTaskMotorHandle);
            if (err != pdPASS){
                sprintf(tempStr, "Failed to Create Task Motor\n");
                UART_WriteTxData(0x0d);
                UART_PutString(tempStr);
                while(1){};
            }
        #endif
        
        #if DEBUG_PRINT_MODE == 1
            sprintf( tempStr, "Main: Start\n\n" );
            UART_PutString( tempStr );
        #endif
        
        vTaskStartScheduler();  /* Should never return, add reset after */
    }
    else 
    {
        /* failed to create semaphore or mutex */
        #if DEBUG_PRINT_MODE == 1
            sprintf( tempStr, "failed to create semaphore or mutex or queue" );
            UART_PutString( tempStr );
        #endif
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
        
        #if DEBUG_PRINT_MODE == 1
            sprintf(tempStr, "longitude: %Lf    latitude: %Lf\n", longitudeInDec, latitudeInDec);
            UART_PutString( tempStr );
        #endif
        
        if ( longitudeInDec == 0 && latitudeInDec == 0 )
        {
            xTaskNotify(vTaskSpeechHandle, (uint32_t)(1<<2)|(1<<0), (eNotifyAction)eSetValueWithOverwrite );
        }
        
        if ( firstFix == 0 && longitudeInDec != 0 && latitudeInDec != 0 && path.checkpointDestSelected == pdTRUE )
        {
            firstFix = 1; // run one time
            
            /* Set Path Details */
            pathStart ( &path, latitudeInDec, longitudeInDec );
            
            BaseType_t err = xTaskCreate( vTaskPath, "task path", TASK_PATH_STK_SIZE, (void*) 0, TASK_PATH_PRIO, &vTaskPathHandle );
            if ( err != pdPASS ){
                #if DEBUG_PRINT_MODE == 1
                    sprintf( tempStr, "Failed to Create Task Path\n" );
                    UART_PutString( tempStr );
                #endif
                while(1){};
            }
            err = xTaskCreate( vTaskDirection, "task direction", TASK_DIRECTION_STK_SIZE, (void*) 0, TASK_DIRECTION_PRIO, &vTaskDirectionHandle );
            if ( err != pdPASS ){
                #if DEBUG_PRINT_MODE == 1
                    sprintf(tempStr, "Failed to Create Task Path Start\n");
                    UART_PutString( tempStr );
                #endif
                while(1){};
            }
        }
        vTaskDelay(xDelay40ms);
    }
}

/*-----------------------------------------------------------*/
static void vTaskPath( void *pvParameter )
{
    (void) pvParameter;
    double diffDistance;
    const TickType_t xDelay2000ms = pdMS_TO_TICKS(2000UL);
    int nextCheckpoint;
    //SOUND(); // sound always on as long as task path is running - ***place this in sound task when written***
    while (1)
    {
        if ( path.checkpointOperation == 0 )
        {
            nextCheckpoint = path.checkpointCurrent+1;
            if ( nextCheckpoint == 15 ) { nextCheckpoint = 0; }
            diffDistance = distance( latitudeInDec, longitudeInDec, 
                path.checkpointLat[nextCheckpoint], path.checkpointLon[nextCheckpoint] );
        }
        if ( path.checkpointOperation == 1 )
        {
            nextCheckpoint = path.checkpointCurrent-1;
            if ( nextCheckpoint == -1 ) { nextCheckpoint = 14; }
            diffDistance = distance(latitudeInDec, longitudeInDec, 
                path.checkpointLat[nextCheckpoint], path.checkpointLon[nextCheckpoint] );
        }
        
        if ( diffDistance < PATH_PROXIMITY || path.atDestination == pdTRUE )
        {
            if ( path.checkpointOperation == 0 && path.atDestination == pdFALSE )
            { 
                path.checkpointCurrent = path.checkpointCurrent + 1;
                if ( path.checkpointCurrent == 15 ) { path.checkpointCurrent = 0; }
                
            }
            if ( path.checkpointOperation == 1 && path.atDestination == pdFALSE ) 
            {
                path.checkpointCurrent = path.checkpointCurrent - 1;
                if ( path.checkpointCurrent == -1 ) { path.checkpointCurrent = 14; }
            }
            else { /* error in checkPointOperation value*/}
            if ( path.checkpointCurrent == path.checkpointDest )/* if atDestination is pdTRUE, this condition must be satisfied */
            {
                /* Vocalize arrived at destination */
                xTaskNotify(vTaskSpeechHandle, (uint32_t)(1<<2)|(1<<1), (eNotifyAction)eSetValueWithOverwrite );

                /* RESTART PROGRAM */
                isr_button_ClearPending();
                path.checkpointDestSelected = pdFALSE; // no destination selected
                path.atDestination = pdFALSE; // not at destination - reset
                firstFix = 0; // to create vTaskPathStart once again in vTaskGPS
                vTaskDelete(vTaskDirectionHandle);
                vTaskDelete(NULL); // delete current task - and all others
            }
        }
        #if DEBUG_PRINT_MODE == 1
            sprintf( tempStr, "Current Checkpoint: H%d \nNext Checkpoint:    H%d \n", 
                checkpointName[path.checkpointCurrent], checkpointName[nextCheckpoint] );
            UART_PutString( tempStr );
            sprintf( tempStr, "Distance to next checkpoint: %.2f \n", diffDistance );
            UART_PutString( tempStr);
        #endif
        vTaskDelay( xDelay2000ms );
    }
}

/*-----------------------------------------------------------*/
static void vTaskSpeech ( void *pvParameter )
{
    (void) pvParameter;
    uint32_t speechNotificationValue;
    while (1)
    {
        if ( xTaskNotifyWait((uint32_t)0, (uint32_t)0, &speechNotificationValue, portMAX_DELAY ) == pdTRUE )
        {
            #if DEBUG_PRINT_MODE == 1
                sprintf( tempStr, "Notification received: %d", speechNotificationValue );
                UART_PutString( tempStr );
            #endif
        }

        SPEECH();
                
        //Prevent the RTOS kernel swapping out the task.
        vTaskSuspendAll();
        //Interrupts will still operate and the tick count will be maintained.
        switch (speechNotificationValue)
        {
            case 1:
                #if DEBUG_PRINT_MODE == 1
                    sprintf( tempStr, "     Campus centre\n" );
                    UART_PutString( tempStr );
                #endif
                sayCampusCentre();
                break;
            case 2:
                #if DEBUG_PRINT_MODE == 1
                    sprintf( tempStr, "     Campbell Hall\n" );
                    UART_PutString( tempStr );
                #endif
                sayCampbellHall();
                break;
            case 3:
                #if DEBUG_PRINT_MODE == 1
                    sprintf( tempStr, "     Hargrave Library\n" );
                    UART_PutString( tempStr );
                #endif
                sayHargraveLibrary();
                break;
            case 4:
                #if DEBUG_PRINT_MODE == 1
                    sprintf( tempStr, "     Battery Level: %d%%\n", batteryLevelValue );
                    UART_PutString(tempStr);
                #endif
                sayBatteryPercent(batteryLevelValue);
                break;
            case 5:
                #if DEBUG_PRINT_MODE == 1
                    sprintf( tempStr, "     No GPS Fix\n" );
                    UART_PutString( tempStr );
                #endif
                sayFix();
                break;
            case 6:
                #if DEBUG_PRINT_MODE == 1
                    sprintf( tempStr, "     Arrived at destination \n" );
                    UART_PutString( tempStr );
                #endif
                sayArrived();
            break;
            default:
                #if DEBUG_PRINT_MODE == 1
                    sprintf( tempStr, "Error in notification value received \n" );
                    UART_PutString( tempStr );
                #endif
                break;
        }
        // Restart the RTOS kernel.  We want to force a context switch, 
        //but there is no point if resuming the scheduler caused a context switch already.
        if( !xTaskResumeAll () )
        {
            taskYIELD ();
        }
        OFF();
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
    uint32_t batteryNotificationValue;
    
    while(1)
    {
        if ( xTaskNotifyWait((uint32_t)0, (uint32_t)0, &batteryNotificationValue, portMAX_DELAY ) == pdTRUE )
        {
            #if DEBUG_PRINT_MODE == 1
                sprintf( tempStr, "Battery Button Pressed" );
                UART_PutString( tempStr );
            #endif
        }
        if ( batteryNotificationValue == 1 )
        {
            batteryLevelValue = readBatteryLevel();
            xTaskNotify(vTaskSpeechHandle, (uint32_t)(1<<2), (eNotifyAction)eSetValueWithOverwrite );
        }
    }
}

/*-----------------------------------------------------------*/
static void vTaskButton ( void *pvParameter )
{
    (void) pvParameter;
    BaseType_t xStatus;
    float buttonHoldTime;
    int buttonCount = -1; // button count mod 3 = ans, if 0-Campus centre, 1-Campbell Hall, 2- Hargrave
    
    while(1)
    {
        xStatus = xQueueReceive( xButtonTimeQueue, &buttonHoldTime, portMAX_DELAY );
        #if DEBUG_PRINT_MODE == 1
            sprintf(tempStr, "Hold time: %.2f\n", buttonHoldTime);
            UART_PutString(tempStr);
        #endif
        if ( path.checkpointDestSelected == pdFALSE && buttonHoldTime > 0 && buttonHoldTime <= 3 )
        {
            buttonCount++;
            switch( buttonCount % 3 ) 
            {
                case 0:
                xTaskNotify(vTaskSpeechHandle, (uint32_t)(1<<0), (eNotifyAction)eSetValueWithOverwrite );
                break;
                case 1:
                xTaskNotify(vTaskSpeechHandle, (uint32_t)(1<<1), (eNotifyAction)eSetValueWithOverwrite );
                break;
                case 2:
                xTaskNotify(vTaskSpeechHandle, (uint32_t)(1<<1)|(1<<0), (eNotifyAction)eSetValueWithOverwrite );
                break;
                default: 
                //error
                #if DEBUG_PRINT_MODE == 1
                    sprintf(tempStr, "Error in button presses\n");
                    UART_PutString(tempStr);
                #endif
                break;
            }
        }
        else if ( path.checkpointDestSelected == pdFALSE && buttonHoldTime > 3 && buttonHoldTime <= 6 )
        {
            switch( buttonCount % 3 ) 
            {
                case 0:
                path.checkpointDestName = 'C';
                #if DEBUG_PRINT_MODE == 1
                    sprintf(tempStr, "Campus Center Destination Selected\n");
                    UART_PutString(tempStr);
                #endif
                break;
                case 1:
                path.checkpointDestName = 'H';
                #if DEBUG_PRINT_MODE == 1
                    sprintf(tempStr, "Campbell Hall Destination Selected\n");
                    UART_PutString(tempStr);
                #endif
                break;
                case 2:
                path.checkpointDestName = 'L';
                #if DEBUG_PRINT_MODE == 1
                    sprintf(tempStr, "Hargrave Library Destination Selected\n");
                    UART_PutString(tempStr);
                #endif
                break;
                default: 
                //error
                #if DEBUG_PRINT_MODE == 1
                    sprintf(tempStr, "Error in button hold (buttonCount = -1)\n");
                    UART_PutString(tempStr);
                #endif
                break;
            }
            path.checkpointDestSelected = pdTRUE;
        }
        else if ( buttonHoldTime > 6)
        {
            xTaskNotify(vTaskBatteryLevelHandle, (uint32_t)(1<<0), (eNotifyAction)eSetValueWithOverwrite );
        }
        vTaskSuspend(NULL);
    }
}


/*-----------------------------------------------------------*/
#if OBJ_DETECT_MODE == 1
void vTaskDistance(void *pvParameter)
{
    (void) pvParameter;
    
    /* start components required for ultrasonic sensors. */
    startUltrasonicSensors();
    
    const TickType_t xDelay1000ms = pdMS_TO_TICKS(1000UL);

    while(1)
    {
        distanceReading( &ultrasonicReadings );
        #if DEBUG_PRINT_MODE == 1
            sprintf(tempStr, "Distance 1 : %lf cm         Distance 2 : %lf cm           Distance 3 : %lf cm\n",
                ultrasonicReadings.distance1, ultrasonicReadings.distance2, ultrasonicReadings.distance3);
            UART_PutString(tempStr);
        #endif
        /* Task should execute every 2000 milliseconds exactly. */
        vTaskDelay( xDelay1000ms );
    }
}

/*-----------------------------------------------------------*/
void vTaskMotor(void *pvParameter)
{
    (void) pvParameter;
    
    /* start components required for motors. */
    startMotors();
    
    const TickType_t xDelay2093ms = pdMS_TO_TICKS(2093UL);
    
    while(1)
    {
        setMotors( (int)ultrasonicReadings.distance1, (int)ultrasonicReadings.distance2, (int)ultrasonicReadings.distance3);
       
        /* Task should execute every 2000 milliseconds exactly. */
        vTaskDelay( xDelay2093ms );
    }
}
#endif
/*-----------------------------------------------------------*/
/* [] END OF FILE */
