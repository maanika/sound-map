/*******************************************************************************
*                               INCLUDED HEADERS
*******************************************************************************/
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

/*******************************************************************************
*                               CONSTANT DEFINITIONS
*******************************************************************************/
#define TABLE_LENGTH 720
#define SOUND_VOLUME 1.6
#define PATH_PROXIMITY 10

#define ON();       { AMux_1_Start(); AMux_2_Start(); }
#define OFF();      { AMux_1_DisconnectAll(); AMux_2_DisconnectAll();}

// Navigation sound output - SOUND for short
#define SOUND();    { AMux_1_FastSelect(0); AMux_2_FastSelect(0); }
// Speech sythesiser output - SPEECH for short
#define SPEECH();   { AMux_1_FastSelect(1); AMux_2_FastSelect(1); } 

/*******************************************************************************
*                               TASK PRIORITIES
*******************************************************************************/
#define TASK_GPS_PRIO           (configMAX_PRIORITIES - 1)
#define TASK_PATH_PRIO          (configMAX_PRIORITIES - 3)
#define TASK_DIRECTION_PRIO     (configMAX_PRIORITIES - 4)
#define TASK_SOUND_PRIO         (configMAX_PRIORITIES - 5)
#define TASK_SPEECH_PRIO        (configMAX_PRIORITIES - 6)
#define TASK_BATTERY_LEVEL_PRIO (configMAX_PRIORITIES - 7)
#define TASK_BUTTON_PRIO        (configMAX_PRIORITIES - 2)
#define TASK_LED_PRIO           (configMAX_PRIORITIES - 10)
#if OBJ_DETECT_MODE == 1
    #define TASK_MOTOR_PRIO     (configMAX_PRIORITIES - 8)
    #define TASK_DIS_PRIO       (configMAX_PRIORITIES - 9)
#endif

/*******************************************************************************
*                               TASK STACK SIZES
*******************************************************************************/
#define TASK_GPS_STK_SIZE           500
#define TASK_SPEECH_STK_SIZE        500
#define TASK_PATH_STK_SIZE          500
#define TASK_DIRECTION_STK_SIZE     500
#define TASK_SOUND_STK_SIZE         500
#define TASK_BATTERY_LEVEL_STK_SIZE 500
#define TASK_BUTTON_STK_SIZE        500
#define TASK_LED_STK_SIZE           200
#if OBJ_DETECT_MODE == 1
    #define TASK_MOTOR_STK_SIZE     200
    #define TASK_DIS_STK_SIZE       200
#endif


/*******************************************************************************
*                               GLOBAL VARIABLES
*******************************************************************************/
/* Path planning variables */
struct Path path = {
    /* Checkpoint latitudes */      /* Checkpoint longitudes */
    .checkpointLat[0] = -37.911547, .checkpointLon[0] = 145.13335,
    .checkpointLat[1] = -37.911685, .checkpointLon[1] = 145.13398,
    .checkpointLat[2] = -37.911286, .checkpointLon[2] = 145.13415,
    .checkpointLat[3] = -37.911307, .checkpointLon[3] = 145.13442,
    .checkpointLat[4] = -37.911286, .checkpointLon[4] = 145.13415,
    .checkpointLat[5] = -37.910313, .checkpointLon[5] = 145.13432,
    .checkpointLat[6] = -37.910275, .checkpointLon[6] = 145.13333,
    .checkpointLat[7] = -37.910235, .checkpointLon[7] = 145.13211,
    .checkpointLat[8] = -37.909966, .checkpointLon[8] = 145.13219,
    .checkpointLat[9] = -37.910235, .checkpointLon[9] = 145.13211,
    .checkpointLat[10] = -37.910215, .checkpointLon[10] = 145.13167,
    .checkpointLat[11] = -37.910688, .checkpointLon[11] = 145.13158,
    .checkpointLat[12] = -37.910862, .checkpointLon[12] = 145.13277,
    .checkpointLat[13] = -37.911202, .checkpointLon[13] = 145.13298,
    .checkpointLat[14] = -37.911295, .checkpointLon[14] = 145.13339,
    .checkpointOperation = 0,   /* addition by default */
    .checkpointDestSelected = pdFALSE,
    .checkpointDestName = ' ',
    .atDestination = pdFALSE,
};

/* GPS variables */
long double latitudeInDec, longitudeInDec;
int firstFix = 0;
double direction = 0;

/* Path variables */
int nextCheckpoint = 0; // stores the next checkpoint array index

/* Sound variables */
BaseType_t soundState = pdFALSE;

/* Battery Level variables */
int batteryLevelValue = 0;

/* Variables for debugging/testing */
#if DEBUG_PRINT_MODE == 1
    /* Checkpoint name printing */
    int checkpointName[15] = { 
        0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14 
    };
    
    /* PC UART printing */
    char tempStr[100]; 
#endif

/* Variables for object detection */
#if OBJ_DETECT_MODE == 1
    ultrasonicSensor ultrasonicReadings;
#endif

/* TEMPORARY FOR TESTING */
double diffDistance;
double distFromPrevCheckpoint;
double distBetweenCheckpoints;

/*******************************************************************************
*                               TASK HANDLERS
*******************************************************************************/
TaskHandle_t vTaskGPSHandle           = NULL;
TaskHandle_t vTaskPathHandle          = NULL;
TaskHandle_t vTaskSpeechHandle        = NULL;
TaskHandle_t vTaskDirectionHandle     = NULL;
TaskHandle_t vTaskSoundHandle         = NULL;
TaskHandle_t vTaskBatteryLevelHandle  = NULL;
TaskHandle_t vTaskButtonHandle        = NULL;
TaskHandle_t vTaskLEDHandle           = NULL;
#if OBJ_DETECT_MODE == 1
    TaskHandle_t xTaskMotorHandle     = NULL;
    TaskHandle_t xTaskDistanceHandle  = NULL;
#endif

/*******************************************************************************
*                               SEMAPHORE HANDLERS
*******************************************************************************/
SemaphoreHandle_t xGPSSemaphore;
SemaphoreHandle_t xUARTMutex;
SemaphoreHandle_t xBatteryLevelMutex;

/*******************************************************************************
*                               QUEUE HANDLERS
*******************************************************************************/
QueueHandle_t xButtonTimeQueue;

/*******************************************************************************
*                             FUNCTION DECLARATIONS
*******************************************************************************/
extern void RTOS_Start( void );

/*******************************************************************************
*                               TASK DECLARATIONS
*******************************************************************************/
static void vTaskGPS            ( void *pvParameter );
static void vTaskPath           ( void *pvParameter );
static void vTaskSpeech         ( void *pvParameter );
static void vTaskDirection      ( void *pvParameter );
static void vTaskSound          ( void *pvParameter );
static void vTaskBatteryLevel   ( void *pvParameter );
static void vTaskButton         ( void *pvParameter );
static void vTaskLED            ( void *pvParameter );

#if OBJ_DETECT_MODE == 1
static void vTaskDistance       ( void *pvParameter );
static void vTaskMotor          ( void *pvParameter );
#endif

/*******************************************************************************
*                           INTERRUPT SERVICE ROUTINES
*******************************************************************************/
/* GPS ISR */
CY_ISR( ISR_GPS_Received )
{
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR( xGPSSemaphore, &xHigherPriorityTaskWoken );
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

/* Button ISR */
CY_ISR( ISR_Button )
{
    BaseType_t xHigherPriorityTaskWoken;
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

/*******************************************************************************
*                               PSOC START FUNCTION
*******************************************************************************/
void PSOC_Start( void )
{
    /* Start UART for GPS and debug printing */
    UART_Start();
    
    /* Start Button */
    Timer_1_Start();
    
    /* Start and Initialize Sound */
    startSoundComponents();
    dmaConfiguration();
    
    /* Start and Initialize Speech */
    synthInitialize();
    
    /* Start and Initialize Compass */
    compassStart();
    
    /* Start Battery Level Monitor */
    batteryLevelMonitorStart();

    /* Turn on System Output */
    ON();
}

/*******************************************************************************
*                                   MAIN
*******************************************************************************/
int main( void )
{
    CyGlobalIntEnable;                            // Enable global interrupts.
    
    /* System Initialization */
    RTOS_Start();
    PSOC_Start();
    
    SPEECH();
    sayWelocome(); sayPause();                    // Vocalize Welcome greeting
    OFF();
    
    /* Interrupts */
    isr_GPS_Received_ClearPending();              // Cancel any pending isr_RxSignal interrupts
    isr_GPS_Received_StartEx( ISR_GPS_Received ); // Enable the interrupt service routine
    
    isr_button_ClearPending();                    // Cancel any pending isr_RxSignal interrupts
    isr_button_StartEx( ISR_Button );             // Enable the interrupt service routine
    
    /* Creating Semaphores and Mutxes */
    xGPSSemaphore = xSemaphoreCreateCounting( 10, 0 );
    xUARTMutex = xSemaphoreCreateMutex();
    xBatteryLevelMutex = xSemaphoreCreateMutex();
    xButtonTimeQueue = xQueueCreate( 1, sizeof(portFLOAT) );
    
    /* Creating Tasks */
    if ( xGPSSemaphore != NULL || xUARTMutex != NULL || xButtonTimeQueue != NULL || xBatteryLevelMutex != NULL )
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
        
        /*err = xTaskCreate ( vTaskLED, "task LED ", TASK_LED_STK_SIZE, (void*) 0, TASK_LED_PRIO, &vTaskLEDHandle );
        if ( err != pdPASS ){
            #if DEBUG_PRINT_MODE == 1
                sprintf( tempStr, "Failed to Create Task LED\n" );
                UART_PutString( tempStr );
            #endif
            while(1){};
        }*/
        
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
        
        vTaskStartScheduler();   // Start the OS
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

/*******************************************************************************
*                                   GPS TASK
*******************************************************************************/
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
            //sprintf(tempStr, "longitude: %Lf    latitude: %Lf\n", longitudeInDec, latitudeInDec);
            //UART_PutString( tempStr );
        #endif
        
        if ( longitudeInDec == 0 && latitudeInDec == 0 )
        {
            xTaskNotify(vTaskSpeechHandle, (uint32_t)(1<<2)|(1<<0), (eNotifyAction)eSetValueWithOverwrite );
        }
        
        if ( firstFix == 0 && longitudeInDec != 0 && latitudeInDec != 0 && path.checkpointDestSelected == pdTRUE )
        {
            firstFix = 1; // run one time
            soundState = pdTRUE; // navigation sound on
            
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
                    sprintf(tempStr, "Failed to Create Task Direction\n");
                    UART_PutString( tempStr );
                #endif
                while(1){};
            }
            err = xTaskCreate( vTaskSound, "task sound", TASK_SOUND_STK_SIZE, (void*) 0, TASK_SOUND_PRIO, &vTaskSoundHandle );
            if ( err != pdPASS ){
                #if DEBUG_PRINT_MODE == 1
                    sprintf(tempStr, "Failed to Create Task Sound\n");
                    UART_PutString( tempStr );
                #endif
                while(1){};
            }           
        }
        vTaskDelay(xDelay40ms);
    }
}

/*******************************************************************************
*                                   PATH TASK
*******************************************************************************/
static void vTaskPath( void *pvParameter )
{
    (void) pvParameter;
    const TickType_t xDelay1000ms = pdMS_TO_TICKS(1000UL);

    while (1)
    {
        if ( path.checkpointOperation == 0 )
        {
            nextCheckpoint = path.checkpointCurrent+1;
            if ( nextCheckpoint == 15 ) { nextCheckpoint = 0; }
            
            diffDistance = distance( latitudeInDec, longitudeInDec, 
                path.checkpointLat[nextCheckpoint],path.checkpointLon[nextCheckpoint] );
            
            distFromPrevCheckpoint = distance( path.checkpointLat[path.checkpointCurrent], path.checkpointLon[path.checkpointCurrent],
                latitudeInDec, longitudeInDec );
            
            distBetweenCheckpoints = distance( path.checkpointLat[path.checkpointCurrent], path.checkpointLon[path.checkpointCurrent],
                path.checkpointLat[nextCheckpoint],path.checkpointLon[nextCheckpoint] );
            
        }
        if ( path.checkpointOperation == 1 )
        {
            nextCheckpoint = path.checkpointCurrent-1;
            if ( nextCheckpoint == -1 ) { nextCheckpoint = 14; }
            
            diffDistance = distance(latitudeInDec, longitudeInDec, 
                path.checkpointLat[nextCheckpoint],path.checkpointLon[nextCheckpoint] );
           
            distFromPrevCheckpoint = distance( path.checkpointLat[path.checkpointCurrent], path.checkpointLon[path.checkpointCurrent],
                latitudeInDec, longitudeInDec );
            
            distBetweenCheckpoints = distance( path.checkpointLat[path.checkpointCurrent], path.checkpointLon[path.checkpointCurrent],
                path.checkpointLat[nextCheckpoint],path.checkpointLon[nextCheckpoint] );
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
            if ( path.checkpointCurrent == path.checkpointDest ) //if atDestination is pdTRUE, this condition must be satisfied
            {
                /* Vocalize arrived at destination */
                xTaskNotify(vTaskSpeechHandle, (uint32_t)(1<<2)|(1<<1), (eNotifyAction)eSetValueWithOverwrite );

                /* Restart Program */
                isr_button_ClearPending();
                path.checkpointDestSelected = pdFALSE;      // no destination selected
                path.atDestination = pdFALSE;               // not at destination - reset
                firstFix = 0;                               // to create vTaskPathStart once again in vTaskGPS
                soundState = pdFALSE;                       // sound navigation off
                vTaskDelete(vTaskDirectionHandle);
                vTaskDelete(vTaskSoundHandle);
                vTaskDelete(NULL);                          // delete current task - and all others
                OFF();                                      // OFF sound output
            }
        }
        #if DEBUG_PRINT_MODE == 1
            //sprintf( tempStr, "Current Checkpoint: H%d      Next Checkpoint:    H%d\n", 
                //checkpointName[path.checkpointCurrent], checkpointName[nextCheckpoint] );
            //UART_PutString( tempStr );
            //sprintf( tempStr, "Distance to next checkpoint: %.2f \n", diffDistance );
            //UART_PutString( tempStr);
        #endif
        vTaskDelay( xDelay1000ms );
    }
}

/*******************************************************************************
*                                   SPEECH TASK
*******************************************************************************/
static void vTaskSpeech ( void *pvParameter )
{
    (void) pvParameter;
    uint32_t speechNotificationValue;
    while (1)
    {
        if ( xTaskNotifyWait((uint32_t)0, (uint32_t)0, &speechNotificationValue, portMAX_DELAY ) == pdTRUE )
        {
            #if DEBUG_PRINT_MODE == 1
                sprintf( tempStr, "Notification received: %u", speechNotificationValue );
                UART_PutString( tempStr );
            #endif
        }

        SPEECH(); // turn on speech (sound is offed automatically - turn on at end if needed)
                
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
        
        if (soundState == pdTRUE) {SOUND();} // turn navigation sound back on if destination is selected and path task running
        else {SPEECH();}
        
        // Restart the RTOS kernel.  We want to force a context switch, 
        //but there is no point if resuming the scheduler caused a context switch already.
        if( !xTaskResumeAll () )
        {
            taskYIELD ();
        }
    }
}

/*******************************************************************************
*                                   DIRECTION TASK
*******************************************************************************/
static void vTaskDirection ( void *pvParameter )
{
    (void) pvParameter;
    char tempStr[100];
    compassRaw compass;
    float Xm_off, Ym_off, Zm_off, Xm_cal, Ym_cal, Zm_cal;
    const float alpha = 0.5;
    float fXm = 0;
    float fYm = 0;
    float fZm = 0;
    double bearing, difference;
    const TickType_t xDelay250ms = pdMS_TO_TICKS(250UL);

    while(1)
    {
        /* Compass Raw data readings */
        compassRead(&compass);
        
        /* Magnetometer calibration */
        Xm_off = compass.m_x*(100000.0/1100.0) + 3190.321761; //X-axis combined bias (Non calibrated data - bias)
        Ym_off = compass.m_y*(100000.0/1100.0) + 6257.765070; //Y-axis combined bias (Default: substracting bias)
        Zm_off = compass.m_z*(100000.0/980.0 ) - 1176.670017; //Z-axis combined bias
            
        Xm_cal =  0.943220*Xm_off +0.005519*Ym_off +0.022864*Zm_off; //X-axis correction for combined scale factors (Default: positive factors)
        Ym_cal =  0.005519*Xm_off +0.920150*Ym_off +0.001051*Zm_off; //Y-axis correction for combined scale factors
        Zm_cal =  0.022864*Xm_off +0.001051*Ym_off +1.170044*Zm_off; //Z-axis correction for combined scale factor
        
        /* Low-Pass filter magnetometer */
        fXm = Xm_cal * alpha + (fXm * (1.0 - alpha));
        fYm = Ym_cal * alpha + (fYm * (1.0 - alpha));
        fZm = Zm_cal * alpha + (fZm * (1.0 - alpha));
        
        /* Calculate bearing */
        bearing = atan2(fYm,fXm);
        if (bearing < 0) bearing += 2*M_PI;
        sprintf(tempStr, " %.2f", bearing*180/M_PI);
        //UART_PutString( tempStr );
        
        /* Calculate angle between current and next checkpoint coordinates (degrees) */
        difference = GPSbearing(latitudeInDec, longitudeInDec ,
            path.checkpointLat[nextCheckpoint], path.checkpointLon[nextCheckpoint]);
        
        //sprintf(tempStr, "      Difference: %.2f",difference );
        //UART_PutString( tempStr );
        
        /* calculate the direction need to walk in to get to destination (radians) */
        if (bearing > M_PI) bearing = bearing - 2*M_PI;     // make bearing  within -M_PI to M_PI
        direction = difference*M_PI/180 - bearing;
        
        //sprintf(tempStr, "      Direction: %.3f", direction * 180/M_PI);
        //UART_PutString( tempStr );
        
        /*// TEST program for offsetAngle
        direction = direction + 2*(M_PI/180); // set angle to direction need to go in
        if (direction > M_PI) direction = -M_PI;
        // End of test program */
        
        vTaskDelay(xDelay250ms);
    }
}

/*******************************************************************************
*                                   SOUND TASK
*******************************************************************************/
static void vTaskSound ( void *pvParameter )
{
    (void) pvParameter;
    const int freqFront = 400;
    const int freqBack = 1000;
    int freq = freqFront;
    float offsetAngle = 0;
    float ITDtime, phaseDelay;
    int phaseDelayCycles;
    int leftFast, rightFast;
    float IIDattenuation;
    const TickType_t xDelay250ms = pdMS_TO_TICKS(250UL);
    int printCount = 0;
    double pathError;
    
    sineWaveInitialize(400);

    SOUND(); // sound always on as long as task path is running 
    
    while (1)
    {
        offsetAngle = direction;
        //restrict angle to range of -pi to pi
        if (offsetAngle > M_PI) offsetAngle = offsetAngle - 2*M_PI; 
        if (offsetAngle < -M_PI) offsetAngle = offsetAngle + 2*M_PI;
        
        // print every second
        if ( ( printCount%2 ) == 0 ){
            
            pathError = diffDistance*sin(
            acos( ( diffDistance*diffDistance + distBetweenCheckpoints*distBetweenCheckpoints - distFromPrevCheckpoint*distFromPrevCheckpoint ) / ( 2*diffDistance*distBetweenCheckpoints) )
            );
            
            //sprintf(tempStr, "%.2f    %.2f    %.2f\n", diffDistance, pathError,offsetAngle*180/M_PI);    
            UART_PutString(tempStr);
        }
        leftFast = offsetAngle < 0; //left is earlier then right
        rightFast = offsetAngle > 0; //right is earlier then left
        
        //set freq back or front
        if (offsetAngle >= -M_PI/2 && offsetAngle <= M_PI/2 ) freq = freqFront;
        else freq = freqBack;
        
        //sprintf(tempStr, "frequency: %d\n", freq);
        //UART_PutString(tempStr);
        
        //calculate time delay
        ITDtime = 0.0002970892271*( offsetAngle + sin(offsetAngle));
        phaseDelay = 2 * M_PI * freq * ITDtime; // in radians
        
        //sprintf(tempStr, "ITDTime: %.7f     phaseDelay: %.2f", ITDtime, phaseDelay*180/M_PI);
        //UART_PutString(tempStr);
        
        // Find offset (cycles) to delay using resolution of sinewave table.
        if (phaseDelay < 0) phaseDelay = phaseDelay * -1;
        phaseDelayCycles = (int) ( phaseDelay * 180/M_PI) / 0.5;
        //sprintf(tempStr, "      phaseDelayCycles: %d\n", phaseDelayCycles);
        //UART_PutString(tempStr);
        
        // Calculate amplitude attenuation
        IIDattenuation = ( cos( offsetAngle ) );
        if  (IIDattenuation < 0) IIDattenuation = IIDattenuation*-1;
        //sprintf(tempStr, "ILDatt: %.2f\n", IIDattenuation);
        //UART_PutString(tempStr);
        
        if(rightFast)
        {
            // implement time delay on left (1)
            // right(2) will lead while left(1) will lag
            // make left side quieter
            updateSineWave(0, IIDattenuation/SOUND_VOLUME, 1);
            updateSineWave(phaseDelayCycles, 1/SOUND_VOLUME, 2);

            // update frequency
            DDS24_1_SetFrequency((freq / 2.4965) * TABLE_LENGTH);
            
            //sprintf(tempStr, "RIGHT");
            //UART_PutString(tempStr);
        }
        else
        {
            // implement time delay on right (2)
            // left (1) will lead while right(2) will lag
            // make right side quieter
            updateSineWave(phaseDelayCycles, 1/SOUND_VOLUME, 1);
            updateSineWave(0, IIDattenuation/SOUND_VOLUME, 2);
            
            // update frequency
            DDS24_1_SetFrequency((freq / 2.4965) * TABLE_LENGTH);   
            //sprintf(tempStr, "LEFT");
            //UART_PutString(tempStr);
        }
        vTaskDelay(xDelay250ms);
    }
}

/*******************************************************************************
*                                 BATTERY LEVEL TASK
*******************************************************************************/
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
            xSemaphoreTake( xBatteryLevelMutex, portMAX_DELAY );
            {
                batteryLevelValue = readBatteryLevel();
            }
            xSemaphoreGive( xBatteryLevelMutex );
            xTaskNotify(vTaskSpeechHandle, (uint32_t)(1<<2), (eNotifyAction)eSetValueWithOverwrite );
        }
    }
}

/*******************************************************************************
*                                  BUTTON TASK
*******************************************************************************/
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

/*******************************************************************************
*                               LED TASK
*******************************************************************************/
static void vTaskLED( void *pvParameter )
{
    (void) pvParameter;
    const TickType_t xDelay2500ms = pdMS_TO_TICKS(2500UL);
    TickType_t xDelayIndicator;
    int batteryLevel;
    while (1)
    {
        xSemaphoreTake( xBatteryLevelMutex, portMAX_DELAY );
        {
            batteryLevel = readBatteryLevel();
        }
        xSemaphoreGive( xBatteryLevelMutex );
        if (batteryLevel > 40)  xDelayIndicator = pdMS_TO_TICKS(10000UL);
        else xDelayIndicator = xDelay2500ms;
        Pin_LED_Write(1);
        vTaskDelay( xDelay2500ms );
        Pin_LED_Write(0);
        vTaskDelay( xDelayIndicator );
    }
}

/*******************************************************************************
*                           DISTANCE AND MOTOR TASK
*******************************************************************************/
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

/* [] END OF FILE */
