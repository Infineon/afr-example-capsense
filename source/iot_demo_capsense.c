/******************************************************************************
* File Name: iot_demo_capsense.c
*
* Description: This code example features a 5-segment CapSense slider and two
*              CapSense buttons. The status of the buttons and slider are stored
*              in an AWS thing shadow. The same publishing device also subscribes
*              to its own shadow updates and controls the LED on the kit depending
*              on the CapSense status received. Button 0 turns the LED ON,
*              Button 1 turns the LED OFF, and the slider controls
*              the brightness of the LED. The code example also features
*              interfacing with Tuner GUI using I2C interface.
*
* Related Document: README.md
*
*******************************************************************************
* Copyright 2019-2021, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#include "cyhal.h"
#include "cybsp.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "iot_mqtt.h"
#include "iot_wifi.h"
#include "aws_demo.h"
#include "mqtt_operation.h"
#include "capsense_operation.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/
/* Priorities of user tasks in this demo */
#define TASK_CAPSENSE_PRIORITY      (tskIDLE_PRIORITY + 2)
#define TASK_MQTT_PRIORITY          (tskIDLE_PRIORITY + 1)

/* Stack sizes of user tasks in this demo */
#define TASK_CAPSENSE_STACK_SIZE    (configMINIMAL_STACK_SIZE * 4)
#define TASK_MQTT_STACK_SIZE        (configMINIMAL_STACK_SIZE * 16)

/* MQTT Topics to Subscribe */
#define TOPIC_SHADOW_UPDATED        "$aws/things/capsense/shadow/update/documents"
#define TOPIC_SHADOW_GET_ACCEPTED   "$aws/things/capsense/shadow/get/accepted"

/*******************************************************************************
 * Global variable
 ******************************************************************************/
QueueHandle_t capsense_command_q;

/* Handle of the MQTT connection used in this demo. */
IotMqttConnection_t mqtt_connection = IOT_MQTT_CONNECTION_INITIALIZER;

/*******************************************************************************
 * Function forward declarations
 ******************************************************************************/
int run_application( bool aws_iot_mqtt_mode,
                 const char * p_identifier,
                 void * p_network_server_info,
                 void * p_network_credential_info,
                 const IotNetworkInterface_t * p_network_interface );

/*******************************************************************************
* Function Name: init_application
********************************************************************************
* Summary:
*  Initializes the tasks required to run the application.
*
*******************************************************************************/
void init_application(void)
{
    static demoContext_t appContext =
    {
        .networkTypes                = AWSIOT_NETWORK_TYPE_WIFI,
        .demoFunction                = run_application,
        .networkConnectedCallback    = NULL,
        .networkDisconnectedCallback = NULL
    };

    Iot_CreateDetachedThread( runDemoTask,
                              &appContext,
                              tskIDLE_PRIORITY,
                              (configMINIMAL_STACK_SIZE * 8) );
}

/*******************************************************************************
* Function Name: run_application
********************************************************************************
* Summary:
*  This thread starts all the necessary threads and does the required
*  configurations for the demo. It is called by runDemoTask().
*
* Parameters:
*  bool aws_iot_mqtt_mode          : Specify if this demo is running with the AWS
*                                    IoT MQTT server. Set this to `false` if using 
*                                    another MQTT server.
*  const char *p_identifier        : p_identifier NULL-terminated MQTT client identifier.
*  void *p_network_server_info     : Passed to the MQTT connect function when
*                                    establishing the MQTT connection.
*  void *p_network_credential_info : Passed to the MQTT connect function when
*                                    establishing the MQTT connection.
*  const IotNetworkInterface_t * p_network_interface : The network interface to use 
*                                                    for this demo
*******************************************************************************/
int run_application(bool aws_iot_mqtt_mode,
                const char * p_identifier,
                void * p_network_server_info,
                void * p_network_credential_info,
                const IotNetworkInterface_t * p_network_interface )
{
    /* Create the queue. See the data-type for details of queue
     * contents
     */
    capsense_command_q  = xQueueCreate(SINGLE_ELEMENT_QUEUE,
                                       sizeof(capsense_command_t));

    /* Topics used as both topic filters and topic names in this demo. */
    const char * pTopics[ TOPIC_FILTER_COUNT ] =
    {
        TOPIC_SHADOW_UPDATED,
        TOPIC_SHADOW_GET_ACCEPTED
    };

    /* Length of topic names as per pTopics array */
    const uint16_t pTopicsSize[ TOPIC_FILTER_COUNT ] =
    {
        (uint16_t)strlen(TOPIC_SHADOW_UPDATED),
        (uint16_t)strlen(TOPIC_SHADOW_GET_ACCEPTED)
    };

    /* Initialize the MQTT library required for this demo. */
    initialize_mqtt();

    /* Establish a new MQTT connection. */
    establish_mqtt_connection( aws_iot_mqtt_mode,
                               p_identifier,
                               p_network_server_info,
                               p_network_credential_info,
                               p_network_interface,
                               &mqtt_connection );

    /* Add the topic filter subscriptions used in this demo. */
    modify_subscriptions( mqtt_connection,
                          IOT_MQTT_SUBSCRIBE,
                          pTopics,
                          pTopicsSize,
                          TOPIC_FILTER_COUNT,
                          NULL );

    /* Create the user tasks. See the respective task definition for more
     * details of these tasks.
     */
    xTaskCreate(task_mqtt, "task_mqtt", TASK_MQTT_STACK_SIZE,
                NULL, TASK_MQTT_PRIORITY, NULL);
    xTaskCreate(task_capsense, "CapSense Task", TASK_CAPSENSE_STACK_SIZE,
                NULL, TASK_CAPSENSE_PRIORITY, NULL);

    /* Suspend this task to prevent network connection resources from being cleaned up */
    vTaskSuspend( NULL );

    return 0;
}

/* [] END OF FILE */
