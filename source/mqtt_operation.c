/******************************************************************************
* File Name: mqtt_operation.c
*
* Description: This file contains threads, functions, and other resources related
* to MQTT operations.
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
#include "aws_demo.h"
#include "cJSON/cJSON.h"
#include "mqtt_operation.h"

/* Set up logging for this demo. */
#include "iot_demo_logging.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/
#define MAX_DUTY_CYCLE              (100)

/* Subtracting from MAX_DUTY_CYCLE since the LED is connected in active low configuration */
#define GET_DUTY_CYCLE(x)           (MAX_DUTY_CYCLE - x)

/* PWM frequency used to drive LED */
#define PWM_LED_FREQ_HZ             (1000000u)

/* Thread delays */
#define MQTT_THREAD_LOOP_DELAY_MS   (100)

/* MQTT Publish Topics */
#define TOPIC_SHADOW_UPDATE         "$aws/things/capsense/shadow/update"
#define TOPIC_SHADOW_GET            "$aws/things/capsense/shadow/get"

/*******************************************************************************
 * Global variable
 ******************************************************************************/
QueueHandle_t led_command_data_q;

cyhal_pwm_t pwm_led;
bool led_on = false;

/*******************************************************************************
 * Forward Declaration
 ******************************************************************************/
void update_led_brightness(uint32_t brightness);
void update_led_state(const char* command);

/*******************************************************************************
* Function Name: task_capsense
********************************************************************************
* Summary:
*  This task receives command from CapSense Task and publishes the state of
*  buttons/slider to an AWS thing shadow
*
* Parameters:
*  void *param : Task parameter defined during task creation (unused)
*
*******************************************************************************/
void task_mqtt(void* param)
{
    /* Suppress warning for unused parameter */
    (void)param;

    mqtt_data_t mqtt_cmd_data;  /* Queue to receive command from CapSense task */
    char json[MAX_JSON_MESSAGE_LENGTH]; /* JSON message to send */
    char topic[MAX_TOPIC_LENGTH];       /* Buffer for topic name */
    uint16_t topic_length = 0;   /* Length of topic */
    uint16_t message_length = 0; /* Length of JSON message */

    /* Configure the TCPWM for driving led */
    cyhal_pwm_init(&pwm_led, CYBSP_USER_LED, NULL);

    /* Create the queue. See the data-type for details of queue
     * contents
     */
    led_command_data_q  = xQueueCreate(SINGLE_ELEMENT_QUEUE,
                                       sizeof(mqtt_data_t));

    /* Send command to get status of LED from the device shadow */
    mqtt_cmd_data.command = GET_LED_STATE;
    xQueueOverwrite(led_command_data_q, &mqtt_cmd_data);

    while (1)
    {
        /* Block until a command has been received over queue */
        xQueueReceive(led_command_data_q, &mqtt_cmd_data, portMAX_DELAY);

        topic_length = snprintf(topic, sizeof(topic), TOPIC_SHADOW_UPDATE);

        switch(mqtt_cmd_data.command)
        {
            /* Turn on the LED. */
            case LED_TURN_ON:
            {
                message_length = snprintf(json, sizeof(json), "{\"state\" : {\"reported\" : {\"Command\": \"LED ON\"}}}");
                break;
            }
            /* Turn off LED */
            case LED_TURN_OFF:
            {
                message_length = snprintf(json, sizeof(json), "{\"state\" : {\"reported\" : {\"Command\": \"LED OFF\"}}}");
                break;
            }
            /* Update LED brightness */
            case LED_UPDATE_BRIGHTNESS:
            {
                message_length = snprintf(json, sizeof(json), "{\"state\" : {\"reported\" : {\"Brightness\": %d}}}", (int)mqtt_cmd_data.brightness);
                break;
            }
            /* Send empty message and set topic to get shadow status */
            case GET_LED_STATE:
            {
                message_length = snprintf(json, sizeof(json), "{}");
                topic_length = snprintf(topic, sizeof(topic), TOPIC_SHADOW_GET);
                break;
            }
            /* Invalid command */
            default:
            {
                /* Handle invalid command here */
                break;
            }
        }

        /* PUBLISH (and wait) for all messages. */
        publish_message( mqtt_connection,
                         topic,
                         topic_length,
                         json,
                         message_length);

        vTaskDelay(pdMS_TO_TICKS(MQTT_THREAD_LOOP_DELAY_MS));
    }
}


/*******************************************************************************
* Function Name: initialize_mqtt
********************************************************************************
* Summary:
*  This function initialize MQTT library.
*
*******************************************************************************/
int initialize_mqtt( void )
{
    int status = EXIT_SUCCESS;
    IotMqttError_t mqtt_init_status = IOT_MQTT_SUCCESS;

    mqtt_init_status = IotMqtt_Init();

    if( mqtt_init_status != IOT_MQTT_SUCCESS )
    {
        /* Failed to initialize MQTT library. */
        configPRINTF(("Failed to initialize MQTT library\r\n"));
        status = EXIT_FAILURE;
    }
    return status;
}

/*******************************************************************************
* Function Name: establish_mqtt_connection
********************************************************************************
* Summary:
*  Establish a new connection to the MQTT server.
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
*                                                      for this demo
*  IotMqttConnection_t * p_mqtt_connection : Set to the handle to the new MQTT connection.
*
* Return:
*  int status : `EXIT_SUCCESS` if the connection is successfully established; `EXIT_FAILURE`
*  otherwise.
*
*******************************************************************************/
int establish_mqtt_connection( bool aws_iot_mqtt_mode,
                               const char * p_identifier,
                               void * p_network_server_info,
                               void * p_network_credential_info,
                               const IotNetworkInterface_t * p_network_interface,
                               IotMqttConnection_t * p_mqtt_connection )
{
    int status = EXIT_SUCCESS;
    IotMqttError_t connect_status = IOT_MQTT_STATUS_PENDING;
    IotMqttNetworkInfo_t network_info = IOT_MQTT_NETWORK_INFO_INITIALIZER;
    IotMqttConnectInfo_t connect_info = IOT_MQTT_CONNECT_INFO_INITIALIZER;

    /*
     * Set the members of the network info not set by the initializer. This
     * struct provided information on the transport layer to the MQTT connection.
     */
    network_info.createNetworkConnection = true;
    network_info.u.setup.pNetworkServerInfo = p_network_server_info;
    network_info.u.setup.pNetworkCredentialInfo = p_network_credential_info;
    network_info.pNetworkInterface = p_network_interface;

    #if ( IOT_MQTT_ENABLE_SERIALIZER_OVERRIDES == 1 ) && defined( IOT_DEMO_MQTT_SERIALIZER )
        network_info.pMqttSerializer = IOT_DEMO_MQTT_SERIALIZER;
    #endif

    /* Set the members of the connection info not set by the initializer. */
    connect_info.awsIotMqttMode = aws_iot_mqtt_mode;
    connect_info.cleanSession = true;
    connect_info.keepAliveSeconds = KEEP_ALIVE_SECONDS;
    connect_info.pWillInfo = NULL;

    /* Use the parameter client identifier provided. */
    if( p_identifier != NULL )
    {
        connect_info.pClientIdentifier = p_identifier;
        connect_info.clientIdentifierLength = ( uint16_t ) strlen( p_identifier );
    }

    connect_status = IotMqtt_Connect( &network_info,
                                      &connect_info,
                                      MQTT_TIMEOUT_MS,
                                      p_mqtt_connection );

    if( connect_status != IOT_MQTT_SUCCESS )
    {
        status = EXIT_FAILURE;
    }

    return status;
}

/*******************************************************************************
* Function Name: modify_subscriptions
********************************************************************************
* Summary:
*  Establish a new connection to the MQTT server.
*
* Parameters:
*  IotMqttConnection_t mqtt_connection   : The MQTT connection to use for subscriptions.
*  IotMqttOperationType_t operation      : Either #IOT_MQTT_SUBSCRIBE or #IOT_MQTT_UNSUBSCRIBE.
*  const char ** p_topic_filters         : Array of topic filters for subscriptions.
*  const uint16_t *p_topic_filter_length : Array of length of topic filters.
*  const uint32_t filter_count           : Number of topic filters. 
*  void * p_callback_parameter           : The parameter to pass to the subscription
*                                          callback.
*
* Return:
*  int status : `EXIT_SUCCESS` if the connection is successfully established; `EXIT_FAILURE`
*  otherwise.
*
*******************************************************************************/
int modify_subscriptions( IotMqttConnection_t mqtt_connection,
                          IotMqttOperationType_t operation,
                          const char ** p_topic_filters,
                          const uint16_t *p_topic_filter_length,
                          const uint32_t filter_count,
                          void * p_callback_parameter )
{
    int status = EXIT_SUCCESS;
    uint32_t i = 0;
    IotMqttError_t subscription_status = IOT_MQTT_STATUS_PENDING;
    IotMqttSubscription_t p_subscriptions[ filter_count ];

    /* Set the members of the subscription list. */
    for( i = 0; i < filter_count; i++ )
    {
        p_subscriptions[ i ].qos = IOT_MQTT_QOS_1;
        p_subscriptions[ i ].pTopicFilter = p_topic_filters[ i ];
        p_subscriptions[ i ].topicFilterLength = p_topic_filter_length[ i ];
        p_subscriptions[ i ].callback.pCallbackContext = p_callback_parameter;
        p_subscriptions[ i ].callback.function = mqtt_subscription_callback;
    }

    /* Modify subscriptions by either subscribing or unsubscribing. */
    if( operation == IOT_MQTT_SUBSCRIBE )
    {
        subscription_status = IotMqtt_TimedSubscribe( mqtt_connection,
                                                      p_subscriptions,
                                                      filter_count,
                                                      0,
                                                      MQTT_TIMEOUT_MS );

        /* Check the status of SUBSCRIBE. */
        switch( subscription_status )
        {
            case IOT_MQTT_SUCCESS:
                IotLogInfo( "All demo topic filter subscriptions accepted.\r\n");
                break;

            case IOT_MQTT_SERVER_REFUSED:

                /* Check which subscriptions were rejected before exiting the demo. */
                for( i = 0; i < filter_count; i++ )
                {
                    if( IotMqtt_IsSubscribed( mqtt_connection,
                                              p_subscriptions[ i ].pTopicFilter,
                                              p_subscriptions[ i ].topicFilterLength,
                                              NULL ) == true )
                    {
                        IotLogInfo("Topic filter %.*s was accepted.\r\n",
                                    p_subscriptions[ i ].topicFilterLength,
                                    p_subscriptions[ i ].pTopicFilter);
                    }
                    else
                    {
                        IotLogInfo("Topic filter %.*s was rejected.\r\n",
                                    p_subscriptions[ i ].topicFilterLength,
                                    p_subscriptions[ i ].pTopicFilter);
                    }
                }

                status = EXIT_FAILURE;
                break;

            default:

                status = EXIT_FAILURE;
                break;
        }
    }
    else if( operation == IOT_MQTT_UNSUBSCRIBE )
    {
        subscription_status = IotMqtt_TimedUnsubscribe( mqtt_connection,
                                                        p_subscriptions,
                                                        filter_count,
                                                        0,
                                                        MQTT_TIMEOUT_MS );

        /* Check the status of UNSUBSCRIBE. */
        if( subscription_status != IOT_MQTT_SUCCESS )
        {
            status = EXIT_FAILURE;
        }
    }
    else
    {
        /* Only SUBSCRIBE and UNSUBSCRIBE are valid for modifying subscriptions. */
        IotLogInfo("MQTT operation %s is not valid for modifying subscriptions.\r\n",
                    IotMqtt_OperationType( operation ) );

        status = EXIT_FAILURE;
    }
    return status;
}

/*******************************************************************************
* Function Name: publish_message
********************************************************************************
* Summary:
*  Publish MQTT Message.
*
* Parameters:
*  IotMqttConnection_t mqtt_connection : The MQTT connection to use for publishing.
*  char* topic                         : Topic name for publishing.
*  uint16_t topic_length               : Topic name length.
*  char* mqtt_message                  : Message for publishing.
*  uint16_t message_length             : Message length. 
*
* Return:
*  int status : `EXIT_SUCCESS` if all messages are published and received; 
*               `EXIT_FAILURE` otherwise.
*
*******************************************************************************/
int publish_message( IotMqttConnection_t mqtt_connection,
                     char* topic,
                     uint16_t topic_length,
                     char* mqtt_message,
                     uint16_t message_length)
{
    int status = EXIT_SUCCESS;
    IotMqttError_t publish_status = IOT_MQTT_STATUS_PENDING;
    IotMqttPublishInfo_t publish_info = IOT_MQTT_PUBLISH_INFO_INITIALIZER;
    IotMqttCallbackInfo_t publish_complete = IOT_MQTT_CALLBACK_INFO_INITIALIZER;

    /* The MQTT library should invoke this callback when a PUBLISH message
     * is successfully transmitted. Assign NULL to not attach a callback */
    publish_complete.function = NULL;

    /* Pass the PUBLISH number to the operation complete callback.
       Assigned NULL since callback is not attached */
    publish_complete.pCallbackContext = NULL;

    /* Set the common members of the publish info. */
    publish_info.qos = IOT_MQTT_QOS_1;
    publish_info.topicNameLength = topic_length;
    publish_info.pPayload = mqtt_message;
    publish_info.retryMs = PUBLISH_RETRY_MS;
    publish_info.retryLimit = PUBLISH_RETRY_LIMIT;
    publish_info.payloadLength = ( size_t ) message_length;
    publish_info.pTopicName = topic;

    /* PUBLISH a message */
    publish_status = IotMqtt_Publish( mqtt_connection,
                                      &publish_info,
                                      0,
                                      &publish_complete,
                                      NULL );

    if( publish_status != IOT_MQTT_STATUS_PENDING )
    {
        status = EXIT_FAILURE;
    }
    return status;
}

/*******************************************************************************
* Function Name: mqtt_subscription_callback
********************************************************************************
* Summary:
*  Called by the MQTT library when an incoming PUBLISH message is received.
*  The demo uses this callback to handle incoming PUBLISH messages.
*
* Parameters:
*  void * param1 : Counts the total number of received PUBLISH messages. This
*                  callback will increment this counter.
*  IotMqttCallbackParam_t * const p_publish : TInformation about the incoming
*                                             PUBLISH message passed by the
*                                             MQTT library.
*
*******************************************************************************/
void mqtt_subscription_callback( void * param1,
                                 IotMqttCallbackParam_t * const p_publish )
{
    ( void )param1;             /* Suppress compiler warning */
    char topic_str[MAX_TOPIC_LENGTH] = {0};    /* String to copy the topic into */
    char pub_type[20] =  {0};    /* String to compare to the publish type */

    /* Copy the message to a null terminated string */
    memcpy(topic_str, p_publish->u.message.info.pTopicName, p_publish->u.message.info.topicNameLength);
    topic_str[p_publish->u.message.info.topicNameLength+1] = 0; /* Add termination */

    /* Copy the message to a null terminated string */
    char *pPayload = (char*)pvPortMalloc(p_publish->u.message.info.payloadLength + 1);
    memcpy(pPayload, p_publish->u.message.info.pPayload, p_publish->u.message.info.payloadLength);
    topic_str[p_publish->u.message.info.topicNameLength] = 0; /* Add termination */


    /* Scan the topic to extract publish type */
    sscanf(topic_str, "$aws/things/capsense/shadow/%19s", pub_type);

    /* Check to see if it is an initial get of the values of other things */
    if(strcmp(pub_type,"get/accepted") == 0)
    {
        /* Parse JSON message */
        cJSON *root = cJSON_Parse(pPayload);
        cJSON *state = cJSON_GetObjectItem(root,"state");
        cJSON *reported = cJSON_GetObjectItem(state,"reported");

        if(reported == NULL)
        {
            /* If there is no previous shadow state, set LED to max brightness and start PWM */
            update_led_state("LED ON");
            update_led_brightness(MAX_DUTY_CYCLE);
        }
        else
        {
            cJSON *command = cJSON_GetObjectItem(reported,"Command");
            cJSON *brightness = cJSON_GetObjectItem(reported,"Brightness");

            if(command != NULL)
            {
                update_led_state(command->valuestring);
            }
            if(brightness != NULL)
            {
                update_led_brightness(brightness->valueint);
            }
        }
        cJSON_Delete(root);
    }

    if(strcmp(pub_type,"update/documents") == 0)
    {
        /* Parse JSON message */
        cJSON *root = cJSON_Parse(pPayload);
        cJSON *current = cJSON_GetObjectItem(root,"current");
        cJSON *state = cJSON_GetObjectItem(current,"state");
        cJSON *reported = cJSON_GetObjectItem(state,"reported");
        cJSON *command = cJSON_GetObjectItem(reported,"Command");
        cJSON *brightness = cJSON_GetObjectItem(reported,"Brightness");

        if(command != NULL)
        {
            update_led_state(command->valuestring);
        }
        if(brightness != NULL)
        {
            update_led_brightness(brightness->valueint);
        }
        cJSON_Delete(root);
    }
    vPortFree(pPayload);
}

/*******************************************************************************
* Function Name: update_led_state
********************************************************************************
* Summary:
*  This function turn ON/OFF LED.
*
* Parameters:
*  const char* command : command to either turn on (LED ON) or turn off (LED OFF) 
*                        the user LED 
*******************************************************************************/
void update_led_state(const char* command)
{
    if(0 == strcmp(command, "LED ON"))
    {
        if (!led_on)
        {
            /* Start PWM to turn the LED on */
            cyhal_pwm_start(&pwm_led);
            led_on = true;
        }
    }
    else if(0 == strcmp(command, "LED OFF"))
    {
        if(led_on)
        {
            /* Stop PWM to turn the LED off */
            cyhal_pwm_stop(&pwm_led);
            led_on = false;
        }
    }

}

/*******************************************************************************
* Function Name: update_led_state
********************************************************************************
* Summary:
*  Change LED Brightness.
*
* Parameters:
*  uint32_t brightness : Drive the LED with brightness value. 
*******************************************************************************/
void update_led_brightness(uint32_t brightness)
{
    cyhal_pwm_set_duty_cycle(&pwm_led, GET_DUTY_CYCLE(brightness), PWM_LED_FREQ_HZ);
}

/* [] END OF FILE */
