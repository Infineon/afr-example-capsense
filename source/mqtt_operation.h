/******************************************************************************
* File Name: mqtt_operation.h
*
* Description: This file contains function declarations related to MQTT
* operations.
*
*******************************************************************************
* Copyright 2018-2021, Cypress Semiconductor Corporation (an Infineon company) or
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
#ifndef MQTT_OPERATION_H_
#define MQTT_OPERATION_H_

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "iot_mqtt.h"
/*******************************************************************************
 * Defines
 ******************************************************************************/
/* Allowed TCPWM compare value for maximum brightness */
#define LED_MAX_BRIGHTNESS          (100u)

/*******************************************************************************
 * Data structure and enumeration
 ******************************************************************************/
/* Available LED commands */
typedef enum
{
    LED_TURN_ON,
    LED_TURN_OFF,
    LED_UPDATE_BRIGHTNESS,
    GET_LED_STATE
} led_command_t;

/* Structure used for storing MQTT data */
typedef struct
{
    led_command_t command;
    uint32_t brightness;
} mqtt_data_t;

/*******************************************************************************
 * Global variable
 ******************************************************************************/
extern QueueHandle_t led_command_data_q;
extern IotMqttConnection_t mqtt_connection;

/* MQTT Broker info */
#define KEEP_ALIVE_SECONDS          (60)
#define MQTT_TIMEOUT_MS             (5000)
#define TOPIC_FILTER_COUNT          (2)
#define PUBLISH_RETRY_LIMIT         (10)
#define PUBLISH_RETRY_MS            (1000)
#define MAX_JSON_MESSAGE_LENGTH     (100)
#define MAX_TOPIC_LENGTH            (50)

/* Queue length of message queues used in this project */
#define SINGLE_ELEMENT_QUEUE        (1u)

/*******************************************************************************
 * MQTT functions
 ******************************************************************************/
int initialize_mqtt(void);

int establish_mqtt_connection( bool aws_iot_mqtt_mode,
                               const char * p_identifier,
                               void * p_network_server_info,
                               void * p_network_credential_info,
                               const IotNetworkInterface_t * p_network_interface,
                               IotMqttConnection_t * p_mqtt_connection );

int modify_subscriptions( IotMqttConnection_t mqtt_connection,
                          IotMqttOperationType_t operation,
                          const char ** p_topic_filters,
                          const uint16_t *p_topic_filter_length,
                          const uint32_t filter_count,
                          void * p_callback_parameter );

int publish_message( IotMqttConnection_t mqtt_connection,
                     char* topic,
                     uint16_t topic_length,
                     char* mqtt_message,
                     uint16_t message_length);

void mqtt_subscription_callback( void * param1,
                                 IotMqttCallbackParam_t * const p_publish );

/* Thread function */
void task_mqtt(void* param);

#endif /* MQTT_OPERATION_H_ */
