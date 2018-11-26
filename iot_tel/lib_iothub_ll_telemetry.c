
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "iothub_client_options.h"

#include "azure_c_shared_utility/threadapi.h"
#include "azure_c_shared_utility/crt_abstractions.h"
#include "azure_c_shared_utility/shared_util_options.h"

// DW ADDED
#include "lib_iothub_ll_telemetry.h"
#include "../str2str/rtklib.h"

#ifdef SET_TRUSTED_CERT_IN_SAMPLES
#include "certs.h"
#endif // SET_TRUSTED_CERT_IN_SAMPLES

//
#define RTK_MQTT

#ifdef RTK_MQTT
    #include "iothubtransportmqtt.h"
#endif // RTK_MQTT

#ifdef SET_TRUSTED_CERT_IN_SAMPLES
#include "certs.h"
#endif // SET_TRUSTED_CERT_IN_SAMPLES

/* Paste in the your iothub connection string  */
/*static const char* connectionString = "HostName=IMU.azure-devices.net;DeviceId=RaspberryPi3_Wuxi;SharedAccessKey=kdFC/6Li9E9BsRt/oc0uRlLfW0hkcqm9RAo2nXZniFo=";*/

/*#define MESSAGE_COUNT        5*/
static size_t g_message_count_send_confirmations = 0;

static rtcm_t rtcm;


static IOTHUBMESSAGE_DISPOSITION_RESULT receive_msg_callback(IOTHUB_MESSAGE_HANDLE message, void* user_context)
{
    (void)user_context;
    const char* messageId;
    const char* correlationId;

    // Message properties
    if ((messageId = IoTHubMessage_GetMessageId(message)) == NULL)
    {
        messageId = "<unavailable>";
    }

    if ((correlationId = IoTHubMessage_GetCorrelationId(message)) == NULL)
    {
        correlationId = "<unavailable>";
    }

    IOTHUBMESSAGE_CONTENT_TYPE content_type = IoTHubMessage_GetContentType(message);
    if (content_type == IOTHUBMESSAGE_BYTEARRAY)
    {
        const unsigned char* buff_msg;
        size_t buff_len;

        if (IoTHubMessage_GetByteArray(message, &buff_msg, &buff_len) != IOTHUB_MESSAGE_OK)
        {
            (void)printf("Failure retrieving byte array message\r\n");
        }
        else
        {
            (void)printf("Received Binary message\r\nMessage ID: %s\r\n Correlation ID: %s\r\n Data: <<<%.*s>>> & Size=%d\r\n", messageId, correlationId, (int)buff_len, buff_msg, (int)buff_len);
        }
    }
    else
    {
        const char* string_msg = IoTHubMessage_GetString(message);
        if (string_msg == NULL)
        {
            (void)printf("Failure retrieving byte array message\r\n");
        }
        else
        {
            (void)printf("Received String Message\r\nMessage ID: %s\r\n Correlation ID: %s\r\n Data: <<<%s>>>\r\n", messageId, correlationId, string_msg);
        }
    }
    return IOTHUBMESSAGE_ACCEPTED;
}



static void send_confirm_callback(IOTHUB_CLIENT_CONFIRMATION_RESULT result, void* userContextCallback)
{
    (void)userContextCallback;
    // When a message is sent this callback will get envoked
    g_message_count_send_confirmations++;
    (void)printf("Confirmation callback received for message %zu with result %s\r\n", g_message_count_send_confirmations, ENUM_TO_STRING(IOTHUB_CLIENT_CONFIRMATION_RESULT, result));
}

static void connection_status_callback(IOTHUB_CLIENT_CONNECTION_STATUS result, IOTHUB_CLIENT_CONNECTION_STATUS_REASON reason, void* user_context)
{
    (void)reason;
    (void)user_context;
    // This sample DOES NOT take into consideration network outages.
    if (result == IOTHUB_CLIENT_CONNECTION_AUTHENTICATED)
    {
        (void)printf("The device client is connected to iothub\r\n");
    }
    else
    {
        (void)printf("The device client has been disconnected\r\n");
    }
}



int init_iothub(iothub_api_t *iothub_api)
{
    IOTHUB_CLIENT_TRANSPORT_PROVIDER protocol;
    /*IOTHUB_MESSAGE_HANDLE message_handle;*/
	/*size_t messages_sent = 0;*/
    /*const char* telemetry_msg = "test_message";*/

    // Select the Protocol to use with the connection
#ifdef RTK_MQTT
	protocol = MQTT_Protocol;
#endif // RTK_MQTT

    // Used to initialize IoTHub SDK subsystem
    (void)IoTHub_Init();


    (void)printf("Creating IoTHub Device handle\r\n");
    // Create the iothub handle here
    iothub_api->device_ll_handle = IoTHubDeviceClient_LL_CreateFromConnectionString(iothub_api->connectionString, protocol);

	IOTHUB_DEVICE_CLIENT_LL_HANDLE device_ll_handle = iothub_api->device_ll_handle;
    if (device_ll_handle == NULL)
    {
        (void)printf("Failure createing Iothub device.  Hint: Check you connection string.\r\n");
    }
    else
    {
        // Set any option that are neccessary.
        // For available options please see the iothub_sdk_options.md documentation

        /*bool traceOn = true;*/
        /*IoTHubDeviceClient_LL_SetOption(device_ll_handle, OPTION_LOG_TRACE, &traceOn);*/

#ifdef SET_TRUSTED_CERT_IN_SAMPLES
        // Setting the Trusted Certificate.  This is only necessary on system with without
        // built in certificate stores.
		IoTHubDeviceClient_LL_SetOption(device_ll_handle, OPTION_TRUSTED_CERT, certificates);
#endif // SET_TRUSTED_CERT_IN_SAMPLES

#if defined RTK_MQTT || defined RTK_MQTT_WS
        //Setting the auto URL Encoder (recommended for MQTT). Please use this option unless
        //you are URL Encoding inputs yourself.
        //ONLY valid for use with MQTT
        //bool urlEncodeOn = true;
        //IoTHubDeviceClient_LL_SetOption(iothub_ll_handle, OPTION_AUTO_URL_ENCODE_DECODE, &urlEncodeOn);
#endif
		// Setting message callback to get C2D messages
        (void)IoTHubDeviceClient_SetMessageCallback(device_ll_handle, receive_msg_callback, NULL);

        // Setting connection status callback to get indication of connection to iothub
        (void)IoTHubDeviceClient_LL_SetConnectionStatusCallback(device_ll_handle, connection_status_callback, NULL);
		
		return 1;
	}

	return 0;
}


int ll_send_telemetry(IOTHUB_DEVICE_CLIENT_LL_HANDLE device_ll_handle, unsigned char *msgText, size_t n)
{

    IOTHUB_MESSAGE_HANDLE message_handle;

	// Construct the iothub message from a string or a byte array
	/*message_handle = IoTHubMessage_CreateFromString(telemetry_msg);*/
	message_handle = IoTHubMessage_CreateFromByteArray((const unsigned char*)msgText, n);

	// Set Message property
	/*(void)IoTHubMessage_SetMessageId(message_handle, "MSG_ID");
	(void)IoTHubMessage_SetCorrelationId(message_handle, "CORE_ID");
	(void)IoTHubMessage_SetContentTypeSystemProperty(message_handle, "application%2fjson");
	(void)IoTHubMessage_SetContentEncodingSystemProperty(message_handle, "utf-8");*/

	// Add custom properties to message
	(void)IoTHubMessage_SetProperty(message_handle, "property_key", "property_value");

	/*(void)printf("Sending message %d to IoTHub\r\n", (int)(messages_sent + 1));*/
	(void)printf("Sending message to IoTHub...\n");
	IoTHubDeviceClient_LL_SendEventAsync(device_ll_handle, message_handle, send_confirm_callback, NULL);
	printf("Telemetry message with %zu bytes sending done.\n", n);

	// The message is copied to the sdk so the we can destroy it
	IoTHubMessage_Destroy(message_handle);

	/*messages_sent++;*/

	IoTHubDeviceClient_LL_DoWork(device_ll_handle);
	ThreadAPI_Sleep(1000);

	return 1;
}


iothub_api_t *openiothub(const char *connect_string, char *msg)
{
	/*sprintf(msg, "Opening stream to IoT Hub.\n");	*/
	
	iothub_api_t *iot_device = (iothub_api_t *)malloc(sizeof(iothub_api_t));
	if (!iot_device) return NULL;

	if (!init_rtcm(&rtcm)) {
		free(iot_device);
		return NULL;
	}

	strcpy(iot_device->connectionString, connect_string);
	if(init_iothub(iot_device))
		return iot_device;
	else
		return NULL;
}



int writeiothub(iothub_api_t *device, unsigned char *buff, int n, char *msg)
{
	int i, ret;
	rtcm_t *rtcm_data = &rtcm;
	/*memset(rtcm )*/

	for (i = 0; i < n; i++) {
		ret = input_rtcm2(rtcm_data, buff[i]);

		switch (ret) {
			case 1: 
				printf("Decoded %d obs from RTCM3.\n", rtcm_data->obs.n);

				break;
			case 2:
				
				printf("Decoded navigation data from RTCM3.\n");
				break;
		}
	}

	/*sprintf(msg, "Stream server writes %d bytes to buffer for up to cloud iothub\n", n);*/

	ll_send_telemetry(device->device_ll_handle, buff, n);

	return n;
}



/*int close_iothub()*/
/*{*/
	/*// Clean up the iothub sdk handle*/
	/*IoTHubDeviceClient_LL_Destroy(device_ll_handle);*/

	/*// Free all the sdk subsystem*/
	/*IoTHub_Deinit();*/

	/*return 0;*/
/*}*/

