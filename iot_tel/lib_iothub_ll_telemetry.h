#ifndef LIB_IOTHUB_LL_TELEMETRY_H
#define LIB_IOTHUB_LL_TELEMETRY_H


//#include "../str2str/rtklib.h"
//#include "../rtk_data_to_iothub_helper.h"


#include "iothub.h"
#include "iothub_message.h"
#include "iothub_device_client_ll.h"


typedef struct 
{
	char connectionString[1024];

	IOTHUB_DEVICE_CLIENT_LL_HANDLE device_ll_handle;
	
	//rtcm_t rtcm;

} iothub_api_t;


iothub_api_t *openiothub(const char *p, char *m);

int init_iothub(iothub_api_t *d);

int ll_send_telemetry(IOTHUB_DEVICE_CLIENT_LL_HANDLE dh, unsigned char *msg, size_t n);

int writeiothub(iothub_api_t *device, unsigned char *buff, int n, char *msg);

#endif
