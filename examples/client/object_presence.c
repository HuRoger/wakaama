/*******************************************************************************

 * 

 * Copyright (c) 2014 Bosch Software Innovations GmbH, Germany.

 *

 * All rights reserved. This program and the accompanying materials

 * are made available under the terms of the Eclipse Public License v1.0

 * and Eclipse Distribution License v1.0 which accompany this distribution.

 *

 * The Eclipse Public License is available at

 *    http://www.eclipse.org/legal/epl-v10.html

 * The Eclipse Distribution License is available at

 *    http://www.eclipse.org/org/documents/edl-v10.php.

 *

 * Contributors:

 *    Bosch Software Innovations GmbH - Please refer to git log

 *    Pascal Rieux - Please refer to git log

 *    

 ******************************************************************************/

/*! \file

  LWM2M object "presence" implementation



  \author Joerg Hubschneider

*/



/*

 *  Object     |      | Multiple  |     | Description                   |

 *  Name       |  ID  | Instances |Mand.|                               |

 *-------------+------+-----------+-----+-------------------------------+

 *  presence   |   6  |    No     |  No |  see TS E.7 page 101          |

 *

 *  Resources:

 *  Name        | ID  | Oper.|Instances|Mand.|  Type   | Range | Units | Description                                                                      |

 * -------------+-----+------+---------+-----+---------+-------+-------+----------------------------------------------------------------------------------+

 *  Latitude    |  0  |  R   | Single  | Yes | String  |       |  Deg  | The decimal notation of latitude  e.g. -  45.5723  [Worlds Geodetic System 1984].|

 *  Longitude   |  1  |  R   | Single  | Yes | String  |       |  Deg  | The decimal notation of longitude e.g. - 153.21760 [Worlds Geodetic System 1984].|

 *  Altidude    |  2  |  R   | Single  | No  | String  |       |   m   | The decimal notation of altidude in meters above sea level.                      |

 *  Uncertainty |  3  |  R   | Single  | No  | String  |       |   m   | The accuracy of the position in meters.                                          |

 *  Velocity    |  4  |  R   | Single  | No  | Opaque  |       |   *   | The velocity of the device as defined in 3GPP 23.032 GAD specification(*).       |

 *              |     |      |         |     |         |       |       | This set of values may not be available if the device is static.                 |

 *              |     |      |         |     |         |       |       | opaque: see OMA_TS 6.3.2                                                         |

 *  Timestamp   |  5  |  R   | Single  | Yes | Time    |       |   s   | The timestamp when the presence measurement was performed.                       |

 */

#include <unistd.h>

#include <prussdrv.h>
#include <pruss_intc_mapping.h>



#include "liblwm2m.h"

#include "BBBIOlib/BBBio_lib/BBBiolib.h"

#include <stdio.h>

#include <stdlib.h>

#include <string.h>

#include <time.h>



#ifdef LWM2M_CLIENT_MODE





// ---- private "object presence" specific defines ----
// Smart Object Id's:

#define SmartObjectID                         3302

// Resource Id's:

#define RES_M_DigInputState                   5500

#define RES_O_DigInputCount                   5501

#define RES_O_Units                           5701

#define RES_O_MinRangeValue                   5603

#define RES_O_MaxRangeValue                   5604

#define RES_O_On_Time                         5852

#define RES_O_Cumulative_active_power         5805



#define PRV_Units                             "centimeter"

#define PRV_MinRangeValue                     0

#define PRV_MaxRangeValue                     100




//-----  3GPP TS 23.032 V11.0.0(2012-09) ---------

#define HORIZONTAL_VELOCITY                  0  // for Octet-1 upper half(..<<4)

#define HORIZONTAL_VELOCITY_VERTICAL         1  // set vertical direction bit!

#define HORIZONTAL_VELOCITY_WITH_UNCERTAINTY 2



#define VELOCITY_OCTETS                      5  // for HORITZOL_VELOCITY_WITH_UNCERTAINTY 

#define DEG_DECIMAL_PLACES                   6  // configuration: degree decimals implementation



void *_thread_get_presence(void *arg);

typedef struct

{
	bool presence;

    unsigned long timestamp;

} presence_data_t;


/**

implementation GPIO control	

*/
#if 0
void gpio_init()
{
	iolib_init();
	iolib_setdir(8,12,BBBIO_DIR_IN);
	
}

#endif

/**

implementation for all read-able resources

*/

static uint8_t prv_res2tlv(lwm2m_data_t* dataP,

                           presence_data_t* locDataP)

{

    //-------------------------------------------------------------------- JH --

    uint8_t ret = COAP_205_CONTENT;  
	
   // locDataP->presence = get_presence();

    switch (dataP->id)   // presence resourceId

    {

    case RES_M_DigInputState:

        lwm2m_data_encode_bool(locDataP->presence, dataP);

        break;
    case RES_O_Units:

        lwm2m_data_encode_string(PRV_Units, dataP);

        break;


    default:

        ret = COAP_404_NOT_FOUND;

        break;

    }

  

    return ret;

}





/**

  * Implementation (callback-) function of reading object resources. For whole

  * object, single resources or a sequence of resources

  * see 3GPP TS 23.032 V11.0.0(2012-09) page 23,24.

  * implemented for: HORIZONTAL_VELOCITY_WITH_UNCERTAINT

  * @param objInstId    in,     instances ID of the presence object to read

  * @param numDataP     in/out, pointer to the number of resource to read. 0 is the

  *                             exception for all readable resource of object instance

  * @param tlvArrayP    in/out, TLV data sequence with initialized resource ID to read

  * @param objectP      in,     private presence data structure

  */

static uint8_t prv_presence_read(uint16_t objInstId,
                                 int*  numDataP,

                                 lwm2m_data_t** tlvArrayP,

                                 lwm2m_object_t*  objectP)

{   

    //-------------------------------------------------------------------- JH --

    int     i;
    uint8_t result = COAP_500_INTERNAL_SERVER_ERROR;
    presence_data_t* locDataP = (presence_data_t*)(objectP->userData);

    // defined as single instance object!

    //if (objInstId != 0) return COAP_404_NOT_FOUND;

printf("test read\n");

    if (*numDataP == 0)     // full object, readable resources!

    {

        uint16_t readResIds[] = {

                RES_M_DigInputState,
                RES_O_Units

        }; // readable resources!

        

        *numDataP  = sizeof(readResIds)/sizeof(uint16_t);

        *tlvArrayP = lwm2m_data_new(*numDataP);

        if (*tlvArrayP == NULL) return COAP_500_INTERNAL_SERVER_ERROR;

        

        // init readable resource id's

        for (i = 0 ; i < *numDataP ; i++)
        {
            (*tlvArrayP)[i].id = readResIds[i];
        }
    }

    

    for (i = 0 ; i < *numDataP ; i++)
    {
        result = prv_res2tlv ((*tlvArrayP)+i, locDataP);
        if (result!=COAP_205_CONTENT) break;
    }
    return result;
}


static uint8_t prv_presence_write(uint16_t instanceId,
                                  int numData,
                                  lwm2m_data_t * dataArray,
                                  lwm2m_object_t * objectP)
{
    int i;
    uint8_t result;
    presence_data_t * data = (presence_data_t*)(objectP->userData);

    // this is a single instance object
    if (instanceId != 0)
    {
        return COAP_404_NOT_FOUND;
    }
    i = 0;
    printf("---------------test write func\n");
    do

    {

        switch (dataArray[i].id)

        {

        case RES_M_DigInputState:

            if (lwm2m_data_decode_bool(&dataArray[i], &(data->presence)) == 1)

            {	
                result = COAP_204_CHANGED;
            }
            else
            {
                result = COAP_400_BAD_REQUEST;
            }
            break;

        default:
	    printf("method not allowed\n");
            result = COAP_405_METHOD_NOT_ALLOWED;
        }

        i++;

    } while (i < numData && result == COAP_204_CHANGED);

    return result;

}

static uint8_t prv_presence_discover(uint16_t instanceId,
                                   int * numDataP,
                                   lwm2m_data_t ** dataArrayP,
                                   lwm2m_object_t * objectP)
{

    uint8_t result;

    int i;



    // this is a single instance object

    if (instanceId != 0)

    {
        return COAP_404_NOT_FOUND;
    }

    result = COAP_205_CONTENT;

    // is the server asking for the full object ?
    if (*numDataP == 0)
    {

        uint16_t resList[] = {
            RES_M_DigInputState,
            RES_O_Units
        };

        int nbRes = sizeof(resList) / sizeof(uint16_t);



        *dataArrayP = lwm2m_data_new(nbRes);

        if (*dataArrayP == NULL) return COAP_500_INTERNAL_SERVER_ERROR;
        *numDataP = nbRes;
        for (i = 0; i < nbRes; i++)

        {
            (*dataArrayP)[i].id = resList[i];
        }

    }

    else
    {

        for (i = 0; i < *numDataP && result == COAP_205_CONTENT; i++)
        {

            switch ((*dataArrayP)[i].id)
            {

            case RES_M_DigInputState:
            case RES_O_Units:   
                break;

            default:
                result = COAP_404_NOT_FOUND;
            }
        }
    }
   return result;

}

void display_presence_object(lwm2m_object_t * object)

{

#ifdef WITH_LOGS

    presence_data_t * data = (presence_data_t *)object->userData;

    fprintf(stdout, "  /%u: presence object:\r\n", object->objID);

    if (NULL != data)

    {
/*
        fprintf(stdout, "    latitude: %s, longitude: %s, altitude: %s, uncertainty: %s, timestamp: %lu\r\n",

                data->latitude, data->longitude, data->altitude, data->uncertainty, data->timestamp);
*/
    }

#endif

}



/**

  * Convenience function to set the velocity attributes.

  * see 3GPP TS 23.032 V11.0.0(2012-09) page 23,24.

  * implemented for: HORIZONTAL_VELOCITY_WITH_UNCERTAINTY

  * @param locationObj location object reference (to be casted!)

  * @param bearing          [Deg]  0 - 359    resolution: 1 degree

  * @param horizontalSpeed  [km/h] 1 - s^16-1 resolution: 1 km/h steps

  * @param speedUncertainty [km/h] 1-254      resolution: 1 km/h (255=undefined!)

  */

void presence_setVelocity(lwm2m_object_t* presenceObj,

                          uint16_t bearing,

                          uint16_t horizontalSpeed,

                          uint8_t speedUncertainty)

{

    //-------------------------------------------------------------------- JH --

    presence_data_t* pData = presenceObj->userData;

	/*

    pData->velocity[0] = HORIZONTAL_VELOCITY_WITH_UNCERTAINTY << 4;

    pData->velocity[0] = (bearing & 0x100) >> 8;

    pData->velocity[1] = (bearing & 0x0FF);

    pData->velocity[2] = horizontalSpeed >> 8;

    pData->velocity[3] = horizontalSpeed & 0xff;

    pData->velocity[4] = speedUncertainty;

	*/

}



/**

  * A convenience function to set the location coordinates with its timestamp.

  * @see testMe()

  * @param locationObj location object reference (to be casted!)

  * @param latitude  the second argument.

  * @param longitude the second argument.

  * @param altitude  the second argument.

  * @param timestamp the related timestamp. Seconds since 1970.

  */

void presence_setpresenceAtTime(lwm2m_object_t* presenceObj,

                             float latitude,

                             float longitude,

                             float altitude,

                             uint64_t timestamp)

{

    //-------------------------------------------------------------------- JH --

    presence_data_t* pData = presenceObj->userData;



#if defined(ARDUINO)

    dtostrf (latitude,  -8, 6, pData->latitude);

    dtostrf (longitude, -8, 6, pData->longitude);

    dtostrf (altitude,  -8, 4, pData->altitude);

#else

    //snprintf(pData->latitude, 5+DEG_DECIMAL_PLACES, "%-8.6f", (double)latitude);

    //snprintf(pData->longitude,5+DEG_DECIMAL_PLACES, "%-8.6f", (double)longitude);

    //snprintf(pData->altitude, 5+DEG_DECIMAL_PLACES, "%-8.4f", (double)altitude);

#endif



    pData->timestamp = timestamp;

}



/**

  * This function creates the LWM2M Location. 

  * @return gives back allocated LWM2M data object structure pointer. On error, 

  * NULL value is returned.

  */

lwm2m_object_t * get_object_presence(void)

{

    //-------------------------------------------------------------------- JH --

    lwm2m_object_t * presenceObj;
	
    pthread_t thread_get_presence;

    presenceObj = (lwm2m_object_t *)lwm2m_malloc(sizeof(lwm2m_object_t));
    
    iolib_init();
	iolib_setdir(8,12,BBBIO_DIR_IN);

    if (NULL != presenceObj)

    {

        memset(presenceObj, 0, sizeof(lwm2m_object_t));



        // It assigns its unique ID

        // The 6 is the standard ID for the optional object "presence".

        presenceObj->objID = SmartObjectID;

        

        // and its unique instance

        presenceObj->instanceList = (lwm2m_list_t *)lwm2m_malloc(sizeof(lwm2m_list_t));

        if (NULL != presenceObj->instanceList)

        {
            memset(presenceObj->instanceList, 0, sizeof(lwm2m_list_t));
        }

        else

        {
            lwm2m_free(presenceObj);

            return NULL;

        }



        // And the private function that will access the object.

        // Those function will be called when a read query is made by the server.

        // In fact the library don't need to know the resources of the object, only the server does.

        //

        presenceObj->readFunc     = prv_presence_read;
	    presenceObj->writeFunc    = prv_presence_write;
        presenceObj->userData     = lwm2m_malloc(sizeof(presence_data_t));
	    presenceObj->discoverFunc = prv_presence_discover; 
	

        // initialize private data structure containing the needed variables

        if (NULL != presenceObj->userData)

        {

            presence_data_t* data = (presence_data_t*)presenceObj->userData;
            
            if(is_high(8,11))
                data->presence = 1; // Mount Everest :)
                
	    	if(is_low(8,11))
	    	    data->presence = 0; // Mount Everest :)
                

           // presence_setVelocity(presenceObj, 0, 0, 255); // 255: speedUncertainty not supported!

            data->timestamp   = time(NULL);

        }

        else

        {

            lwm2m_free(presenceObj);

            presenceObj = NULL;

        }

    }

    pthread_create(&thread_get_presence, NULL, &_thread_get_presence, (void *)presenceObj);	
    pthread_detach(thread_get_presence);   

    return presenceObj;

}



void free_object_presence(lwm2m_object_t * object)

{

    lwm2m_list_free(object->instanceList);

    lwm2m_free(object->userData);

    lwm2m_free(object);

}


void *_thread_get_presence(void *arg)

{
	lwm2m_object_t * presenceObj = (lwm2m_object_t *)arg;
	presence_data_t* data = (presence_data_t*)presenceObj->userData;
	
	while(1)
	{

	   
	   
	   if(is_high(8,11))
        data->presence = true; // Mount Everest :)
	   if(is_low(8,11))
	    data->presence = false; // Mount Everest :)
	    
	   printf("presence = %d\n",data->presence);
	   	
	   sleep(1);
	}

	/* Disable PRU and close memory mapping*/

	prussdrv_pru_disable(0);

	prussdrv_exit();

	printf(">> PRU Disabled.\r\n");

	pthread_exit(0);

}



#endif  //LWM2M_CLIENT_MODE



