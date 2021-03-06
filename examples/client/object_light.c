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

  LWM2M object "light" implementation



  \author Joerg Hubschneider

*/



/*

 *  Object     |      | Multiple  |     | Description                   |

 *  Name       |  ID  | Instances |Mand.|                               |

 *-------------+------+-----------+-----+-------------------------------+

 *  light   |   6  |    No     |  No |  see TS E.7 page 101          |

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

 *  Timestamp   |  5  |  R   | Single  | Yes | Time    |       |   s   | The timestamp when the light measurement was performed.                       |

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





// ---- private "object light" specific defines ----

// Resource Id's:

#define RES_M_On_Off                          5850

#define RES_O_Dimmer                          5851

#define RES_O_Colour                          5706

#define RES_O_Units                           5701

#define RES_O_On_Time                         5852

#define RES_O_Cumulative_active_power         5805

#define RES_O_Power_factor                    5820



//-----  3GPP TS 23.032 V11.0.0(2012-09) ---------

#define HORIZONTAL_VELOCITY                  0  // for Octet-1 upper half(..<<4)

#define HORIZONTAL_VELOCITY_VERTICAL         1  // set vertical direction bit!

#define HORIZONTAL_VELOCITY_WITH_UNCERTAINTY 2



#define VELOCITY_OCTETS                      5  // for HORITZOL_VELOCITY_WITH_UNCERTAINTY 

#define DEG_DECIMAL_PLACES                   6  // configuration: degree decimals implementation



typedef struct

{

	bool On_off;

    unsigned long timestamp;

} light_data_t;


/**

implementation GPIO control	

*/

void gpio_init()
{
	iolib_init();
	iolib_setdir(8,16,BBBIO_DIR_OUT);
	
}

void gpio_high()
{
	pin_high(8,16);
	printf("set led light hight\n");
}

void gpio_low()
{
        pin_low(8,16);
	 printf("set led light low\n");
}


/**

implementation for all read-able resources

*/

static uint8_t prv_res2tlv(lwm2m_data_t* dataP,

                           light_data_t* locDataP)

{

    //-------------------------------------------------------------------- JH --

    uint8_t ret = COAP_205_CONTENT;  

    switch (dataP->id)   // light resourceId

    {

    case RES_M_On_Off:

        lwm2m_data_encode_bool(locDataP->On_off, dataP);

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

  * @param objInstId    in,     instances ID of the light object to read

  * @param numDataP     in/out, pointer to the number of resource to read. 0 is the

  *                             exception for all readable resource of object instance

  * @param tlvArrayP    in/out, TLV data sequence with initialized resource ID to read

  * @param objectP      in,     private light data structure

  */

static uint8_t prv_light_read(uint16_t objInstId,
                                 int*  numDataP,

                                 lwm2m_data_t** tlvArrayP,

                                 lwm2m_object_t*  objectP)

{   

    //-------------------------------------------------------------------- JH --

    int     i;
    uint8_t result = COAP_500_INTERNAL_SERVER_ERROR;
    light_data_t* locDataP = (light_data_t*)(objectP->userData);

    // defined as single instance object!

    //if (objInstId != 0) return COAP_404_NOT_FOUND;

printf("test read\n");

    if (*numDataP == 0)     // full object, readable resources!

    {

        uint16_t readResIds[] = {

                RES_M_On_Off,

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


static uint8_t prv_light_write(uint16_t instanceId,
                                  int numData,
                                  lwm2m_data_t * dataArray,
                                  lwm2m_object_t * objectP)
{
    int i;
    uint8_t result;
    light_data_t * data = (light_data_t*)(objectP->userData);

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

        case RES_M_On_Off:

            if (lwm2m_data_decode_bool(&dataArray[i], &data->On_off) == 1)

            {
		gpio_init();

		if(data->On_off)
		  gpio_high();
		else
	          gpio_low();
		
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

static uint8_t prv_light_discover(uint16_t instanceId,
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
            RES_M_On_Off
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

            case RES_M_On_Off:
                break;

            default:
                result = COAP_404_NOT_FOUND;
            }
        }
    }
   return result;

}

void display_light_object(lwm2m_object_t * object)

{

#ifdef WITH_LOGS

    light_data_t * data = (light_data_t *)object->userData;

    fprintf(stdout, "  /%u: light object:\r\n", object->objID);

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

void light_setVelocity(lwm2m_object_t* lightObj,

                          uint16_t bearing,

                          uint16_t horizontalSpeed,

                          uint8_t speedUncertainty)

{

    //-------------------------------------------------------------------- JH --

    light_data_t* pData = lightObj->userData;

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

void light_setlightAtTime(lwm2m_object_t* lightObj,

                             float latitude,

                             float longitude,

                             float altitude,

                             uint64_t timestamp)

{

    //-------------------------------------------------------------------- JH --

    light_data_t* pData = lightObj->userData;



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

lwm2m_object_t * get_object_light(void)

{

    //-------------------------------------------------------------------- JH --

    lwm2m_object_t * lightObj;



    lightObj = (lwm2m_object_t *)lwm2m_malloc(sizeof(lwm2m_object_t));

    if (NULL != lightObj)

    {

        memset(lightObj, 0, sizeof(lwm2m_object_t));



        // It assigns its unique ID

        // The 6 is the standard ID for the optional object "light".

        lightObj->objID = 3311;//3311

        

        // and its unique instance

        lightObj->instanceList = (lwm2m_list_t *)lwm2m_malloc(sizeof(lwm2m_list_t));

        if (NULL != lightObj->instanceList)

        {

            memset(lightObj->instanceList, 0, sizeof(lwm2m_list_t));

        }

        else

        {

            lwm2m_free(lightObj);

            return NULL;

        }



        // And the private function that will access the object.

        // Those function will be called when a read query is made by the server.

        // In fact the library don't need to know the resources of the object, only the server does.

        //

        lightObj->readFunc     = prv_light_read;
	    lightObj->writeFunc    = prv_light_write;
        lightObj->userData     = lwm2m_malloc(sizeof(light_data_t));
	    lightObj->discoverFunc = prv_light_discover; 
	

        // initialize private data structure containing the needed variables

        if (NULL != lightObj->userData)

        {

            light_data_t* data = (light_data_t*)lightObj->userData;

            data->On_off = 1; // Mount Everest :)
	        gpio_init();
	        gpio_high(); 

           // light_setVelocity(lightObj, 0, 0, 255); // 255: speedUncertainty not supported!

            data->timestamp   = time(NULL);

        }

        else

        {

            lwm2m_free(lightObj);

            lightObj = NULL;

        }

    }

#if 0
	/* Initialize the PRU */

	printf(">> Initializing PRU\n");

	tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;

	prussdrv_init();


	/* Open PRU Interrupt */

	if (prussdrv_open (PRU_EVTOUT_0)) {

		// Handle failure
		fprintf(stderr, ">> PRU open failed\n");
	}
	/* Get the interrupt initialized */

	prussdrv_pruintc_init(&pruss_intc_initdata);



	/* Get pointers to PRU local memory */

	void *pruDataMem;

	prussdrv_map_prumem(PRUSS0_PRU0_DATARAM, &pruDataMem);

	unsigned int *pruData = (unsigned int *) pruDataMem;



	/* Execute code on PRU */

	printf(">> Executing HCSR-04 code\n");

	prussdrv_exec_program(0, "hcsr04.bin");



	/* Get measurements */

	int i = 0;

	while (i++ < 20) {

	
		// Wait for the PRU interrupt
		prussdrv_pru_wait_event (PRU_EVTOUT_0);
		prussdrv_pru_clear_event(PRU_EVTOUT_0, PRU0_ARM_INTERRUPT);
		

		// Print the distance received from the sonar

		// At 20 degrees in dry air the speed of sound is 342.2 cm/sec

		// so it takes 29.12 us to make 1 cm, i.e. 58.44 us for a roundtrip of 1 cm

		printf("%3d: Distance = %.2f cm\n", i, (float) pruData[0] / 58.44);

		sleep(1);

	}



	/* Disable PRU and close memory mapping*/

	prussdrv_pru_disable(0);

	prussdrv_exit();

	printf(">> PRU Disabled.\r\n");


#endif

    

    return lightObj;

}



void free_object_light(lwm2m_object_t * object)

{

    lwm2m_list_free(object->instanceList);

    lwm2m_free(object->userData);

    lwm2m_free(object);

}



#endif  //LWM2M_CLIENT_MODE


