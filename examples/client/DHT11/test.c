#include "bbb_dht_read.h"
#include "common_dht_read.h"
#include "bbb_mmio.h"

#include <stdio.h>

void main()
{
    float humidity = 0, temperature = 0;
    int result;
    
    while(1)
    {
        
        result = bbb_dht_read(DHT11, 1, 13, &humidity, &temperature);
        if(result==0)
        printf("humidity:[%f], temperature:[%f] result[%d]\n",humidity,temperature,result);
        sleep(1);
    }

}