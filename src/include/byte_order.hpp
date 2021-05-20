#ifndef FDSHIM_NET_FLOAT_BYTE_ORDER_H_INCLUDED
#define FDSHIM_NET_FLOAT_BYTE_ORDER_H_INCLUDED

#include <arpa/inet.h>

double htond(double x)	
{
    static_assert(sizeof(int) == 4,"");
    static_assert(sizeof(double) == 8,""); 
  
    int * p = (int*)&x;
    int tmp = p[0];
    p[0] = htonl(p[1]);
    p[1] = htonl(tmp);

    return x;
}

float htonf(float x)	
{
    static_assert(sizeof(int) == 4,"");
    static_assert(sizeof(float) == 4,""); 

    int * p = (int *)&x;
    *p = htonl(*p);
    return x;
}


#endif // FDSHIM_NET_FLOAT_BYTE_ORDER_H_INCLUDED
