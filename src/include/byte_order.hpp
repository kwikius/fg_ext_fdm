#ifndef FDSHIM_NET_FLOAT_BYTE_ORDER_H_INCLUDED
#define FDSHIM_NET_FLOAT_BYTE_ORDER_H_INCLUDED

#include <arpa/inet.h>
/*

   uint32_t htonl(uint32_t hostlong);

   uint16_t htons(uint16_t hostshort);

   uint32_t ntohl(uint32_t netlong);

   uint16_t ntohs(uint16_t netshort);
*/

inline double htond(double x)	
{
    static_assert(sizeof(int) == 4,"");
    static_assert(sizeof(double) == 8,""); 
  
    int * p = (int*)&x;
    int tmp = p[0];
    p[0] = htonl(p[1]);
    p[1] = htonl(tmp);

    return x;
}

inline double ntohd(double x)	
{
    static_assert(sizeof(int) == 4,"");
    static_assert(sizeof(double) == 8,""); 
  
    int * p = (int*)&x;
    int tmp = p[0];
    p[0] = ntohl(p[1]);
    p[1] = ntohl(tmp);

    return x;
}

inline float htonf(float x)	
{
    static_assert(sizeof(int) == 4,"");
    static_assert(sizeof(float) == 4,""); 

    int * p = (int *)&x;
    *p = htonl(*p);
    return x;
}

inline float ntohf(float x)	
{
    static_assert(sizeof(int) == 4,"");
    static_assert(sizeof(float) == 4,""); 

    int * p = (int *)&x;
    *p = ntohl(*p);
    return x;
}

#endif // FDSHIM_NET_FLOAT_BYTE_ORDER_H_INCLUDED
