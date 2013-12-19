#include "EcUtil.h"

namespace cpp4ec
{

char* dtype2string(uint16 dtype)
{
   switch(dtype)
   {
      case ECT_BOOLEAN:
         sprintf(hstr, "BOOLEAN");
         break;
      case ECT_INTEGER8:
         sprintf(hstr, "INTEGER8");
         break;
      case ECT_INTEGER16:
         sprintf(hstr, "INTEGER16");
         break;
      case ECT_INTEGER32:
         sprintf(hstr, "INTEGER32");
         break;
      case ECT_INTEGER24:
         sprintf(hstr, "INTEGER24");
         break;
      case ECT_INTEGER64:
         sprintf(hstr, "INTEGER64");
         break;
      case ECT_UNSIGNED8:
         sprintf(hstr, "UNSIGNED8");
         break;
      case ECT_UNSIGNED16:
         sprintf(hstr, "UNSIGNED16");
         break;
      case ECT_UNSIGNED32:
         sprintf(hstr, "UNSIGNED32");
         break;
      case ECT_UNSIGNED24:
         sprintf(hstr, "UNSIGNED24");
         break;
      case ECT_UNSIGNED64:
         sprintf(hstr, "UNSIGNED64");
         break;
      case ECT_REAL32:
         sprintf(hstr, "REAL32");
         break;
      case ECT_REAL64:
         sprintf(hstr, "REAL64");
         break;
      case ECT_BIT1:
         sprintf(hstr, "BIT1");
         break;
      case ECT_BIT2:
         sprintf(hstr, "BIT2");
         break;
      case ECT_BIT3:
         sprintf(hstr, "BIT3");
         break;
      case ECT_BIT4:
         sprintf(hstr, "BIT4");
         break;
      case ECT_BIT5:
         sprintf(hstr, "BIT5");
         break;
      case ECT_BIT6:
         sprintf(hstr, "BIT6");
         break;
      case ECT_BIT7:
         sprintf(hstr, "BIT7");
         break;
      case ECT_BIT8:
         sprintf(hstr, "BIT8");
         break;
      case ECT_VISIBLE_STRING:
         sprintf(hstr, "VISIBLE_STRING");
         break;
      case ECT_OCTET_STRING:
         sprintf(hstr, "OCTET_STRING");
         break;
      default:
         sprintf(hstr, "Type 0x%4.4X", dtype);
   }
   return hstr;
}               

char* SDO2string(uint16 slave, uint16 index, uint8 subidx, uint16 dtype)
{
   int l = sizeof(usdo) - 1, i;
   uint8 *u8;
   int8 *i8;
   uint16 *u16;
   int16 *i16;
   uint32 *u32;
   int32 *i32;
   uint64 *u64;
   int64 *i64;
   float *sr;
   double *dr;
   char es[32];
   
   memset(&usdo, 0, 128);
   ec_SDOread(slave, index, subidx, FALSE, &l, &usdo, EC_TIMEOUTRXM);
   if (EcatError)
   {
      return ec_elist2string();
   }
   else
   {
      switch(dtype)
      {
         case ECT_BOOLEAN:
            u8 = (uint8*) &usdo[0];
            if (*u8) sprintf(hstr, "TRUE"); 
                 else sprintf(hstr, "FALSE");
                 break;
         case ECT_INTEGER8:
            i8 = (int8*) &usdo[0];
            sprintf(hstr, "0x%2.2x %d", *i8, *i8); 
            break;
         case ECT_INTEGER16:
            i16 = (int16*) &usdo[0];
            sprintf(hstr, "0x%4.4x %d", *i16, *i16); 
            break;
         case ECT_INTEGER32:
         case ECT_INTEGER24:
            i32 = (int32*) &usdo[0];
            sprintf(hstr, "0x%8.8x %d", *i32, *i32); 
            break;
         case ECT_INTEGER64:
            i64 = (int64*) &usdo[0];
            sprintf(hstr, "0x%16.16llx %lld", *i64, *i64); 
            break;
         case ECT_UNSIGNED8:
            u8 = (uint8*) &usdo[0];
            sprintf(hstr, "0x%2.2x %u", *u8, *u8); 
            break;
         case ECT_UNSIGNED16:
            u16 = (uint16*) &usdo[0];
            sprintf(hstr, "0x%4.4x %u", *u16, *u16); 
            break;
         case ECT_UNSIGNED32:
         case ECT_UNSIGNED24:
            u32 = (uint32*) &usdo[0];
            sprintf(hstr, "0x%8.8x %u", *u32, *u32); 
            break;
         case ECT_UNSIGNED64:
            u64 = (uint64*) &usdo[0];
            sprintf(hstr, "0x%16.16llx %llu", *u64, *u64); 
            break;
         case ECT_REAL32:
            sr = (float*) &usdo[0];
            sprintf(hstr, "%f", *sr); 
            break;
         case ECT_REAL64:
            dr = (double*) &usdo[0];
            sprintf(hstr, "%f", *dr); 
            break;
         case ECT_BIT1:
         case ECT_BIT2:
         case ECT_BIT3:
         case ECT_BIT4:
         case ECT_BIT5:
         case ECT_BIT6:
         case ECT_BIT7:
         case ECT_BIT8:
            u8 = (uint8*) &usdo[0];
            sprintf(hstr, "0x%x", *u8); 
            break;
         case ECT_VISIBLE_STRING:
            strcpy(hstr, usdo);
            break;
         case ECT_OCTET_STRING:
            hstr[0] = 0x00;
            for (i = 0 ; i < l ; i++)
            { 
               sprintf(es, "0x%2.2x ", usdo[i]);
               strcat( hstr, es);
            }
            break;
         default:
            sprintf(hstr, "Unknown type");
      }
      return hstr;
   }
}


}