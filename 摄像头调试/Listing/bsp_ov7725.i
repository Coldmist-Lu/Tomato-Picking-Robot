#line 1 "..\\..\\User\\ov7725\\bsp_ov7725.c"














  


#line 1 "..\\..\\User\\./ov7725/bsp_ov7725.h"


	   
#line 1 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"







































 



 



 
    






  


 
  


 

#line 75 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"


















 





#line 107 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"







            
#line 122 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"





 






 
#line 143 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"



 



 



 
#line 162 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"




 
typedef enum IRQn
{
 
  NonMaskableInt_IRQn         = -14,     
  MemoryManagement_IRQn       = -12,     
  BusFault_IRQn               = -11,     
  UsageFault_IRQn             = -10,     
  SVCall_IRQn                 = -5,      
  DebugMonitor_IRQn           = -4,      
  PendSV_IRQn                 = -2,      
  SysTick_IRQn                = -1,      

 
  WWDG_IRQn                   = 0,       
  PVD_IRQn                    = 1,       
  TAMPER_IRQn                 = 2,       
  RTC_IRQn                    = 3,       
  FLASH_IRQn                  = 4,       
  RCC_IRQn                    = 5,       
  EXTI0_IRQn                  = 6,       
  EXTI1_IRQn                  = 7,       
  EXTI2_IRQn                  = 8,       
  EXTI3_IRQn                  = 9,       
  EXTI4_IRQn                  = 10,      
  DMA1_Channel1_IRQn          = 11,      
  DMA1_Channel2_IRQn          = 12,      
  DMA1_Channel3_IRQn          = 13,      
  DMA1_Channel4_IRQn          = 14,      
  DMA1_Channel5_IRQn          = 15,      
  DMA1_Channel6_IRQn          = 16,      
  DMA1_Channel7_IRQn          = 17,      

#line 221 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 242 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 270 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 296 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"


  ADC1_2_IRQn                 = 18,      
  USB_HP_CAN1_TX_IRQn         = 19,      
  USB_LP_CAN1_RX0_IRQn        = 20,      
  CAN1_RX1_IRQn               = 21,      
  CAN1_SCE_IRQn               = 22,      
  EXTI9_5_IRQn                = 23,      
  TIM1_BRK_IRQn               = 24,      
  TIM1_UP_IRQn                = 25,      
  TIM1_TRG_COM_IRQn           = 26,      
  TIM1_CC_IRQn                = 27,      
  TIM2_IRQn                   = 28,      
  TIM3_IRQn                   = 29,      
  TIM4_IRQn                   = 30,      
  I2C1_EV_IRQn                = 31,      
  I2C1_ER_IRQn                = 32,      
  I2C2_EV_IRQn                = 33,      
  I2C2_ER_IRQn                = 34,      
  SPI1_IRQn                   = 35,      
  SPI2_IRQn                   = 36,      
  USART1_IRQn                 = 37,      
  USART2_IRQn                 = 38,      
  USART3_IRQn                 = 39,      
  EXTI15_10_IRQn              = 40,      
  RTCAlarm_IRQn               = 41,      
  USBWakeUp_IRQn              = 42,      
  TIM8_BRK_IRQn               = 43,      
  TIM8_UP_IRQn                = 44,      
  TIM8_TRG_COM_IRQn           = 45,      
  TIM8_CC_IRQn                = 46,      
  ADC3_IRQn                   = 47,      
  FSMC_IRQn                   = 48,      
  SDIO_IRQn                   = 49,      
  TIM5_IRQn                   = 50,      
  SPI3_IRQn                   = 51,      
  UART4_IRQn                  = 52,      
  UART5_IRQn                  = 53,      
  TIM6_IRQn                   = 54,      
  TIM7_IRQn                   = 55,      
  DMA2_Channel1_IRQn          = 56,      
  DMA2_Channel2_IRQn          = 57,      
  DMA2_Channel3_IRQn          = 58,      
  DMA2_Channel4_5_IRQn        = 59       


#line 381 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 426 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 472 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"
} IRQn_Type;



 

#line 1 "..\\..\\Libraries\\CMSIS\\core_cm3.h"
 




















 





































 

 
 
 
 
 
 
 
 








 











#line 1 "D:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
 
 





 









     
#line 27 "D:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
     











#line 46 "D:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"





 

     

     
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

     

     
     
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef   signed           int int_least32_t;
typedef   signed       __int64 int_least64_t;

     
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef unsigned           int uint_least32_t;
typedef unsigned       __int64 uint_least64_t;

     

     
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef   signed       __int64 int_fast64_t;

     
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef unsigned       __int64 uint_fast64_t;

     




typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;


     
typedef   signed     long long intmax_t;
typedef unsigned     long long uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     






     






     






     

     


     


     


     

     
#line 216 "D:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     



     






     
    
 



#line 241 "D:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     







     










     











#line 305 "D:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"






 
#line 91 "..\\..\\Libraries\\CMSIS\\core_cm3.h"

















 

#line 117 "..\\..\\Libraries\\CMSIS\\core_cm3.h"





 


 





 
typedef struct
{
  volatile uint32_t ISER[8];                       
       uint32_t RESERVED0[24];                                   
  volatile uint32_t ICER[8];                       
       uint32_t RSERVED1[24];                                    
  volatile uint32_t ISPR[8];                       
       uint32_t RESERVED2[24];                                   
  volatile uint32_t ICPR[8];                       
       uint32_t RESERVED3[24];                                   
  volatile uint32_t IABR[8];                       
       uint32_t RESERVED4[56];                                   
  volatile uint8_t  IP[240];                       
       uint32_t RESERVED5[644];                                  
  volatile  uint32_t STIR;                          
}  NVIC_Type;                                               
   





 
typedef struct
{
  volatile const  uint32_t CPUID;                         
  volatile uint32_t ICSR;                          
  volatile uint32_t VTOR;                          
  volatile uint32_t AIRCR;                         
  volatile uint32_t SCR;                           
  volatile uint32_t CCR;                           
  volatile uint8_t  SHP[12];                       
  volatile uint32_t SHCSR;                         
  volatile uint32_t CFSR;                          
  volatile uint32_t HFSR;                          
  volatile uint32_t DFSR;                          
  volatile uint32_t MMFAR;                         
  volatile uint32_t BFAR;                          
  volatile uint32_t AFSR;                          
  volatile const  uint32_t PFR[2];                        
  volatile const  uint32_t DFR;                           
  volatile const  uint32_t ADR;                           
  volatile const  uint32_t MMFR[4];                       
  volatile const  uint32_t ISAR[5];                       
} SCB_Type;                                                

 












 






























 






 





















 









 


















 
































                                     









 









 









 














   





 
typedef struct
{
  volatile uint32_t CTRL;                          
  volatile uint32_t LOAD;                          
  volatile uint32_t VAL;                           
  volatile const  uint32_t CALIB;                         
} SysTick_Type;

 












 



 



 








   





 
typedef struct
{
  volatile  union  
  {
    volatile  uint8_t    u8;                        
    volatile  uint16_t   u16;                       
    volatile  uint32_t   u32;                       
  }  PORT [32];                                
       uint32_t RESERVED0[864];                                 
  volatile uint32_t TER;                           
       uint32_t RESERVED1[15];                                  
  volatile uint32_t TPR;                           
       uint32_t RESERVED2[15];                                  
  volatile uint32_t TCR;                           
       uint32_t RESERVED3[29];                                  
  volatile uint32_t IWR;                           
  volatile uint32_t IRR;                           
  volatile uint32_t IMCR;                          
       uint32_t RESERVED4[43];                                  
  volatile uint32_t LAR;                           
  volatile uint32_t LSR;                           
       uint32_t RESERVED5[6];                                   
  volatile const  uint32_t PID4;                          
  volatile const  uint32_t PID5;                          
  volatile const  uint32_t PID6;                          
  volatile const  uint32_t PID7;                          
  volatile const  uint32_t PID0;                          
  volatile const  uint32_t PID1;                          
  volatile const  uint32_t PID2;                          
  volatile const  uint32_t PID3;                          
  volatile const  uint32_t CID0;                          
  volatile const  uint32_t CID1;                          
  volatile const  uint32_t CID2;                          
  volatile const  uint32_t CID3;                          
} ITM_Type;                                                

 



 
























 



 



 



 








   





 
typedef struct
{
       uint32_t RESERVED0;
  volatile const  uint32_t ICTR;                          



       uint32_t RESERVED1;

} InterruptType_Type;

 



 








   


#line 614 "..\\..\\Libraries\\CMSIS\\core_cm3.h"





 
typedef struct
{
  volatile uint32_t DHCSR;                         
  volatile  uint32_t DCRSR;                         
  volatile uint32_t DCRDR;                         
  volatile uint32_t DEMCR;                         
} CoreDebug_Type;

 




































 






 






































   


 
#line 721 "..\\..\\Libraries\\CMSIS\\core_cm3.h"

#line 728 "..\\..\\Libraries\\CMSIS\\core_cm3.h"






   




 





#line 758 "..\\..\\Libraries\\CMSIS\\core_cm3.h"


 


 




#line 783 "..\\..\\Libraries\\CMSIS\\core_cm3.h"


 
 
 
 








 
extern uint32_t __get_PSP(void);








 
extern void __set_PSP(uint32_t topOfProcStack);








 
extern uint32_t __get_MSP(void);








 
extern void __set_MSP(uint32_t topOfMainStack);








 
extern uint32_t __REV16(uint16_t value);








 
extern int32_t __REVSH(int16_t value);


#line 933 "..\\..\\Libraries\\CMSIS\\core_cm3.h"





 








 
static __inline uint32_t  __get_BASEPRI(void)
{
  register uint32_t __regBasePri         __asm("basepri");
  return(__regBasePri);
}







 
static __inline void __set_BASEPRI(uint32_t basePri)
{
  register uint32_t __regBasePri         __asm("basepri");
  __regBasePri = (basePri & 0xff);
}







 
static __inline uint32_t __get_PRIMASK(void)
{
  register uint32_t __regPriMask         __asm("primask");
  return(__regPriMask);
}







 
static __inline void __set_PRIMASK(uint32_t priMask)
{
  register uint32_t __regPriMask         __asm("primask");
  __regPriMask = (priMask);
}







 
static __inline uint32_t __get_FAULTMASK(void)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  return(__regFaultMask);
}







 
static __inline void __set_FAULTMASK(uint32_t faultMask)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  __regFaultMask = (faultMask & 1);
}







 
static __inline uint32_t __get_CONTROL(void)
{
  register uint32_t __regControl         __asm("control");
  return(__regControl);
}







 
static __inline void __set_CONTROL(uint32_t control)
{
  register uint32_t __regControl         __asm("control");
  __regControl = control;
}





#line 1445 "..\\..\\Libraries\\CMSIS\\core_cm3.h"







 
 

 











 
static __inline void NVIC_SetPriorityGrouping(uint32_t PriorityGroup)
{
  uint32_t reg_value;
  uint32_t PriorityGroupTmp = (PriorityGroup & 0x07);                          
  
  reg_value  =  ((SCB_Type *) ((0xE000E000) + 0x0D00))->AIRCR;                                                    
  reg_value &= ~((0xFFFFul << 16) | (7ul << 8));              
  reg_value  =  (reg_value                       |
                (0x5FA << 16) | 
                (PriorityGroupTmp << 8));                                      
  ((SCB_Type *) ((0xE000E000) + 0x0D00))->AIRCR =  reg_value;
}








 
static __inline uint32_t NVIC_GetPriorityGrouping(void)
{
  return ((((SCB_Type *) ((0xE000E000) + 0x0D00))->AIRCR & (7ul << 8)) >> 8);    
}








 
static __inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000) + 0x0100))->ISER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}








 
static __inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000) + 0x0100))->ICER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}









 
static __inline uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t) ((((NVIC_Type *) ((0xE000E000) + 0x0100))->ISPR[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));  
}








 
static __inline void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000) + 0x0100))->ISPR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}








 
static __inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000) + 0x0100))->ICPR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}









 
static __inline uint32_t NVIC_GetActive(IRQn_Type IRQn)
{
  return((uint32_t)((((NVIC_Type *) ((0xE000E000) + 0x0100))->IABR[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));  
}












 
static __inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if(IRQn < 0) {
    ((SCB_Type *) ((0xE000E000) + 0x0D00))->SHP[((uint32_t)(IRQn) & 0xF)-4] = ((priority << (8 - 4)) & 0xff); }  
  else {
    ((NVIC_Type *) ((0xE000E000) + 0x0100))->IP[(uint32_t)(IRQn)] = ((priority << (8 - 4)) & 0xff);    }         
}















 
static __inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if(IRQn < 0) {
    return((uint32_t)(((SCB_Type *) ((0xE000E000) + 0x0D00))->SHP[((uint32_t)(IRQn) & 0xF)-4] >> (8 - 4)));  }  
  else {
    return((uint32_t)(((NVIC_Type *) ((0xE000E000) + 0x0100))->IP[(uint32_t)(IRQn)]           >> (8 - 4)));  }  
}
















 
static __inline uint32_t NVIC_EncodePriority (uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & 0x07);           
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7 - PriorityGroupTmp) > 4) ? 4 : 7 - PriorityGroupTmp;
  SubPriorityBits     = ((PriorityGroupTmp + 4) < 7) ? 0 : PriorityGroupTmp - 7 + 4;
 
  return (
           ((PreemptPriority & ((1 << (PreemptPriorityBits)) - 1)) << SubPriorityBits) |
           ((SubPriority     & ((1 << (SubPriorityBits    )) - 1)))
         );
}
















 
static __inline void NVIC_DecodePriority (uint32_t Priority, uint32_t PriorityGroup, uint32_t* pPreemptPriority, uint32_t* pSubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & 0x07);           
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7 - PriorityGroupTmp) > 4) ? 4 : 7 - PriorityGroupTmp;
  SubPriorityBits     = ((PriorityGroupTmp + 4) < 7) ? 0 : PriorityGroupTmp - 7 + 4;
  
  *pPreemptPriority = (Priority >> SubPriorityBits) & ((1 << (PreemptPriorityBits)) - 1);
  *pSubPriority     = (Priority                   ) & ((1 << (SubPriorityBits    )) - 1);
}



 












 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{ 
  if (ticks > (0xFFFFFFul << 0))  return (1);             
                                                               
  ((SysTick_Type *) ((0xE000E000) + 0x0010))->LOAD  = (ticks & (0xFFFFFFul << 0)) - 1;       
  NVIC_SetPriority (SysTick_IRQn, (1<<4) - 1);   
  ((SysTick_Type *) ((0xE000E000) + 0x0010))->VAL   = 0;                                           
  ((SysTick_Type *) ((0xE000E000) + 0x0010))->CTRL  = (1ul << 2) | 
                   (1ul << 1)   | 
                   (1ul << 0);                     
  return (0);                                                   
}






 





 
static __inline void NVIC_SystemReset(void)
{
  ((SCB_Type *) ((0xE000E000) + 0x0D00))->AIRCR  = ((0x5FA << 16)      | 
                 (((SCB_Type *) ((0xE000E000) + 0x0D00))->AIRCR & (7ul << 8)) | 
                 (1ul << 2));                    
  __dsb(0);                                                                    
  while(1);                                                     
}

   



 






 
 

extern volatile int ITM_RxBuffer;                     












 
static __inline uint32_t ITM_SendChar (uint32_t ch)
{
  if ((((CoreDebug_Type *) (0xE000EDF0))->DEMCR & (1ul << 24))  &&       
      (((ITM_Type *) (0xE0000000))->TCR & (1ul << 0))                  &&       
      (((ITM_Type *) (0xE0000000))->TER & (1ul << 0)        )                    )      
  {
    while (((ITM_Type *) (0xE0000000))->PORT[0].u32 == 0);
    ((ITM_Type *) (0xE0000000))->PORT[0].u8 = (uint8_t) ch;
  }  
  return (ch);
}










 
static __inline int ITM_ReceiveChar (void) {
  int ch = -1;                                

  if (ITM_RxBuffer != 0x5AA55AA5) {
    ch = ITM_RxBuffer;
    ITM_RxBuffer = 0x5AA55AA5;        
  }
  
  return (ch); 
}









 
static __inline int ITM_CheckChar (void) {

  if (ITM_RxBuffer == 0x5AA55AA5) {
    return (0);                                  
  } else {
    return (1);                                  
  }
}

   






   



 
#line 479 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"
#line 1 "D:\\Keil\\ARM\\PACK\\Keil\\STM32F1xx_DFP\\2.2.0\\Device\\Include\\system_stm32f10x.h"



















 



 



   
  


 









 



 




 

extern uint32_t SystemCoreClock;           



 



 



 



 



 



 
  
extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);


 









 
  


   
 
#line 480 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"
#line 481 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"



   

 
typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef const int32_t sc32;   
typedef const int16_t sc16;   
typedef const int8_t sc8;    

typedef volatile int32_t  vs32;
typedef volatile int16_t  vs16;
typedef volatile int8_t   vs8;

typedef volatile const int32_t vsc32;   
typedef volatile const int16_t vsc16;   
typedef volatile const int8_t vsc8;    

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef const uint32_t uc32;   
typedef const uint16_t uc16;   
typedef const uint8_t uc8;    

typedef volatile uint32_t  vu32;
typedef volatile uint16_t vu16;
typedef volatile uint8_t  vu8;

typedef volatile const uint32_t vuc32;   
typedef volatile const uint16_t vuc16;   
typedef volatile const uint8_t vuc8;    

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;


typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;

 





 



    



 

typedef struct
{
  volatile uint32_t SR;
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t SMPR1;
  volatile uint32_t SMPR2;
  volatile uint32_t JOFR1;
  volatile uint32_t JOFR2;
  volatile uint32_t JOFR3;
  volatile uint32_t JOFR4;
  volatile uint32_t HTR;
  volatile uint32_t LTR;
  volatile uint32_t SQR1;
  volatile uint32_t SQR2;
  volatile uint32_t SQR3;
  volatile uint32_t JSQR;
  volatile uint32_t JDR1;
  volatile uint32_t JDR2;
  volatile uint32_t JDR3;
  volatile uint32_t JDR4;
  volatile uint32_t DR;
} ADC_TypeDef;



 

typedef struct
{
  uint32_t  RESERVED0;
  volatile uint16_t DR1;
  uint16_t  RESERVED1;
  volatile uint16_t DR2;
  uint16_t  RESERVED2;
  volatile uint16_t DR3;
  uint16_t  RESERVED3;
  volatile uint16_t DR4;
  uint16_t  RESERVED4;
  volatile uint16_t DR5;
  uint16_t  RESERVED5;
  volatile uint16_t DR6;
  uint16_t  RESERVED6;
  volatile uint16_t DR7;
  uint16_t  RESERVED7;
  volatile uint16_t DR8;
  uint16_t  RESERVED8;
  volatile uint16_t DR9;
  uint16_t  RESERVED9;
  volatile uint16_t DR10;
  uint16_t  RESERVED10; 
  volatile uint16_t RTCCR;
  uint16_t  RESERVED11;
  volatile uint16_t CR;
  uint16_t  RESERVED12;
  volatile uint16_t CSR;
  uint16_t  RESERVED13[5];
  volatile uint16_t DR11;
  uint16_t  RESERVED14;
  volatile uint16_t DR12;
  uint16_t  RESERVED15;
  volatile uint16_t DR13;
  uint16_t  RESERVED16;
  volatile uint16_t DR14;
  uint16_t  RESERVED17;
  volatile uint16_t DR15;
  uint16_t  RESERVED18;
  volatile uint16_t DR16;
  uint16_t  RESERVED19;
  volatile uint16_t DR17;
  uint16_t  RESERVED20;
  volatile uint16_t DR18;
  uint16_t  RESERVED21;
  volatile uint16_t DR19;
  uint16_t  RESERVED22;
  volatile uint16_t DR20;
  uint16_t  RESERVED23;
  volatile uint16_t DR21;
  uint16_t  RESERVED24;
  volatile uint16_t DR22;
  uint16_t  RESERVED25;
  volatile uint16_t DR23;
  uint16_t  RESERVED26;
  volatile uint16_t DR24;
  uint16_t  RESERVED27;
  volatile uint16_t DR25;
  uint16_t  RESERVED28;
  volatile uint16_t DR26;
  uint16_t  RESERVED29;
  volatile uint16_t DR27;
  uint16_t  RESERVED30;
  volatile uint16_t DR28;
  uint16_t  RESERVED31;
  volatile uint16_t DR29;
  uint16_t  RESERVED32;
  volatile uint16_t DR30;
  uint16_t  RESERVED33; 
  volatile uint16_t DR31;
  uint16_t  RESERVED34;
  volatile uint16_t DR32;
  uint16_t  RESERVED35;
  volatile uint16_t DR33;
  uint16_t  RESERVED36;
  volatile uint16_t DR34;
  uint16_t  RESERVED37;
  volatile uint16_t DR35;
  uint16_t  RESERVED38;
  volatile uint16_t DR36;
  uint16_t  RESERVED39;
  volatile uint16_t DR37;
  uint16_t  RESERVED40;
  volatile uint16_t DR38;
  uint16_t  RESERVED41;
  volatile uint16_t DR39;
  uint16_t  RESERVED42;
  volatile uint16_t DR40;
  uint16_t  RESERVED43;
  volatile uint16_t DR41;
  uint16_t  RESERVED44;
  volatile uint16_t DR42;
  uint16_t  RESERVED45;    
} BKP_TypeDef;
  


 

typedef struct
{
  volatile uint32_t TIR;
  volatile uint32_t TDTR;
  volatile uint32_t TDLR;
  volatile uint32_t TDHR;
} CAN_TxMailBox_TypeDef;



 
  
typedef struct
{
  volatile uint32_t RIR;
  volatile uint32_t RDTR;
  volatile uint32_t RDLR;
  volatile uint32_t RDHR;
} CAN_FIFOMailBox_TypeDef;



 
  
typedef struct
{
  volatile uint32_t FR1;
  volatile uint32_t FR2;
} CAN_FilterRegister_TypeDef;



 
  
typedef struct
{
  volatile uint32_t MCR;
  volatile uint32_t MSR;
  volatile uint32_t TSR;
  volatile uint32_t RF0R;
  volatile uint32_t RF1R;
  volatile uint32_t IER;
  volatile uint32_t ESR;
  volatile uint32_t BTR;
  uint32_t  RESERVED0[88];
  CAN_TxMailBox_TypeDef sTxMailBox[3];
  CAN_FIFOMailBox_TypeDef sFIFOMailBox[2];
  uint32_t  RESERVED1[12];
  volatile uint32_t FMR;
  volatile uint32_t FM1R;
  uint32_t  RESERVED2;
  volatile uint32_t FS1R;
  uint32_t  RESERVED3;
  volatile uint32_t FFA1R;
  uint32_t  RESERVED4;
  volatile uint32_t FA1R;
  uint32_t  RESERVED5[8];

  CAN_FilterRegister_TypeDef sFilterRegister[14];



} CAN_TypeDef;



 
typedef struct
{
  volatile uint32_t CFGR;
  volatile uint32_t OAR;
  volatile uint32_t PRES;
  volatile uint32_t ESR;
  volatile uint32_t CSR;
  volatile uint32_t TXD;
  volatile uint32_t RXD;  
} CEC_TypeDef;



 

typedef struct
{
  volatile uint32_t DR;
  volatile uint8_t  IDR;
  uint8_t   RESERVED0;
  uint16_t  RESERVED1;
  volatile uint32_t CR;
} CRC_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t SWTRIGR;
  volatile uint32_t DHR12R1;
  volatile uint32_t DHR12L1;
  volatile uint32_t DHR8R1;
  volatile uint32_t DHR12R2;
  volatile uint32_t DHR12L2;
  volatile uint32_t DHR8R2;
  volatile uint32_t DHR12RD;
  volatile uint32_t DHR12LD;
  volatile uint32_t DHR8RD;
  volatile uint32_t DOR1;
  volatile uint32_t DOR2;



} DAC_TypeDef;



 

typedef struct
{
  volatile uint32_t IDCODE;
  volatile uint32_t CR;	
}DBGMCU_TypeDef;



 

typedef struct
{
  volatile uint32_t CCR;
  volatile uint32_t CNDTR;
  volatile uint32_t CPAR;
  volatile uint32_t CMAR;
} DMA_Channel_TypeDef;

typedef struct
{
  volatile uint32_t ISR;
  volatile uint32_t IFCR;
} DMA_TypeDef;



 

typedef struct
{
  volatile uint32_t MACCR;
  volatile uint32_t MACFFR;
  volatile uint32_t MACHTHR;
  volatile uint32_t MACHTLR;
  volatile uint32_t MACMIIAR;
  volatile uint32_t MACMIIDR;
  volatile uint32_t MACFCR;
  volatile uint32_t MACVLANTR;              
       uint32_t RESERVED0[2];
  volatile uint32_t MACRWUFFR;              
  volatile uint32_t MACPMTCSR;
       uint32_t RESERVED1[2];
  volatile uint32_t MACSR;                  
  volatile uint32_t MACIMR;
  volatile uint32_t MACA0HR;
  volatile uint32_t MACA0LR;
  volatile uint32_t MACA1HR;
  volatile uint32_t MACA1LR;
  volatile uint32_t MACA2HR;
  volatile uint32_t MACA2LR;
  volatile uint32_t MACA3HR;
  volatile uint32_t MACA3LR;                
       uint32_t RESERVED2[40];
  volatile uint32_t MMCCR;                  
  volatile uint32_t MMCRIR;
  volatile uint32_t MMCTIR;
  volatile uint32_t MMCRIMR;
  volatile uint32_t MMCTIMR;                
       uint32_t RESERVED3[14];
  volatile uint32_t MMCTGFSCCR;             
  volatile uint32_t MMCTGFMSCCR;
       uint32_t RESERVED4[5];
  volatile uint32_t MMCTGFCR;
       uint32_t RESERVED5[10];
  volatile uint32_t MMCRFCECR;
  volatile uint32_t MMCRFAECR;
       uint32_t RESERVED6[10];
  volatile uint32_t MMCRGUFCR;
       uint32_t RESERVED7[334];
  volatile uint32_t PTPTSCR;
  volatile uint32_t PTPSSIR;
  volatile uint32_t PTPTSHR;
  volatile uint32_t PTPTSLR;
  volatile uint32_t PTPTSHUR;
  volatile uint32_t PTPTSLUR;
  volatile uint32_t PTPTSAR;
  volatile uint32_t PTPTTHR;
  volatile uint32_t PTPTTLR;
       uint32_t RESERVED8[567];
  volatile uint32_t DMABMR;
  volatile uint32_t DMATPDR;
  volatile uint32_t DMARPDR;
  volatile uint32_t DMARDLAR;
  volatile uint32_t DMATDLAR;
  volatile uint32_t DMASR;
  volatile uint32_t DMAOMR;
  volatile uint32_t DMAIER;
  volatile uint32_t DMAMFBOCR;
       uint32_t RESERVED9[9];
  volatile uint32_t DMACHTDR;
  volatile uint32_t DMACHRDR;
  volatile uint32_t DMACHTBAR;
  volatile uint32_t DMACHRBAR;
} ETH_TypeDef;



 

typedef struct
{
  volatile uint32_t IMR;
  volatile uint32_t EMR;
  volatile uint32_t RTSR;
  volatile uint32_t FTSR;
  volatile uint32_t SWIER;
  volatile uint32_t PR;
} EXTI_TypeDef;



 

typedef struct
{
  volatile uint32_t ACR;
  volatile uint32_t KEYR;
  volatile uint32_t OPTKEYR;
  volatile uint32_t SR;
  volatile uint32_t CR;
  volatile uint32_t AR;
  volatile uint32_t RESERVED;
  volatile uint32_t OBR;
  volatile uint32_t WRPR;
#line 920 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"
} FLASH_TypeDef;



 
  
typedef struct
{
  volatile uint16_t RDP;
  volatile uint16_t USER;
  volatile uint16_t Data0;
  volatile uint16_t Data1;
  volatile uint16_t WRP0;
  volatile uint16_t WRP1;
  volatile uint16_t WRP2;
  volatile uint16_t WRP3;
} OB_TypeDef;



 

typedef struct
{
  volatile uint32_t BTCR[8];   
} FSMC_Bank1_TypeDef; 



 
  
typedef struct
{
  volatile uint32_t BWTR[7];
} FSMC_Bank1E_TypeDef;



 
  
typedef struct
{
  volatile uint32_t PCR2;
  volatile uint32_t SR2;
  volatile uint32_t PMEM2;
  volatile uint32_t PATT2;
  uint32_t  RESERVED0;   
  volatile uint32_t ECCR2; 
} FSMC_Bank2_TypeDef;  



 
  
typedef struct
{
  volatile uint32_t PCR3;
  volatile uint32_t SR3;
  volatile uint32_t PMEM3;
  volatile uint32_t PATT3;
  uint32_t  RESERVED0;   
  volatile uint32_t ECCR3; 
} FSMC_Bank3_TypeDef; 



 
  
typedef struct
{
  volatile uint32_t PCR4;
  volatile uint32_t SR4;
  volatile uint32_t PMEM4;
  volatile uint32_t PATT4;
  volatile uint32_t PIO4; 
} FSMC_Bank4_TypeDef; 



 

typedef struct
{
  volatile uint32_t CRL;
  volatile uint32_t CRH;
  volatile uint32_t IDR;
  volatile uint32_t ODR;
  volatile uint32_t BSRR;
  volatile uint32_t BRR;
  volatile uint32_t LCKR;
} GPIO_TypeDef;



 

typedef struct
{
  volatile uint32_t EVCR;
  volatile uint32_t MAPR;
  volatile uint32_t EXTICR[4];
  uint32_t RESERVED0;
  volatile uint32_t MAPR2;  
} AFIO_TypeDef;


 

typedef struct
{
  volatile uint16_t CR1;
  uint16_t  RESERVED0;
  volatile uint16_t CR2;
  uint16_t  RESERVED1;
  volatile uint16_t OAR1;
  uint16_t  RESERVED2;
  volatile uint16_t OAR2;
  uint16_t  RESERVED3;
  volatile uint16_t DR;
  uint16_t  RESERVED4;
  volatile uint16_t SR1;
  uint16_t  RESERVED5;
  volatile uint16_t SR2;
  uint16_t  RESERVED6;
  volatile uint16_t CCR;
  uint16_t  RESERVED7;
  volatile uint16_t TRISE;
  uint16_t  RESERVED8;
} I2C_TypeDef;



 

typedef struct
{
  volatile uint32_t KR;
  volatile uint32_t PR;
  volatile uint32_t RLR;
  volatile uint32_t SR;
} IWDG_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t CSR;
} PWR_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t CFGR;
  volatile uint32_t CIR;
  volatile uint32_t APB2RSTR;
  volatile uint32_t APB1RSTR;
  volatile uint32_t AHBENR;
  volatile uint32_t APB2ENR;
  volatile uint32_t APB1ENR;
  volatile uint32_t BDCR;
  volatile uint32_t CSR;










} RCC_TypeDef;



 

typedef struct
{
  volatile uint16_t CRH;
  uint16_t  RESERVED0;
  volatile uint16_t CRL;
  uint16_t  RESERVED1;
  volatile uint16_t PRLH;
  uint16_t  RESERVED2;
  volatile uint16_t PRLL;
  uint16_t  RESERVED3;
  volatile uint16_t DIVH;
  uint16_t  RESERVED4;
  volatile uint16_t DIVL;
  uint16_t  RESERVED5;
  volatile uint16_t CNTH;
  uint16_t  RESERVED6;
  volatile uint16_t CNTL;
  uint16_t  RESERVED7;
  volatile uint16_t ALRH;
  uint16_t  RESERVED8;
  volatile uint16_t ALRL;
  uint16_t  RESERVED9;
} RTC_TypeDef;



 

typedef struct
{
  volatile uint32_t POWER;
  volatile uint32_t CLKCR;
  volatile uint32_t ARG;
  volatile uint32_t CMD;
  volatile const uint32_t RESPCMD;
  volatile const uint32_t RESP1;
  volatile const uint32_t RESP2;
  volatile const uint32_t RESP3;
  volatile const uint32_t RESP4;
  volatile uint32_t DTIMER;
  volatile uint32_t DLEN;
  volatile uint32_t DCTRL;
  volatile const uint32_t DCOUNT;
  volatile const uint32_t STA;
  volatile uint32_t ICR;
  volatile uint32_t MASK;
  uint32_t  RESERVED0[2];
  volatile const uint32_t FIFOCNT;
  uint32_t  RESERVED1[13];
  volatile uint32_t FIFO;
} SDIO_TypeDef;



 

typedef struct
{
  volatile uint16_t CR1;
  uint16_t  RESERVED0;
  volatile uint16_t CR2;
  uint16_t  RESERVED1;
  volatile uint16_t SR;
  uint16_t  RESERVED2;
  volatile uint16_t DR;
  uint16_t  RESERVED3;
  volatile uint16_t CRCPR;
  uint16_t  RESERVED4;
  volatile uint16_t RXCRCR;
  uint16_t  RESERVED5;
  volatile uint16_t TXCRCR;
  uint16_t  RESERVED6;
  volatile uint16_t I2SCFGR;
  uint16_t  RESERVED7;
  volatile uint16_t I2SPR;
  uint16_t  RESERVED8;  
} SPI_TypeDef;



 

typedef struct
{
  volatile uint16_t CR1;
  uint16_t  RESERVED0;
  volatile uint16_t CR2;
  uint16_t  RESERVED1;
  volatile uint16_t SMCR;
  uint16_t  RESERVED2;
  volatile uint16_t DIER;
  uint16_t  RESERVED3;
  volatile uint16_t SR;
  uint16_t  RESERVED4;
  volatile uint16_t EGR;
  uint16_t  RESERVED5;
  volatile uint16_t CCMR1;
  uint16_t  RESERVED6;
  volatile uint16_t CCMR2;
  uint16_t  RESERVED7;
  volatile uint16_t CCER;
  uint16_t  RESERVED8;
  volatile uint16_t CNT;
  uint16_t  RESERVED9;
  volatile uint16_t PSC;
  uint16_t  RESERVED10;
  volatile uint16_t ARR;
  uint16_t  RESERVED11;
  volatile uint16_t RCR;
  uint16_t  RESERVED12;
  volatile uint16_t CCR1;
  uint16_t  RESERVED13;
  volatile uint16_t CCR2;
  uint16_t  RESERVED14;
  volatile uint16_t CCR3;
  uint16_t  RESERVED15;
  volatile uint16_t CCR4;
  uint16_t  RESERVED16;
  volatile uint16_t BDTR;
  uint16_t  RESERVED17;
  volatile uint16_t DCR;
  uint16_t  RESERVED18;
  volatile uint16_t DMAR;
  uint16_t  RESERVED19;
} TIM_TypeDef;



 
 
typedef struct
{
  volatile uint16_t SR;
  uint16_t  RESERVED0;
  volatile uint16_t DR;
  uint16_t  RESERVED1;
  volatile uint16_t BRR;
  uint16_t  RESERVED2;
  volatile uint16_t CR1;
  uint16_t  RESERVED3;
  volatile uint16_t CR2;
  uint16_t  RESERVED4;
  volatile uint16_t CR3;
  uint16_t  RESERVED5;
  volatile uint16_t GTPR;
  uint16_t  RESERVED6;
} USART_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t CFR;
  volatile uint32_t SR;
} WWDG_TypeDef;



 
  


 











 




#line 1312 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 1335 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"



#line 1354 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"




















 
  


   

#line 1454 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"



 



 
  
  

 
    
 
 
 

 
 
 
 
 

 



 



 


 
 
 
 
 

 











 
#line 1515 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"




 





 
 
 
 
 

 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 





 



 






 
 
 
 
 

 
#line 1691 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 1698 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
 








 








 






#line 1734 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 











 











 













 






#line 1850 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"




#line 1870 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 





#line 1883 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 1902 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 1911 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 1919 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"



















#line 1944 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"












 













#line 1976 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"





#line 1990 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 1997 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 2007 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"











 


















#line 2043 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 2051 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"



















#line 2076 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"












 













#line 2108 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"





#line 2122 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 2129 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 2139 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"











 








 








   
#line 2178 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 2273 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 2300 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"
 
 
 
 
 
 

 




































































 




































































 
#line 2462 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 2480 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 2498 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 2515 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 2533 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 2552 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 

 






 
#line 2579 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"






 








 









 








 








 









 










 




#line 2654 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 










#line 2685 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 





 
#line 2700 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 2709 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

   
#line 2718 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 2727 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 





 
#line 2742 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 2751 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

   
#line 2760 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 2769 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 





 
#line 2784 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 2793 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

   
#line 2802 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 2811 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 





 
#line 2826 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 2835 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

   
#line 2844 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 2853 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 2862 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 2871 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 2881 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
 
 
 
 

 





 


 


 




 
 
 
 
 

 
#line 2945 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 2980 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 3015 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 3050 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 3085 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 





 





 





 





 





 





 





 





 






 
#line 3152 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 



 









 
#line 3176 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"




 




 
#line 3192 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 





 
#line 3214 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
 





 
#line 3229 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"
 
#line 3236 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 




 






 


 


 


 
 
 
 
 

 
#line 3285 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 3307 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 3329 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 3351 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 3373 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 3395 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
 
 
 
 

 
#line 3431 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 3461 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 3471 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"















 
#line 3495 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"















 
#line 3519 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"















 
#line 3543 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"















 
#line 3567 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"















 
#line 3591 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"















 
#line 3615 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"















 


 


 


 


 


 


 


 


 


 



 


 


 



 


 


 


 



 


 


 


 


 
 
 
 
 

 






 
#line 3716 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 3725 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"















  
 
#line 3748 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"


















 








































 


















































 


 


 


 


 


 


 
#line 3883 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 3890 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 3897 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 3904 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"







 
#line 3918 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 3925 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 3932 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 3939 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 3946 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 3953 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 3961 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 3968 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 3975 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 3982 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 3989 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 3996 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 4004 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 4011 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 4018 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 4025 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"





 


 


 


 


 



 
 
 
 
 

 









































 



 


 


 


 


 


 


 



 



 



 


 


 



 
 
 
 
 
 





 






 


 
#line 4167 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 4177 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 


 


 
 
 
 
 

 
















 









#line 4225 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 

























 
#line 4268 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 4282 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 4292 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 




























 





















 




























 





















 
#line 4411 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 


 


 


 


 


 


 


 


 
#line 4446 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"





#line 4457 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 4465 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 4472 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 


 
 
 
 
 

 




 
#line 4494 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 


 


 


 


 


 


 


 


 
 
 
 
 

 


 





 


 



 
 
 
 
 

 
#line 4556 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"



 
#line 4568 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"







 


 
 
 
 
 

 











#line 4606 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 











#line 4629 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 











#line 4652 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 











#line 4675 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 








































 








































 








































 








































 


































 


































 


































 


































 



























 



























 



























 
#line 5072 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 5081 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 5090 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 5101 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 5111 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 5121 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 5131 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 5142 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 5152 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 5162 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 5172 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 5183 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 5193 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 5203 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 5213 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 5224 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 5234 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 5244 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 5254 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 5265 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 5275 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 5285 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 5295 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 5306 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 5316 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 5326 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 5336 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 5347 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 5357 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 5367 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

#line 5377 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 


 


 
 
 
 
 

 




 












 


 






#line 5425 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 


 


 


 


 


 


 


 


 
















 


 
#line 5495 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 5510 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 5536 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 


 


 
 
 
 
 

 
 























 























 























 























 























 























 























 























 
 
#line 5757 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 5769 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 






 
#line 5786 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"



     


 
 


 


 


 


 


 


 


 


 

 


 


 


 


 


 


 


 


 

 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 

 


 


 


 


 


 


 


 


 

 


#line 5930 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"



 


#line 5942 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"



 


#line 5954 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"



 


#line 5966 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"



 


#line 5978 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"



 


#line 5990 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"



 


#line 6002 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"



 


#line 6014 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"



 

 


#line 6028 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"



 


#line 6040 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"



 


#line 6052 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"



 


#line 6064 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"



 


#line 6076 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"



 


#line 6088 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"



 


#line 6100 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"



 


#line 6112 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"



 


#line 6124 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"



 


#line 6136 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"



 


#line 6148 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"



 


#line 6160 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"



 


#line 6172 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"



 


#line 6184 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"



 


#line 6196 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"



 


#line 6208 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"



 
 
 
 
 

 
 
#line 6228 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 6239 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 6257 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"











 





 





 
#line 6295 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 












 
#line 6316 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
 






 




 





 





 






 




 





 





 






   




 





 





 





 




 





 





 





 




 





 





 
 


 
#line 6456 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 6473 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 6490 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 6507 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 6541 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 6575 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 6609 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 6643 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 6677 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 6711 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 6745 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 6779 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 6813 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 6847 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 6881 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 6915 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 6949 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 6983 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 7017 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 7051 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 7085 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 7119 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 7153 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 7187 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 7221 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 7255 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 7289 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 7323 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 7357 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 7391 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 7425 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 7459 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
 
 
 
 

 









#line 7486 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 7494 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 7504 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 


 


 


 


 





















 




 
 
 
 
 

 
#line 7565 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 7574 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"







 



#line 7595 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"



 



 


 
#line 7620 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 7630 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 




 


 
 
 
 
 

 
#line 7656 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 


 



 
#line 7680 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 7689 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"







 
#line 7709 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
#line 7720 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"



 
 
 
 
 

 


#line 7749 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 









#line 7783 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 
 
 
 
 

 









 


 


 





 
#line 7823 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"

 


 









 


 

 



 



 



 



 



 



 



 



#line 8287 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"



 

 

  

#line 1 "..\\..\\User\\stm32f10x_conf.h"



















 

 



 
 
#line 1 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_adc.h"




















 

 







 
#line 1 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"







































 



 



 
    
#line 8327 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"



 

  

 

 
#line 33 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_adc.h"



 



 



 



 

typedef struct
{
  uint32_t ADC_Mode;                      

 

  FunctionalState ADC_ScanConvMode;       

 

  FunctionalState ADC_ContinuousConvMode; 

 

  uint32_t ADC_ExternalTrigConv;          

 

  uint32_t ADC_DataAlign;                 
 

  uint8_t ADC_NbrOfChannel;               

 
}ADC_InitTypeDef;


 



 










 

#line 104 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_adc.h"

#line 115 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_adc.h"


 



 

#line 129 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_adc.h"




#line 139 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_adc.h"

#line 154 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_adc.h"


 



 







 



 

#line 192 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_adc.h"




#line 205 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_adc.h"


 



 

#line 229 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_adc.h"


 



 

















#line 266 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_adc.h"


 



 

#line 282 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_adc.h"


 



 

#line 297 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_adc.h"

#line 305 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_adc.h"


 



 











 



 

#line 338 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_adc.h"


 



 





 



 





 



 





 



 





  




 




 



 





 



 





 



 



 



 



 

void ADC_DeInit(ADC_TypeDef* ADCx);
void ADC_Init(ADC_TypeDef* ADCx, ADC_InitTypeDef* ADC_InitStruct);
void ADC_StructInit(ADC_InitTypeDef* ADC_InitStruct);
void ADC_Cmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_DMACmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_ITConfig(ADC_TypeDef* ADCx, uint16_t ADC_IT, FunctionalState NewState);
void ADC_ResetCalibration(ADC_TypeDef* ADCx);
FlagStatus ADC_GetResetCalibrationStatus(ADC_TypeDef* ADCx);
void ADC_StartCalibration(ADC_TypeDef* ADCx);
FlagStatus ADC_GetCalibrationStatus(ADC_TypeDef* ADCx);
void ADC_SoftwareStartConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
FlagStatus ADC_GetSoftwareStartConvStatus(ADC_TypeDef* ADCx);
void ADC_DiscModeChannelCountConfig(ADC_TypeDef* ADCx, uint8_t Number);
void ADC_DiscModeCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_RegularChannelConfig(ADC_TypeDef* ADCx, uint8_t ADC_Channel, uint8_t Rank, uint8_t ADC_SampleTime);
void ADC_ExternalTrigConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
uint16_t ADC_GetConversionValue(ADC_TypeDef* ADCx);
uint32_t ADC_GetDualModeConversionValue(void);
void ADC_AutoInjectedConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_InjectedDiscModeCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_ExternalTrigInjectedConvConfig(ADC_TypeDef* ADCx, uint32_t ADC_ExternalTrigInjecConv);
void ADC_ExternalTrigInjectedConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_SoftwareStartInjectedConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
FlagStatus ADC_GetSoftwareStartInjectedConvCmdStatus(ADC_TypeDef* ADCx);
void ADC_InjectedChannelConfig(ADC_TypeDef* ADCx, uint8_t ADC_Channel, uint8_t Rank, uint8_t ADC_SampleTime);
void ADC_InjectedSequencerLengthConfig(ADC_TypeDef* ADCx, uint8_t Length);
void ADC_SetInjectedOffset(ADC_TypeDef* ADCx, uint8_t ADC_InjectedChannel, uint16_t Offset);
uint16_t ADC_GetInjectedConversionValue(ADC_TypeDef* ADCx, uint8_t ADC_InjectedChannel);
void ADC_AnalogWatchdogCmd(ADC_TypeDef* ADCx, uint32_t ADC_AnalogWatchdog);
void ADC_AnalogWatchdogThresholdsConfig(ADC_TypeDef* ADCx, uint16_t HighThreshold, uint16_t LowThreshold);
void ADC_AnalogWatchdogSingleChannelConfig(ADC_TypeDef* ADCx, uint8_t ADC_Channel);
void ADC_TempSensorVrefintCmd(FunctionalState NewState);
FlagStatus ADC_GetFlagStatus(ADC_TypeDef* ADCx, uint8_t ADC_FLAG);
void ADC_ClearFlag(ADC_TypeDef* ADCx, uint8_t ADC_FLAG);
ITStatus ADC_GetITStatus(ADC_TypeDef* ADCx, uint16_t ADC_IT);
void ADC_ClearITPendingBit(ADC_TypeDef* ADCx, uint16_t ADC_IT);









 



 



 

 
#line 29 "..\\..\\User\\stm32f10x_conf.h"
#line 1 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_bkp.h"




















 

 







 
#line 33 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_bkp.h"



 



 



 



 



 



 







 



 

#line 78 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_bkp.h"


 



 

#line 128 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_bkp.h"

#line 143 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_bkp.h"




 



 



 



 



 

void BKP_DeInit(void);
void BKP_TamperPinLevelConfig(uint16_t BKP_TamperPinLevel);
void BKP_TamperPinCmd(FunctionalState NewState);
void BKP_ITConfig(FunctionalState NewState);
void BKP_RTCOutputConfig(uint16_t BKP_RTCOutputSource);
void BKP_SetRTCCalibrationValue(uint8_t CalibrationValue);
void BKP_WriteBackupRegister(uint16_t BKP_DR, uint16_t Data);
uint16_t BKP_ReadBackupRegister(uint16_t BKP_DR);
FlagStatus BKP_GetFlagStatus(void);
void BKP_ClearFlag(void);
ITStatus BKP_GetITStatus(void);
void BKP_ClearITPendingBit(void);








 



 



 

 
#line 30 "..\\..\\User\\stm32f10x_conf.h"
#line 1 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_can.h"




















 

 







 
#line 33 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_can.h"



 



 



 






 

typedef struct
{
  uint16_t CAN_Prescaler;   
 
  
  uint8_t CAN_Mode;         

 

  uint8_t CAN_SJW;          



 

  uint8_t CAN_BS1;          

 

  uint8_t CAN_BS2;          


 
  
  FunctionalState CAN_TTCM; 

 
  
  FunctionalState CAN_ABOM;  

 

  FunctionalState CAN_AWUM;  

 

  FunctionalState CAN_NART;  

 

  FunctionalState CAN_RFLM;  

 

  FunctionalState CAN_TXFP;  

 
} CAN_InitTypeDef;



 

typedef struct
{
  uint16_t CAN_FilterIdHigh;         

 

  uint16_t CAN_FilterIdLow;          

 

  uint16_t CAN_FilterMaskIdHigh;     


 

  uint16_t CAN_FilterMaskIdLow;      


 

  uint16_t CAN_FilterFIFOAssignment; 
 
  
  uint8_t CAN_FilterNumber;           

  uint8_t CAN_FilterMode;            
 

  uint8_t CAN_FilterScale;           
 

  FunctionalState CAN_FilterActivation; 
 
} CAN_FilterInitTypeDef;



 

typedef struct
{
  uint32_t StdId;  
 

  uint32_t ExtId;  
 

  uint8_t IDE;     

 

  uint8_t RTR;     

 

  uint8_t DLC;     

 

  uint8_t Data[8]; 
 
} CanTxMsg;



 

typedef struct
{
  uint32_t StdId;  
 

  uint32_t ExtId;  
 

  uint8_t IDE;     

 

  uint8_t RTR;     

 

  uint8_t DLC;     
 

  uint8_t Data[8]; 
 

  uint8_t FMI;     

 
} CanRxMsg;



 



 



 






 



 












 





   










 
  



   







 



 










 



 

#line 301 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_can.h"




 



 

#line 319 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_can.h"





 



 





 



 







 



 








 



 









 



 







 



 



 



 








 



 







 



 







 



 








 



 








 



 






 



 






 




   
                                                                
#line 493 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_can.h"




 



 

 
 

 




 
#line 518 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_can.h"

 



 

 





#line 539 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_can.h"








 

  


 


  


 
#line 565 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_can.h"

 



 






 





#line 590 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_can.h"

#line 597 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_can.h"



 



 
#line 621 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_can.h"



 



 



 



 



 
  
void CAN_DeInit(CAN_TypeDef* CANx);

  
uint8_t CAN_Init(CAN_TypeDef* CANx, CAN_InitTypeDef* CAN_InitStruct);
void CAN_FilterInit(CAN_FilterInitTypeDef* CAN_FilterInitStruct);
void CAN_StructInit(CAN_InitTypeDef* CAN_InitStruct);
void CAN_SlaveStartBank(uint8_t CAN_BankNumber); 
void CAN_DBGFreeze(CAN_TypeDef* CANx, FunctionalState NewState);
void CAN_TTComModeCmd(CAN_TypeDef* CANx, FunctionalState NewState);

 
uint8_t CAN_Transmit(CAN_TypeDef* CANx, CanTxMsg* TxMessage);
uint8_t CAN_TransmitStatus(CAN_TypeDef* CANx, uint8_t TransmitMailbox);
void CAN_CancelTransmit(CAN_TypeDef* CANx, uint8_t Mailbox);

 
void CAN_Receive(CAN_TypeDef* CANx, uint8_t FIFONumber, CanRxMsg* RxMessage);
void CAN_FIFORelease(CAN_TypeDef* CANx, uint8_t FIFONumber);
uint8_t CAN_MessagePending(CAN_TypeDef* CANx, uint8_t FIFONumber);


 
uint8_t CAN_OperatingModeRequest(CAN_TypeDef* CANx, uint8_t CAN_OperatingMode);
uint8_t CAN_Sleep(CAN_TypeDef* CANx);
uint8_t CAN_WakeUp(CAN_TypeDef* CANx);

 
uint8_t CAN_GetLastErrorCode(CAN_TypeDef* CANx);
uint8_t CAN_GetReceiveErrorCounter(CAN_TypeDef* CANx);
uint8_t CAN_GetLSBTransmitErrorCounter(CAN_TypeDef* CANx);

 
void CAN_ITConfig(CAN_TypeDef* CANx, uint32_t CAN_IT, FunctionalState NewState);
FlagStatus CAN_GetFlagStatus(CAN_TypeDef* CANx, uint32_t CAN_FLAG);
void CAN_ClearFlag(CAN_TypeDef* CANx, uint32_t CAN_FLAG);
ITStatus CAN_GetITStatus(CAN_TypeDef* CANx, uint32_t CAN_IT);
void CAN_ClearITPendingBit(CAN_TypeDef* CANx, uint32_t CAN_IT);








 



 



 

 
#line 31 "..\\..\\User\\stm32f10x_conf.h"
#line 1 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_cec.h"




















 

 







 
#line 33 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_cec.h"



 



 
  



 
   


  
typedef struct
{
  uint16_t CEC_BitTimingMode; 
 
  uint16_t CEC_BitPeriodMode; 
 
}CEC_InitTypeDef;



 



  
  


  







 



  







  




  
#line 100 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_cec.h"


  




  



  



  




 



 
   


  
#line 136 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_cec.h"



  
#line 147 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_cec.h"


                               
#line 157 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_cec.h"



  



  



 
 


 



  
void CEC_DeInit(void);
void CEC_Init(CEC_InitTypeDef* CEC_InitStruct);
void CEC_Cmd(FunctionalState NewState);
void CEC_ITConfig(FunctionalState NewState);
void CEC_OwnAddressConfig(uint8_t CEC_OwnAddress);
void CEC_SetPrescaler(uint16_t CEC_Prescaler);
void CEC_SendDataByte(uint8_t Data);
uint8_t CEC_ReceiveDataByte(void);
void CEC_StartOfMessage(void);
void CEC_EndOfMessageCmd(FunctionalState NewState);
FlagStatus CEC_GetFlagStatus(uint32_t CEC_FLAG);
void CEC_ClearFlag(uint32_t CEC_FLAG);
ITStatus CEC_GetITStatus(uint8_t CEC_IT);
void CEC_ClearITPendingBit(uint16_t CEC_IT);









  



  



  

 
#line 32 "..\\..\\User\\stm32f10x_conf.h"
#line 1 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_crc.h"




















 

 







 
#line 33 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_crc.h"



 



 



 



 



 



 



 



 



 

void CRC_ResetDR(void);
uint32_t CRC_CalcCRC(uint32_t Data);
uint32_t CRC_CalcBlockCRC(uint32_t pBuffer[], uint32_t BufferLength);
uint32_t CRC_GetCRC(void);
void CRC_SetIDRegister(uint8_t IDValue);
uint8_t CRC_GetIDRegister(void);








 



 



 

 
#line 33 "..\\..\\User\\stm32f10x_conf.h"
#line 1 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_dac.h"




















 

 







 
#line 33 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_dac.h"



 



 



 



 

typedef struct
{
  uint32_t DAC_Trigger;                      
 

  uint32_t DAC_WaveGeneration;               

 

  uint32_t DAC_LFSRUnmask_TriangleAmplitude; 

 

  uint32_t DAC_OutputBuffer;                 
 
}DAC_InitTypeDef;



 



 



 

#line 94 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_dac.h"

#line 104 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_dac.h"



 



 

#line 119 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_dac.h"


 



 

#line 151 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_dac.h"

#line 176 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_dac.h"


 



 







 



 







 



 

#line 214 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_dac.h"


 



 







 



 




 
#line 261 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_dac.h"



 



 



 



 

void DAC_DeInit(void);
void DAC_Init(uint32_t DAC_Channel, DAC_InitTypeDef* DAC_InitStruct);
void DAC_StructInit(DAC_InitTypeDef* DAC_InitStruct);
void DAC_Cmd(uint32_t DAC_Channel, FunctionalState NewState);



void DAC_DMACmd(uint32_t DAC_Channel, FunctionalState NewState);
void DAC_SoftwareTriggerCmd(uint32_t DAC_Channel, FunctionalState NewState);
void DAC_DualSoftwareTriggerCmd(FunctionalState NewState);
void DAC_WaveGenerationCmd(uint32_t DAC_Channel, uint32_t DAC_Wave, FunctionalState NewState);
void DAC_SetChannel1Data(uint32_t DAC_Align, uint16_t Data);
void DAC_SetChannel2Data(uint32_t DAC_Align, uint16_t Data);
void DAC_SetDualChannelData(uint32_t DAC_Align, uint16_t Data2, uint16_t Data1);
uint16_t DAC_GetDataOutputValue(uint32_t DAC_Channel);
#line 299 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_dac.h"








 



 



 

 
#line 34 "..\\..\\User\\stm32f10x_conf.h"
#line 1 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_dbgmcu.h"




















 

 







 
#line 33 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_dbgmcu.h"



 



 



 



 



 

#line 80 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_dbgmcu.h"
                                              



  



 



 



 

uint32_t DBGMCU_GetREVID(void);
uint32_t DBGMCU_GetDEVID(void);
void DBGMCU_Config(uint32_t DBGMCU_Periph, FunctionalState NewState);








 



 



 

 
#line 35 "..\\..\\User\\stm32f10x_conf.h"
#line 1 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_dma.h"




















 

 







 
#line 33 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_dma.h"



 



 



 



 

typedef struct
{
  uint32_t DMA_PeripheralBaseAddr;  

  uint32_t DMA_MemoryBaseAddr;      

  uint32_t DMA_DIR;                
 

  uint32_t DMA_BufferSize;         

 

  uint32_t DMA_PeripheralInc;      
 

  uint32_t DMA_MemoryInc;          
 

  uint32_t DMA_PeripheralDataSize; 
 

  uint32_t DMA_MemoryDataSize;     
 

  uint32_t DMA_Mode;               


 

  uint32_t DMA_Priority;           
 

  uint32_t DMA_M2M;                
 
}DMA_InitTypeDef;



 



 

#line 107 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_dma.h"



 







 



 







 



 







 



 

#line 154 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_dma.h"


 



 

#line 168 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_dma.h"


 



 






 



 

#line 195 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_dma.h"


 



 







 



 






#line 248 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_dma.h"

#line 269 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_dma.h"



#line 296 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_dma.h"



 



 
#line 332 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_dma.h"

#line 353 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_dma.h"



#line 380 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_dma.h"


 



 





 



 



 



 



 

void DMA_DeInit(DMA_Channel_TypeDef* DMAy_Channelx);
void DMA_Init(DMA_Channel_TypeDef* DMAy_Channelx, DMA_InitTypeDef* DMA_InitStruct);
void DMA_StructInit(DMA_InitTypeDef* DMA_InitStruct);
void DMA_Cmd(DMA_Channel_TypeDef* DMAy_Channelx, FunctionalState NewState);
void DMA_ITConfig(DMA_Channel_TypeDef* DMAy_Channelx, uint32_t DMA_IT, FunctionalState NewState);
void DMA_SetCurrDataCounter(DMA_Channel_TypeDef* DMAy_Channelx, uint16_t DataNumber); 
uint16_t DMA_GetCurrDataCounter(DMA_Channel_TypeDef* DMAy_Channelx);
FlagStatus DMA_GetFlagStatus(uint32_t DMAy_FLAG);
void DMA_ClearFlag(uint32_t DMAy_FLAG);
ITStatus DMA_GetITStatus(uint32_t DMAy_IT);
void DMA_ClearITPendingBit(uint32_t DMAy_IT);








 



 



 

 
#line 36 "..\\..\\User\\stm32f10x_conf.h"
#line 1 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_exti.h"




















 

 







 
#line 33 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_exti.h"



 



 



 



 

typedef enum
{
  EXTI_Mode_Interrupt = 0x00,
  EXTI_Mode_Event = 0x04
}EXTIMode_TypeDef;





 

typedef enum
{
  EXTI_Trigger_Rising = 0x08,
  EXTI_Trigger_Falling = 0x0C,  
  EXTI_Trigger_Rising_Falling = 0x10
}EXTITrigger_TypeDef;






 

typedef struct
{
  uint32_t EXTI_Line;               
 
   
  EXTIMode_TypeDef EXTI_Mode;       
 

  EXTITrigger_TypeDef EXTI_Trigger; 
 

  FunctionalState EXTI_LineCmd;     
  
}EXTI_InitTypeDef;



 



 



 

#line 124 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_exti.h"
                                          
#line 136 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_exti.h"

                    


 



 



 



 



 

void EXTI_DeInit(void);
void EXTI_Init(EXTI_InitTypeDef* EXTI_InitStruct);
void EXTI_StructInit(EXTI_InitTypeDef* EXTI_InitStruct);
void EXTI_GenerateSWInterrupt(uint32_t EXTI_Line);
FlagStatus EXTI_GetFlagStatus(uint32_t EXTI_Line);
void EXTI_ClearFlag(uint32_t EXTI_Line);
ITStatus EXTI_GetITStatus(uint32_t EXTI_Line);
void EXTI_ClearITPendingBit(uint32_t EXTI_Line);








 



 



 

 
#line 37 "..\\..\\User\\stm32f10x_conf.h"
#line 1 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_flash.h"




















 

 







 
#line 33 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_flash.h"



 



 



 



 

typedef enum
{ 
  FLASH_BUSY = 1,
  FLASH_ERROR_PG,
  FLASH_ERROR_WRP,
  FLASH_COMPLETE,
  FLASH_TIMEOUT
}FLASH_Status;



 



 



 

#line 77 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_flash.h"


 



 







 



 







 



 

 
#line 118 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_flash.h"

 
#line 144 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_flash.h"

 
#line 211 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_flash.h"











 



 







 



 







 



 





#line 270 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_flash.h"


 


 
#line 291 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_flash.h"






 



 
#line 333 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_flash.h"





 
#line 346 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_flash.h"



 



 



 



 



 

 
void FLASH_SetLatency(uint32_t FLASH_Latency);
void FLASH_HalfCycleAccessCmd(uint32_t FLASH_HalfCycleAccess);
void FLASH_PrefetchBufferCmd(uint32_t FLASH_PrefetchBuffer);
void FLASH_Unlock(void);
void FLASH_Lock(void);
FLASH_Status FLASH_ErasePage(uint32_t Page_Address);
FLASH_Status FLASH_EraseAllPages(void);
FLASH_Status FLASH_EraseOptionBytes(void);
FLASH_Status FLASH_ProgramWord(uint32_t Address, uint32_t Data);
FLASH_Status FLASH_ProgramHalfWord(uint32_t Address, uint16_t Data);
FLASH_Status FLASH_ProgramOptionByteData(uint32_t Address, uint8_t Data);
FLASH_Status FLASH_EnableWriteProtection(uint32_t FLASH_Pages);
FLASH_Status FLASH_ReadOutProtection(FunctionalState NewState);
FLASH_Status FLASH_UserOptionByteConfig(uint16_t OB_IWDG, uint16_t OB_STOP, uint16_t OB_STDBY);
uint32_t FLASH_GetUserOptionByte(void);
uint32_t FLASH_GetWriteProtectionOptionByte(void);
FlagStatus FLASH_GetReadOutProtectionStatus(void);
FlagStatus FLASH_GetPrefetchBufferStatus(void);
void FLASH_ITConfig(uint32_t FLASH_IT, FunctionalState NewState);
FlagStatus FLASH_GetFlagStatus(uint32_t FLASH_FLAG);
void FLASH_ClearFlag(uint32_t FLASH_FLAG);
FLASH_Status FLASH_GetStatus(void);
FLASH_Status FLASH_WaitForLastOperation(uint32_t Timeout);

 
void FLASH_UnlockBank1(void);
void FLASH_LockBank1(void);
FLASH_Status FLASH_EraseAllBank1Pages(void);
FLASH_Status FLASH_GetBank1Status(void);
FLASH_Status FLASH_WaitForLastBank1Operation(uint32_t Timeout);

#line 408 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_flash.h"








 



 



 

 
#line 38 "..\\..\\User\\stm32f10x_conf.h"
#line 1 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_fsmc.h"




















 

 







 
#line 33 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_fsmc.h"



 



 



 



 

typedef struct
{
  uint32_t FSMC_AddressSetupTime;       


 

  uint32_t FSMC_AddressHoldTime;        


 

  uint32_t FSMC_DataSetupTime;          


 

  uint32_t FSMC_BusTurnAroundDuration;  


 

  uint32_t FSMC_CLKDivision;            

 

  uint32_t FSMC_DataLatency;            





 

  uint32_t FSMC_AccessMode;             
 
}FSMC_NORSRAMTimingInitTypeDef;



 

typedef struct
{
  uint32_t FSMC_Bank;                
 

  uint32_t FSMC_DataAddressMux;      

 

  uint32_t FSMC_MemoryType;          

 

  uint32_t FSMC_MemoryDataWidth;     
 

  uint32_t FSMC_BurstAccessMode;     

 
                                       
  uint32_t FSMC_AsynchronousWait;     

 

  uint32_t FSMC_WaitSignalPolarity;  

 

  uint32_t FSMC_WrapMode;            

 

  uint32_t FSMC_WaitSignalActive;    


 

  uint32_t FSMC_WriteOperation;      
 

  uint32_t FSMC_WaitSignal;          

 

  uint32_t FSMC_ExtendedMode;        
 

  uint32_t FSMC_WriteBurst;          
  

  FSMC_NORSRAMTimingInitTypeDef* FSMC_ReadWriteTimingStruct;    

  FSMC_NORSRAMTimingInitTypeDef* FSMC_WriteTimingStruct;            
}FSMC_NORSRAMInitTypeDef;



 

typedef struct
{
  uint32_t FSMC_SetupTime;      



 

  uint32_t FSMC_WaitSetupTime;  



 

  uint32_t FSMC_HoldSetupTime;  




 

  uint32_t FSMC_HiZSetupTime;   



 
}FSMC_NAND_PCCARDTimingInitTypeDef;



 

typedef struct
{
  uint32_t FSMC_Bank;              
 

  uint32_t FSMC_Waitfeature;      
 

  uint32_t FSMC_MemoryDataWidth;  
 

  uint32_t FSMC_ECC;              
 

  uint32_t FSMC_ECCPageSize;      
 

  uint32_t FSMC_TCLRSetupTime;    

 

  uint32_t FSMC_TARSetupTime;     

  

  FSMC_NAND_PCCARDTimingInitTypeDef*  FSMC_CommonSpaceTimingStruct;     

  FSMC_NAND_PCCARDTimingInitTypeDef*  FSMC_AttributeSpaceTimingStruct;  
}FSMC_NANDInitTypeDef;



 

typedef struct
{
  uint32_t FSMC_Waitfeature;    
 

  uint32_t FSMC_TCLRSetupTime;  

 

  uint32_t FSMC_TARSetupTime;   

  

  
  FSMC_NAND_PCCARDTimingInitTypeDef*  FSMC_CommonSpaceTimingStruct;  

  FSMC_NAND_PCCARDTimingInitTypeDef*  FSMC_AttributeSpaceTimingStruct;    
  
  FSMC_NAND_PCCARDTimingInitTypeDef*  FSMC_IOSpaceTimingStruct;    
}FSMC_PCCARDInitTypeDef;



 



 



 






 



   




 



     



 



















 



 








 



 

#line 317 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_fsmc.h"



 



 








 



 







 
  


 







 
  


 








 



 








 



 








 



 





                              


 



 







 



 









 



 







 



 





 



 





 



 





 



 





 



 





 



 





 



 

#line 521 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_fsmc.h"



 



 
  


 



 








 




 








 



 

#line 577 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_fsmc.h"



 



 





 



 





 



 





 



 





 



 





 



 





 



 

#line 653 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_fsmc.h"


 



 

#line 669 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_fsmc.h"





 



 



 



 



 



 

void FSMC_NORSRAMDeInit(uint32_t FSMC_Bank);
void FSMC_NANDDeInit(uint32_t FSMC_Bank);
void FSMC_PCCARDDeInit(void);
void FSMC_NORSRAMInit(FSMC_NORSRAMInitTypeDef* FSMC_NORSRAMInitStruct);
void FSMC_NANDInit(FSMC_NANDInitTypeDef* FSMC_NANDInitStruct);
void FSMC_PCCARDInit(FSMC_PCCARDInitTypeDef* FSMC_PCCARDInitStruct);
void FSMC_NORSRAMStructInit(FSMC_NORSRAMInitTypeDef* FSMC_NORSRAMInitStruct);
void FSMC_NANDStructInit(FSMC_NANDInitTypeDef* FSMC_NANDInitStruct);
void FSMC_PCCARDStructInit(FSMC_PCCARDInitTypeDef* FSMC_PCCARDInitStruct);
void FSMC_NORSRAMCmd(uint32_t FSMC_Bank, FunctionalState NewState);
void FSMC_NANDCmd(uint32_t FSMC_Bank, FunctionalState NewState);
void FSMC_PCCARDCmd(FunctionalState NewState);
void FSMC_NANDECCCmd(uint32_t FSMC_Bank, FunctionalState NewState);
uint32_t FSMC_GetECC(uint32_t FSMC_Bank);
void FSMC_ITConfig(uint32_t FSMC_Bank, uint32_t FSMC_IT, FunctionalState NewState);
FlagStatus FSMC_GetFlagStatus(uint32_t FSMC_Bank, uint32_t FSMC_FLAG);
void FSMC_ClearFlag(uint32_t FSMC_Bank, uint32_t FSMC_FLAG);
ITStatus FSMC_GetITStatus(uint32_t FSMC_Bank, uint32_t FSMC_IT);
void FSMC_ClearITPendingBit(uint32_t FSMC_Bank, uint32_t FSMC_IT);








 



 



  

 
#line 39 "..\\..\\User\\stm32f10x_conf.h"
#line 1 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_gpio.h"




















 

 







 
#line 33 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_gpio.h"



 



 



 

#line 53 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_gpio.h"
                                     


 

typedef enum
{ 
  GPIO_Speed_10MHz = 1,
  GPIO_Speed_2MHz, 
  GPIO_Speed_50MHz
}GPIOSpeed_TypeDef;





 

typedef enum
{ GPIO_Mode_AIN = 0x0,
  GPIO_Mode_IN_FLOATING = 0x04,
  GPIO_Mode_IPD = 0x28,
  GPIO_Mode_IPU = 0x48,
  GPIO_Mode_Out_OD = 0x14,
  GPIO_Mode_Out_PP = 0x10,
  GPIO_Mode_AF_OD = 0x1C,
  GPIO_Mode_AF_PP = 0x18
}GPIOMode_TypeDef;








 

typedef struct
{
  uint16_t GPIO_Pin;             
 

  GPIOSpeed_TypeDef GPIO_Speed;  
 

  GPIOMode_TypeDef GPIO_Mode;    
 
}GPIO_InitTypeDef;




 

typedef enum
{ Bit_RESET = 0,
  Bit_SET
}BitAction;





 



 



 

#line 144 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_gpio.h"



#line 163 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_gpio.h"



 



 

#line 204 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_gpio.h"







#line 217 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_gpio.h"






#line 245 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_gpio.h"
                              


  



 

#line 266 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_gpio.h"

#line 274 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_gpio.h"



 



 

#line 299 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_gpio.h"

#line 316 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_gpio.h"



 



  








                                                 


 



 



 



 

void GPIO_DeInit(GPIO_TypeDef* GPIOx);
void GPIO_AFIODeInit(void);
void GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_InitStruct);
void GPIO_StructInit(GPIO_InitTypeDef* GPIO_InitStruct);
uint8_t GPIO_ReadInputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
uint16_t GPIO_ReadInputData(GPIO_TypeDef* GPIOx);
uint8_t GPIO_ReadOutputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
uint16_t GPIO_ReadOutputData(GPIO_TypeDef* GPIOx);
void GPIO_SetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void GPIO_ResetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void GPIO_WriteBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, BitAction BitVal);
void GPIO_Write(GPIO_TypeDef* GPIOx, uint16_t PortVal);
void GPIO_PinLockConfig(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void GPIO_EventOutputConfig(uint8_t GPIO_PortSource, uint8_t GPIO_PinSource);
void GPIO_EventOutputCmd(FunctionalState NewState);
void GPIO_PinRemapConfig(uint32_t GPIO_Remap, FunctionalState NewState);
void GPIO_EXTILineConfig(uint8_t GPIO_PortSource, uint8_t GPIO_PinSource);
void GPIO_ETH_MediaInterfaceConfig(uint32_t GPIO_ETH_MediaInterface);








 



 



 

 
#line 40 "..\\..\\User\\stm32f10x_conf.h"
#line 1 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_i2c.h"




















 

 







 
#line 33 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_i2c.h"



 



 



 



 

typedef struct
{
  uint32_t I2C_ClockSpeed;          
 

  uint16_t I2C_Mode;                
 

  uint16_t I2C_DutyCycle;           
 

  uint16_t I2C_OwnAddress1;         
 

  uint16_t I2C_Ack;                 
 

  uint16_t I2C_AcknowledgedAddress; 
 
}I2C_InitTypeDef;



  




 





 

#line 92 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_i2c.h"


 



 







  



 







 



 







 



 







  



 

#line 166 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_i2c.h"


 



 







 



 







  



 







  



 







  



 

#line 236 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_i2c.h"



#line 246 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_i2c.h"


 



 



 

#line 265 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_i2c.h"



 

#line 284 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_i2c.h"



#line 298 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_i2c.h"


 



 




 







 
 

























 

 


 





























 

  
 


 
 

 






 
























 

    
 



 



 



























 

  
 

 


 
 


 


 

#line 496 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_i2c.h"


 



 




 



 




 



 



 



 



 

void I2C_DeInit(I2C_TypeDef* I2Cx);
void I2C_Init(I2C_TypeDef* I2Cx, I2C_InitTypeDef* I2C_InitStruct);
void I2C_StructInit(I2C_InitTypeDef* I2C_InitStruct);
void I2C_Cmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_DMACmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_DMALastTransferCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_GenerateSTART(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_GenerateSTOP(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_AcknowledgeConfig(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_OwnAddress2Config(I2C_TypeDef* I2Cx, uint8_t Address);
void I2C_DualAddressCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_GeneralCallCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_ITConfig(I2C_TypeDef* I2Cx, uint16_t I2C_IT, FunctionalState NewState);
void I2C_SendData(I2C_TypeDef* I2Cx, uint8_t Data);
uint8_t I2C_ReceiveData(I2C_TypeDef* I2Cx);
void I2C_Send7bitAddress(I2C_TypeDef* I2Cx, uint8_t Address, uint8_t I2C_Direction);
uint16_t I2C_ReadRegister(I2C_TypeDef* I2Cx, uint8_t I2C_Register);
void I2C_SoftwareResetCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_NACKPositionConfig(I2C_TypeDef* I2Cx, uint16_t I2C_NACKPosition);
void I2C_SMBusAlertConfig(I2C_TypeDef* I2Cx, uint16_t I2C_SMBusAlert);
void I2C_TransmitPEC(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_PECPositionConfig(I2C_TypeDef* I2Cx, uint16_t I2C_PECPosition);
void I2C_CalculatePEC(I2C_TypeDef* I2Cx, FunctionalState NewState);
uint8_t I2C_GetPEC(I2C_TypeDef* I2Cx);
void I2C_ARPCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_StretchClockCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_FastModeDutyCycleConfig(I2C_TypeDef* I2Cx, uint16_t I2C_DutyCycle);













































































 





 
ErrorStatus I2C_CheckEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT);




 
uint32_t I2C_GetLastEvent(I2C_TypeDef* I2Cx);




 
FlagStatus I2C_GetFlagStatus(I2C_TypeDef* I2Cx, uint32_t I2C_FLAG);



 

void I2C_ClearFlag(I2C_TypeDef* I2Cx, uint32_t I2C_FLAG);
ITStatus I2C_GetITStatus(I2C_TypeDef* I2Cx, uint32_t I2C_IT);
void I2C_ClearITPendingBit(I2C_TypeDef* I2Cx, uint32_t I2C_IT);








  



  



  

 
#line 41 "..\\..\\User\\stm32f10x_conf.h"
#line 1 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_iwdg.h"




















 

 







 
#line 33 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_iwdg.h"



 



 



 



 



 



 







 



 

#line 84 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_iwdg.h"


 



 







 



 



 



 



 

void IWDG_WriteAccessCmd(uint16_t IWDG_WriteAccess);
void IWDG_SetPrescaler(uint8_t IWDG_Prescaler);
void IWDG_SetReload(uint16_t Reload);
void IWDG_ReloadCounter(void);
void IWDG_Enable(void);
FlagStatus IWDG_GetFlagStatus(uint16_t IWDG_FLAG);








 



 



 

 
#line 42 "..\\..\\User\\stm32f10x_conf.h"
#line 1 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_pwr.h"




















 

 







 
#line 33 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_pwr.h"



 



  



  



  



  



  

#line 70 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_pwr.h"


 



 







 



 




 


 



 










 



 



 



 



 

void PWR_DeInit(void);
void PWR_BackupAccessCmd(FunctionalState NewState);
void PWR_PVDCmd(FunctionalState NewState);
void PWR_PVDLevelConfig(uint32_t PWR_PVDLevel);
void PWR_WakeUpPinCmd(FunctionalState NewState);
void PWR_EnterSTOPMode(uint32_t PWR_Regulator, uint8_t PWR_STOPEntry);
void PWR_EnterSTANDBYMode(void);
FlagStatus PWR_GetFlagStatus(uint32_t PWR_FLAG);
void PWR_ClearFlag(uint32_t PWR_FLAG);








 



 



 

 
#line 43 "..\\..\\User\\stm32f10x_conf.h"
#line 1 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_rcc.h"




















 

 







 
#line 33 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_rcc.h"



 



 



 

typedef struct
{
  uint32_t SYSCLK_Frequency;   
  uint32_t HCLK_Frequency;     
  uint32_t PCLK1_Frequency;    
  uint32_t PCLK2_Frequency;    
  uint32_t ADCCLK_Frequency;   
}RCC_ClocksTypeDef;



 



 



 









  



 



#line 94 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_rcc.h"



  



 
#line 126 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_rcc.h"

#line 141 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_rcc.h"


 



 
#line 175 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_rcc.h"


 




 
#line 196 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_rcc.h"


 

#line 283 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_rcc.h"




 

#line 295 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_rcc.h"


 



 

#line 317 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_rcc.h"


  



 

#line 333 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_rcc.h"


 



 

#line 347 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_rcc.h"

#line 364 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_rcc.h"




 




 








 
#line 396 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_rcc.h"


#line 423 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_rcc.h"
  



 

#line 435 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_rcc.h"


 



 








 



 

#line 462 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_rcc.h"


 



 







#line 489 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_rcc.h"


 



 

#line 518 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_rcc.h"




  



 

#line 553 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_rcc.h"
 




 



 







#line 586 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_rcc.h"



 



 

#line 606 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_rcc.h"

#line 625 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_rcc.h"




 



 



 



 



 

void RCC_DeInit(void);
void RCC_HSEConfig(uint32_t RCC_HSE);
ErrorStatus RCC_WaitForHSEStartUp(void);
void RCC_AdjustHSICalibrationValue(uint8_t HSICalibrationValue);
void RCC_HSICmd(FunctionalState NewState);
void RCC_PLLConfig(uint32_t RCC_PLLSource, uint32_t RCC_PLLMul);
void RCC_PLLCmd(FunctionalState NewState);





#line 666 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_rcc.h"

void RCC_SYSCLKConfig(uint32_t RCC_SYSCLKSource);
uint8_t RCC_GetSYSCLKSource(void);
void RCC_HCLKConfig(uint32_t RCC_SYSCLK);
void RCC_PCLK1Config(uint32_t RCC_HCLK);
void RCC_PCLK2Config(uint32_t RCC_HCLK);
void RCC_ITConfig(uint8_t RCC_IT, FunctionalState NewState);


 void RCC_USBCLKConfig(uint32_t RCC_USBCLKSource);




void RCC_ADCCLKConfig(uint32_t RCC_PCLK2);






void RCC_LSEConfig(uint8_t RCC_LSE);
void RCC_LSICmd(FunctionalState NewState);
void RCC_RTCCLKConfig(uint32_t RCC_RTCCLKSource);
void RCC_RTCCLKCmd(FunctionalState NewState);
void RCC_GetClocksFreq(RCC_ClocksTypeDef* RCC_Clocks);
void RCC_AHBPeriphClockCmd(uint32_t RCC_AHBPeriph, FunctionalState NewState);
void RCC_APB2PeriphClockCmd(uint32_t RCC_APB2Periph, FunctionalState NewState);
void RCC_APB1PeriphClockCmd(uint32_t RCC_APB1Periph, FunctionalState NewState);





void RCC_APB2PeriphResetCmd(uint32_t RCC_APB2Periph, FunctionalState NewState);
void RCC_APB1PeriphResetCmd(uint32_t RCC_APB1Periph, FunctionalState NewState);
void RCC_BackupResetCmd(FunctionalState NewState);
void RCC_ClockSecuritySystemCmd(FunctionalState NewState);
void RCC_MCOConfig(uint8_t RCC_MCO);
FlagStatus RCC_GetFlagStatus(uint8_t RCC_FLAG);
void RCC_ClearFlag(void);
ITStatus RCC_GetITStatus(uint8_t RCC_IT);
void RCC_ClearITPendingBit(uint8_t RCC_IT);








 



 



  

 
#line 44 "..\\..\\User\\stm32f10x_conf.h"
#line 1 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_rtc.h"




















 

 







 
#line 33 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_rtc.h"



 



  



  



  



 



 

#line 64 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_rtc.h"


  



 

#line 82 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_rtc.h"



 



 



 



 



 

void RTC_ITConfig(uint16_t RTC_IT, FunctionalState NewState);
void RTC_EnterConfigMode(void);
void RTC_ExitConfigMode(void);
uint32_t  RTC_GetCounter(void);
void RTC_SetCounter(uint32_t CounterValue);
void RTC_SetPrescaler(uint32_t PrescalerValue);
void RTC_SetAlarm(uint32_t AlarmValue);
uint32_t  RTC_GetDivider(void);
void RTC_WaitForLastTask(void);
void RTC_WaitForSynchro(void);
FlagStatus RTC_GetFlagStatus(uint16_t RTC_FLAG);
void RTC_ClearFlag(uint16_t RTC_FLAG);
ITStatus RTC_GetITStatus(uint16_t RTC_IT);
void RTC_ClearITPendingBit(uint16_t RTC_IT);








 



 



 

 
#line 45 "..\\..\\User\\stm32f10x_conf.h"
#line 1 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_sdio.h"




















 

 







 
#line 33 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_sdio.h"



 



 



 

typedef struct
{
  uint32_t SDIO_ClockEdge;            
 

  uint32_t SDIO_ClockBypass;          

 

  uint32_t SDIO_ClockPowerSave;       

 

  uint32_t SDIO_BusWide;              
 

  uint32_t SDIO_HardwareFlowControl;  
 

  uint8_t SDIO_ClockDiv;              
 
                                           
} SDIO_InitTypeDef;

typedef struct
{
  uint32_t SDIO_Argument;  


 

  uint32_t SDIO_CmdIndex;   

  uint32_t SDIO_Response;  
 

  uint32_t SDIO_Wait;      
 

  uint32_t SDIO_CPSM;      

 
} SDIO_CmdInitTypeDef;

typedef struct
{
  uint32_t SDIO_DataTimeOut;     

  uint32_t SDIO_DataLength;      
 
  uint32_t SDIO_DataBlockSize;  
 
 
  uint32_t SDIO_TransferDir;    

 
 
  uint32_t SDIO_TransferMode;   
 
 
  uint32_t SDIO_DPSM;           

 
} SDIO_DataInitTypeDef;



  



 



 







 



 







  



 







 



 









 



 







 



 






  




 

#line 222 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_sdio.h"


  



 




 



 

#line 245 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_sdio.h"


 



 








 



 






  



 

#line 283 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_sdio.h"


 



 




 



 

#line 330 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_sdio.h"


 



 







 



 







 



 






 



 

#line 421 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_sdio.h"



#line 448 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_sdio.h"





 



 







 



 



 



 



 

void SDIO_DeInit(void);
void SDIO_Init(SDIO_InitTypeDef* SDIO_InitStruct);
void SDIO_StructInit(SDIO_InitTypeDef* SDIO_InitStruct);
void SDIO_ClockCmd(FunctionalState NewState);
void SDIO_SetPowerState(uint32_t SDIO_PowerState);
uint32_t SDIO_GetPowerState(void);
void SDIO_ITConfig(uint32_t SDIO_IT, FunctionalState NewState);
void SDIO_DMACmd(FunctionalState NewState);
void SDIO_SendCommand(SDIO_CmdInitTypeDef *SDIO_CmdInitStruct);
void SDIO_CmdStructInit(SDIO_CmdInitTypeDef* SDIO_CmdInitStruct);
uint8_t SDIO_GetCommandResponse(void);
uint32_t SDIO_GetResponse(uint32_t SDIO_RESP);
void SDIO_DataConfig(SDIO_DataInitTypeDef* SDIO_DataInitStruct);
void SDIO_DataStructInit(SDIO_DataInitTypeDef* SDIO_DataInitStruct);
uint32_t SDIO_GetDataCounter(void);
uint32_t SDIO_ReadData(void);
void SDIO_WriteData(uint32_t Data);
uint32_t SDIO_GetFIFOCount(void);
void SDIO_StartSDIOReadWait(FunctionalState NewState);
void SDIO_StopSDIOReadWait(FunctionalState NewState);
void SDIO_SetSDIOReadWaitMode(uint32_t SDIO_ReadWaitMode);
void SDIO_SetSDIOOperation(FunctionalState NewState);
void SDIO_SendSDIOSuspendCmd(FunctionalState NewState);
void SDIO_CommandCompletionCmd(FunctionalState NewState);
void SDIO_CEATAITCmd(FunctionalState NewState);
void SDIO_SendCEATACmd(FunctionalState NewState);
FlagStatus SDIO_GetFlagStatus(uint32_t SDIO_FLAG);
void SDIO_ClearFlag(uint32_t SDIO_FLAG);
ITStatus SDIO_GetITStatus(uint32_t SDIO_IT);
void SDIO_ClearITPendingBit(uint32_t SDIO_IT);








 



 



 

 
#line 46 "..\\..\\User\\stm32f10x_conf.h"
#line 1 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_spi.h"




















 

 







 
#line 33 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_spi.h"



 



  



 



 

typedef struct
{
  uint16_t SPI_Direction;           
 

  uint16_t SPI_Mode;                
 

  uint16_t SPI_DataSize;            
 

  uint16_t SPI_CPOL;                
 

  uint16_t SPI_CPHA;                
 

  uint16_t SPI_NSS;                 

 
 
  uint16_t SPI_BaudRatePrescaler;   



 

  uint16_t SPI_FirstBit;            
 

  uint16_t SPI_CRCPolynomial;        
}SPI_InitTypeDef;



 

typedef struct
{

  uint16_t I2S_Mode;         
 

  uint16_t I2S_Standard;     
 

  uint16_t I2S_DataFormat;   
 

  uint16_t I2S_MCLKOutput;   
 

  uint32_t I2S_AudioFreq;    
 

  uint16_t I2S_CPOL;         
 
}I2S_InitTypeDef;



 



 










 
  
#line 136 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_spi.h"


 



 







 



 







  



 







 



 







 



 







  



 

#line 220 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_spi.h"


  



 







 



 

#line 248 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_spi.h"


 



 

#line 266 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_spi.h"


 



 

#line 282 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_spi.h"


  



 







 



 

#line 312 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_spi.h"






  



 







 



 






 



 







 



 






 



 







 



 

#line 396 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_spi.h"


 



 

#line 417 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_spi.h"


 



 




 



 



 



 



 

void SPI_I2S_DeInit(SPI_TypeDef* SPIx);
void SPI_Init(SPI_TypeDef* SPIx, SPI_InitTypeDef* SPI_InitStruct);
void I2S_Init(SPI_TypeDef* SPIx, I2S_InitTypeDef* I2S_InitStruct);
void SPI_StructInit(SPI_InitTypeDef* SPI_InitStruct);
void I2S_StructInit(I2S_InitTypeDef* I2S_InitStruct);
void SPI_Cmd(SPI_TypeDef* SPIx, FunctionalState NewState);
void I2S_Cmd(SPI_TypeDef* SPIx, FunctionalState NewState);
void SPI_I2S_ITConfig(SPI_TypeDef* SPIx, uint8_t SPI_I2S_IT, FunctionalState NewState);
void SPI_I2S_DMACmd(SPI_TypeDef* SPIx, uint16_t SPI_I2S_DMAReq, FunctionalState NewState);
void SPI_I2S_SendData(SPI_TypeDef* SPIx, uint16_t Data);
uint16_t SPI_I2S_ReceiveData(SPI_TypeDef* SPIx);
void SPI_NSSInternalSoftwareConfig(SPI_TypeDef* SPIx, uint16_t SPI_NSSInternalSoft);
void SPI_SSOutputCmd(SPI_TypeDef* SPIx, FunctionalState NewState);
void SPI_DataSizeConfig(SPI_TypeDef* SPIx, uint16_t SPI_DataSize);
void SPI_TransmitCRC(SPI_TypeDef* SPIx);
void SPI_CalculateCRC(SPI_TypeDef* SPIx, FunctionalState NewState);
uint16_t SPI_GetCRC(SPI_TypeDef* SPIx, uint8_t SPI_CRC);
uint16_t SPI_GetCRCPolynomial(SPI_TypeDef* SPIx);
void SPI_BiDirectionalLineConfig(SPI_TypeDef* SPIx, uint16_t SPI_Direction);
FlagStatus SPI_I2S_GetFlagStatus(SPI_TypeDef* SPIx, uint16_t SPI_I2S_FLAG);
void SPI_I2S_ClearFlag(SPI_TypeDef* SPIx, uint16_t SPI_I2S_FLAG);
ITStatus SPI_I2S_GetITStatus(SPI_TypeDef* SPIx, uint8_t SPI_I2S_IT);
void SPI_I2S_ClearITPendingBit(SPI_TypeDef* SPIx, uint8_t SPI_I2S_IT);








 



 



 

 
#line 47 "..\\..\\User\\stm32f10x_conf.h"
#line 1 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_tim.h"




















 

 







 
#line 33 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_tim.h"



 



  



  




 

typedef struct
{
  uint16_t TIM_Prescaler;         
 

  uint16_t TIM_CounterMode;       
 

  uint16_t TIM_Period;            

  

  uint16_t TIM_ClockDivision;     
 

  uint8_t TIM_RepetitionCounter;  






 
} TIM_TimeBaseInitTypeDef;       



 

typedef struct
{
  uint16_t TIM_OCMode;        
 

  uint16_t TIM_OutputState;   
 

  uint16_t TIM_OutputNState;  

 

  uint16_t TIM_Pulse;         
 

  uint16_t TIM_OCPolarity;    
 

  uint16_t TIM_OCNPolarity;   

 

  uint16_t TIM_OCIdleState;   

 

  uint16_t TIM_OCNIdleState;  

 
} TIM_OCInitTypeDef;



 

typedef struct
{

  uint16_t TIM_Channel;      
 

  uint16_t TIM_ICPolarity;   
 

  uint16_t TIM_ICSelection;  
 

  uint16_t TIM_ICPrescaler;  
 

  uint16_t TIM_ICFilter;     
 
} TIM_ICInitTypeDef;




 

typedef struct
{

  uint16_t TIM_OSSRState;        
 

  uint16_t TIM_OSSIState;        
 

  uint16_t TIM_LOCKLevel;        
  

  uint16_t TIM_DeadTime;         

 

  uint16_t TIM_Break;            
 

  uint16_t TIM_BreakPolarity;    
 

  uint16_t TIM_AutomaticOutput;  
 
} TIM_BDTRInitTypeDef;



 

#line 186 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_tim.h"

 



 






 
#line 205 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_tim.h"
									                                 
 
#line 216 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_tim.h"

                                             
#line 225 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_tim.h"

 
#line 236 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_tim.h"

 
#line 249 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_tim.h"

                                         
#line 266 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_tim.h"

 
#line 279 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_tim.h"
                                                                                                                                                                                                                          


  



 

#line 308 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_tim.h"


 



 







  



 

#line 341 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_tim.h"


  



 

#line 355 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_tim.h"


 



 

#line 373 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_tim.h"


  



 







 



 
  






 



 







  



 







  



 







  



 







  



 







  



 







  



 







  



 

#line 497 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_tim.h"


  



 







 



 







  



 







  



 







  



 

#line 561 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_tim.h"


  



 

#line 577 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_tim.h"


  



 

#line 593 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_tim.h"


  



 

#line 610 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_tim.h"

#line 619 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_tim.h"


  



 

#line 665 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_tim.h"


  



 

#line 709 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_tim.h"


  



 

#line 725 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_tim.h"



  



 

#line 742 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_tim.h"


  



 

#line 770 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_tim.h"


  



 

#line 784 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_tim.h"


  



  






 



 







  



 







  



 

#line 833 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_tim.h"


  




 

#line 851 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_tim.h"



  



 

#line 866 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_tim.h"


  



 







  



 





                                     


  



 







  



 

#line 927 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_tim.h"


  



 

#line 943 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_tim.h"


  



 







  



 

#line 987 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_tim.h"
                               
                               



  



 




  



 




  



 

#line 1034 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_tim.h"


 



 



 



  



 

void TIM_DeInit(TIM_TypeDef* TIMx);
void TIM_TimeBaseInit(TIM_TypeDef* TIMx, TIM_TimeBaseInitTypeDef* TIM_TimeBaseInitStruct);
void TIM_OC1Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_OC2Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_OC3Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_OC4Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_ICInit(TIM_TypeDef* TIMx, TIM_ICInitTypeDef* TIM_ICInitStruct);
void TIM_PWMIConfig(TIM_TypeDef* TIMx, TIM_ICInitTypeDef* TIM_ICInitStruct);
void TIM_BDTRConfig(TIM_TypeDef* TIMx, TIM_BDTRInitTypeDef *TIM_BDTRInitStruct);
void TIM_TimeBaseStructInit(TIM_TimeBaseInitTypeDef* TIM_TimeBaseInitStruct);
void TIM_OCStructInit(TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_ICStructInit(TIM_ICInitTypeDef* TIM_ICInitStruct);
void TIM_BDTRStructInit(TIM_BDTRInitTypeDef* TIM_BDTRInitStruct);
void TIM_Cmd(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_CtrlPWMOutputs(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_ITConfig(TIM_TypeDef* TIMx, uint16_t TIM_IT, FunctionalState NewState);
void TIM_GenerateEvent(TIM_TypeDef* TIMx, uint16_t TIM_EventSource);
void TIM_DMAConfig(TIM_TypeDef* TIMx, uint16_t TIM_DMABase, uint16_t TIM_DMABurstLength);
void TIM_DMACmd(TIM_TypeDef* TIMx, uint16_t TIM_DMASource, FunctionalState NewState);
void TIM_InternalClockConfig(TIM_TypeDef* TIMx);
void TIM_ITRxExternalClockConfig(TIM_TypeDef* TIMx, uint16_t TIM_InputTriggerSource);
void TIM_TIxExternalClockConfig(TIM_TypeDef* TIMx, uint16_t TIM_TIxExternalCLKSource,
                                uint16_t TIM_ICPolarity, uint16_t ICFilter);
void TIM_ETRClockMode1Config(TIM_TypeDef* TIMx, uint16_t TIM_ExtTRGPrescaler, uint16_t TIM_ExtTRGPolarity,
                             uint16_t ExtTRGFilter);
void TIM_ETRClockMode2Config(TIM_TypeDef* TIMx, uint16_t TIM_ExtTRGPrescaler, 
                             uint16_t TIM_ExtTRGPolarity, uint16_t ExtTRGFilter);
void TIM_ETRConfig(TIM_TypeDef* TIMx, uint16_t TIM_ExtTRGPrescaler, uint16_t TIM_ExtTRGPolarity,
                   uint16_t ExtTRGFilter);
void TIM_PrescalerConfig(TIM_TypeDef* TIMx, uint16_t Prescaler, uint16_t TIM_PSCReloadMode);
void TIM_CounterModeConfig(TIM_TypeDef* TIMx, uint16_t TIM_CounterMode);
void TIM_SelectInputTrigger(TIM_TypeDef* TIMx, uint16_t TIM_InputTriggerSource);
void TIM_EncoderInterfaceConfig(TIM_TypeDef* TIMx, uint16_t TIM_EncoderMode,
                                uint16_t TIM_IC1Polarity, uint16_t TIM_IC2Polarity);
void TIM_ForcedOC1Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction);
void TIM_ForcedOC2Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction);
void TIM_ForcedOC3Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction);
void TIM_ForcedOC4Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction);
void TIM_ARRPreloadConfig(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_SelectCOM(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_SelectCCDMA(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_CCPreloadControl(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_OC1PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
void TIM_OC2PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
void TIM_OC3PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
void TIM_OC4PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
void TIM_OC1FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast);
void TIM_OC2FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast);
void TIM_OC3FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast);
void TIM_OC4FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast);
void TIM_ClearOC1Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear);
void TIM_ClearOC2Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear);
void TIM_ClearOC3Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear);
void TIM_ClearOC4Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear);
void TIM_OC1PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity);
void TIM_OC1NPolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCNPolarity);
void TIM_OC2PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity);
void TIM_OC2NPolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCNPolarity);
void TIM_OC3PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity);
void TIM_OC3NPolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCNPolarity);
void TIM_OC4PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity);
void TIM_CCxCmd(TIM_TypeDef* TIMx, uint16_t TIM_Channel, uint16_t TIM_CCx);
void TIM_CCxNCmd(TIM_TypeDef* TIMx, uint16_t TIM_Channel, uint16_t TIM_CCxN);
void TIM_SelectOCxM(TIM_TypeDef* TIMx, uint16_t TIM_Channel, uint16_t TIM_OCMode);
void TIM_UpdateDisableConfig(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_UpdateRequestConfig(TIM_TypeDef* TIMx, uint16_t TIM_UpdateSource);
void TIM_SelectHallSensor(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_SelectOnePulseMode(TIM_TypeDef* TIMx, uint16_t TIM_OPMode);
void TIM_SelectOutputTrigger(TIM_TypeDef* TIMx, uint16_t TIM_TRGOSource);
void TIM_SelectSlaveMode(TIM_TypeDef* TIMx, uint16_t TIM_SlaveMode);
void TIM_SelectMasterSlaveMode(TIM_TypeDef* TIMx, uint16_t TIM_MasterSlaveMode);
void TIM_SetCounter(TIM_TypeDef* TIMx, uint16_t Counter);
void TIM_SetAutoreload(TIM_TypeDef* TIMx, uint16_t Autoreload);
void TIM_SetCompare1(TIM_TypeDef* TIMx, uint16_t Compare1);
void TIM_SetCompare2(TIM_TypeDef* TIMx, uint16_t Compare2);
void TIM_SetCompare3(TIM_TypeDef* TIMx, uint16_t Compare3);
void TIM_SetCompare4(TIM_TypeDef* TIMx, uint16_t Compare4);
void TIM_SetIC1Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC);
void TIM_SetIC2Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC);
void TIM_SetIC3Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC);
void TIM_SetIC4Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC);
void TIM_SetClockDivision(TIM_TypeDef* TIMx, uint16_t TIM_CKD);
uint16_t TIM_GetCapture1(TIM_TypeDef* TIMx);
uint16_t TIM_GetCapture2(TIM_TypeDef* TIMx);
uint16_t TIM_GetCapture3(TIM_TypeDef* TIMx);
uint16_t TIM_GetCapture4(TIM_TypeDef* TIMx);
uint16_t TIM_GetCounter(TIM_TypeDef* TIMx);
uint16_t TIM_GetPrescaler(TIM_TypeDef* TIMx);
FlagStatus TIM_GetFlagStatus(TIM_TypeDef* TIMx, uint16_t TIM_FLAG);
void TIM_ClearFlag(TIM_TypeDef* TIMx, uint16_t TIM_FLAG);
ITStatus TIM_GetITStatus(TIM_TypeDef* TIMx, uint16_t TIM_IT);
void TIM_ClearITPendingBit(TIM_TypeDef* TIMx, uint16_t TIM_IT);








  



  



 

 
#line 48 "..\\..\\User\\stm32f10x_conf.h"
#line 1 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_usart.h"




















 

 







 
#line 33 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_usart.h"



 



  



  



  
  
typedef struct
{
  uint32_t USART_BaudRate;            


 

  uint16_t USART_WordLength;          
 

  uint16_t USART_StopBits;            
 

  uint16_t USART_Parity;              




 
 
  uint16_t USART_Mode;                
 

  uint16_t USART_HardwareFlowControl; 

 
} USART_InitTypeDef;



  
  
typedef struct
{

  uint16_t USART_Clock;   
 

  uint16_t USART_CPOL;    
 

  uint16_t USART_CPHA;    
 

  uint16_t USART_LastBit; 

 
} USART_ClockInitTypeDef;



  



  
  
















  
  


                                    




  



  
  
#line 146 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_usart.h"


  



  
  
#line 160 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_usart.h"


  



  
  





  



  
#line 187 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_usart.h"


  



  






  



 
  






  



 







 



 







  



 
  
#line 264 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_usart.h"


 



 







  



 







 



 
  







 



 







  



 

#line 336 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_usart.h"
                              
#line 344 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_usart.h"



  



  



  



  



 

void USART_DeInit(USART_TypeDef* USARTx);
void USART_Init(USART_TypeDef* USARTx, USART_InitTypeDef* USART_InitStruct);
void USART_StructInit(USART_InitTypeDef* USART_InitStruct);
void USART_ClockInit(USART_TypeDef* USARTx, USART_ClockInitTypeDef* USART_ClockInitStruct);
void USART_ClockStructInit(USART_ClockInitTypeDef* USART_ClockInitStruct);
void USART_Cmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_ITConfig(USART_TypeDef* USARTx, uint16_t USART_IT, FunctionalState NewState);
void USART_DMACmd(USART_TypeDef* USARTx, uint16_t USART_DMAReq, FunctionalState NewState);
void USART_SetAddress(USART_TypeDef* USARTx, uint8_t USART_Address);
void USART_WakeUpConfig(USART_TypeDef* USARTx, uint16_t USART_WakeUp);
void USART_ReceiverWakeUpCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_LINBreakDetectLengthConfig(USART_TypeDef* USARTx, uint16_t USART_LINBreakDetectLength);
void USART_LINCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_SendData(USART_TypeDef* USARTx, uint16_t Data);
uint16_t USART_ReceiveData(USART_TypeDef* USARTx);
void USART_SendBreak(USART_TypeDef* USARTx);
void USART_SetGuardTime(USART_TypeDef* USARTx, uint8_t USART_GuardTime);
void USART_SetPrescaler(USART_TypeDef* USARTx, uint8_t USART_Prescaler);
void USART_SmartCardCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_SmartCardNACKCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_HalfDuplexCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_OverSampling8Cmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_OneBitMethodCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_IrDAConfig(USART_TypeDef* USARTx, uint16_t USART_IrDAMode);
void USART_IrDACmd(USART_TypeDef* USARTx, FunctionalState NewState);
FlagStatus USART_GetFlagStatus(USART_TypeDef* USARTx, uint16_t USART_FLAG);
void USART_ClearFlag(USART_TypeDef* USARTx, uint16_t USART_FLAG);
ITStatus USART_GetITStatus(USART_TypeDef* USARTx, uint16_t USART_IT);
void USART_ClearITPendingBit(USART_TypeDef* USARTx, uint16_t USART_IT);








  



  



  

 
#line 49 "..\\..\\User\\stm32f10x_conf.h"
#line 1 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_wwdg.h"




















 

 







 
#line 33 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_wwdg.h"



 



  



  
  


  



  
  


  
  
#line 68 "..\\..\\Libraries\\FWlib\\inc\\stm32f10x_wwdg.h"



  



  



  


  



  
  
void WWDG_DeInit(void);
void WWDG_SetPrescaler(uint32_t WWDG_Prescaler);
void WWDG_SetWindowValue(uint8_t WindowValue);
void WWDG_EnableIT(void);
void WWDG_SetCounter(uint8_t Counter);
void WWDG_Enable(uint8_t Counter);
FlagStatus WWDG_GetFlagStatus(void);
void WWDG_ClearFlag(void);









  



  



  

 
#line 50 "..\\..\\User\\stm32f10x_conf.h"
#line 1 "..\\..\\Libraries\\FWlib\\inc\\misc.h"




















 

 







 
#line 33 "..\\..\\Libraries\\FWlib\\inc\\misc.h"



 



 



 



 

typedef struct
{
  uint8_t NVIC_IRQChannel;                    


 

  uint8_t NVIC_IRQChannelPreemptionPriority;  

 

  uint8_t NVIC_IRQChannelSubPriority;         

 

  FunctionalState NVIC_IRQChannelCmd;         

    
} NVIC_InitTypeDef;
 


 



 
























 



 



 



 







 



 

#line 133 "..\\..\\Libraries\\FWlib\\inc\\misc.h"


 



 

#line 151 "..\\..\\Libraries\\FWlib\\inc\\misc.h"















 



 







 



 



 



 



 

void NVIC_PriorityGroupConfig(uint32_t NVIC_PriorityGroup);
void NVIC_Init(NVIC_InitTypeDef* NVIC_InitStruct);
void NVIC_SetVectorTable(uint32_t NVIC_VectTab, uint32_t Offset);
void NVIC_SystemLPConfig(uint8_t LowPowerMode, FunctionalState NewState);
void SysTick_CLKSourceConfig(uint32_t SysTick_CLKSource);









 



 



 

 
#line 51 "..\\..\\User\\stm32f10x_conf.h"


 
 

 
 

 
#line 75 "..\\..\\User\\stm32f10x_conf.h"



 
#line 8298 "..\\..\\Libraries\\CMSIS\\stm32f10x.h"




 

















 









 

  

 

 
#line 5 "..\\..\\User\\./ov7725/bsp_ov7725.h"


 
typedef struct
{	

	
	uint8_t QVGA_VGA; 
	
	 
	 
	uint16_t cam_sx; 
	uint16_t cam_sy; 
	
	uint16_t cam_width;
	uint16_t cam_height;
	
	uint16_t lcd_sx;
	uint16_t lcd_sy;
	uint8_t lcd_scan;
	
	uint8_t light_mode;
	int8_t saturation;
	int8_t brightness;
	int8_t contrast;
	uint8_t effect;	


}OV7725_MODE_PARAM;


 
#line 185 "..\\..\\User\\./ov7725/bsp_ov7725.h"



 



























#line 226 "..\\..\\User\\./ov7725/bsp_ov7725.h"































#line 266 "..\\..\\User\\./ov7725/bsp_ov7725.h"

#line 275 "..\\..\\User\\./ov7725/bsp_ov7725.h"


																		
																		
																		

 


#line 290 "..\\..\\User\\./ov7725/bsp_ov7725.h"

																		

void OV7725_GPIO_Config(void);
ErrorStatus OV7725_Init(void);
uint8_t color_check(uint16_t tmp[], uint16_t es);																					
uint8_t ImagDisp(uint16_t sx,uint16_t sy,uint16_t width,uint16_t height, uint16_t bf[], uint16_t es, uint16_t *cnt);
void OV7725_Light_Mode(uint8_t mode);
void OV7725_Color_Saturation(int8_t sat);
void OV7725_Brightness(int8_t bri);
void OV7725_Contrast(int8_t cnst);
void OV7725_Special_Effect(uint8_t eff);
void VSYNC_Init(void);				
void OV7725_Window_Set(uint16_t sx,uint16_t sy,uint16_t width,uint16_t height,uint8_t QVGA_VGA);

























#line 19 "..\\..\\User\\ov7725\\bsp_ov7725.c"
#line 1 "..\\..\\User\\./sccb/bsp_sccb.h"





#line 7 "..\\..\\User\\./sccb/bsp_sccb.h"



 














   










void SCCB_GPIO_Config(void);
int SCCB_WriteByte( u16 WriteAddress , u8 SendByte);
int SCCB_ReadByte(u8* pBuffer,   u16 length,   u8 ReadAddress);



#line 20 "..\\..\\User\\ov7725\\bsp_ov7725.c"
#line 1 "..\\..\\User\\./lcd/bsp_ili9341_lcd.h"




#line 6 "..\\..\\User\\./lcd/bsp_ili9341_lcd.h"
#line 1 "..\\..\\User\\./font/fonts.h"



#line 5 "..\\..\\User\\./font/fonts.h"
#line 1 "..\\..\\User\\./font/fonts.h"
#line 6 "..\\..\\User\\./font/fonts.h"







  
typedef struct _tFont
{    
  const uint8_t *table;
  uint16_t Width;
  uint16_t Height;
  
} sFONT;

extern sFONT Font24x32;
extern sFONT Font16x24;
extern sFONT Font8x16;





#line 7 "..\\..\\User\\./lcd/bsp_ili9341_lcd.h"














 

 











 
 































 
































































 


 








extern uint16_t LCD_X_LENGTH,LCD_Y_LENGTH; 



extern uint8_t LCD_SCAN_MODE;

 


#line 167 "..\\..\\User\\./lcd/bsp_ili9341_lcd.h"



 







 
void                     ILI9341_Init                    ( void );
void                     ILI9341_Rst                     ( void );
void                     ILI9341_BackLed_Control         ( FunctionalState enumState );
void                     ILI9341_GramScan                ( uint8_t ucOtion );
void                     ILI9341_OpenWindow              ( uint16_t usX, uint16_t usY, uint16_t usWidth, uint16_t usHeight );
void                     ILI9341_Clear                   ( uint16_t usX, uint16_t usY, uint16_t usWidth, uint16_t usHeight );
void                     ILI9341_SetPointPixel           ( uint16_t usX, uint16_t usY );
uint16_t                 ILI9341_GetPointPixel           ( uint16_t usX , uint16_t usY );
void                     ILI9341_DrawLine                ( uint16_t usX1, uint16_t usY1, uint16_t usX2, uint16_t usY2 );
void                     ILI9341_DrawRectangle           ( uint16_t usX_Start, uint16_t usY_Start, uint16_t usWidth, uint16_t usHeight,uint8_t ucFilled );
void                     ILI9341_DrawCircle              ( uint16_t usX_Center, uint16_t usY_Center, uint16_t usRadius, uint8_t ucFilled );
void                     ILI9341_DispChar_EN             ( uint16_t usX, uint16_t usY, const char cChar );
void                     ILI9341_DispStringLine_EN      ( uint16_t line, char * pStr );
void                     ILI9341_DispString_EN      			( uint16_t usX, uint16_t usY, char * pStr );
void 											ILI9341_DispString_EN_YDir 		(   uint16_t usX,uint16_t usY ,  char * pStr );

void 											LCD_SetFont											(sFONT *fonts);
sFONT 										*LCD_GetFont											(void);
void 											LCD_ClearLine										(uint16_t Line);
void 											LCD_SetBackColor								(uint16_t Color);
void 											LCD_SetTextColor								(uint16_t Color)	;
void 											LCD_SetColors										(uint16_t TextColor, uint16_t BackColor);
void 											LCD_GetColors										(uint16_t *TextColor, uint16_t *BackColor);


void                 ILI9341_Write_Cmd           ( uint16_t usCmd );
void                 ILI9341_Write_Data          ( uint16_t usData );
uint16_t             ILI9341_Read_Data           ( void );




#line 21 "..\\..\\User\\ov7725\\bsp_ov7725.c"
#line 1 "..\\..\\User\\./usart/bsp_usart.h"




#line 6 "..\\..\\User\\./usart/bsp_usart.h"
#line 1 "D:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"
 
 
 





 






 







 




  
 








#line 47 "D:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"


  



    typedef unsigned int size_t;    









 
 

 



    typedef struct __va_list __va_list;






   




 




typedef struct __fpos_t_struct {
    unsigned __int64 __pos;
    



 
    struct {
        unsigned int __state1, __state2;
    } __mbstate;
} fpos_t;
   


 


   

 

typedef struct __FILE FILE;
   






 

#line 136 "D:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"


extern FILE __stdin, __stdout, __stderr;
extern FILE *__aeabi_stdin, *__aeabi_stdout, *__aeabi_stderr;

#line 166 "D:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"
    

    

    





     



   


 


   


 

   



 

   


 




   


 





    


 






extern __declspec(__nothrow) int remove(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int rename(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) FILE *tmpfile(void);
   




 
extern __declspec(__nothrow) char *tmpnam(char *  );
   











 

extern __declspec(__nothrow) int fclose(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) int fflush(FILE *  );
   







 
extern __declspec(__nothrow) FILE *fopen(const char * __restrict  ,
                           const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   








































 
extern __declspec(__nothrow) FILE *freopen(const char * __restrict  ,
                    const char * __restrict  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(2,3)));
   








 
extern __declspec(__nothrow) void setbuf(FILE * __restrict  ,
                    char * __restrict  ) __attribute__((__nonnull__(1)));
   




 
extern __declspec(__nothrow) int setvbuf(FILE * __restrict  ,
                   char * __restrict  ,
                   int  , size_t  ) __attribute__((__nonnull__(1)));
   















 
#pragma __printf_args
extern __declspec(__nothrow) int fprintf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   


















 
#pragma __printf_args
extern __declspec(__nothrow) int _fprintf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   




 
#pragma __printf_args
extern __declspec(__nothrow) int _printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






 
#pragma __printf_args
extern __declspec(__nothrow) int _sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

#pragma __printf_args
extern __declspec(__nothrow) int snprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   















 

#pragma __printf_args
extern __declspec(__nothrow) int _snprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int fscanf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






























 
#pragma __scanf_args
extern __declspec(__nothrow) int _fscanf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   






 
#pragma __scanf_args
extern __declspec(__nothrow) int _scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int sscanf(const char * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   








 
#pragma __scanf_args
extern __declspec(__nothrow) int _sscanf(const char * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

 
extern __declspec(__nothrow) int vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int _vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int _vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int _vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int _vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) int vfprintf(FILE * __restrict  ,
                    const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int vsprintf(char * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 

extern __declspec(__nothrow) int vsnprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   







 

extern __declspec(__nothrow) int _vsprintf(char * __restrict  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vfprintf(FILE * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vsnprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   



 
extern __declspec(__nothrow) int fgetc(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) char *fgets(char * __restrict  , int  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   










 
extern __declspec(__nothrow) int fputc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   







 
extern __declspec(__nothrow) int fputs(const char * __restrict  , FILE * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int getc(FILE *  ) __attribute__((__nonnull__(1)));
   







 




    extern __declspec(__nothrow) int (getchar)(void);

   





 
extern __declspec(__nothrow) char *gets(char *  ) __attribute__((__nonnull__(1)));
   









 
extern __declspec(__nothrow) int putc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   





 




    extern __declspec(__nothrow) int (putchar)(int  );

   



 
extern __declspec(__nothrow) int puts(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int ungetc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   






















 

extern __declspec(__nothrow) size_t fread(void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   











 

extern __declspec(__nothrow) size_t __fread_bytes_avail(void * __restrict  ,
                    size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   











 

extern __declspec(__nothrow) size_t fwrite(const void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   







 

extern __declspec(__nothrow) int fgetpos(FILE * __restrict  , fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) int fseek(FILE *  , long int  , int  ) __attribute__((__nonnull__(1)));
   














 
extern __declspec(__nothrow) int fsetpos(FILE * __restrict  , const fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   










 
extern __declspec(__nothrow) long int ftell(FILE *  ) __attribute__((__nonnull__(1)));
   











 
extern __declspec(__nothrow) void rewind(FILE *  ) __attribute__((__nonnull__(1)));
   





 

extern __declspec(__nothrow) void clearerr(FILE *  ) __attribute__((__nonnull__(1)));
   




 

extern __declspec(__nothrow) int feof(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) int ferror(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) void perror(const char *  );
   









 

extern __declspec(__nothrow) int _fisatty(FILE *   ) __attribute__((__nonnull__(1)));
    
 

extern __declspec(__nothrow) void __use_no_semihosting_swi(void);
extern __declspec(__nothrow) void __use_no_semihosting(void);
    





 











#line 985 "D:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"



 

#line 7 "..\\..\\User\\./usart/bsp_usart.h"





 
	









    



















































































void USART_Config(void);

#line 22 "..\\..\\User\\ov7725\\bsp_ov7725.c"
#line 1 "..\\..\\User\\./led/bsp_led.h"




#line 6 "..\\..\\User\\./led/bsp_led.h"


 



















 



 
















 





 












 



















					
















					





					






void LED_GPIO_Config(void);

#line 23 "..\\..\\User\\ov7725\\bsp_ov7725.c"



OV7725_MODE_PARAM cam_mode =
{
	
 
 
	
	.QVGA_VGA = 0,	
	.cam_sx = 0,
	.cam_sy = 0,	
	
	.cam_width = 320,
	.cam_height = 240,
	
	.lcd_sx = 0,
	.lcd_sy = 0,
	.lcd_scan = 3, 
	
	
	.light_mode = 0,
	.saturation = 0,	
	.brightness = 0,
	.contrast = 0,
	.effect = 0,		
	
	
 	
 
 
 



















	
	
	 	
	 
	




















};


typedef struct Reg
{
	uint8_t Address;			        
	uint8_t Value;		            
}Reg_Info;

 
Reg_Info Sensor_Config[] =
{
	{0x11,     0x00},  
	{0x12,      0x46},  
	{0x17,    0x3f},
	{0x18,     0x50},
	{0x19,     0x03},
	{0x1A,     0x78},
	{0x32,      0x00},
	{0x29,  0x50},
	{0x2C,  0x78},
	{0x2A,     0x00},
	

	 
	{0x42,     0x7f},
	{0x4D,   0x09},
	{0x63, 0xe0},
	{0x64, 0xff},
	{0x65, 0x20},
	{0x66,	0x00},
	{0x67, 0x00},

	 
	{0x13,		0xf0},
	{0x0D,		0x81},  
	{0x0F,		0xc5},
	{0x14,		0x21},
	{0x22,	0xFF},
	{0x23,	0x01},
	{0x24,		0x34},
	{0x25,		0x3c},
	{0x26,		0xa1},
	{0x2B,		0x00},
	{0x6B,  0xaa},
	{0x13,		0xff},
	{0x69,  0x5d},

	{0x90,		0x0a},
	{0x91,	0x01},
	{0x92,		0x01},
	{0x93,		0x01},

	{0x94,		0x5f},
	{0x95,		0x53},
	{0x96,		0x11},
	{0x97,		0x1a},
	{0x98,		0x3d},
	{0x99,		0x5a},
	{0x9A,  0x1e},

	{0x9B,	0x00},
	{0x9C,		0x25},
	{0xA7,		0x65},
	{0xA8,		0x65},
	{0x9E,	0x81},
	
	{0xA6,		  0x06},	
	
     
	{0x7E,		0x0c},
	{0x7F,		0x16},
	{0x80,		0x2a},
	{0x81,		0x4e},
	{0x82,		0x61},
	{0x83,		0x6f},
	{0x84,		0x7b},
	{0x85,		0x86},
	{0x86,		0x8e},
	{0x87,		0x97},
	{0x88,		0xa4},
	{0x89,		0xaf},
	{0x8A,		0xc5},
	{0x8B,		0xd7},
	{0x8C,		0xe8},
	{0x8D,		0x20},

	{0xA9,	0x80},
	{0xAA,	0x80},
	{0xAC,	0xff},
	{0x33,	0x00},
	{0x22,	0x99},
	{0x23,	0x03},
	{0x4A,	0x00},
	{0x49,	0x13},
	{0x47,		0x08},
	{0x4B,  0x14},
	{0x4C,  0x17},
	{0x46,	0x05},
	
	{0x0C,		0xd0}, 

	 
	{0x0E,		0xf5},	  
	
};

uint8_t OV7725_REG_NUM = sizeof(Sensor_Config)/sizeof(Sensor_Config[0]);	   

volatile uint8_t Ov7725_vsync ;	  









 
static void FIFO_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	
		 
	  RCC_APB2PeriphClockCmd (((uint32_t)0x00000004)|((uint32_t)0x00000010)|
															((uint32_t)0x00000004)|((uint32_t)0x00000010)|
															((uint32_t)0x00000020)|((uint32_t)0x00000008), ENABLE );
	
		 
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
		GPIO_InitStructure.GPIO_Pin = ((uint16_t)0x0008);
		GPIO_Init(((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x0800)), &GPIO_InitStructure);
	
			 
		GPIO_InitStructure.GPIO_Pin = ((uint16_t)0x0010);
		GPIO_Init(((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x1000)), &GPIO_InitStructure);
	
			 
		GPIO_InitStructure.GPIO_Pin = ((uint16_t)0x0004);
		GPIO_Init(((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x0800)), &GPIO_InitStructure);
		
		 
		GPIO_InitStructure.GPIO_Pin = ((uint16_t)0x0020);
		GPIO_Init(((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x1000)), &GPIO_InitStructure);

		 
		GPIO_InitStructure.GPIO_Pin = ((uint16_t)0x0008);	
		GPIO_Init(((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x1400)), &GPIO_InitStructure);
	

     
		GPIO_InitStructure.GPIO_Pin = 	((uint16_t)0x0100) | ((uint16_t)0x0200) |
																			((uint16_t)0x0400) | ((uint16_t)0x0800) |
																			((uint16_t)0x1000) | 	((uint16_t)0x2000) |
																			((uint16_t)0x4000) | 	((uint16_t)0x8000);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x0C00)), &GPIO_InitStructure);
		
		
    ((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x0800))->BRR =((uint16_t)0x0008);	  					 
    ((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x1400))->BSRR =((uint16_t)0x0008);   						 
		
		
}








 
static void VSYNC_GPIO_Config(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
	  EXTI_InitTypeDef EXTI_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
	
		 
	  RCC_APB2PeriphClockCmd ( ((uint32_t)0x00000001)|((uint32_t)0x00000040), ENABLE );	 
    
		 
		GPIO_InitStructure.GPIO_Pin =  ((uint16_t)0x0008);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x1000)), &GPIO_InitStructure);

		 
    GPIO_EXTILineConfig(((uint8_t)0x02), ((uint8_t)0x03));
    EXTI_InitStructure.EXTI_Line = ((uint32_t)0x00008);
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling ; 
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    EXTI_GenerateSWInterrupt(((uint32_t)0x00008));		
	
		 
    NVIC_PriorityGroupConfig(((uint32_t)0x600));
    NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}








 
void OV7725_GPIO_Config(void)
{
	SCCB_GPIO_Config();
	FIFO_GPIO_Config();
	VSYNC_GPIO_Config();
	
}







 
ErrorStatus OV7725_Init(void)
{
	uint16_t i = 0;
	uint8_t Sensor_IDCode = 0;	
	
	
	
	if( 0 == SCCB_WriteByte ( 0x12, 0x80 ) )  
	{
		
		return ERROR ;
	}	

	if( 0 == SCCB_ReadByte( &Sensor_IDCode, 1, 0x0b ) )	  
	{
		
		return ERROR;
	}
	
	
	if(Sensor_IDCode == 0x21)
	{
		for( i = 0 ; i < OV7725_REG_NUM ; i++ )
		{
			if( 0 == SCCB_WriteByte(Sensor_Config[i].Address, Sensor_Config[i].Value) )
			{                
				
				return ERROR;
			}
		}
	}
	else
	{
		return ERROR;
	}
	
	
	return SUCCESS;
}













 
void OV7725_Light_Mode(uint8_t mode)
{
	switch(mode)
	{
		case 0:	
			SCCB_WriteByte(0x13, 0xff); 
			SCCB_WriteByte(0x0e, 0x65);
			SCCB_WriteByte(0x2d, 0x00);
			SCCB_WriteByte(0x2e, 0x00);
			break;
		case 1:
			SCCB_WriteByte(0x13, 0xfd); 
			SCCB_WriteByte(0x01, 0x5a);
			SCCB_WriteByte(0x02, 0x5c);
			SCCB_WriteByte(0x0e, 0x65);
			SCCB_WriteByte(0x2d, 0x00);
			SCCB_WriteByte(0x2e, 0x00);
			break;	
		case 2:
			SCCB_WriteByte(0x13, 0xfd); 
			SCCB_WriteByte(0x01, 0x58);
			SCCB_WriteByte(0x02, 0x60);
			SCCB_WriteByte(0x0e, 0x65);
			SCCB_WriteByte(0x2d, 0x00);
			SCCB_WriteByte(0x2e, 0x00);
			break;	
		case 3:
			SCCB_WriteByte(0x13, 0xfd); 
			SCCB_WriteByte(0x01, 0x84);
			SCCB_WriteByte(0x02, 0x4c);
			SCCB_WriteByte(0x0e, 0x65);
			SCCB_WriteByte(0x2d, 0x00);
			SCCB_WriteByte(0x2e, 0x00);
			break;	
		case 4:
			SCCB_WriteByte(0x13, 0xfd); 
			SCCB_WriteByte(0x01, 0x96);
			SCCB_WriteByte(0x02, 0x40);
			SCCB_WriteByte(0x0e, 0x65);
			SCCB_WriteByte(0x2d, 0x00);
			SCCB_WriteByte(0x2e, 0x00);
			break;	
		
		case 5:
			SCCB_WriteByte(0x13, 0xff); 
			SCCB_WriteByte(0x0e, 0xe5);
			break;	
		
		default:
			 do{ if(0) printf("<<-OV7725-DEBUG->> [%d]" "Light Mode parameter error!" "\n",433); }while(0); 

			break;
	}

}			






 
void OV7725_Color_Saturation(int8_t sat)
{

 	if(sat >=-4 && sat<=4)
	{	
		SCCB_WriteByte(0xA7, (sat+4)<<4); 
		SCCB_WriteByte(0xA8, (sat+4)<<4);
	}
	else
	{
		do{ if(0) printf("<<-OV7725-DEBUG->> [%d]" "Color Saturation parameter error!" "\n",456); }while(0);
	}
	
}			






 
void OV7725_Brightness(int8_t bri)
{
	uint8_t BRIGHT_Value,SIGN_Value;	
	
	switch(bri)
	{
		case 4:
				BRIGHT_Value = 0x48;
				SIGN_Value = 0x06;
			break;
		
		case 3:
				BRIGHT_Value = 0x38;
				SIGN_Value = 0x06;		
		break;	
		
		case 2:
				BRIGHT_Value = 0x28;
				SIGN_Value = 0x06;			
		break;	
		
		case 1:
				BRIGHT_Value = 0x18;
				SIGN_Value = 0x06;			
		break;	
		
		case 0:
				BRIGHT_Value = 0x08;
				SIGN_Value = 0x06;			
		break;	
		
		case -1:
				BRIGHT_Value = 0x08;
				SIGN_Value = 0x0e;		
		break;	
		
		case -2:
				BRIGHT_Value = 0x18;
				SIGN_Value = 0x0e;		
		break;	
		
		case -3:
				BRIGHT_Value = 0x28;
				SIGN_Value = 0x0e;		
		break;	
		
		case -4:
				BRIGHT_Value = 0x38;
				SIGN_Value = 0x0e;		
		break;	
		
		default:
			do{ if(0) printf("<<-OV7725-DEBUG->> [%d]" "Brightness parameter error!" "\n",519); }while(0);
			break;
	}

		SCCB_WriteByte(0x9B, BRIGHT_Value); 
		SCCB_WriteByte(0xAB, SIGN_Value);
}		





 
void OV7725_Contrast(int8_t cnst)
{
	if(cnst >= -4 && cnst <=4)
	{
		SCCB_WriteByte(0x9C, (0x30-(4-cnst)*4));
	}
	else
	{
		do{ if(0) printf("<<-OV7725-DEBUG->> [%d]" "Contrast parameter error!" "\n",540); }while(0);
	}
}		













 
void OV7725_Special_Effect(uint8_t eff)
{
	switch(eff)
	{
		case 0:
			SCCB_WriteByte(0xa6, 0x06);
			SCCB_WriteByte(0x60, 0x80);
			SCCB_WriteByte(0x61, 0x80);
		break;
		
		case 1:
			SCCB_WriteByte(0xa6, 0x26);
			SCCB_WriteByte(0x60, 0x80);
			SCCB_WriteByte(0x61, 0x80);
		break;	
		
		case 2:
			SCCB_WriteByte(0xa6, 0x1e);
			SCCB_WriteByte(0x60, 0xa0);
			SCCB_WriteByte(0x61, 0x40);	
		break;	
		
		case 3:
			SCCB_WriteByte(0xa6, 0x1e);
			SCCB_WriteByte(0x60, 0x40);
			SCCB_WriteByte(0x61, 0xa0);	
		break;	
		
		case 4:
			SCCB_WriteByte(0xa6, 0x1e);
			SCCB_WriteByte(0x60, 0x80);
			SCCB_WriteByte(0x61, 0xc0);		
		break;	
		
		case 5:
			SCCB_WriteByte(0xa6, 0x1e);
			SCCB_WriteByte(0x60, 0x60);
			SCCB_WriteByte(0x61, 0x60);		
		break;	
		
		case 6:
			SCCB_WriteByte(0xa6, 0x46);
		break;	
				
		default:
			do{ if(0) printf("<<-OV7725-DEBUG->> [%d]" "Special Effect error!" "\n",602); }while(0);
			break;
	}
}		


















 
void OV7725_Window_Set(uint16_t sx,uint16_t sy,uint16_t width,uint16_t height,uint8_t QVGA_VGA)
{
	uint8_t reg_raw,cal_temp;

	 
	if(QVGA_VGA == 0)
	{
		 
		SCCB_WriteByte(0x12,0x46); 
	}
	else
	{
			 
		SCCB_WriteByte(0x12,0x06); 
	}

	 
	
	SCCB_ReadByte(&reg_raw,1,0x17);
	
	
	cal_temp = (reg_raw + (sx>>2));	
	SCCB_WriteByte(0x17,cal_temp ); 
	
	 
	
	SCCB_WriteByte(0x18,width>>2);
	
	
	 
	
	SCCB_ReadByte(&reg_raw,1,0x19);	
	
	cal_temp = (reg_raw + (sy>>1));	
	
	SCCB_WriteByte(0x19,cal_temp);
	
	 
	
	SCCB_WriteByte(0x1A,height>>1);
	
	 
	
	SCCB_ReadByte(&reg_raw,1,0x32);	
	
	cal_temp = (reg_raw |(width&0x03)|((height&0x01)<<2)|((sx&0x03)<<4)|((sy&0x01)<<6));	
	
	SCCB_WriteByte(0x32,cal_temp);
	
	 
	SCCB_WriteByte(0x29,width>>2);
	SCCB_WriteByte(0x2C,height>>1);
	
	
	SCCB_ReadByte(&reg_raw,1,0x2A);	
	cal_temp = (reg_raw |(width&0x03)|((height&0x01)<<2));	

	SCCB_WriteByte(0x2A,cal_temp);	
}














 
void OV7725_Window_VGA_Set(uint16_t sx,uint16_t sy,uint16_t width,uint16_t height)
{
	
	uint8_t reg_raw,cal_temp;

	 
	 
	SCCB_WriteByte(0x12,0x06); 

	 
	
	SCCB_ReadByte(&reg_raw,1,0x17);
	
	
	cal_temp = (reg_raw + (sx>>2));	
	SCCB_WriteByte(0x17,cal_temp ); 
	
	 
	
	SCCB_WriteByte(0x18,width>>2);
	
	
	 
	
	SCCB_ReadByte(&reg_raw,1,0x19);	
	
	cal_temp = (reg_raw + (sy>>1));	
	
	SCCB_WriteByte(0x19,cal_temp);
	
	 
	
	SCCB_WriteByte(0x1A,height>>1);
	
	 
	
	SCCB_ReadByte(&reg_raw,1,0x32);	
	
	cal_temp = (reg_raw |(width&0x03)|((height&0x01)<<2)|((sx&0x03)<<4)|((sy&0x01)<<6));	
	
	SCCB_WriteByte(0x19,cal_temp);
	
	 
	SCCB_WriteByte(0x29,width>>2);
	SCCB_WriteByte(0x2C,height>>1);
	
	
	SCCB_ReadByte(&reg_raw,1,0x2A);	
	
	cal_temp = (reg_raw |(width&0x03)|((height&0x01)<<2));	

	SCCB_WriteByte(0x2A,cal_temp);	
}




 
uint8_t color_check(uint16_t tmp[], uint16_t es) {
	int r = tmp[0], g = tmp[1], b = tmp[2];
	uint16_t maxrgb = tmp[0], minrgb = tmp[0];
	maxrgb = maxrgb > tmp[1] ? maxrgb : tmp[1];
	maxrgb = maxrgb > tmp[2] ? maxrgb : tmp[2];
	minrgb = minrgb < tmp[1] ? minrgb : tmp[1];
	minrgb = minrgb < tmp[2] ? minrgb : tmp[2];
	long h;
	int bot = maxrgb - minrgb;
	if(bot == 0) h = 0;
	if(maxrgb == r && g >= b) h = (g - b) * 100 * 60 / bot;
	if(maxrgb == r && g < b) h = 36000 - (b - g) * 100 * 60 / bot;
	if(maxrgb == g) h = (b - r) * 100 * 60 / bot + 12000;
	if(maxrgb == b) h = (r - g) * 100 * 60 / bot + 24000;
	if(h < 3600 || h > 5100) return 0;
	return 1;
}








 
uint8_t ImagDisp(uint16_t sx,uint16_t sy,uint16_t width,uint16_t height, uint16_t bf[], uint16_t es, uint16_t *cnt)
{
	uint16_t i, j; 
	uint16_t Camera_Data;
	uint16_t tmp[3];
	uint16_t tmpcnt = 0;
	uint8_t batch[320];	
	
	ILI9341_OpenWindow(sx,sy,width,height);
	ILI9341_Write_Cmd ( 0x2C );	
	
	for(i = 0; i < width; i++) {
		batch[i] = 0;
	}
	for(i = 0; i < height; i++)
	{
		for(j = 0; j < width; j++)
		{
			do{ Camera_Data=0; ((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x1000))->BRR =((uint16_t)0x0020); Camera_Data = (((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x0C00))->IDR) & 0xff00; ((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x1000))->BSRR =((uint16_t)0x0020); ((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x1000))->BRR =((uint16_t)0x0020); Camera_Data |= (((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x0C00))->IDR >>8) & 0x00ff; ((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x1000))->BSRR =((uint16_t)0x0020); }while(0);		 
			tmp[0] = (Camera_Data & 0xf800) >> 10;
			tmp[1] = (Camera_Data & 0x07e0) >> 5;
			tmp[2] = (Camera_Data & 0x001f) << 1;
			
			if(color_check(tmp, es)) {
				Camera_Data = 0xF800;
				tmpcnt++;
				batch[j]++;	
			}
			ILI9341_Write_Data(Camera_Data);
		}
	}
	*cnt = tmpcnt;






	for(i = 0; i < width; i++) {
		tmpcnt = tmpcnt - batch[i];
		if(tmpcnt < (*cnt)/2 ) {
			printf("共有 %d 个点，中心在 %d 处\n", *cnt, i);
			break;
		}
	}
	if(tmpcnt <= 1400) {
		{((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x0C00))->BSRR=((uint16_t)0x0020);}; {((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x0C00))->BSRR=((uint16_t)0x0001);} {((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x0C00))->BSRR=((uint16_t)0x0002);};
		return 0;
	}
	if(i < width/2 - 20) {
		{((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x0C00))->BSRR=((uint16_t)0x0020);}; {((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x0C00))->BSRR=((uint16_t)0x0001);} {((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x0C00))->BRR=((uint16_t)0x0002);};
		return 2;
	} else if(i > width/2 + 20) {
		{((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x0C00))->BSRR=((uint16_t)0x0020);}; {((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x0C00))->BRR=((uint16_t)0x0001);} {((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x0C00))->BSRR=((uint16_t)0x0002);};
		return 3;
	} else {
		{((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x0C00))->BRR=((uint16_t)0x0020);}; {((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x0C00))->BSRR=((uint16_t)0x0001);} {((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x0C00))->BSRR=((uint16_t)0x0002);};
		return 1;
	}
	
	
}






 



 
