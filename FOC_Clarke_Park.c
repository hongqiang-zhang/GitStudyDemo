/**
 * @file    FOC_Clarke_Park.c
 * @brief   coordinate transformation ,Obtain the sine and cosine value for coordinate transformation 
 * @author  XXX
 * @version 1.0
 * @date    2020-01-17
 * @copyright Copyright (c) 2020  ninebot
 * 
 * @par 修改日志:
 * <table>
 * <tr><th>Date       <th>Version <th>Author  <th>Description
 * <tr><td>2020-01-17 <td>1.0     <td>wangh   <td>内容
 * </table>
 */
#include "FOC_Globle.h"
#include "FOC_Clarke_Park.h"
#include "Globle.h"
/* M4M5白车 DVT1-1、 EVT2-HF EVT2-5 */
//#define SIN_MASK 0x0300
//#define U0_90    0x0000
//#define U90_180  0x0100
//#define U180_270 0x0200
//#define U270_360 0x0300

/* EVT2-3 */
//#define SIN_MASK 0x0300
//#define U0_90    0x0000
//#define U90_180  0x0100
//#define U180_270 0x0200
//#define U270_360 0x0300
/* M3M6 黑车 */
//#define SIN_MASK 0x0300
//#define U0_90    0x0200
//#define U90_180  0x0300
//#define U180_270 0x0000
//#define U270_360 0x0100

/* DVT1-2 */
//#define SIN_MASK 0x0300
//#define U0_90    0x0100
//#define U90_180  0x0200
//#define U180_270 0x0300
//#define U270_360 0x0000

//#define U0_90_2    0x0300
//#define U90_180_2  0x0000
//#define U180_270_2 0x0100
//#define U270_360_2 0x0200

/* DVT1-3 */
//#define SIN_MASK 0x0300
//#define U0_90    0x0100
//#define U90_180  0x0200
//#define U180_270 0x0300
//#define U270_360 0x0000

//#define U0_90_2    0x0100
//#define U90_180_2  0x0200
//#define U180_270_2 0x0300
//#define U270_360_2 0x0000

/* M4M5白车 EVT1-HF EVT2-5 */
//#define U0_90_2    0x0100
//#define U90_180_2  0x0200
//#define U180_270_2 0x0300
//#define U270_360_2 0x0000

/* EVT2-3 */
//#define U0_90_2    0x0000
//#define U90_180_2  0x0100
//#define U180_270_2 0x0200
//#define U270_360_2 0x0300

/* DVT2_1 DVT2_2 DVT2_5  DVT2_4*/
#define SIN_MASK 0x0300
#define U0_90    0x0000
#define U90_180  0x0100
#define U180_270 0x0200
#define U270_360 0x0300

#define U0_90_2    0x0000
#define U90_180_2  0x0100
#define U180_270_2 0x0200
#define U270_360_2 0x0300

/* DVT2_3 */
//#define SIN_MASK 0x0300
//#define U0_90    0x0000
//#define U90_180  0x0100
//#define U180_270 0x0200
//#define U270_360 0x0300

//#define U0_90_2    0x0200
//#define U90_180_2  0x0300
//#define U180_270_2 0x0000
//#define U270_360_2 0x0100

/* DVT2_1 */
//#define SIN_MASK 0x0300
//#define U0_90    0x0200
//#define U90_180  0x0300
//#define U180_270 0x0000
//#define U270_360 0x0100

//#define U0_90_2    0x0000
//#define U90_180_2  0x0100
//#define U180_270_2 0x0200
//#define U270_360_2 0x0300

/* EVT2-4 */
//#define SIN_MASK 0x0300
//#define U0_90    0x0200
//#define U90_180  0x0300
//#define U180_270 0x0000
//#define U270_360 0x0100

//#define U0_90_2    0x0300
//#define U90_180_2  0x0000
//#define U180_270_2 0x0100
//#define U270_360_2 0x0200

//#define SIN_MASK   0x0300
//#define U0_90      0x0200
//#define U90_180    0x0300
//#define U180_270   0x0000
//#define U270_360   0x0100

//115电机
//#define SIN_MASK 0x0300
//#define U0_90    0x0200
//#define U90_180  0x0300
//#define U180_270 0x0000
//#define U270_360 0x0100

//#define U0_90_2    0x0100
//#define U90_180_2  0x0200
//#define U180_270_2 0x0300
//#define U270_360_2 0x0000

const s16 hSin_Cos_Table[256] = SIN_COS_TABLE; //正余弦表

Sin_Cos_Value GetSinCosByAngle(u16 hAngle); //函数声明，利用电角度获取正余弦值
Sin_Cos_Value GetSinCosByAngle_2(u16 hAngle); //函数声明，利用电角度获取正余弦值

/**
 * @brief  Clarke transform
 * @param  Curr_Input   Motor phase current (two phase current when MOS is on )
 * @return Curr_Components  Ialpha, Ibeta ;
 */
Curr_Components Clarke(Curr_Components Curr_Input)
{
  Curr_Components Curr_Output = {0};

  Curr_Output.C1 = Curr_Input.C1; // Ialpha = ia
  Curr_Output.C2 = ((Curr_Input.C1 + (Curr_Input.C2 << 1)) * 2365) >> 12; // Ibeta = (ia+2ib)/squr(3) //最大电流范围120A
  return (Curr_Output);
}

/**
 * @brief  Park transform
 * @param  Curr_Input       Ialpha, Ibeta ;
 * @param  SinCosMap        Electric angle (==The angle between q axis and α axis==)
 * @return Curr_Components  Iq , Id
 */
Curr_Components Park(Curr_Components Curr_Input, Sin_Cos_Value SinCosMap)
{
  Curr_Components Curr_Output = {0};

  SinCosMap.hSin >>= 3; //防止计算溢出，电流可最大输入2^19=524288（mA）
  SinCosMap.hCos >>= 3;
  Curr_Output.C1 = (SinCosMap.hCos * Curr_Input.C1 + SinCosMap.hSin * Curr_Input.C2) >> 12;//Iq = cos(theta)*Ialpha + sin(theta)*Ibeta
  Curr_Output.C2 = (SinCosMap.hSin * Curr_Input.C1 - SinCosMap.hCos * Curr_Input.C2) >> 12;//Id = sin(theta)*Ialpha - cos(theta)*Ibeta
  return (Curr_Output);
}

/**
 * @brief  Voltage Park Transformation 
 * @param  Volt_Input  two of the Ua,Ub and Uc
 * @param  SinCosMap   sine and consine value
 * @return Volt_Components  Uα and Uβ
 * @note   Not used in the program
 */
Volt_Components Park_Volt(Volt_Components Volt_Input, Sin_Cos_Value SinCosMap)
{
  Volt_Components Volt_Output = {0};

  SinCosMap.hSin >>= 3;
  SinCosMap.hCos >>= 3;
  Volt_Output.V1 = (SinCosMap.hCos * Volt_Input.V1 + SinCosMap.hSin * Volt_Input.V2) >> 12;
  Volt_Output.V2 = (SinCosMap.hSin * Volt_Input.V1 - SinCosMap.hCos * Volt_Input.V2) >> 12;
  return (Volt_Output);
}

/**
 * @brief  Park inverse transformation 
 * @param  Volt_Input  input voltage :Uq、Ud
 * @param  SinCosMap   sine and consine value
 * @return Volt_Components  Uα and Uβ
 */
Volt_Components Rev_Park(Volt_Components Volt_Input, Sin_Cos_Value SinCosMap)
{
  Volt_Components Volt_Output = {0};

  //电角度定义为Q轴与Alpha轴的夹角
  Volt_Output.V1 = ((s32)SinCosMap.hCos * Volt_Input.V1 + (s32)SinCosMap.hSin * Volt_Input.V2) >> 15;//Valpha = cos(theta)*Vq + sin(theta)*Vd
  Volt_Output.V2 = ((s32)SinCosMap.hSin * Volt_Input.V1 - (s32)SinCosMap.hCos * Volt_Input.V2) >> 15;//Vbeta = sin(theta)*Vq - cos(theta)*Vd
  
  return (Volt_Output);
}

/**
 * @brief  Get the Sin Cos By Angle object
 * @param  hAngle  Electric angle (s16)
 * @return Sin_Cos_Value  The cosine and sine value calculated from the electric angle in an electric period 
 * @note   The electrical angle data type obtained from HALL_IncElectricalAngle_2() is u16,
 *         but the data type of GetSinCosByAngle()'s input parameter is S16 
 */
Sin_Cos_Value GetSinCosByAngle(u16 hAngle)
{
  static u16 hindex;
  Sin_Cos_Value Local_Components = {0};

  /* 10 bit index computation  */
  //从HALL_IncElectricalAngle_2()中得到的ElectricAngle的取值范围是0--65535;
  //将其转化成s16,即为0--32767，(-32768)--(-1);
  //所以(u16)(hAngle + 32768)的范围是32768--65535,0--32767；
//  hindex = (u16)(hAngle + 32768); 
//  hindex /= 64; //u16 -> u10; 数据范围是512--1023,0--511；
  hindex = hAngle;
  /**
   * @brief  Construct a new switch object
   * @param  SIN_MASK  SIN_MASK 0x0300
   * @note   将0--1023按照第9、10位的数据分成四个部分分别是
   *         512--767： U0_90(10)    ||                               0--255 即原始数据中的 0--16383    对应  512--767  对应   0--90度                       
   *         768--1023：U90_180(11)  || ==》》在table表里转化成u8==》》 0--255              16384--32767 对应  768--1023 对应  90--180度
   *         0----255： U180_270(00) ||                               0--255              32768--49151 对应   0--255   对应  180--270度
   *         256--511： U270_360(01) ||                               0--255              49152--65535 对应  256--512  对应  270--360度
   *         因为table表只有0-90度的值，所以，在0-90度sin值为正，180-270度sin值为负；
   */
  switch (hindex & SIN_MASK)
  {
  case U0_90:
    Local_Components.hSin = hSin_Cos_Table[(u8)(hindex)];
    Local_Components.hCos = hSin_Cos_Table[(u8)(0xFF - (u8)(hindex))];
    break;

  case U90_180:
    Local_Components.hSin = hSin_Cos_Table[(u8)(0xFF - (u8)(hindex))];
    Local_Components.hCos = -hSin_Cos_Table[(u8)(hindex)];
    break;

  case U180_270:
    Local_Components.hSin = -hSin_Cos_Table[(u8)(hindex)];
    Local_Components.hCos = -hSin_Cos_Table[(u8)(0xFF - (u8)(hindex))];
    break;

  case U270_360:
    Local_Components.hSin = -hSin_Cos_Table[(u8)(0xFF - (u8)(hindex))];
    Local_Components.hCos = hSin_Cos_Table[(u8)(hindex)];
    break;
  default:
    break;
  }
  return (Local_Components);
}

/**
 * @brief  Get the Sin Cos By Angle object
 * @param  hAngle  Electric angle (s16)
 * @return Sin_Cos_Value  The cosine and sine value calculated from the electric angle in an electric period 
 * @note   The electrical angle data type obtained from HALL_IncElectricalAngle_2() is u16,
 *         but the data type of GetSinCosByAngle()'s input parameter is S16 
 */
Sin_Cos_Value GetSinCosByAngle_2(u16 hAngle)
{
  static u16 hindex_2;
  Sin_Cos_Value Local_Components_2 = {0};

  /* 10 bit index computation  */
  //从HALL_IncElectricalAngle_2()中得到的ElectricAngle的取值范围是0--65535;
  //将其转化成s16,即为0--32767，(-32768)--(-1);
  //所以(u16)(hAngle + 32768)的范围是32768--65535,0--32767；
//  hindex = (u16)(hAngle + 32768); 
//  hindex /= 64; //u16 -> u10; 数据范围是512--1023,0--511；
  hindex_2 = hAngle;
  /**
   * @brief  Construct a new switch object
   * @param  SIN_MASK  SIN_MASK 0x0300
   * @note   将0--1023按照第9、10位的数据分成四个部分分别是
   *         512--767： U0_90(10)    ||                               0--255 即原始数据中的 0--16383    对应  512--767  对应   0--90度                       
   *         768--1023：U90_180(11)  || ==》》在table表里转化成u8==》》 0--255              16384--32767 对应  768--1023 对应  90--180度
   *         0----255： U180_270(00) ||                               0--255              32768--49151 对应   0--255   对应  180--270度
   *         256--511： U270_360(01) ||                               0--255              49152--65535 对应  256--512  对应  270--360度
   *         因为table表只有0-90度的值，所以，在0-90度sin值为正，180-270度sin值为负；
   */
  switch (hindex_2 & SIN_MASK)
  {
  case U0_90_2:
    Local_Components_2.hSin = hSin_Cos_Table[(u8)(hindex_2)];
    Local_Components_2.hCos = hSin_Cos_Table[(u8)(0xFF - (u8)(hindex_2))];
    break;

  case U90_180_2:
    Local_Components_2.hSin = hSin_Cos_Table[(u8)(0xFF - (u8)(hindex_2))];
    Local_Components_2.hCos = -hSin_Cos_Table[(u8)(hindex_2)];
    break;

  case U180_270_2:
    Local_Components_2.hSin = -hSin_Cos_Table[(u8)(hindex_2)];
    Local_Components_2.hCos = -hSin_Cos_Table[(u8)(0xFF - (u8)(hindex_2))];
    break;

  case U270_360_2:
    Local_Components_2.hSin = -hSin_Cos_Table[(u8)(0xFF - (u8)(hindex_2))];
    Local_Components_2.hCos = hSin_Cos_Table[(u8)(hindex_2)];
    break;
  default:
    break;
  }
  return (Local_Components_2);
}
