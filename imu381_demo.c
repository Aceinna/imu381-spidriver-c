/*****************************************************************************
 * @file   imu381_demo.c
 * @Author
 * @date   Dec, 2018
 * @brief  IMU38x module functions with p3-B board spi host interface.
 *         parse IMU data/1pps/remapping etc. 		
 * @version  0.0.1
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR
 * PURPOSE.
 *
 ******************************************************************************/


/*******************************************************************/
/* library includes */

#include <stdint.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <sys/ioctl.h>
#include <asm/ioctl.h>
#include <stdio.h>
#include <unistd.h>
#include <linux/spi/spidev.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <softPwm.h>


/*******************************************************************/
/* local macro definitions */

#define SPI_CLOCK                           (500000) //HZ
#define PI_SPI_CHANNEL                      (0x01)
#define BUFFER_SIZE                         (0x64)
#define CP_MODE                             (0x03) //CPOL,CPHA:00,01,10,11
#define CHIP_ID                             (14352)//(0x3810)

#define REG_PRD                             (0x56)
#define REG_ACC_X                           (0x0A)
#define REG_ACC_Y                           (0x0C)
#define REG_ACC_Z                           (0x0E)
#define REG_GYRO_X                          (0x04)
#define REG_GYRO_Y                          (0x06)
#define REG_GYRO_Z                          (0x08)
#define REG_DIAGST_STA                      (0x3C)
#define REG_SELF_TEST_DATA_READY            (0x34)
#define REG_OUTPUT_DATA_RATE                (0x36)
#define REG_RS_DYNAMIC_RANGE                (0x39)
#define REG_LOW_PASS_FILTER                 (0x38)
#define REG_SERIAL_NUMBER                   (0x58)

#define BURST_MODE_STANDARD_8WORDS          (0x3E)//standard mode
#define BURST_MODE_SCALED_SENSR_0_15WORDS   (0x41)//S0 payload
#define BURST_MODE_SCALED_SENSR_1_12WORDS   (0x42)//S1 payload
#define BURST_MODE_ANGLE_DATA1_16WORDS      (0x43)//A1 payload
#define BURST_MODE_ANGLE_DATA2_15WORDS      (0x44)//A2 payload
#define BURST_MODE_NAV0_16WORDS             (0x45)//A3 payload

//Use physical pin 32(gpio 26),which is Pin 26 for wiringPi library
#define INT_HIGH_TO_LOW_TRG                 (29)
#define ONE_PPS_PIN                         (28)
#define PWM_FREQ                            (100)//note:there is software PWM frequency limitation with wiringPi  
#define PWM_INTERVAL                        (50)
#define M_PI                                (3.1415926)

#define SUCCESS                             (0)
#define FAIL                                (-1)
                    
#define DEBUG_SYS_STATUS
#define SINGLE_SPI_OPERATION


/*******************************************************************/
/* type definitions */

/*!
 *
 *@brief: acc raw data
 *
 */
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
    uint32_t time_stamp;
}acc_raw_t;


/*!
 *
 *@brief: gyro raw data
 *
 */
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
    uint32_t time_stamp;
}gyro_raw_t;


/*******************************************************************/
/* variable */

volatile int32_t event_counter = 0;



/*******************************************************************/
/* function definition */

/*!
 *@brief: signal spi read. IMU381 spi read performed 2 byte at one time 
 *        while write are a single byte in length.
 *@param[in/out]: in,reg_addr. out, *read_buf_p
 *
 *@return: no
 */
void single_spi_read(uint8_t reg_addr,uint16_t *read_buf_p)
{
    uint8_t buf[2] = {0};

    buf[0] = reg_addr;
    buf[1] = 0x00;
    wiringPiSPIDataRW(PI_SPI_CHANNEL,buf,2);
    wiringPiSPIDataRW(PI_SPI_CHANNEL,buf,2);

    *read_buf_p = (buf[0] << 8) + buf[1];
    //printf("buf[0]=%x,buf[1]=%x \r\n",buf[0],buf[1]);
}


/*!
 *@brief: single spi write, different with imu381 spi read.
 *        select the desired register address only
 *@param[in]: reg_addr data
 *
 *@return: no
 */
void single_spi_write(uint8_t reg_addr,uint8_t data)
{
    uint8_t buf[2] = {0};

    buf[0] = reg_addr | 0x80;
    buf[1] = data;
    wiringPiSPIDataRW(PI_SPI_CHANNEL,buf,2);
}


/*!
 *@brief: pi3 board initialize
 *
 *@param: no 
 *
 *@return: FAIL/SUCCESS
 */
void pi3_board_init(void)
{
    // sets up the wiringPi library
    if (wiringPiSetup () < 0) 
    {
      printf ("Unable to setup wiringPi\n");
    }
}


/*!
 *@brief: spi initialization
 *
 *@param: no 
 *
 *@return: FAIL/SUCCESS
 */
uint8_t spi_init(void)
{
    int32_t fd = 0;
    uint8_t buf[2] = {0};
    uint16_t reg_v = 0; 
    
    fd = wiringPiSetup();
    if(fd < 0)
    {
        printf("fail wiringPisetup! \r\n");
        return FAIL;
    }

    fd = wiringPiSPISetupMode(PI_SPI_CHANNEL,SPI_CLOCK,CP_MODE);    
    if(fd < 0)
    {
        printf("fail wiringPiSPIsetup! ");
        return FAIL;
    }
  
    buf[0] = REG_PRD;
    buf[1] = 0x00;
    wiringPiSPIDataRW(PI_SPI_CHANNEL,buf,2);
    wiringPiSPIDataRW(PI_SPI_CHANNEL,buf,2);
    reg_v = (buf[0] << 8) + buf[1];//buf[0]->MSB
    if(CHIP_ID != reg_v)
    {
        printf("fail read chip id! reg_v = %d\r\n",reg_v);
        return FAIL;
    }

    buf[0] = REG_DIAGST_STA;
    buf[1] = 0x00;
    wiringPiSPIDataRW(PI_SPI_CHANNEL,buf,2);
    wiringPiSPIDataRW(PI_SPI_CHANNEL,buf,2);
    reg_v = (buf[0] << 8) + buf[1];
    printf("read status reg: %d \r\n", reg_v);

    return SUCCESS ;
}

/*!
 *@brief: burst read mode,store the data payload in buf_p
 *        
 *@param: [in]mode,
 *               BURST_MODE_STANDARD_8WORDS                 (0x3E)//standard mode
 *               BURST_MODE_SCALED_SENSR_0_15WORDS   (0x41)//S0 payload
 *               BURST_MODE_SCALED_SENSR_1_12WORDS   (0x42)//S1 payload
 *               BURST_MODE_ANGLE_DATA1_16WORDS          (0x43)//A1 payload
 *               BURST_MODE_ANGLE_DATA2_15WORDS          (0x44)//A2 payload
 *               BURST_MODE_NAV0_16BIT                                   (0x45)//A3 payload
 *               [out]buf_p.
 *
 *@return: no
 */
void burst_read_spi(uint8_t mode,uint8_t *buf_p)
{
    int16_t int16_arr[36] = {0}; 
    float flot_arr[36] = {0.0}; 

    switch(mode)
    {
        case BURST_MODE_STANDARD_8WORDS:
        {
            buf_p[0] = BURST_MODE_STANDARD_8WORDS;
            buf_p[1] = 0x00;
            wiringPiSPIDataRW(PI_SPI_CHANNEL,buf_p,8*2+2);//8(16-bit word)
            //acc, unit:mg
            int16_arr[0] = (int16_t)((buf_p[8] << 8) + buf_p[9])/4;
            int16_arr[1] = (int16_t)((buf_p[10] << 8) + buf_p[11])/4;        
            int16_arr[2] = (int16_t)((buf_p[12] << 8) + buf_p[13])/4;
            printf("mg:accx=%d,accy=%d,accz=%d \r\n",int16_arr[0],int16_arr[1],int16_arr[2]);
            //gyro, unit:degree/s
            flot_arr[0] = (int16_t)((buf_p[2] << 8) + buf_p[1])/200.0;
            flot_arr[1] = (int16_t)((buf_p[4] << 8) + buf_p[3])/200.0;        
            flot_arr[2] = (int16_t)((buf_p[6] << 8) + buf_p[5])/200.0;
            printf("deg/s:gx=%f,gy=%f,gz=%f \r\n",flot_arr[0],flot_arr[1],flot_arr[2]);
        }
        break;
        
        case BURST_MODE_SCALED_SENSR_0_15WORDS:
        {
            buf_p[0] = BURST_MODE_SCALED_SENSR_0_15WORDS;//15(16-bit word)
            buf_p[1] = 0x00;
            wiringPiSPIDataRW(PI_SPI_CHANNEL,buf_p,15*2+2);
            //acc
            flot_arr[0] = 20.0/65536*((buf_p[0] << 8) + buf_p[1]);
            flot_arr[1] = 20.0/65536*((buf_p[2] << 8) + buf_p[3]);
            flot_arr[2] = 20.0/65536*((buf_p[4] << 8) + buf_p[5]);
            printf("s0_acc_x=%f,s0_acc_y=%f,s0_acc_z=%f \r\n",flot_arr[0],flot_arr[1],flot_arr[2]);
            //gyro
            flot_arr[0] = 7*M_PI/65536*((buf_p[6] << 8) + buf_p[7]);
            flot_arr[1] = 7*M_PI/65536*((buf_p[8] << 8) + buf_p[9]);
            flot_arr[2] = 7*M_PI/65536*((buf_p[10] << 8) + buf_p[11]);
            printf("s0_gyro_x=%f,s0_gyro_y=%f,s0_gyro_z=%f \r\n",flot_arr[0],flot_arr[1],flot_arr[2]);
        }
        break;

        case BURST_MODE_SCALED_SENSR_1_12WORDS:
        {
            buf_p[0] = BURST_MODE_SCALED_SENSR_1_12WORDS;
            buf_p[1] = 0x00;
            wiringPiSPIDataRW(PI_SPI_CHANNEL,buf_p,12*2+2);
            //TODO: parse
        }
        break;

        case BURST_MODE_ANGLE_DATA1_16WORDS:
        {
            buf_p[0] = BURST_MODE_ANGLE_DATA1_16WORDS;
            buf_p[1] = 0x00;
            wiringPiSPIDataRW(PI_SPI_CHANNEL,buf_p,16*2+2);
            //TODO: parse
        }
        break;

        case BURST_MODE_ANGLE_DATA2_15WORDS:
        {
            buf_p[0] = BURST_MODE_ANGLE_DATA2_15WORDS;
            buf_p[1] = 0x00;
            wiringPiSPIDataRW(PI_SPI_CHANNEL,buf_p,15*2+2);
            //TODO: parse
        }
        break;

        case BURST_MODE_NAV0_16WORDS :
        {
            buf_p[0] = BURST_MODE_NAV0_16WORDS;
            buf_p[1] = 0x00;
            wiringPiSPIDataRW(PI_SPI_CHANNEL,buf_p,16*2+2);
            //TODO: parse
        }
        break;

        default :
        {
            printf("error package mode \r\n");
        }
        break;
    }
}


/*!
 *@brief: read acc rawdata, unit mg.
 *
 *@param[out]: acc_raw_t *rawdata_out_p
 *
 *@return: no
 */
void acc_readxyz_raw(acc_raw_t *rawdata_out_p)
{
    uint8_t buf_x[2] = {0};
    uint8_t buf_y[2] = {0};
    uint8_t buf_z[2] = {0};

    buf_x[0] = REG_ACC_X;
    buf_x[1] = 0x00;
    wiringPiSPIDataRW(PI_SPI_CHANNEL,buf_x,2);
    wiringPiSPIDataRW(PI_SPI_CHANNEL,buf_x,2);

    buf_y[0] = REG_ACC_Y;
    buf_y[1] = 0x00;
    wiringPiSPIDataRW(PI_SPI_CHANNEL,buf_y,2);
    wiringPiSPIDataRW(PI_SPI_CHANNEL,buf_y,2);

    buf_z[0] = REG_ACC_Z;
    buf_z[1] = 0x00;
    wiringPiSPIDataRW(PI_SPI_CHANNEL,buf_z,2);
    wiringPiSPIDataRW(PI_SPI_CHANNEL,buf_z,2);

    rawdata_out_p->x = (int16_t)((buf_x[0] << 8) + buf_x[1])/4;
    rawdata_out_p->y = (int16_t)((buf_y[0] << 8) + buf_y[1])/4;
    rawdata_out_p->z = (int16_t)((buf_z[0] << 8) + buf_z[1])/4;
}

/*!
 *@brief: read gyro rawdata, unit: deg/s
 *
 *@param[out]: acc_raw_t *rawdata_out_p
 *
 *@return: no
 */
void gyro_readxyz_raw(gyro_raw_t *rawdata_out_p)
{
    uint8_t buf_x[2] = {0};
    uint8_t buf_y[2] = {0};
    uint8_t buf_z[2] = {0};

    buf_x[0] = REG_GYRO_X;
    buf_x[1] = 0x00;
    wiringPiSPIDataRW(PI_SPI_CHANNEL,buf_x,2);
    wiringPiSPIDataRW(PI_SPI_CHANNEL,buf_x,2);

    buf_y[0] = REG_GYRO_Y;
    buf_y[1] = 0x00;
    wiringPiSPIDataRW(PI_SPI_CHANNEL,buf_y,2);
    wiringPiSPIDataRW(PI_SPI_CHANNEL,buf_y,2);

    buf_z[0] = REG_GYRO_Z;
    buf_z[1] = 0x00;
    wiringPiSPIDataRW(PI_SPI_CHANNEL,buf_z,2);
    wiringPiSPIDataRW(PI_SPI_CHANNEL,buf_z,2);

    rawdata_out_p->x = (int16_t)((buf_x[0] << 8) + buf_x[1])/200;
    rawdata_out_p->y = (int16_t)((buf_y[0] << 8) + buf_y[1])/200;
    rawdata_out_p->z = (int16_t)((buf_z[0] << 8) + buf_z[1])/200;
}

/*!
 *@brief: register the interrupt
 *
 *@param[in]: pin_number. 
 *                     edge_type, 
 *                     INT_EDGE_SETUP         (0)
 *                     INT_EDGE_FALLING      (1)
 *                     INT_EDGE_RISING         (2)
 *                     INT_EDGE_BOTH           (3)
 *                     function_p.
 *
 *@return: result.
 */
int32_t pi3_isr_register(int32_t pin_number, int32_t edge_type,  void (*function_p)(void))
{
    return wiringPiISR(pin_number,edge_type,function_p);
}

/*!
 *@brief: generate software pwm to slave, 100HZ max 
 *
 *@param[in]: pin, init_value, pwm_range
 *
 *@return: no
 */
int32_t pi3_pwm_generate(int32_t pin, int32_t init_value, int32_t pwm_range)
{
      return softPwmCreate(pin,init_value,pwm_range);
}

/*!
 *@brief: generate hardware pwm, 1Khz, accuracy rate < +- 0.1%
 *        note: only wipin 1 could generate hardware pwm, other GPIO soft PWM 
 *              accuracy can not match the IMU381 1pps requirement.
 *@param[in/out]: no
 *
 *@return: no
 */
void pwm_1pps_generate(void)
{
    pinMode(1,PWM_OUTPUT); //set wiPin 1 as pwm output
    pwmSetMode(PWM_MODE_MS); //set pwm pin as duty ratio could be changed  
    pwmSetRange(600); //frequency = 600 000/600 = 1Khz
    pwmWrite(1,300); //duty ratio = 300/600 = 50%
}

/*!
 *@brief: disable 1pps
 *
 *@param[in/out]: no
 *
 *@return: no
 */
void pwm_1pps_disable(void)
{
    pinMode(1,OUTPUT); //set wiPin 1 as pin output
}

/*!
 *@brief: called every time an event occurs
 *
 *@param: no
 *
 *@return: no
 */
void dady_interrupt(void)
{
    event_counter++;
    //printf("interrupt \r\n");
}


int32_t main(void)
{
    uint8_t res = 0;
    uint32_t time_stmp = 0;
    uint8_t buf_package[36] = {0}; 
    uint8_t buf_pack[10] = {0};    
    acc_raw_t acc_rawdata;
    gyro_raw_t gyro_rawdata;
     
    pi3_board_init();
    pwm_1pps_disable();
    delay(300);  

    res = spi_init();
    if(SUCCESS != res)
    {
        printf("Initialize fail! \r\n");
    }
   
    //set accuracy 1kHz 1pps
    pwm_1pps_generate();
    printf("add 1KHz pwm end \r\n");
    delay(800);

    //set pin29 generate an interrupt on high-to-low transitions.
    //and attach dady_interrupt() to the interrupt.
    if (pi3_isr_register(INT_HIGH_TO_LOW_TRG,INT_EDGE_FALLING, &dady_interrupt) < 0) 
    {
        printf ("Unable to setup ISR\n");
        return 1;
    }


    while(1)
    {    
#if defined(SINGLE_SPI_OPERATION)
        if(0 != event_counter)
        {  
            acc_readxyz_raw(&acc_rawdata); 
            gyro_readxyz_raw(&gyro_rawdata);
            printf("acc_x=%hd,acc_y=%hd,acc_z=%hd,ts=%d \r\n",acc_rawdata.x,acc_rawdata.y,acc_rawdata.z,time_stmp);
            printf("gyro_x=%hd,gyro_y=%hd,gyro_z=%hd,ts=%d \r\n",gyro_rawdata.x,gyro_rawdata.y,gyro_rawdata.z,time_stmp);
            time_stmp = millis();
        }  
#endif

#if defined(BURST_SPI_OPERATION)
        if(0 != event_counter)
        {
            burst_read_spi(BURST_MODE_STANDARD_8WORDS,buf_package);
            event_counter = 0;
        }
#endif
    }

    return 0;
}



