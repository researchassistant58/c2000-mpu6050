//*************************************************************************************
//Includes
//*************************************************************************************
#include "f2802x_headers/include/F2802x_Device.h"
#include "f2802x_common/include/f2802x_examples.h"
#include "f2802x_common/include/clk.h"
#include "f2802x_common/include/flash.h"
#include "f2802x_common/include/cpu.h"
#include "f2802x_common/include/pie.h"
#include "f2802x_common/include/pll.h"
#include "f2802x_common/include/wdog.h"
#include "f2802x_common/include/sci.h"
#include "f2802x_common/include/gpio.h"
#include "MPU6050.h"
#include "Math.h"
//*************************************************************************************
//functions
//*************************************************************************************
//init functions
void setup_handles();
void init_system();
void I2CA_Init(void);
void init_gpio_scia();
void scia_init(void);
void Setup_MPU6050();
void init_gpio_led0();
//general purpose functions
void I2CA_WriteData(uint8_t yazilacakaddress, uint8_t veri);
uint8_t I2CA_ReadData(uint8_t reg);
uint16_t I2CA_Read2Bytes(uint8_t reg_low, uint8_t reg_high);
void sci_msg_int(uint16_t val,int digit);
void scia_msg(char * msg);
int powxy(int x,int y);
//*************************************************************************************
//Global variables
//handles
CPU_Handle myCpu;
CLK_Handle myClk;
FLASH_Handle myFlash;
PIE_Handle myPie;
PLL_Handle myPll;
WDOG_Handle myWDog;
GPIO_Handle myGpio;
SCI_Handle mySci;
//mpu6050 variables
uint8_t acc_x_h, acc_x_l, acc_y_h, acc_y_l, acc_z_h, acc_z_l;
uint8_t gyro_x_h, gyro_x_l, gyro_y_h, gyro_y_l, gyro_z_h, gyro_z_l;
uint16_t accel_x, accel_y, accel_z;
uint16_t gyro_x, gyro_y, gyro_z;
uint8_t who_m_i_reg;
//*************************************************************************************
//MAIN
//*************************************************************************************
void main()
{
	//setup
    setup_handles();
    init_system();
    I2CA_Init();
    init_gpio_scia();
    scia_init();
    Setup_MPU6050();
    init_gpio_led0();
    //
    scia_msg("Slave address::");
    who_m_i_reg=I2CA_ReadData(MPU6050_RA_WHO_AM_I);
    sci_msg_int(who_m_i_reg,3);
    //loop
    while (1){
       DELAY_US(100);
    	   //gyro_z_h = I2CA_ReadData(MPU6050_RA_GYRO_ZOUT_H);
       	   accel_x=I2CA_Read2Bytes(MPU6050_RA_ACCEL_XOUT_L,MPU6050_RA_ACCEL_XOUT_H);
       	   scia_msg("ax:\n");
    	   sci_msg_int(accel_x,5);
    	   SCI_putDataBlocking (mySci,'\n');//nl
    	   SCI_putDataBlocking (mySci,'\r');//cr

       	   accel_y=I2CA_Read2Bytes(MPU6050_RA_ACCEL_YOUT_L,MPU6050_RA_ACCEL_YOUT_H);
       	   scia_msg("ay:\n");
    	   sci_msg_int(accel_y,5);
    	   SCI_putDataBlocking (mySci,'\n');//nl
    	   SCI_putDataBlocking (mySci,'\r');//cr

       	   accel_z=I2CA_Read2Bytes(MPU6050_RA_ACCEL_ZOUT_L,MPU6050_RA_ACCEL_ZOUT_H);
       	   scia_msg("az:\n");
    	   sci_msg_int(accel_z,5);
    	   SCI_putDataBlocking (mySci,'\n');//nl
    	   SCI_putDataBlocking (mySci,'\r');//cr

       	   accel_x=I2CA_Read2Bytes(MPU6050_RA_GYRO_XOUT_L,MPU6050_RA_GYRO_XOUT_H);
       	   scia_msg("gx:\n");
    	   sci_msg_int(gyro_x,5);
    	   SCI_putDataBlocking (mySci,'\n');//nl
    	   SCI_putDataBlocking (mySci,'\r');//cr

       	   accel_x=I2CA_Read2Bytes(MPU6050_RA_GYRO_YOUT_L,MPU6050_RA_GYRO_YOUT_L);
       	   scia_msg("gy:\n");
    	   sci_msg_int(gyro_y,5);
    	   SCI_putDataBlocking (mySci,'\n');//nl
    	   SCI_putDataBlocking (mySci,'\r');//cr

       	   accel_x=I2CA_Read2Bytes(MPU6050_RA_GYRO_ZOUT_L,MPU6050_RA_GYRO_ZOUT_L);
       	   scia_msg("gz:\n");
    	   sci_msg_int(gyro_z,5);
    	   SCI_putDataBlocking (mySci,'\n');//nl
    	   SCI_putDataBlocking (mySci,'\r');//cr


       }
}
//*************************************************************************************
//END MAIN
//*************************************************************************************
void setup_handles()
{
    myClk = CLK_init((void *) CLK_BASE_ADDR, sizeof(CLK_Obj));
    myCpu = CPU_init((void *) NULL, sizeof(CPU_Obj));
    myPie = PIE_init((void *) PIE_BASE_ADDR, sizeof(PIE_Obj));
    myPll = PLL_init((void *) PLL_BASE_ADDR, sizeof(PLL_Obj));
    myWDog = WDOG_init((void *) WDOG_BASE_ADDR, sizeof(WDOG_Obj));
    myGpio = GPIO_init((void *) GPIO_BASE_ADDR, sizeof(GPIO_Obj));
    mySci = SCI_init((void *) SCIA_BASE_ADDR, sizeof(SCI_Obj));

}
//*************************************************************************************
void init_system()
{
    // running from flash - copy RAM based functions to RAM
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t) &RamfuncsLoadSize);

    // disable watchdog
    WDOG_disable(myWDog);

    // load factory calibration
    CLK_enableAdcClock(myClk);
    (*Device_cal )();
    CLK_disableAdcClock(myClk);

    // select the internal oscillator 1 (10 MHz) as the clock source
    CLK_setOscSrc(myClk, CLK_OscSrc_Internal);

    // setup the PLL for 10 MHz * 12 / 2 = 60 MHz
    PLL_setup(myPll, PLL_Multiplier_12, PLL_DivideSelect_ClkIn_by_2);

    // disable PIE and all interrupts
    PIE_disable(myPie);
    PIE_disableAllInts(myPie);
    CPU_disableGlobalInts(myCpu);
    CPU_clearIntFlags(myCpu);
}
//*************************************************************************************
void I2CA_Init(void)
{
		EALLOW;
		SysCtrlRegs.PCLKCR0.bit.I2CAENCLK 	= 0x0001;  // The I2C module is clocked.
		EDIS;

		EALLOW;
		GpioCtrlRegs.GPBPUD.bit.GPIO32 		= 0x0000;
		GpioCtrlRegs.GPBPUD.bit.GPIO33 		= 0x0000;
		GpioCtrlRegs.GPBQSEL1.bit.GPIO32 	= 0x0003;
	    GpioCtrlRegs.GPBQSEL1.bit.GPIO33 	= 0x0003;
		GpioCtrlRegs.GPBMUX1.bit.GPIO32 	= 0x0001;  // 1 atanmasi demek = SDAA - I2C Data open drain bidirectional port (I/O)
		GpioCtrlRegs.GPBMUX1.bit.GPIO33 	= 0x0001;  // 1 atanmasi demek = SCLA - I2C Clock open drain bidirectional port (I/O)

	    EDIS;
	   	// Initialize I2C Module
	   	// Note : 	The prescaler must be initialized only while the I2C module is in the reset state (IRS = 0 in I2CMDR).
	   	// 			The prescaled frequency takes effect only when IRS is changed to 1.
	   	//			Changing the IPSC value while IRS = 1 has no effect.

	   	I2caRegs.I2CPSC.all 	= 0x0005;	// Prescaler - need 7-12 Mhz on module clk (fmod = SYSCLKOUT / (I2CPSC+1) = 60 Mhz / 6 = 10 Mhz
	   	I2caRegs.I2CCLKL 		= 0x002D;	// Low time duration = (1/fmod)*(I2CCLKL+d) NOTE: must be non zero
	   	I2caRegs.I2CCLKH 		= 0x002D;	// High Time Duration= (1/fmod)*(I2CCLKH+d) NOTE: must be non zero
											// I2CPSC = 0-> (d = 7) , I2CPSC = 1-> (d = 6), I2CPSC > 1 -> (d = 5)
											// TSCL = (I2CPSC+1)*[(I2CCLKL+d)+(I2CCLKH+d)]/SYSCLKOUT   fSCL is configured to 100 Khz
	   	I2caRegs.I2CIER.all 	= 0x0000;	// Disable all interrupts


	  	I2caRegs.I2CFFTX.all 	= 0x6000;	// Enable FIFO mode and TXFIFO
	   	I2caRegs.I2CFFRX.all 	= 0x2040;	// Enable RXFIFO, clear RXFFINT,

	   	I2caRegs.I2CMDR.all 	= 0x0020;	// 1: Take I2C out of reset
	   										// 0: Stop I2C when suspended
}
//*************************************************************************************
void init_gpio_scia()
{
    // set SCI pull-ups
    GPIO_setPullUp(myGpio, GPIO_Number_28, GPIO_PullUp_Enable);
    GPIO_setPullUp(myGpio, GPIO_Number_29, GPIO_PullUp_Disable);

    // use asynchronous qualification for the SCI input signal
    GPIO_setQualification(myGpio, GPIO_Number_28, GPIO_Qual_ASync);

    // mux gpio28/29 to SCI-A
    GPIO_setMode(myGpio, GPIO_Number_28, GPIO_28_Mode_SCIRXDA);
    GPIO_setMode(myGpio, GPIO_Number_29, GPIO_29_Mode_SCITXDA);
}
//*************************************************************************************
void scia_init()
{
    // enable the SCI-A clock
    CLK_enableSciaClock(myClk);

    // disable parity
    SCI_disableParity(mySci);

    // 1 stop bit
    SCI_setNumStopBits(mySci, SCI_NumStopBits_One);

    // 8 data bits
    SCI_setCharLength(mySci, SCI_CharLength_8_Bits);

    // 9600 baud rate
    SCI_setBaudRate(mySci, SCI_BaudRate_9_6_kBaud);

    // enable free run - continue SCI operation regardless of emulation suspend
    SCI_setPriority(mySci, SCI_Priority_FreeRun);

    // enable TX and RX
    SCI_enableTx(mySci);
    SCI_enableRx(mySci);

    // enable the SCI interface
    SCI_enable(mySci);
}
//*************************************************************************************
void Setup_MPU6050()
{
    //Sets sample rate to 8000/1+7 = 1000Hz
	I2CA_WriteData( MPU6050_RA_SMPLRT_DIV, 0x07);
    //Disable FSync, 256Hz DLPF
	I2CA_WriteData( MPU6050_RA_CONFIG, 0x00);
    //Disable gyro self tests, scale of 500 degrees/s
	I2CA_WriteData( MPU6050_RA_GYRO_CONFIG, 0b00001000);
    //Disable accel self tests, scale of +-2g, no DHPF
	I2CA_WriteData( MPU6050_RA_ACCEL_CONFIG, 0x00);
    //Freefall threshold of |0mg|
	I2CA_WriteData( MPU6050_RA_FF_THR, 0x00);
    //Freefall duration limit of 0
	I2CA_WriteData( MPU6050_RA_FF_DUR, 0x00);
    //Motion threshold of 0mg
	I2CA_WriteData( MPU6050_RA_MOT_THR, 0x00);
    //Motion duration of 0s
	I2CA_WriteData( MPU6050_RA_MOT_DUR, 0x00);
    //Zero motion threshold
	I2CA_WriteData( MPU6050_RA_ZRMOT_THR, 0x00);
    //Zero motion duration threshold
	I2CA_WriteData( MPU6050_RA_ZRMOT_DUR, 0x00);
    //Disable sensor output to FIFO buffer
	I2CA_WriteData( MPU6050_RA_FIFO_EN, 0x00);


    //AUX I2C setup
    //Sets AUX I2C to single master control, plus other config
	I2CA_WriteData( MPU6050_RA_I2C_MST_CTRL, 0x00);
    //Setup AUX I2C slaves
	I2CA_WriteData( MPU6050_RA_I2C_SLV0_ADDR, 0x00);
	I2CA_WriteData( MPU6050_RA_I2C_SLV0_REG, 0x00);
	I2CA_WriteData( MPU6050_RA_I2C_SLV0_CTRL, 0x00);
	I2CA_WriteData( MPU6050_RA_I2C_SLV1_ADDR, 0x00);
	I2CA_WriteData( MPU6050_RA_I2C_SLV1_REG, 0x00);
	I2CA_WriteData( MPU6050_RA_I2C_SLV1_CTRL, 0x00);
	I2CA_WriteData( MPU6050_RA_I2C_SLV2_ADDR, 0x00);
	I2CA_WriteData( MPU6050_RA_I2C_SLV2_REG, 0x00);
	I2CA_WriteData( MPU6050_RA_I2C_SLV2_CTRL, 0x00);
	I2CA_WriteData( MPU6050_RA_I2C_SLV3_ADDR, 0x00);
	I2CA_WriteData( MPU6050_RA_I2C_SLV3_REG, 0x00);
	I2CA_WriteData( MPU6050_RA_I2C_SLV3_CTRL, 0x00);
	I2CA_WriteData( MPU6050_RA_I2C_SLV4_ADDR, 0x00);
	I2CA_WriteData( MPU6050_RA_I2C_SLV4_REG, 0x00);
	I2CA_WriteData( MPU6050_RA_I2C_SLV4_DO, 0x00);
	I2CA_WriteData( MPU6050_RA_I2C_SLV4_CTRL, 0x00);
	I2CA_WriteData( MPU6050_RA_I2C_SLV4_DI, 0x00);


    //MPU6050_RA_I2C_MST_STATUS //Read-only
    //Setup INT pin and AUX I2C pass through
	I2CA_WriteData( MPU6050_RA_INT_PIN_CFG, 0x00);
    //Enable data ready interrupt
	I2CA_WriteData( MPU6050_RA_INT_ENABLE, 0x00);

    //Slave out, dont care
	I2CA_WriteData( MPU6050_RA_I2C_SLV0_DO, 0x00);
	I2CA_WriteData( MPU6050_RA_I2C_SLV1_DO, 0x00);
	I2CA_WriteData( MPU6050_RA_I2C_SLV2_DO, 0x00);
	I2CA_WriteData( MPU6050_RA_I2C_SLV3_DO, 0x00);
    //More slave config
	I2CA_WriteData( MPU6050_RA_I2C_MST_DELAY_CTRL, 0x00);
    //Reset sensor signal paths
	I2CA_WriteData( MPU6050_RA_SIGNAL_PATH_RESET, 0x00);
    //Motion detection control
	I2CA_WriteData( MPU6050_RA_MOT_DETECT_CTRL, 0x00);
    //Disables FIFO, AUX I2C, FIFO and I2C reset bits to 0
	I2CA_WriteData( MPU6050_RA_USER_CTRL, 0x00);
    //Sets clock source to gyro reference w/ PLL
	I2CA_WriteData( MPU6050_RA_PWR_MGMT_1, 0b00000010);
    //Controls frequency of wakeups in accel low power mode plus the sensor standby modes
	I2CA_WriteData( MPU6050_RA_PWR_MGMT_2, 0x00);
    //Data transfer to and from the FIFO buffer
	I2CA_WriteData( MPU6050_RA_FIFO_R_W, 0x00);
    //MPU6050_RA_WHO_AM_I             //Read-only, I2C address


}
//*************************************************************************************
void init_gpio_led0()
{
    GPIO_setMode(myGpio, GPIO_Number_0, GPIO_0_Mode_GeneralPurpose);
    GPIO_setDirection(myGpio, GPIO_Number_0, GPIO_Direction_Output);
}
//*************************************************************************************
void I2CA_WriteData(uint8_t yazilacakaddress, uint8_t veri)
{
	I2caRegs.I2CSAR 	= 0x0068;	// slave adress
	I2caRegs.I2CCNT		= 2	;	// indicate how many data bytes to transfer when the I2C module is configured as a transmitter,
	                            // or to receive when configured as a master receiver
	I2caRegs.I2CDXR		= yazilacakaddress;
	I2caRegs.I2CDXR		= veri;
	I2caRegs.I2CMDR.all = 0x2E20;
	while(!I2caRegs.I2CSTR.bit.SCD);			//
		I2caRegs.I2CSTR.bit.SCD 	= 1;        //
}
//*************************************************************************************
uint8_t I2CA_ReadData(uint8_t reg)
{
	uint8_t tempdata;
	I2caRegs.I2CSAR 	= 0x0068;		// slave adress
	I2caRegs.I2CCNT		= 1;		    //
	I2caRegs.I2CDXR		= reg;
	I2caRegs.I2CMDR.all = 0x2620;	    //
	while(!I2caRegs.I2CSTR.bit.ARDY);	//
                                        //

	I2caRegs.I2CCNT		= 1;
	I2caRegs.I2CMDR.all = 0x2C20;		//
	while(!I2caRegs.I2CSTR.bit.SCD);    //
	tempdata	     	= I2caRegs.I2CDRR;     //
	return(tempdata);
}
//*************************************************************************************
uint16_t I2CA_Read2Bytes(uint8_t reg_low, uint8_t reg_high)
{
	uint8_t val1,val2;
	uint16_t result;
	val1=I2CA_ReadData(reg_low);
	val2=I2CA_ReadData(reg_high);
	result=val2;
	result=result<<8;
	result+=val1;
	return result;
}
//*************************************************************************************
void sci_msg_int(uint16_t val,int digit)
{
	int n=0;
	for(n=(digit);n>0;n--)
		SCI_putDataBlocking(mySci,(int) (val%powxy(10,n))/(powxy(10,n-1))+0x30);
}
//*************************************************************************************
void scia_msg(char * msg)
{
    int i;
    i = 0;
    while(msg[i] != '\0')
    {
    	SCI_putDataBlocking (mySci,msg[i]);
        i++;
    }
}
//*************************************************************************************
int powxy(int x,int y){
	int res=1;
	int n=0;
	if(y==0)
	{
		return 1;
	}
	else
	{
		for (n=0;n<y;n++)
			res=res*x;
		return res;
	}
}
