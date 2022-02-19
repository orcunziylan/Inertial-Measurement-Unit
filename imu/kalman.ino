
//The Wire library is used for I2C communication
#include <Wire.h>

//This is a list of registers in the ITG-3200. Registers are parameters that determine how the sensor will behave, or they can hold data that represent the
//sensors current status.
//To learn more about the registers on the ITG-3200, download and read the datasheet.
char WHO_AM_I = 0x00; //register 0
char SMPLRT_DIV= 0x15; //register 21
char DLPF_FS = 0x16; //register 22

char TEMP_OUT_H = 0x1B;
char TEMP_OUT_L = 0x1C;
char GYRO_XOUT_H = 0x1D;
char GYRO_XOUT_L = 0x1E;
char GYRO_YOUT_H = 0x1F;
char GYRO_YOUT_L = 0x20;
char GYRO_ZOUT_H = 0x21;
char GYRO_ZOUT_L = 0x22;

//These offset values are measured from sensor measurements [mbg: 12 Oct. 2016]
int wxOffset = -42;
int wyOffset = 61;
int wzOffset = 8;


//Accelorometer offset asc11
double asc11=-6028.1 ,  asc12=-85.23  , asc13=-541.23;
double asc21=-88.12,    asc22=-6076.85, asc23=-64.52;
double asc31=518.99,  asc32=185.69,   asc33=-6203.28;
double cdasc=100000;//common divisor accelorometer
double ax0=31.73;
double ay0=-21.42;
double az0=-30.87;



//Magnetometer offset msc-x,y,z=magnetometer scale; magnetometer offset=m-x,y,z-0 
double mx0=-71.69;
double my0=-13.57;
double mz0=-0.98;
double mscx=485.39;
double mscy=528.93;
double mscz=523.08;
//sensivity magnetometer
double msex=1.1,msey=1.1,msez=0.9;
//misaligment error between the magnetic sensor sensing axes and the device body axes
//double  mm11=-75.92, mm12=-1.78, mm13=-8.43;
//double  mm21=-1.79,  mm22=97.86, mm23=-2.38;
//double  mm31=-10.04, mm32=20.49, mm33=99.61;
//double  cdmm=100;//common divisor misalignment

double pi = 3.14159265359; // the number pi

//This is a list of settings that can be loaded into the registers.
//DLPF, Full Scale Register Bits

//Set DLPF_CFG is a 3 bit parameter. It can be set to 3 for 1kHz Fint and 42 Hz Low Pass Filter
char DLPF_CFG_0 = 1<<0;
char DLPF_CFG_1 = 1<<1;
char DLPF_CFG_2 = 0<<2;

//FS_SEL is a 2 bit parameter. It must be set to 3 for proper operation. This corresponds to +-2000 degrees/s for the full range
char DLPF_FS_SEL_0 = 1<<3;
char DLPF_FS_SEL_1 = 1<<4;

//I2C devices each have an address. The address is defined in the datasheet for the device. The ITG-3200 breakout board can have different address depending on how
//the jumper on top of the board is configured. By default, the jumper is connected to the VDD pin. When the jumper is connected to the VDD pin the I2C address
//is 0x69.
char gyroDevice = 0x69;

//In the setup section of the sketch the serial port will be configured, the i2c communication will be initialized, and the itg-3200 will be configured.

  float Wxx=0, Wyy=0, Wzz=0, Wxxx=0, Wyyy=0, Wzzz=0;
  double Q_angle  =  0.001; 
  double Q_gyro   =  0.003;  
  double R_angle  =  0.03;
  double dt = 0.1180555555;
  double fixedrollt=0;
  double fixedpitcht=0;
  double fixedyawt=0;
  int countlap=0;
  //x or.
  double x_angle = 0;  
  double x_angle_deg;
  double x_bias = 0;
  double P_00 = 1, P_01 = 0, P_10 = 0, P_11 = 1;  
  double y, S;
  double K_0,K_1;
  //y or.
  double y_angle = 0;  
  double y_angle_deg;
  double y_bias = 0;
  double Py_00 = 1, Py_01 = 0, Py_10 = 0, Py_11 = 1;  
  double Ky_0,Ky_1;
  //z or.
  double z_angle = 0;  
  double z_angle_deg;
  double z_bias = 0;
  double Pz_00 = 1, Pz_01 = 0, Pz_10 = 0, Pz_11 = 1;  
  double Kz_0,Kz_1;
  


//acc+mag reg



/* LSM303dlm Address definitions */
char magDevice =  0x1E;  
char accDevice = 0x18;  


/* LSM303dlm Register definitions */
char CTRL_REG1_A = 0x20;
char CTRL_REG2_A =0x21;
char CTRL_REG3_A = 0x22;
char CTRL_REG4_A= 0x23;
char CTRL_REG5_A =0x24;

char HP_FILTER_RESET_A =0x25;
char REFERENCE_A =0x26;
char STATUS_REG_A =0x27;

char OUT_X_L_A =0x28;
char OUT_X_H_A= 0x29;

char OUT_Y_L_A =0x2A;
char OUT_Y_H_A =0x2B;

char OUT_Z_L_A =0x2C;
char OUT_Z_H_A= 0x2D;

char INT1_CFG_A =0x30;
char INT1_SOURCE_A =0x31;
char INT1_THS_A =0x32;
char INT1_DURATION_A= 0x33;
char INT2_CFG_A =0x34;
char INT2_SOURCE_A =0x35;
char INT2_THS_A = 0x36;
char INT2_DURATION_A =0x37;


//magnetometer register

char CRA_REG_M= 0x00;
char CRB_REG_M =0x01;
char MR_REG_M =0x02;

char OUT_X_H_M = 0x03;
char OUT_X_L_M= 0x04;

char OUT_Z_H_M =0x05;
char OUT_Z_L_M =0x06;

char OUT_Y_H_M =0x07;
char OUT_Y_L_M =0x08;

char SR_REG_M =0x09;
char IRA_REG_M =0x0A;
char IRB_REG_M =0x0B;
char IRC_REG_M =0x0C;


void setup()
{
  //Create a serial connection using a 9600bps baud rate.
  Serial.begin(28800);

  //Initialize the I2C communication. This will set the Arduino up as the 'Master' device.
  Wire.begin();

  //Read the WHO_AM_I register and print the result
  //char id1 = 0; 
 // id1 = readRegister(gyroDevice, 0x00);  
  //Serial.print("ID: ");
  //Serial.println(id1, HEX);
  //  char id2 = 0; 
  //id2 = readRegister(accDevice, 0x0F);  
  //Serial.print("ID: ");
  //Serial.println(id2, HEX);
    //char id3 = 0; 
  //id3 = readRegister(magDevice, 0x0F);  
  //Serial.print("ID: ");
  //Serial.println(id3, HEX);

 
  
  //Configure the gyroscope
  //Set the gyroscope scale for the outputs to +/-2000 degrees per second
  writeRegister(gyroDevice, DLPF_FS, (DLPF_FS_SEL_0|DLPF_FS_SEL_1|DLPF_CFG_2|DLPF_CFG_1|DLPF_CFG_0));
  //Set the sample rate to 10 hz
  writeRegister(gyroDevice, SMPLRT_DIV, 99);

  //char id4 = 0;
  //id4 = readRegister(gyroDevice, 0x16);
 // Serial.print("DLPS_FS: ");
  //Serial.println(id4, BIN);





//config   acceloremeter
// 0x27 = 0b00101111
// ODR = 00101 (100 Hz ODR) ;  Zen = Yen = Xen = 1 (all axes enabled)
writeRegister( accDevice,   CTRL_REG1_A , 0x2F );  // 0x27 = normal power mode, all accel axes on


 //boot=1 calibrated device
 //fds =0 bilmiyorum
 //HPM1,HPM0=00 normal mode
 //HPCF0,1 = 00 high pass filtre uygularsak gravity okunmaz  degeri fark etmez
//0b1000 0000; 
 writeRegister(accDevice,CTRL_REG2_A,0x80);

 //IHL =0
 //PP_OD = 1 open drain
 //LIR2=1
 //LIR1=1
 //l1(2)_CFG1(2)=11 device ayarlasın
 //0b0111 1111
 writeRegister(accDevice,CTRL_REG3_A,0x7F);

 // 0x00 = 0b00000000
 // FS = 00 (+/- 2 g full scale)
  writeRegister(accDevice,CTRL_REG4_A, 0x00);
 
  
 //config magnetometer
 
  // 0x0C = 0b00001100
  // DO = 011 (7.5 Hz ODR)
  writeRegister(magDevice,CRA_REG_M, 0x0C);

    // 0x20 = 0b0010 0000
    // GN = 001 (+/- 1.3 gauss full scale)( Gain X/Y and Z [LSB/Gauss]= 1100 )( Gain Z [LSB/Gauss] =980)
  writeRegister(magDevice,CRB_REG_M, 0x20);
 
  // 0x00 = 0b00000000
  // MD = 00 (continuous-conversion mode)
  writeRegister(magDevice,MR_REG_M, 0x00);



}

//The loop section of the sketch will read the X,Y and Z output rates from the gyroscope and output them in the Serial Terminal
void loop()
{
  

  double Wx, Wy, Wz,Wx1,Wy1,Wz1,temp;
  double Ax1,Ay1,Az1,Ax,Ay,Az,Axs,Ays,Azs;
  double Mx,My,Mz,Mx1,My1,Mz1,Mx2,My2,Mz2,Mxs,Mys,Mzs;
  double fixedpitch,fixedroll,fixedyaw;
   double fixedpitch1,fixedroll1;
  double norm;

  
  //At the website https://forum.sparkfun.com/viewtopic.php?f=14&t=22207 it is mentioned to perform the below normalizations
  
  
  
  Ax1 = readAx(); //the value mg
  Ay1 = readAy(); // the value mg
  Az1 = readAz(); // the value mg
  Ax  = (((asc11*Ax1+asc12*Ay1+asc13*Az1)/cdasc)+ax0);
  Ay  = (((asc21*Ax1+asc22*Ay1+asc23*Az1)/cdasc)+ay0);
  Az  = (((asc31*Ax1+asc32*Ay1+asc33*Az1)/cdasc)+az0);  
    
  
  
  Mx1 = readMx();//read and convert sensivity
  My1 = readMy();
  Mz1 = readMz();
  Mx  = (((Mx1/msex)-mx0)/mscx);// mgauss
  My  = (((My1/msey)-my0)/mscy);//mgauss
  Mz  = (((Mz1/msez)-mz0)/mscz);//mgauss
  //Mx  = (mm11*Mx1+mm12*My1+mm13*Mz1)/cdmm;
  //My  = (mm21*Mx1+mm22*My1+mm23*Mz1)/cdmm;
  //Mz  = (mm31*Mx1+mm32*My1+mm33*Mz1)/cdmm;
  
  temp = 35.0+((readGyroT()+13200)/280); //temperature is in deg. celcius [mbg: 18 Nov. 16]
  
  
  
  
  //Rate = (readX()-xOffset)/14.375; //angular velocity is in deg/s [mbg: 18 Nov. 16]
  //Rate = (readY()-yOffset)/14.375;
  //zRate = (readZ()-zOffset)/14.375;

  Wx1 = readWx(); //angular velocity is in deg/s [mbg: 18 Nov. 16]
  Wy1 = readWy();
  Wz1 = readWz();
  
  Wx = ((Wx1-wxOffset)/14.375); //angular velocity is in deg/s [mbg: 18 Nov. 16]
  Wy = ((Wy1-wyOffset)/14.375);
  Wz = ((Wz1-wzOffset)/14.375);
  
  //calculation static part
  norm = sqrt(Ax*Ax+Ay*Ay+Az*Az);
  Axs =Ax/norm;
  Ays =Ay/norm;
  Azs =Az/norm;
  norm = sqrt(Mx*Mx+My*My+Mz*Mz);
  Mxs =Mx/norm;
  Mys =My/norm;
  Mzs =Mz/norm;
  fixedpitch=asin(-Axs);
  fixedroll=asin((Ays)/(cos(fixedpitch)));

  
  
  Mx2=Mxs*cos(fixedpitch)+Mzs*sin(fixedpitch);
  My2=Mxs*sin(fixedroll)*sin(fixedpitch)+Mys*cos(fixedroll)-Mzs*sin(fixedroll)*cos(fixedpitch);
  Mz2=-Mxs*cos(fixedroll)*sin(fixedpitch)+Mys*sin(fixedroll)+Mzs*cos(fixedroll)*cos(fixedpitch);  
  if(Mx2>0 && My2>=0)
    fixedyaw=atan(My2/Mx2)*180/pi;
   else if(Mx2<0)
    fixedyaw=180+atan(My2/Mx2)*180/pi;
   else if(Mx2>0 && My2<=0)
    fixedyaw=360+atan(My2/Mx2)*180/pi;
   else if(Mx2==0 && My2<0)
    fixedyaw=90;
   else if(Mx2==0 && My2>0)
    fixedyaw=90;
  
  
  fixedpitch1=fixedpitch*(180/pi);
  fixedroll1=fixedroll*(180/pi);
  
  // calibration
  if(countlap<10){
    
      if(Wx<1 && Wx>(-1) && Wy<1 && Wy>(-1) && Wz<1 && Wz>(-1)){
        
      Wxxx+=Wx;   
      Wyyy+=Wy;
      Wzzz+=Wz;
      fixedyawt+=fixedyaw; 
      
      countlap++;
      }
    }else if(countlap==10){
      Wxxx=Wxxx/10;
      Wyyy=Wyyy/10;
      Wzzz=Wzzz/10;
      fixedyawt=fixedyawt/10;        
      countlap++;    
    }else if(countlap>10){
      
      // Gyro X Calibre
      Wx-=Wxxx;      
      if(Wx>0.1 || Wx<(-0.1)){
           Wxx+=Wx*dt;
      }
      if( fixedroll > -0.0349 ){
        if (fixedroll < 0.0349 ){
          if(Wxx<(-2))
            Wxx+=2;
          else if(Wxx>2)
            Wxx-=2;
        }
      }

      // Gyro Y Calibre 
      Wy-=Wyyy;      
      if(Wy>0.1 || Wy<(-0.1)){
           Wyy+=Wy*dt;
      }
      if( fixedpitch > -0.0349 ){
        if (fixedpitch < 0.0349 ){
         if(Wyy<(-2))
            Wyy+=2;
          else if(Wyy>2)
            Wyy-=2;
        }
      }


      // Gyro Z Calibre
      Wz-=Wzzz;      
      if(Wz>0.1 || Wz<(-0.1)){
           Wzz+=Wz*dt;
      }
      if(Wzz>360)
           Wzz-=360;
      if(Wzz<-360)
           Wzz+=360;
           
      fixedyaw=fixedyaw-fixedyawt;
	
  //Degree ==> Rad
  Wx=Wx*(pi/180);   Wy=Wy*(pi/180);   Wz=Wz*(pi/180);
  Wxx=Wxx*(pi/180);   Wyy=Wyy*(pi/180);   Wzz=Wzz*(pi/180);
  fixedyaw=fixedyaw*(pi/180); 
  

  
  //Orientation   
    //  Roll Kalman 
    
          x_angle +=  dt * (Wxx - x_bias)  ;
                              
          P_00 += dt * ( dt * P_11 - P_01 - P_10 + Q_angle );
          P_01 -=  dt * P_11;
          P_10 -=  dt * P_11;
          P_11 +=  Q_gyro * dt;
          
          y = fixedroll - x_angle;
          
          S = P_00 + R_angle;
          
          K_0 = P_00 / S;
          K_1 = P_10 / S;
          
          x_angle += K_0 * y;
          x_bias  += K_1 * y;

          float P00_temp = P_00;
          float P01_temp = P_01;
          
          P_00 -= K_0 * P00_temp;
          P_01 -= K_0 * P01_temp;
          P_10 -= K_1 * P00_temp;
          P_11 -= K_1 * P01_temp;  


      //  Pitch Kalman
              
          y_angle +=  dt * (Wyy - y_bias)  ;
                              
          Py_00 += dt * ( dt * Py_11 - Py_01 - Py_10 + Q_angle );
          Py_01 -=  dt * Py_11;
          Py_10 -=  dt * Py_11;
          Py_11 +=  Q_gyro * dt;
          
          y = fixedpitch - y_angle;
          
          S = Py_00 + R_angle;
          
          Ky_0 = Py_00 / S;
          Ky_1 = Py_10 / S;
          
          y_angle += Ky_0 * y;
          y_bias  += Ky_1 * y;

          float Py00_temp = Py_00;
          float Py01_temp = Py_01;
          
          Py_00 -= Ky_0 * Py00_temp;
          Py_01 -= Ky_0 * Py01_temp;
          Py_10 -= Ky_1 * Py00_temp;
          Py_11 -= Ky_1 * Py01_temp;

      //  Yaw Kalman
              
          z_angle +=  dt * (Wzz - z_bias)  ;
                              
          Pz_00 += dt * ( dt * Pz_11 - Pz_01 - Pz_10 + Q_angle );
          Pz_01 -=  dt * Pz_11;
          Pz_10 -=  dt * Pz_11;
          Pz_11 +=  Q_gyro * dt;
          
          y = fixedpitch - z_angle;
          
          S = Pz_00 + R_angle;
          
          Kz_0 = Pz_00 / S;
          Kz_1 = Pz_10 / S;
          
          z_angle += Kz_0 * y;
          z_bias  += Kz_1 * y;

          float Pz00_temp = Pz_00;
          float Pz01_temp = Pz_01;
          
          Pz_00 -= Kz_0 * Pz00_temp;
          Pz_01 -= Kz_0 * Pz01_temp;
          Pz_10 -= Kz_1 * Pz00_temp;
          Pz_11 -= Kz_1 * Pz01_temp;


  // Rad to Deg
  x_angle_deg=x_angle*(180/pi);
  y_angle_deg=y_angle*(180/pi);  
  z_angle_deg=z_angle*(180/pi);        
  Wx=Wx*(180/pi);   Wy=Wy*(180/pi);   Wz=Wz*(180/pi);  
  Mx=Mx*(180/pi);   My=My*(180/pi);   Mz=Mz*(180/pi);      
  Wxx=Wxx*(180/pi);   Wyy=Wyy*(180/pi);   Wzz=Wzz*(180/pi); 
  fixedroll=fixedroll*(180/pi); fixedpitch=fixedpitch*(180/pi);   fixedyaw=fixedyaw*(180/pi);    
  }
  //---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

  //Print the output rates accordingly plx daq.
  
  Serial.print("DATA,TIME,");  
  Serial.print(temp);  Serial.print(',');  
    
  //Serial.print(x_angle_deg);  Serial.print(',');  
  //Serial.print(Wx);  Serial.print(','); 
  //Serial.print(Wxx);  Serial.print(',');
  //Serial.print(Wxxx);  Serial.print(',');
  //Serial.print(fixedroll);  Serial.print(',');
  Serial.print(K_0);  Serial.print(',');

  //Serial.print(y_angle_deg);  Serial.print(',');  
  //Serial.print(Wy);  Serial.print(','); 
  //Serial.print(Wyy);  Serial.print(',');
  //Serial.print(Wyyy);  Serial.print(',');
  //Serial.print(fixedpitch);  Serial.print(',');
  Serial.print(Ky_0);  Serial.print(',');

  //Serial.print(z_angle_deg);  Serial.print(',');  
  //Serial.print(Wz);  Serial.print(','); 
  //Serial.print(Wzz);  Serial.print(',');
  //Serial.print(Wzzz);  Serial.print(',');
  //Serial.print(fixedyaw);  Serial.print(',');
  //Serial.print(fixedyawt);  Serial.print(',');
  Serial.print(Kz_0);  Serial.print(',');
   
  //Serial.print(y);  Serial.print(',');
  //Serial.print(Wz);  Serial.print(',');    
  //Serial.print(countlap);  Serial.print(','); 
  //Serial.print(Wx);  Serial.print(','); 
   
  //Serial.print(fixedrollt);  Serial.print(',');
  //Serial.print(countlap);  Serial.print(',');
  //Serial.print(Wxxx);  Serial.print(',');
  //Serial.print(Wyyy);  Serial.print(',');
  //Serial.print(Wzzz);  Serial.print(',');
  
  //Serial.print(K_0);  Serial.print(',');  Serial.print(P_00);  Serial.print(',');Serial.print(P_01);  Serial.print(',');  Serial.print(P_10);  Serial.print(',');Serial.print(P_11);  Serial.print(',');
  //Serial.print(y_angle_deg);  Serial.print(',');  Serial.print(Wyy);  Serial.print(',');
  //Serial.print(fixedpitch);  Serial.print(','); 
  //Serial.print(Ky_0); Serial.print(',');
  
  //Serial.print(Wx);  Serial.print(',');  Serial.print(Wy);  Serial.print(','); 
  //Serial.print(Wzz);  Serial.print(',');
  //Serial.print(Ax);  Serial.print(',');  Serial.print(Ay);  Serial.print(',');  Serial.print(Az);  Serial.print(',');
  //Serial.print(Mx);  Serial.print(',');  
  //Serial.print(My);  Serial.print(',');
  //Serial.print(Mz);  Serial.print(','); 
  //Serial.print(fixedyaw); Serial.print(','); 
  //Serial.print(fixedyawt); Serial.print(','); 
  //Serial.print(fixedpitch1); Serial.print(',');Serial.print(fixedroll1);  Serial.print(',');
  Serial.print('\n');
  
  
    

  //Wait 10ms before reading the values again. (Remember, the output rate was set to 100hz and 1reading per 10ms = 100hz.)
  delay(100);
}

//This function will write a value to a register.
//Parameters:
// char address: The I2C address of the sensor. For the ITG-3200 breakout the address is 0x69.
// char registerAddress: The address of the register on the sensor that should be written to.
// char data: The value to be written to the specified register.
void writeRegister(char device, char registerAddress, char data)
{
  //Initiate a communication sequence with the desired i2c device
  Wire.beginTransmission(device);
  //Tell the I2C address which register we are writing to
  Wire.write(registerAddress);
  //Send the value to write to the specified register
  Wire.write(data);
  //End the communication sequence
  Wire.endTransmission();
}

//This function will read the data from a specified register on the ITG-3200 and return the value.
//Parameters:
// char address: The I2C address of the sensor. For the ITG-3200 breakout the address is 0x69.
// char registerAddress: The address of the register on the sensor that should be read
//Return:
// unsigned char: The value currently residing in the specified register
unsigned char readRegister(char device, char registerAddress)
{
  //This variable will hold the contents read from the i2c device.
  unsigned char data=0;

  //Send the register address to be read.
  Wire.beginTransmission(device);
  //Send the Register Address
  Wire.write(registerAddress);
  //End the communication sequence.
  Wire.endTransmission();

  //Ask the I2C device for data
  Wire.beginTransmission(device);
  Wire.requestFrom(device, 1);

  //Wait for a response from the I2C device
  if(Wire.available()){
    //Save the data sent from the I2C device
    data = Wire.read();
  }

  //End the communication sequence.
  Wire.endTransmission();

  //Return the data read during the operation
  return data;
}

//This function is used to read the X-Axis rate of the gyroscope. The function returns the ADC value from the Gyroscope
//NOTE: This value is NOT in degrees per second.
//Usage: int xRate = readX();
int readWx(void)
{
  int data=0;
  data = readRegister(gyroDevice , GYRO_XOUT_H)<<8;
  data |= readRegister(gyroDevice , GYRO_XOUT_L);

  return data;
}

//This function is used to read the Y-Axis rate of the gyroscope. The function returns the ADC value from the Gyroscope
//NOTE: This value is NOT in degrees per second.
//Usage: int yRate = readY();
int readWy(void)
{
  int data=0;
  data = readRegister(gyroDevice, GYRO_YOUT_H)<<8;
  data |= readRegister(gyroDevice, GYRO_YOUT_L);

  return data;
}

//This function is used to read the Z-Axis rate of the gyroscope. The function returns the ADC value from the Gyroscope
//NOTE: This value is NOT in degrees per second.
//Usage: int zRate = readZ();
int readWz(void)
{
  int data=0;
  data = readRegister(gyroDevice, GYRO_ZOUT_H)<<8;
  data |= readRegister(gyroDevice, GYRO_ZOUT_L);

  return data;
}

//This function is used to read the temperature from the gyroscope. The function returns the ADC value from the Gyroscope
//NOTE: This value is NOT in degrees Celcius.
//Usage: int temp = readT();
int readGyroT(void)
{
  int data=0;
  data = readRegister(gyroDevice, TEMP_OUT_H)<<8;
  data |= readRegister(gyroDevice, TEMP_OUT_L);

  return data;
}



int readAx(void)
{
  int data =0;
  data = readRegister(accDevice ,OUT_X_H_A )<<8;
  data |= readRegister(accDevice, OUT_X_L_A);
return data;
  }


int readAy(void)
{
  int data =0;
  data = readRegister(accDevice ,OUT_Y_H_A )<<8;
  data |= readRegister(accDevice, OUT_Y_L_A);
return data;
  }
  int readAz(void)
{
  int data =0;
  data = readRegister(accDevice ,OUT_Z_H_A )<<8;
  data |= readRegister(accDevice, OUT_Z_L_A);
return data;
  }
  
  int readMx(void)
{
  int data =0;
  data = readRegister(magDevice ,OUT_X_H_M )<<8;
  data |= readRegister(magDevice, OUT_X_L_M);
return data;
  }
    int readMy(void)
{
  int data =0;
  data = readRegister(magDevice ,OUT_Y_H_M )<<8;
  data |= readRegister(magDevice, OUT_Y_L_M);
return data;
  }
    int readMz(void)
{
  int data =0;
  data = readRegister(magDevice ,OUT_Z_H_M )<<8;
  data |= readRegister(magDevice, OUT_Z_L_M);
return data;
  }
