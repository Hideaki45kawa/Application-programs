
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#include <math.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#define BME280_S32_t long signed int
#define BME280_U32_t long unsigned int
#define BME280_S64_t long long signed int

#define BME280_ADDRESS 0x76
#define INA219_ADDRESS 0x40
#define CURRENT_REGISTANCE 0.1

int main();

//BME280 device control routine
int setup_BME280(int fd);
void readTrim(int fd);
void readRawData(int fd);

//INA219 device control routine
int setup_INA219(int fd);
void read_INA219_RawData(int fd,short int *current,unsigned short int *voltage);
void cnv_INA219(int fd,double *current,double *voltage);


// I2C system routine
int setupi2c();
int setupi2cdev(int fd,int dev);
int writei2c(int fd,unsigned char buf[],int databyte);
int readi2c(int fd,unsigned char buf[],int databyte);
int writei2c8bits(int fd,unsigned char addr,unsigned char data);
unsigned char  readi2c8bits(int fd,unsigned char data);
int writei2c16bits(int fd,unsigned char addr,unsigned short int data);
unsigned short  readi2c16bits(int fd,unsigned char addr);


// CPU system get parameter routine
void get_daytime (char Time_string[]);
float get_cputemp(void);


// BME280 Calibrate 

BME280_S32_t cali_T(BME280_S32_t adc_T);
BME280_U32_t cali_P(BME280_S32_t adc_P);
BME280_U32_t cali_H(BME280_S32_t adc_H);


BME280_S32_t hum_raw,temp_raw,pres_raw;
BME280_S32_t t_fine;

 unsigned short dig_T1;
 signed short dig_T2;
 signed short dig_T3;

 unsigned short dig_P1;
 signed short dig_P2;
 signed short dig_P3;
 signed short dig_P4;
 signed short dig_P5;
 signed short dig_P6;
 signed short dig_P7;
 signed short dig_P8;
 signed short dig_P9;

 unsigned char dig_H1;
 signed short dig_H2;
 unsigned char dig_H3;
 signed short dig_H4;
 signed short dig_H5;
 signed char  dig_H6;


int main(int argc,char *argv[])
{
        int ret,fd;
    char daytime[255];
     float cputemp;
double current=0.0,voltage=0.0;

    double temp_act = 0.0, press_act = 0.0,hum_act=0.0;
    BME280_S32_t temp_cal;
    BME280_U32_t press_cal,hum_cal;


 get_daytime (daytime);
cputemp= get_cputemp();

fd=setupi2c();
 if (fd < -1 ) { printf ("Cannot use I2C device!\n"); exit(-1); }

    ret=setup_BME280(fd);
      readTrim(fd);
        readRawData(fd);

    temp_cal = cali_T(temp_raw);
    press_cal = cali_P(pres_raw);
    hum_cal = cali_H(hum_raw);

    temp_act = (double)temp_cal / 100.0;
    press_act = (double)press_cal / (256*100);
    hum_act = (double)hum_cal / 1024.0;

         printf("%s %f %f %f %f ",daytime,cputemp,temp_act,press_act,hum_act);
close(fd);

fd=setupi2c();
     ret=setup_INA219(fd);
  cnv_INA219(fd,&current,&voltage);
    printf("%f %f \n",current,voltage);

}

int setup_BME280(int fd)
{
    unsigned int osrs_t =  0b001;             //Temperature oversampling x 1
    unsigned int osrs_p =  0b001;             //Pressure oversampling x 1
    unsigned int osrs_h =  0b001;             //Humidity oversampling x 1
    unsigned int mode =    0b01;              //Forced mode
    unsigned int t_sb =    0b101;             //Tstandby 1000ms
    unsigned int filter =  0b000;             //Filter off
    unsigned int spi3w_en =0b0;               //3-wire SPI Disable

    unsigned int ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | mode;
    unsigned int config_reg    = (t_sb << 5) | (filter << 2) | spi3w_en;
    unsigned int ctrl_hum_reg  = osrs_h;

     int ret;

          ret =setupi2cdev(fd,BME280_ADDRESS);
                if (ret <0 ) { printf("Cannot find %0X\n",BME280_ADDRESS);
                                           exit(-1);}
  // write to configretion reg.

    ret= writei2c8bits(fd,0xf4,ctrl_meas_reg);

    ret= writei2c8bits(fd,0xf5,config_reg);

    ret= writei2c8bits(fd,0xf2,ctrl_hum_reg);

      return(ret);
}


void readTrim(int fd)
{
   char data[32],i,j;

j=0;
  for (i=0x88;i<=0xa1;i++){data[j]=0;
//    data[j] =wiringPiI2CReadReg8(fd,i);
    data[j]= readi2c8bits(fd,i);
     j++;
   }

  for (i=0xE1;i<=0xe7;i++){data[j]=0;

    data[j]= readi2c8bits(fd,i);
   j++;
   }
 
    dig_T1 = (data[1] << 8) | data[0];
    dig_T2 = (data[3] << 8) | data[2];
    dig_T3 = (data[5] << 8) | data[4];

    dig_P1 = (data[7] << 8) | data[6];
    dig_P2 = (data[9] << 8) | data[8];
    dig_P3 = (data[11]<< 8) | data[10];
    dig_P4 = (data[13]<< 8) | data[12];
    dig_P5 = (data[15]<< 8) | data[14];
    dig_P6 = (data[17]<< 8) | data[16];
    dig_P7 = (data[19]<< 8) | data[18];
    dig_P8 = (data[21]<< 8) | data[20];
    dig_P9 = (data[23]<< 8) | data[22];

    dig_H1 = data[25];
    dig_H2 = (data[27]<< 8) | data[26];
    dig_H3 = data[28];
    dig_H4 = (data[29]<< 4) | (data[30] & 0x0F);
    dig_H5 = (data[31] << 4) | ((data[30] >> 4) & 0x0F); 
    dig_H6 = data[32];  
}

void readRawData(int fd)
{
    int i=0,j=0;
    unsigned char data[8];
           for (i=0xf7;i<=0xfe;i++){
                 data[j]= readi2c8bits(fd,i);
  j++;
}

    pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
    hum_raw  = (data[6] << 8) | data[7];
}




BME280_S32_t cali_T(BME280_S32_t adc_T)
{

    BME280_S32_t var1, var2, T;

    var1 = ((((adc_T >> 3) - ((BME280_S32_t)dig_T1 << 1))) * ((BME280_S32_t)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((BME280_S32_t)dig_T1)) * ((adc_T>>4) - ((BME280_S32_t)dig_T1))) >> 12) *
    ((BME280_S32_t)dig_T3)) >> 14;

    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T; 
}

BME280_U32_t cali_P(BME280_S32_t adc_P)
{
    BME280_S64_t var1, var2,p;
    
    var1 = ((BME280_S64_t)t_fine) - 128000;
    var2 = var1 * var1 * (BME280_S64_t)dig_P6;
    var2 = var2 + ((var1*(BME280_S64_t)dig_P5)<<17);
    var2 = var2 + (((BME280_S64_t)dig_P4)<<35);
    var1 = (( var1 * var1* (BME280_S64_t)dig_P3)>>8) + (var1* (BME280_S64_t)dig_P2<<12);
    var1 = (((((BME280_S64_t)1)<<47)+var1))*((BME280_S64_t)dig_P1)>>33;
    if (var1 == 0)
    {
        return 0;
    }    
    p = 1048576-adc_P;
         p=(((p<<31)-var2)*3125)/var1;
             var1=(((BME280_S64_t)dig_P9)*(p>>13)*(p>>13)) >>25;
                                var2=((BME280_S64_t)dig_P8 *p) >> 19;
            p=((p+var1+var2)>>8)+(((BME280_S64_t)dig_P7)<<4);
       return (BME280_U32_t)p;
      }
  
BME280_U32_t cali_H(BME280_S32_t adc_H)
{
    BME280_S32_t v_x1_u32r;
    v_x1_u32r = (t_fine - ((BME280_S32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((BME280_S32_t)dig_H4) << 20)-(((BME280_S32_t)dig_H5) * v_x1_u32r)) + 
        ((BME280_S32_t)16384)) >> 15) * (((((((v_x1_u32r * ((BME280_S32_t)dig_H6)) >> 10) * (((v_x1_u32r *
        ((BME280_S32_t)dig_H3)) >> 11)+((BME280_S32_t)32768))) >> 10) + ((BME280_S32_t)2097152)) * 
        ((BME280_S32_t)dig_H2)+8192) >> 14));


         v_x1_u32r=(v_x1_u32r-(((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7)*((BME280_S32_t)dig_H1)) >> 4));
 
                   v_x1_u32r=(v_x1_u32r < 0 ? 0 : v_x1_u32r);
                              v_x1_u32r =(v_x1_u32r > 419430400 ? 41930400 : v_x1_u32r);

   return (BME280_U32_t)(v_x1_u32r >> 12);   
}



int setup_INA219(int fd)
{
    unsigned short conf;

    unsigned char rst =  0b00;             //Reset
    unsigned char brng = 0b00;             //maximum voltage *16v or 32v
    unsigned char pg =  0b011;             //gain 1/8

    unsigned char badc = 0b00011;          //voltage resolution
    unsigned char sadc = 0b00011;          //current resolution
    unsigned char mode = 0b0111;           //Set  operating mode


     int ret;


    conf= rst << 15 | brng <<13 | pg << 11 | badc << 7 | sadc << 3 | mode;

          ret =setupi2cdev(fd,INA219_ADDRESS);
                    if (ret <0 ) { printf("Cannot find %0X\n",INA219_ADDRESS);
      exit(-1);
}

  // write to configretion reg.
  ret= writei2c16bits(fd,0x00,conf);

      return(ret);
}


void read_INA219_RawData(int fd,short int *current,unsigned short int *voltage)
{
                *current=readi2c16bits(fd,0x01);
               *voltage=readi2c16bits(fd,0x02);

}

void  cnv_INA219(int fd,double *current,double *voltage)
{
  short int shunt=0,volt=0;

while (! (volt & 0x02 )){
                   read_INA219_RawData(fd,&shunt,&volt);
}
  volt = volt >> 3;

  *current =(double) shunt * 10e-6/CURRENT_REGISTANCE;
 *voltage  =(double) volt * 4e-3;

}

  int  setupi2c()
{
      int fd;
      fd=open("/dev/i2c-1",O_RDWR);
       if (fd < 0) {printf("Cannot open i2c-1 device\n");
        return(-1);
      }
      return(fd);
}


    int  setupi2cdev(int fd,int addr)
{      int ret;
      ret=ioctl(fd,I2C_SLAVE,addr);
  return(ret);
}

      int  writei2c(int fd,unsigned char buf[],int databyte)
{
       int ret;
          ret=write(fd,buf,databyte);
     if ( ret != databyte ) {
    printf("write error /n");
       return (-1);
}
     return(ret);
}
 
     int readi2c(int fd,unsigned char buf[],int databyte)
{
     int  ret;
      ret=read (fd,buf,databyte);
         if (ret != databyte){
     printf("read error!/n");
       return(-1);
}

       return(ret);
}

    int writei2c8bits(int fd,unsigned char addr,unsigned char data)
{
   int ret;
    unsigned char  buf[2];
       buf[0]=addr;
       buf[1]=data;
       buf[2]=0;
  ret=writei2c(fd,buf,2);
     return(ret);
}

 unsigned char  readi2c8bits(int fd,unsigned char data)
{
   int ret;
    unsigned char  buf[1];
      buf[0]=data;
     buf[1]=0;
     writei2c(fd,buf,1);
       readi2c(fd,buf,1);
return(buf[0]);
}

 int writei2c16bits(int fd,unsigned char addr,unsigned short int data)
{
   int ret;
    unsigned char  buf[2];
       buf[0]=addr;
         buf[1]=data >>8;
     buf[2]=data & 0x00ff;
   buf[3]=0;
  ret=   writei2c(fd,buf,3);

     return(ret);
}


 unsigned short  readi2c16bits(int fd,unsigned char addr)
{
   int ret;
    unsigned char  buf[1];
      buf[0]=addr;
   buf[1]=0;
     writei2c(fd,buf,1);
       readi2c(fd,buf,2);
return( buf[0] << 8 | buf[1]);
}


// get daytime 

void get_daytime (char Time_string[])
{
    time_t now;
    struct tm *tm;
    now = time(0);
    if ((tm = localtime (&now)) == NULL) {
        printf ("Error extracting time stuff\n");
        return ;
    }

 //   sprintf (Time_string,"%04d-%02d-%02d %02d:%02d:%02d",
 //       tm->tm_year+1900, tm->tm_mon+1, tm->tm_mday,
 //       tm->tm_hour, tm->tm_min, tm->tm_sec);

    sprintf (Time_string,"%02d:%02d",
        tm->tm_hour, tm->tm_min);
    return ;
}

float get_cputemp(void)
{  FILE *fd;
     int temp;
      float temp_act;
     fd=fopen ("/sys/class/thermal/thermal_zone0/temp","r");
       fscanf(fd,"%d",&temp);
          temp_act=(float)temp/1000.0;
    fclose(fd);
  return(temp_act);
}
