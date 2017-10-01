
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/i2c-dev.h>
#include <wiringPi.h>



#define ADXL345_ADDRESS  0x53


// prottype
int init_adxl(int fd);

int setupi2c();
int setupi2cdev(int fd,int dev);
int writei2c(int fd,unsigned char buf[],int databyte);
int readi2c(int fd,unsigned char buf[],int databyte);
int writei2c8bits(int fd,unsigned char addr,unsigned char data);
int writei2c16bits(int fd,unsigned char addr,unsigned int data);
unsigned char readi2c8bits(int fd,unsigned char data);
short readi2c16bits(int fd,unsigned char data);
//get daytime
void get_daytime (char Time_string[]);

// get cputemp
float get_cputemp(void);

// measure routine
void start_measure(int fd);
void stop_measure(int fd);
void measure_accl(short *x,short *y,short *z,int fd);


int init_adxl(int fd)
{
 int ret;
unsigned char bw_rate     =0b000001011; // time step 0.005sec
unsigned char init_enable =0b010000000; // data ready
unsigned char init_map    =0b000000000; // enable int1
unsigned char data_format =0b000001000; // dataformat full res.
unsigned char fifo_ctl    =0b001011111; // fifo ctrl samples:32  
unsigned char power_ctl   =0b000000000; // no power ctrl 

    //init Hardware settings
    wiringPiSetup();
    pinMode(0,INPUT);

// I2C device initilize

 fd=setupi2c();
 ret=setupi2cdev(fd,ADXL345_ADDRESS);
  if (ret <0 ) {
printf("Cannot find %0X\n",ADXL345_ADDRESS);
    exit(-1);
}

ret= writei2c8bits(fd,0x2C,bw_rate);
ret= writei2c8bits(fd,0x2E,init_enable);
ret= writei2c8bits(fd,0x2F,init_map);
ret= writei2c8bits(fd,0x31,data_format);
ret= writei2c8bits(fd,0x38,fifo_ctl);
ret= writei2c8bits(fd,0x2D,power_ctl);

  return(fd);

}


void start_measure(int fd)
{
unsigned char power_ctl;

int ret;
 power_ctl=readi2c8bits(fd,0x2D);

power_ctl =power_ctl | 0x08;

ret= writei2c8bits(fd,0x2D,power_ctl);

}


void stop_measure(int fd)
{
unsigned char power_ctl;

int ret;
 power_ctl=readi2c8bits(fd,0x2D);

power_ctl =power_ctl | 0xF7;

ret= writei2c8bits(fd,0x2D,power_ctl);

}




void measure_accl(short *x,short *y,short *z,int fd)
{

*x=readi2c16bits(fd,0x32);

*y=readi2c16bits(fd,0x34);

*z=readi2c16bits(fd,0x36);

}



// SYSTEM routine

int  setupi2c()
{
      int fd;
      fd=open("/dev/i2c-1",O_RDWR);
       if (fd < 0) {printf("Cannot open i2c-1 device\n");
        exit(-1);
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


   int writei2c16bits(int fd,unsigned char addr,unsigned int data)
  {
int ret;
      unsigned char buf[3];
        buf[0]=addr;
        buf[1]=(unsigned char) (data & 0xff);
        buf[2]=(unsigned char) (data & 0xff00) >> 8;
        buf[3]=0;
      ret=writei2c(fd,buf,3);
        return(ret);
}



 unsigned char readi2c8bits(int fd,unsigned char data)
{
   int ret;
    unsigned char  buf[1];
      buf[0]=data;
     buf[1]=0;
     writei2c(fd,buf,1);
       readi2c(fd,buf,1);
return(buf[0]);
}

short  readi2c16bits(int fd,unsigned char data)
{
     int ret;
      unsigned char buf[2];
      buf[0]=data;
      writei2c(fd,buf,1);
         readi2c(fd,buf,2);
    return( (short) ( buf[1]<<8) | buf[0] );
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

    sprintf (Time_string,"%04d-%02d-%02d-%02d-%02d-%02d",
        tm->tm_year+1900, tm->tm_mon+1, tm->tm_mday,
        tm->tm_hour, tm->tm_min, tm->tm_sec);

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

