#include <string.h>

/* XDCtools Header files */
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>

/* TI-RTOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/GPIO.h>
#include <ti/net/http/httpcli.h>
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <ti/drivers/I2C.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Idle.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/drivers/UART.h>

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include <sys/socket.h>

/* Example/Board Header file */
#include "Board.h"


char rx_buffer[200];
char resp_str[20][100];

#define ESP8266_OK          1
#define ESP8266_READY       2
#define ESP8266_ERROR       3
#define ESP8266_NOCHANGE    4
#define ESP8266_LINKED      5
#define ESP8266_UNLINK      6
#define ESP8266_FAIL        7
#define ESP8266_UNKNOWN     100

#define ESP8266_AP_CONNECT_TIMEOUT              1
#define ESP8266_AP_CONNECT_WRONG_PASSWORD       2
#define ESP8266_AP_CONNECT_AP_NOT_FOUND         3
#define ESP8266_AP_CONNECT_FAIL                 4
#define ESP8266_AP_CONNECT_SUCCESS              5

#define ESP8266_CONNECTION_STATUS_GOT_IP        2
#define ESP8266_CONNECTION_STATUS_CONNECTED     3
#define ESP8266_CONNECTION_STATUS_DISCONNECTED  4
#define ESP8266_CONNECTION_STATUS_WIFI_FAIL     5

#define serverIP "192.168.1.27"



#define HOSTNAME          "api.openweathermap.org"
#define REQUEST_URI       "/data/2.5/weather?q=eskisehir&appid=b9bdaf75a7b1e96362a172ec83cb9303"
#define USER_AGENT        "HTTPCli (ARM; TI-RTOS)"
#define HTTPTASKSTACKSIZE 10000
extern Semaphore_Handle sem0, sem1, sem2;
extern Mailbox_Handle mailbox0, mailbox1, mailbox2;
//extern Event ev1;
Task_Struct task0Struct;
Char task0Stack[HTTPTASKSTACKSIZE];



#define Board_BMP180_ADDR 0x77

I2C_Handle      i2c;
I2C_Params      i2cParams;
I2C_Transaction i2cTransaction;

uint8_t         txBuffer[4];
uint8_t         rxBuffer[30];

short AC1, AC2, AC3, B1, B2, MB, MC, MD;    // calibration variables
unsigned short AC4, AC5, AC6;               // calibration variables
long UT, UP;    //uncompensated temperature and pressure
float B3, B4, B6, B7, X1t, X1p, X2t, X2p, X3p, B5t, B5p;


Void Hwi1(UArg arg1){
    Semaphore_post(sem0);

}
Void httpTask(UArg arg0, UArg arg1)
{
    bool moreFlag = false;
    char data[100];
    int ret;
    int len;
    struct sockaddr_in addr;
    char *s1, *s2,*s3,*s4, temp_str[20],pressure_str[20];

    HTTPCli_Struct cli;
    HTTPCli_Field fields[3] = {
        { HTTPStd_FIELD_NAME_HOST, HOSTNAME },
        { HTTPStd_FIELD_NAME_USER_AGENT, USER_AGENT },
        { NULL, NULL }
    };
    while(1){
        Semaphore_pend(sem0, BIOS_WAIT_FOREVER);


    System_printf("Sending a HTTP GET request to '%s'\n", HOSTNAME);
    System_flush();

    HTTPCli_construct(&cli);

    HTTPCli_setRequestFields(&cli, fields);

    ret = HTTPCli_initSockAddr((struct sockaddr *)&addr, HOSTNAME, 0);
    if (ret < 0) {
        printError("httpTask: address resolution failed", ret);
    }

    ret = HTTPCli_connect(&cli, (struct sockaddr *)&addr, 0, NULL);
    if (ret < 0) {
        printError("httpTask: connect failed", ret);
    }

    ret = HTTPCli_sendRequest(&cli, HTTPStd_GET, REQUEST_URI, false);
    if (ret < 0) {
        printError("httpTask: send failed", ret);
    }

    ret = HTTPCli_getResponseStatus(&cli);
    if (ret != HTTPStd_OK) {
        printError("httpTask: cannot get status", ret);
    }

    System_printf("HTTP Response Status Code: %d\n", ret);

    ret = HTTPCli_getResponseField(&cli, data, sizeof(data), &moreFlag);
    if (ret != HTTPCli_FIELD_ID_END) {
        printError("httpTask: response field processing failed", ret);
    }

    len = 0;
    do {
        ret = HTTPCli_readResponseBody(&cli, data, sizeof(data), &moreFlag);
        if (ret < 0) {
            printError("httpTask: response body processing failed", ret);
        }
        s1 = strstr(data, "temp");
                            if(s1 != NULL) {
                                if(s1[5] == ':') {
                                    s2 = strstr(s1+6, ",");
                                    if(s2 != NULL) {
                                        *s2=0;
                                        strcpy(temp_str, s1+6);

                                        System_printf("temp: %s\n", temp_str);
                                        System_flush();
                                        Mailbox_post(mailbox0, &temp_str, BIOS_NO_WAIT);
                                    }
                                }
                             }

          s3 = strstr(data, "pressure");
                  if(s3 != NULL) {
                     if(s3[9] == ':') {
                         s4 = strstr(s3+10, ",");
                         if(s4 != NULL) {
                             *s4=0;
                              strcpy(pressure_str, s3+10);
                              System_printf("pressure: %s\n", pressure_str);
                              System_flush();
                              Mailbox_post(mailbox1, &pressure_str, BIOS_NO_WAIT);
                          }

                       }
                    }




        len += ret;
    } while (moreFlag);
   //System_printf("Recieved %d bytes of payload\n", len);
     //System_flush();
    Semaphore_post(sem1);
    HTTPCli_disconnect(&cli);
    //Event_post(ev1, Event_Id_01);

        }

    HTTPCli_destruct(&cli);

}

void BMP180_getPressureCalibration(void)
{
    txBuffer[0] = 0xAA;
    i2cTransaction.slaveAddress = Board_BMP180_ADDR;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 22;

    if (I2C_transfer(i2c, &i2cTransaction)) {
           //System_printf("Calibration data acquired\n");

           AC1 = rxBuffer[0]<<8 | rxBuffer[1];
           AC2 = rxBuffer[2]<<8 | rxBuffer[3];
           AC3 = rxBuffer[4]<<8 | rxBuffer[5];
           AC4 = rxBuffer[6]<<8 | rxBuffer[7];
           AC5 = rxBuffer[8]<<8 | rxBuffer[9];
           AC6 = rxBuffer[10]<<8 | rxBuffer[11];
           B1 = rxBuffer[12]<<8 | rxBuffer[13];
           B2 = rxBuffer[14]<<8 | rxBuffer[15];
           MB = rxBuffer[16]<<8 | rxBuffer[17];
           MC = rxBuffer[18]<<8 | rxBuffer[19];
           MD = rxBuffer[20]<<8 | rxBuffer[21];
    }
}

void BMP180_startTemperatureAcquisition(void)
{
    txBuffer[0] = 0xf4;                                 // control register
    txBuffer[1] = 0x2e;                                 // temperature conversion command
    i2cTransaction.slaveAddress = Board_BMP180_ADDR;    // 0x77
    i2cTransaction.writeBuf = txBuffer;                 // transmit buffer
    i2cTransaction.writeCount = 2;                      // two bytes will be sent
    i2cTransaction.readBuf = rxBuffer;                  // receive buffer
    i2cTransaction.readCount = 0;                       // we are expecting 2 bytes

    if (I2C_transfer(i2c, &i2cTransaction)) {
      // System_printf("Temperature acquisition initiated\n");
    }
}

float BMP180_getTemperature(void)
{
    float temp;

    txBuffer[0] = 0xf6;                                 // temperature register
    i2cTransaction.slaveAddress = Board_BMP180_ADDR;    // 0x77
    i2cTransaction.writeBuf = txBuffer;                 // transmit buffer
    i2cTransaction.writeCount = 1;                      // two bytes will be sent
    i2cTransaction.readBuf = rxBuffer;                  // receive buffer
    i2cTransaction.readCount = 2;                       // we are expecting 2 bytes

    if (I2C_transfer(i2c, &i2cTransaction)) {
       //System_printf("Temperature value acquired\n");
    }

    UT = rxBuffer[0]<<8 | rxBuffer[1];  //UT = raw temperature data
    //System_printf("Uncompansated Temperature : %d\n", UT);

    //compute temperature
    X1t = ((UT - AC6) * AC5) >> 15;
    X2t = (MC << 11) / (X1t + MD);
    B5t = X1t + X2t;
    temp = ((B5t + 8) / 16) / 10;

    return temp;
}

void BMP180_startPressureAcquisition(void)
{
    txBuffer[0] = 0xf4;                                 // control register
    txBuffer[1] = 0x34;                                 // pressure conversion command
    i2cTransaction.slaveAddress = Board_BMP180_ADDR;    // 0x77
    i2cTransaction.writeBuf = txBuffer;                 // transmit buffer
    i2cTransaction.writeCount = 2;                      // two bytes will be sent
    i2cTransaction.readBuf = rxBuffer;                  // receive buffer
    i2cTransaction.readCount = 0;                       // we are expecting 2 bytes

    if (I2C_transfer(i2c, &i2cTransaction)) {
       //System_printf("Pressure acquisition initiated\n");
    }
}

float BMP180_getPressure(void)
{
    float pressure;

    txBuffer[0] = 0xf6;                                 // temperature register
    i2cTransaction.slaveAddress = Board_BMP180_ADDR;    // 0x77
    i2cTransaction.writeBuf = txBuffer;                 // transmit buffer
    i2cTransaction.writeCount = 1;                      // two bytes will be sent
    i2cTransaction.readBuf = rxBuffer;                  // receive buffer
    i2cTransaction.readCount = 2;                       // we are expecting 2 bytes

    if (I2C_transfer(i2c, &i2cTransaction)) {
       //System_printf("Pressure value acquired\n");
    }

    UP = rxBuffer[0]<<8 | rxBuffer[1];  //UT = raw pressure data
    //System_printf("Uncompansated Pressure : %d\n", UP);

    //compute pressure
    B6 = B5t - 4000;
    X1p = (B2 * (B6 * B6 / 4096)) / 2048;
    X2p = AC2 * B6 / 2048;
    X3p = X1p = X2p;
    B3 = ((((long)AC1 * 4 + X3p)) + 2) / 4;
    X1p = AC3 * B6 / 8192;
    X2p = (B1 * (B6 * B6 / 4096)) / 65536;
    X3p = ((X1p + X2p) + 2) / 4;
    B4 = AC4 * (unsigned long)(X3p + 32768) / 32768;
    B7 = ((unsigned long)UP - B3) * (50000);
    if (B7 < 0x80000000) {
        pressure = (B7 * 2) / B4;
    }
    else {
        pressure = (B7 / B4) * 2;
    }
    X1p = (pressure / 256) * (pressure / 256);
    X1p = (X1p * 3038) / 65536;
    X2p = (-7357 * pressure) / 65536;
    pressure = pressure + (X1p + X2p + 3791) / 16;
    pressure = pressure / 100.0f ; // Pa to mbar

    return pressure;
}

void initializeI2C()
{
    // Create I2C interface for sensor usage
    //
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_100kHz;  // It can be I2C_400kHz orI2C_100kHz

    // Let's open the I2C interface
    //
    i2c = I2C_open(Board_I2C0, &i2cParams);  // actually I2C7
    if (i2c == NULL) {
        // error initializing IIC
        //
        System_abort("Error Initializing I2C\n");
    }

    //System_printf("I2C Initialized!\n");
}

void closeI2C(void)
{
    // close the interface
    //
    I2C_close(i2c);

    //System_printf("I2C interface closed\n");
}

Void taskFxn(UArg arg0, UArg arg1)
{
    while(1){
    Semaphore_pend(sem1, BIOS_WAIT_FOREVER);
    float temp, press;

    // initialize I2C interface
    //
    initializeI2C();

    // get pressure calibration data
    //
    BMP180_getPressureCalibration();

    // start temperature acquisition
    //
    BMP180_startTemperatureAcquisition();
    System_flush();

    // wait for 5 mseconds for the acquisition
    //
    Task_sleep(5);

    // get the uncompensated temperature value
    //
    temp = BMP180_getTemperature();

    // start pressure acquisition
    //
    BMP180_startPressureAcquisition();
    System_flush();

    // wait for 5 mseconds for the acquisition
    //
    Task_sleep(5);

    // get the uncompensated pressure value
    // The sea level pressure is 101325 pascal
    //
    press = BMP180_getPressure();

    // get the altitude
    //

    // Close I2C connection
    //
    closeI2C();

    Mailbox_post(mailbox2, &press, BIOS_NO_WAIT);
    System_printf("BMP180 Pressure: %d hPa\n   ", (int)press);
    System_flush();
    //Event_post(ev1, Event_Id_00);
    Task_sleep(2000);
    }
}

/*
 *  ======== netIPAddrHook ========
 *  This function is called when IP Addr is added/deleted
 */
void netIPAddrHook(unsigned int IPAddr, unsigned int IfIdx, unsigned int fAdd)
{
    static Task_Handle taskHandle;
    Task_Params taskParams;
    Error_Block eb;

    /* Create a HTTP task when the IP address is added */
    if (fAdd && !taskHandle) {
        Error_init(&eb);

        Task_Params_init(&taskParams);
        taskParams.stackSize = HTTPTASKSTACKSIZE;
        taskParams.priority = 1;
        taskHandle = Task_create((Task_FuncPtr)httpTask, &taskParams, &eb);
        if (taskHandle == NULL) {
            printError("netIPAddrHook: Failed to create HTTP Task\n", -1);
        }
    }
}

void printError(char *errString, int code)
{
    System_printf("Error! code = %d, desc = %s\n", code, errString);
    BIOS_exit(code);
}








int ESP8266_waitForResponse(UART_Handle uart)
{
    char str[300];
    int retval;

    while(1) {
        ESP8266_waitForLine(uart, str, sizeof(str));
        if(strstr(str, "OK")) {
            retval = ESP8266_OK;
            break;
        }
        else if(strstr(str, "ERROR")) {
            retval = ESP8266_ERROR;
            break;
        }
        else if(strstr(str, "FAIL")) {
            retval = ESP8266_FAIL;
            break;
        }
        else if(strstr(str, "ready")) {
            retval = ESP8266_READY;
            break;
        }
    }

    return retval;
}

int ESP8266_waitForLine(UART_Handle uart, char *line_str, int max_line_str)
{

    int ndx=0;

    line_str[0]=0;
    while(1) {
        UART_read(uart, &line_str[ndx], 1);
        if(ndx>=1) {
            if(line_str[ndx]==10 && line_str[ndx-1]==13) {
                line_str[ndx-1]=0;
                break;
            }
        }
        ndx++;
        if(ndx>=max_line_str) {
            line_str[max_line_str-1]=0;
            break;
        }

    }

    return true;
}

void ESP8266_disableEnable(UART_Handle uart)
{
    // disable ESP8266 by making PE0 pin 0
    //
    GPIOPinWrite(GPIO_PORTE_BASE,(GPIO_PIN_0 | GPIO_PIN_1),GPIO_PIN_1);

    // wait for a while allowing ESP822 to restart
    //
    Task_sleep(100);

    // enable ESP822 by setting PE0 1
    //
    GPIOPinWrite(GPIO_PORTE_BASE,(GPIO_PIN_0 | GPIO_PIN_1),GPIO_PIN_0);

   ESP8266_waitForResponse(uart);
}

bool ESP8266_turnOffEcho(UART_Handle uart)
{
    char str[20];
    int how_many_waiting, i, retval;

    // write ATE0 to turn off the echo mode
    //
    UART_write(uart, "ATE0\r\n", 6);

    // wait for a while to let ESP8266 return response
    //
    retval  = ESP8266_waitForResponse(uart);

    return retval;
}

bool ESP8266_reset(UART_Handle uart)
{
    char str[20];

    // write ATE0 to turn off the echo mode
    //
    strcpy(str, "AT+RST\r\n");
    UART_write(uart, str, strlen(str));

    ESP8266_waitForResponse(uart);
    Task_sleep(100);
    ESP8266_waitForResponse(uart);

    return false;
}

int ESP8266_setWifiMode(UART_Handle uart, char mode)
{
    char str[20];
    int retval;

    // write AT+CWMODE=? to change the Wifi mode
    //
    sprintf(str, "AT+CWMODE=%c\r\n", mode);
    UART_write(uart, str, strlen(str));

    // ESP8266_waitResponseNew(uart);
    retval = ESP8266_waitForResponse(uart);

    return retval;
}

int ESP8266_connectWifiAP(UART_Handle uart, char *ssid, char *passwd)
{
    char str[100];
    char *ptr1;
    int err_no, err_code=ESP8266_AP_CONNECT_FAIL;

    sprintf(str, "AT+CWJAP=\"%s\",\"%s\"\r\n", ssid, passwd);
    UART_write(uart, str, strlen(str));

    while(1) {
       ESP8266_waitForLine(uart,str,sizeof(str));
       if(strstr(str, "+CWJAP:")) {
           // error situation
           ptr1=strchr(str,':');
           if(ptr1) {
               err_no=atoi(ptr1+1);
               switch(err_no) {
                   case 1: err_code=ESP8266_AP_CONNECT_TIMEOUT; break;
                   case 2: err_code=ESP8266_AP_CONNECT_WRONG_PASSWORD; break;
                   case 3: err_code=ESP8266_AP_CONNECT_AP_NOT_FOUND; break;
                   case 4: err_code=ESP8266_AP_CONNECT_FAIL; break;
               }
           }
       }
       else if(strstr(str,"OK")) {
           return ESP8266_AP_CONNECT_SUCCESS;
       }
       else if(strstr(str,"FAIL")) {
           return err_code;
       }

    }
    return err_code;
}

bool ESP8266_disconnectWifiAP(UART_Handle uart)
{
    char str[20];
    char *ptr1, *ptr2;
    int ln1,ln2;

    sprintf(str, "AT+CWQAP\r\n");
    UART_write(uart, str, strlen(str));

    if(ESP8266_waitForResponse(uart) == ESP8266_OK) {
        return true;
    }
    return false;
}

int ESP8266_getWifiList(UART_Handle uart, char *AP_list_str, int max_len)
{
    char str[100];
    char *ptr1, *ptr2;
    int ln1,ln2;

    sprintf(str, "AT+CWLAP\r\n");
    UART_write(uart, str, strlen(str));

    AP_list_str[0]=0;
    while(1) {
       ESP8266_waitForLine(uart,str,sizeof(str));
       if(strstr(str, "+CWLAP:")) {
           ptr1=strchr(str,'\"');
           if(ptr1) {
               ptr2=strchr(ptr1+1, '\"');
               if(ptr2) {
                   *ptr2=';';
                   *(ptr2+1)=0;
                   ln1=strlen(ptr1+1);
                   ln2=strlen(AP_list_str);

                   if((ln1+ln2)<max_len-2) {
                       strcat(AP_list_str, ptr1+1);
                   }
               }
           }
       }
       else if(strstr(str,"OK")) {
           return ESP8266_OK;
       }
       else if(strstr(str,"ERROR")) {
           return ESP8266_ERROR;
       }
       else if(strstr(str,"FAIL")) {
           return ESP8266_FAIL;
       }
    }

    return ESP8266_UNKNOWN;
}

int ESP8266_checkNetworkStatus(UART_Handle uart)
{
    char str[100];
    char *ptr1;
    int retval;

    sprintf(str, "AT+CIPSTATUS\r\n");
    UART_write(uart, str, strlen(str));

    while(1) {
        ESP8266_waitForLine(uart,str,sizeof(str));
        if(ptr1=strstr(str,"STATUS:")) {
            retval=atoi(ptr1+7);
        }
        else if(!strcmp(str,"OK") || !strcmp(str,"ERROR") || !strcmp(str,"FAIL")) {
            break;
        }
    }
    return retval;
}

int ESP8266_getIP(UART_Handle uart, char *ip_str)
{
    char str[100], str2[100];
    char *ptr1, *ptr2, *ptr3;
    int retval;

    sprintf(str, "AT+CIFSR\r\n");
    UART_write(uart, str, strlen(str));

    ip_str[0]=0;
    while(1) {
        ESP8266_waitForLine(uart,str,sizeof(str));
        if(ptr1=strstr(str,"+CIFSR:STAIP")) {
            ptr2=strchr(ptr1+7, '\"');
            ptr3=strchr(ptr2+1, '\"');
            *ptr3=0;
            strcpy(ip_str, ptr2+1);
            return ESP8266_OK;
        }
        else if(!strcmp(str,"OK")) {
            return ESP8266_OK;
        }
        else if(!strcmp(str,"ERROR"))  {
            return ESP8266_ERROR;
        }
        else if(!strcmp(str,"FAIL")) {
            return ESP8266_FAIL;
         }
    }
    return ESP8266_UNKNOWN;
}

int ESP8266_setDNS(UART_Handle uart, const char *ip_str)
{
    char str[100], str2[100];
    char *ptr1, *ptr2, *ptr3;
    int retval;

    sprintf(str, "AT+CIPDOMAIN=\"%s\"\r\n", ip_str);
    UART_write(uart, str, strlen(str));

    while(1) {
        ESP8266_waitForLine(uart,str,sizeof(str));
        if(!strcmp(str,"OK")) {
            return ESP8266_OK;
        }
    }
    return ESP8266_ERROR;
}

Void ESP8266_TCPConnect(UART_Handle uart, const char *ip_str, int port)
{
    char str[100], str2[100];
    char *ptr1, *ptr2, *ptr3;
    int retval;

    sprintf(str, "AT+CIPSTART=\"TCP\",\"%s\",%d\r\n", ip_str, port);
    UART_write(uart, str, strlen(str));

    while(1) {
        ESP8266_waitForLine(uart,str,sizeof(str));
        if(!strcmp(str,"OK")) {
            return ESP8266_OK;
        }
        else if(!strcmp(str,"ERROR")) {
            return ESP8266_OK;
        }
    }
    return ESP8266_ERROR;
}

Void ESP8266_TCPGetLine(UART_Handle uart, const char *ip_str, int port)
{
    char str[100], str2[100];
    char *ptr1, *ptr2, *ptr3;
    int retval,len;
    char temp_str[20],pressure_str[20];
    float press;
    char press1[20];
    //char addStr[5] = "+IPD,";

    while(1) {



        Mailbox_pend(mailbox0, &temp_str, BIOS_WAIT_FOREVER);
              Mailbox_pend(mailbox1, &pressure_str, BIOS_WAIT_FOREVER);
              Mailbox_pend(mailbox2, &press, BIOS_WAIT_FOREVER);
              sprintf(press1,"%f", press);
              System_printf("Incoming data: %s & %s & %s" , temp_str, pressure_str, press1);
              System_flush();

              //strncat(addStr,temp_str, 5);
              //System_printf("data %s" , addStr);
              //System_flush();
              //strcpy(line_str, addStr);


        //Event_pend(ev1, Event_Id_00 + Event_Id_01,Event_Id_NONE, BIOS_WAIT_FOREVER);



        ESP8266_waitForLine(uart,str,sizeof(str));
        if(!strncmp(str,"+IPD,",5)) {
            ptr1=&str[5];
            ptr2=strchr(ptr1+1, ':');
            strncpy(str2, ptr1, ptr2-ptr1);
            str2[ptr2-ptr1]=0;
            len=atoi(str2);


            // EKSIK KISIMLAR  tamamlanacak
            ///
        }
    }
    return ESP8266_ERROR;
}

Void ESP8266_task(UArg arg0, UArg arg1)
{
    UART_Handle uart;
    UART_Params uartParams;
    char ap_list[200], str[100];
    int res;

    UART_Params_init(&uartParams);
    uartParams.writeDataMode =  UART_DATA_BINARY;
    uartParams.readDataMode =  UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_NEWLINE;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.dataLength = UART_LEN_8;
    uartParams.parityType = UART_PAR_NONE;
    uartParams.stopBits = UART_STOP_ONE;
    uartParams.baudRate = 115200;
    uart = UART_open(Board_UART7, &uartParams);

    if (uart == NULL) {
        // something terribly went wrong. abort!!!
        //
        System_abort("Error opening the UART");
    }

    ESP8266_disableEnable(uart);
    ESP8266_turnOffEcho(uart);
    ESP8266_setWifiMode(uart, '3');
    if(ESP8266_getWifiList(uart,ap_list, sizeof(ap_list))==ESP8266_OK) {
        System_printf(ap_list);
        System_printf("\n");
        System_flush();
    }

    res=ESP8266_connectWifiAP(uart, "SUPERONLINE-WiFi_1139", "LA3VNUPJNETM");
    System_printf("Ret Code: %d\n", res);
    System_flush();

    res=ESP8266_checkNetworkStatus(uart);
    System_printf("status Code: %d\n", res);
    System_flush();

    ESP8266_getIP(uart, str);
    System_printf("IP: %s\n", str);
    System_flush();

    ESP8266_TCPGetLine(uart, str, 5000);



    if(ESP8266_setDNS(uart,"8.8.8.8")==ESP8266_OK) {  // set Google DNS
        System_printf("Set DNS OK\n");
        System_flush();
    }


}
/*void sendData2Server(char *serverIP, int serverPort, char *data, int size)
{
    int sockfd;
    struct sockaddr_in serverAddr;
    sockfd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sockfd == -1) {
        System_printf("Socket not created");
        BIOS_exit(-1);
    }
    memset(&serverAddr, 0, sizeof(serverAddr)); /* clear serverAddr structure */
    /*serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(serverPort); /* convert port # to network order */
    /*inet_pton(AF_INET, serverIP, &(serverAddr.sin_addr));
    int connStat = connect(sockfd,(struct sockaddr *)&serverAddr,sizeof(serverAddr));
    if(connStat < 0) {
        System_printf("Error while connecting to server\n");
        if (sockfd > 0)
            close(sockfd);
        BIOS_exit(-1);
    }
    int numSend = send(sockfd, data, size, 0); /* send data to the server*/
    /*if(numSend < 0) {
        System_printf("Error while sending data to server\n");
        if (sockfd > 0) close(sockfd);
        BIOS_exit(-1);
    }
    if (sockfd > 0) close(sockfd);
    }*/

/*Void socketTask(UArg arg0, UArg arg1)
{
    while(1) {
            // wait for the semaphore that httpTask() will signal
            // when temperature string is retrieved from api.openweathermap.org site
            //
            Semaphore_pend(semaphore0, BIOS_WAIT_FOREVER);
            GPIO_write(Board_LED0, 1); // turn on the LED
            // connect to SocketTest program on the system with given IP/port
            // send hello message whihc has a length of 5.
            //
            sendData2Server("192.168.1.25", 5011, tempstr, strlen(tempstr));
            GPIO_write(Board_LED0, 0); // turn off the LED
            // wait for 5 seconds (5000 ms)
            //
            Task_sleep(5000);
    }
}

/*Void clientSocketTask(UArg arg0, UArg arg1)
{
    while(1) {
        // wait for the semaphore that httpTask() will signal
        // when temperature string is retrieved from api.openweathermap.org site
        //
        Semaphore_pend(semaphore0, BIOS_WAIT_FOREVER);

        GPIO_write(Board_LED0, 1); // turn on the LED

        // connect to SocketTest program on the system with given IP/port
        // send hello message whihc has a length of 5.
        //
        if(sendData2Server(SOCKETTEST_IP, OUTGOING_PORT, tempstr, strlen(tempstr))) {
            System_printf("clientSocketTask:: Temperature is sent to the server\n");
            System_flush();
        }

        GPIO_write(Board_LED0, 0);  // turn off the LED
    }
}*/


/*
 *  ======== main ========
 */
int main(void)
{
    /* Call board init functions */
    Board_initGeneral();
    Board_initGPIO();
    Board_initEMAC();
    Board_initI2C();
    Board_initUART();

    Task_Params taskParams;

     Task_Params_init(&taskParams);
     taskParams.stackSize = HTTPTASKSTACKSIZE;
     taskParams.stack = &task0Stack;
     taskParams.instance->name = "esp8266";
     Task_construct(&task0Struct, (Task_FuncPtr)ESP8266_task, &taskParams, NULL);

    /* Turn on user LED */
    GPIO_write(Board_LED0, Board_LED_ON);

    System_printf("Starting the HTTP GET example\nSystem provider is set to "
            "SysMin. Halt the target to view any SysMin contents in ROV.\n");
    /* SysMin will only print to the console when you call flush or exit */
    System_flush();

    /* Start BIOS */
    BIOS_start();

    return (0);
}
