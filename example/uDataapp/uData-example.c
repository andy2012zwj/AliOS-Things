/*
 * Copyright (c) 2014-2016 Alibaba Group. All rights reserved.
 * License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include <aos/aos.h>
#include <aos/yloop.h>
#include "netmgr.h"
#include "aos/uData.h"

#ifdef DATA_TO_CLOUD

#include "iot_import.h"
#include "iot_export.h"
#include "iot_export_mqtt.h"
#ifdef AOS_ATCMD
#include <atparser.h>
#endif
#ifdef CSP_LINUXHOST
#include <signal.h>
#endif

#endif/*end DATA_TO_CLOUD*/


#include "stdio.h"
#include "string.h"
#include "math.h"

#define MATRIX_SIZE 7
  

#define UDATA_PRINT    printf

#define UDATA_SHOW_UINT_1(TYPE,TIME,DATA1) \
do{\
    UDATA_PRINT(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n"); \
    UDATA_PRINT("uData_application::::::::::::::type = (%d)\n", (TYPE)); \
    UDATA_PRINT("uData_application::::::::::::::data = (%d)\n", (DATA1)); \
    UDATA_PRINT("uData_application:::::::::timestamp = (%d)\n", (uint32_t)(TIME)); \
	UDATA_PRINT("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n"); \
}while(0);

#define UDATA_SHOW_UINT_3(TYPE,TIME,DATA1,DATA2,DATA3) \
do{\
    UDATA_PRINT(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n"); \
    UDATA_PRINT("uData_application::::::::::::::type = (%d)\n", (TYPE)); \
    UDATA_PRINT("uData_application::::::::::::::data = (%d) (%d) (%d)\n", (DATA1),(DATA2),(DATA3)); \
    UDATA_PRINT("uData_application:::::::::timestamp = (%d)\n", (uint32_t)(TIME)); \
	UDATA_PRINT("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n"); \
}while(0);

#define UDATA_SHOW_FLOAT_3(TYPE,TIME,DATA1,DATA2,DATA3) \
do{\
    UDATA_PRINT(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n"); \
    UDATA_PRINT("uData_application::::::::::::::type = (%d)\n", (TYPE)); \
    UDATA_PRINT("uData_application::::::::::::::data = (%f) (%f) (%f)\n", (DATA1),(DATA2),(DATA3)); \
    UDATA_PRINT("uData_application:::::::::timestamp = (%d)\n", (uint32_t)(TIME)); \
	UDATA_PRINT("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n"); \
}while(0);


#define  GYTO_FILTER_ZERO_DATA(idx,data)   \
do{ \
    if ((data < 2000000) && (data > -2000000)){ \
        data = 0; \
    } \
}while(0);


typedef struct _sensor_clib_para{
    double gain_x;
    double gain_y;
    double gain_z;
    double offset_x;
    double offset_y;
    double offset_z;
}sensor_clib_para;

extern double m_result[MATRIX_SIZE];  

int test_start_gyro = 0;
int g_acc_clib_flag = 0;
int cali_test_sum = 0;

int m = MATRIX_SIZE;        
int n = MATRIX_SIZE+1;

double acc_test_value[3] = {0.0};
double m_matrix[MATRIX_SIZE][MATRIX_SIZE+1];
double m_result[MATRIX_SIZE];   

sensor_clib_para g_sensor_clib_para;


uint32_t  udata_sub_type = UDATA_SERVICE_MAG;

void Calc_Process(double radius);
void DispMatrix(void);

#define ACC_CLIB_PARA_NAME  "acc_para"
extern int aos_kv_set(const char *key, const void *val, int len, int sync);
extern int aos_kv_get(const char *key, void *buffer, int *buffer_len);




#ifdef DATA_TO_CLOUD
// comments
static int linkkit_started = 0;
static int awss_running = 0;
extern int linkkit_app(void);
void reboot_system(void *parms);

int awss_success_notify();
static void wifi_service_event(input_event_t *event, void *priv_data)
{
    if (event->type != EV_WIFI) {
        return;
    }

    if (event->code != CODE_WIFI_ON_GOT_IP) {
        return;
    }

    netmgr_ap_config_t config;
    memset(&config, 0, sizeof(netmgr_ap_config_t));
    netmgr_get_ap_config(&config);
    LOG("wifi_service_event config.ssid %s", config.ssid);
    if (strcmp(config.ssid, "adha") == 0 || strcmp(config.ssid, "aha") == 0) {
        //clear_wifi_ssid();
        return;
    }

    if (awss_running) {
#ifdef AWSS_NEED_REBOOT
        aos_post_delayed_action(200, reboot_system, NULL);
#endif
        return;
    }
    if (!linkkit_started) {
        linkkit_app();
        awss_success_notify();
        linkkit_started = 1;
    }
}

void reboot_system(void *parms)
{
    LOG("reboot system");
    aos_reboot();
}

static void cloud_service_event(input_event_t *event, void *priv_data)
{
    static uint8_t awss_reported = 0;
    if (event->type != EV_YUNIO) {
        return;
    }

    LOG("cloud_service_event %d", event->code);

    if (event->code == CODE_YUNIO_ON_CONNECTED) {
        LOG("user sub and pub here");
        if (!awss_reported) {
            awss_report_cloud();
            awss_reported = 1;
        }
        return;
    }

    if (event->code == CODE_YUNIO_ON_DISCONNECTED) {
    }
}

static void start_netmgr(void *p)
{
    netmgr_start(true);
    //aos_task_exit(0);
}

extern int awss_report_reset();

void do_awss_active()
{
    LOG("do_awss_active %d\n", awss_running);
    awss_running = 1;
    awss_config_press();
}

static void do_awss_reset()
{
    if (linkkit_started) {
        aos_task_new("reset", awss_report_reset, NULL, 2048);
    }
    netmgr_clear_ap_config();
    LOG("SSID cleared. Please reboot the system.\n");
    aos_post_delayed_action(1000, reboot_system, NULL);
}

void linkkit_key_process(input_event_t *eventinfo, void *priv_data)
{
    if (eventinfo->type != EV_KEY) {
        return;
    }
    LOG("awss config press %d\n", eventinfo->value);

    if (eventinfo->code == CODE_BOOT) {
        if (eventinfo->value == VALUE_KEY_CLICK) {
            do_awss_active();
        } else if (eventinfo->value == VALUE_KEY_LTCLICK) {
            do_awss_reset();
        }
    }
}

#ifdef CONFIG_AOS_CLI
static void handle_reset_cmd(char *pwbuf, int blen, int argc, char **argv)
{
    aos_schedule_call(do_awss_reset, NULL);
}

static void handle_active_cmd(char *pwbuf, int blen, int argc, char **argv)
{
    aos_schedule_call(do_awss_active, NULL);
}

static struct cli_command resetcmd = {
    .name = "reset",
    .help = "factory reset",
    .function = handle_reset_cmd
};

static struct cli_command ncmd = {
    .name = "active_awss",
    .help = "active_awss [start]",
    .function = handle_active_cmd
};
#endif
#endif


#define TEST_GYRO
#ifdef TEST_GYRO

#define  GYRO_COUNT_THRESHOLD        (2000)
#define  GYRO_COUNT_NUM              (2000)

#define  GYRO_CALC_SCALE             (1000000)

#define  GYRO_CALC_IDLE              (0)
#define  GYRO_CALC_START             (1)
#define  GYRO_CALC_PROC              (2)
#define  GYRO_CALC_SHOW              (3)
#define  GYRO_CALC_STOP              (4)



int64_t   gyro_offset[3] = {0,0,0};
uint32_t  gyro_cnt = 0;
uint32_t  gyro_threshold = 0;

void gyro_filter(uint32_t time,int data0, int data1, int data2)
{
    gyro_threshold++;

    if ((gyro_threshold < GYRO_COUNT_THRESHOLD) || (gyro_threshold > (GYRO_COUNT_THRESHOLD + GYRO_COUNT_NUM))){
        return;
    }

    if (gyro_threshold == (GYRO_COUNT_THRESHOLD + GYRO_COUNT_NUM)){
        gyro_offset[0] = gyro_offset[0] / gyro_cnt;
        gyro_offset[1] = gyro_offset[1] / gyro_cnt;
        gyro_offset[2] = gyro_offset[2] / gyro_cnt;
        printf("  $$$$ %d %d %d \n",(int32_t)gyro_offset[0] ,(int32_t)gyro_offset[1] ,(int32_t)gyro_offset[2] );
        return;
    }

    gyro_offset[0] += (int64_t)data0;
    gyro_offset[1] += (int64_t)data1;
    gyro_offset[2] += (int64_t)data2;

    gyro_cnt ++;

}
    

int64_t   gyro_angle[3] = {0,0,0};
uint32_t  gyro_time = 0;
uint32_t  gyro_flag = GYRO_CALC_IDLE;
int64_t   g_angle_save = 360000;



void gyro_data_show()
{
    int64_t angle = 0;
    angle = gyro_angle[0]*gyro_angle[0] + gyro_angle[1]*gyro_angle[1] + gyro_angle[2]*gyro_angle[2] ;
    angle = sqrt(angle);
    printf("gyro_angle = %d  %d  %d\n",(int32_t)gyro_angle[0] ,(int32_t)gyro_angle[1] ,(int32_t)gyro_angle[2] );
    g_angle_save = angle;
    UDATA_SHOW_UINT_3(UDATA_SERVICE_GYRO,0,(uint32_t)angle,0,0);
}

void gyro_data_read()
{
    UDATA_SHOW_UINT_3(UDATA_SERVICE_GYRO,0,(uint32_t)g_angle_save,0,0);
}

void gyro_calc_init()
{
    gyro_angle[0] = 0;
    gyro_angle[1] = 0;
    gyro_angle[2] = 0;

    gyro_time = 0;
    gyro_flag = GYRO_CALC_IDLE;
}

void gyro_calc(uint32_t time,int data0, int data1, int data2)
{
    if(GYRO_CALC_IDLE == gyro_flag){
        return;
    }
    else if(GYRO_CALC_STOP == gyro_flag){
        gyro_flag = GYRO_CALC_IDLE;
        gyro_data_show();
        return;
    }
    else if(GYRO_CALC_SHOW == gyro_flag){
        gyro_flag = GYRO_CALC_PROC;
        //gyro_data_show();
    }
    else if(GYRO_CALC_START == gyro_flag){
        gyro_time = time;
        gyro_flag = GYRO_CALC_PROC;
        //gyro_data_show();
        return;
    }

    gyro_angle[0] += (int64_t)(((int64_t)data0 - gyro_offset[0]) * (time - gyro_time)) / GYRO_CALC_SCALE;
    gyro_angle[1] += (int64_t)(((int64_t)data1 - gyro_offset[1]) * (time - gyro_time)) / GYRO_CALC_SCALE;
    gyro_angle[2] += (int64_t)(((int64_t)data2 - gyro_offset[2]) * (time - gyro_time)) / GYRO_CALC_SCALE;

    gyro_time = time;
}


#endif



void uData_report_demo(input_event_t *event, void *priv_data)
{
    udata_pkg_t buf;
    if ((event == NULL) || (event->type != EV_UDATA)) {
        return;
    }

    if (event->code == CODE_UDATA_REPORT_PUBLISH) {
        int ret = 0;
        ret = uData_report_publish(event, &buf);
        if (ret != 0) {
            return;
        }
        switch (buf.type) {

            case UDATA_SERVICE_ACC: {
                accel_data_t *acc = (accel_data_t *)buf.payload;

                if(0 == g_acc_clib_flag){
                    
                    //UDATA_SHOW_UINT_3(buf.type, (uint32_t)acc->timestamp, acc->data[0], acc->data[1], acc->data[2]);
                    CalcData_Input((double)acc->data[0],(double)acc->data[1],(double)acc->data[2]);
                    printf("cali_test_sum == %d\n",cali_test_sum);

                    if(6 == cali_test_sum){
                        printf("caliCalc_Process_test_sum start == %d\n",cali_test_sum);
                        Calc_Process(1000.0);
                        g_acc_clib_flag = 1;
                        printf("caliCalc_Process_test_sum end == %d\n",cali_test_sum);
                    }
                }
                else{

                    for(int i = 0; i < 3; i++){
                        acc_test_value[i] = (double)acc->data[i];
                    }
                    
                    acc->data[0] = (int32_t)((acc_test_value[0] + g_sensor_clib_para.offset_x) * g_sensor_clib_para.gain_x);
                    acc->data[1] = (int32_t)((acc_test_value[1] + g_sensor_clib_para.offset_y) * g_sensor_clib_para.gain_y);
                    acc->data[2] = (int32_t)((acc_test_value[2] + g_sensor_clib_para.offset_z) * g_sensor_clib_para.gain_z);
                                
                    UDATA_SHOW_UINT_3(buf.type, (uint32_t)acc->timestamp, acc->data[0], acc->data[1], acc->data[2]);
                    acc->data[0] = acc->data[0]*acc->data[0] + acc->data[1]*acc->data[1] + acc->data[2]*acc->data[2];
                    printf("output  == %f\n",sqrt((double)(acc->data[0])));
                }

                break;
            }


            case UDATA_SERVICE_MAG: {
                mag_data_t *mag = (mag_data_t *)buf.payload;
                int mag_value[3] = {0};
                double value = 0.0;
                
                mag_value[0] = mag->data[0];
                mag_value[1] = mag->data[1];
                mag_value[2] = mag->data[2];
                mag_value[0] = (mag_value[0] * mag_value[0]) + (mag_value[1]*mag_value[1]) + (mag_value[2]*mag_value[2]);
                UDATA_SHOW_UINT_3(buf.type, (uint32_t)mag->timestamp, mag->data[0], mag->data[1], mag->data[2]);

                printf("total mag = %f\n",sqrt((double)mag_value[0]));
                break;
            }

            case UDATA_SERVICE_GYRO: {
                gyro_data_t *gyro = (gyro_data_t *)buf.payload;
                //

                GYTO_FILTER_ZERO_DATA(1,gyro->data[0]);
                GYTO_FILTER_ZERO_DATA(2,gyro->data[1]);
                GYTO_FILTER_ZERO_DATA(3,gyro->data[2]);
                //UDATA_SHOW_UINT_3(buf.type, (uint32_t)gyro->timestamp, gyro->data[0], gyro->data[1], gyro->data[2]);
                gyro_filter((uint32_t)gyro->timestamp, gyro->data[0], gyro->data[1], gyro->data[2]);

                gyro_calc((uint32_t)gyro->timestamp, gyro->data[0], gyro->data[1], gyro->data[2]);
#if 0
                if(test_start_gyro == 1){
                    gyro_calc((uint32_t)gyro->timestamp, gyro->data[0], gyro->data[1], gyro->data[2]);
                }
                else if(test_start_gyro == 2){
                    test_start_gyro = 0;
                    gyro_data_show();
                }
#endif

                break;
            }

            case UDATA_SERVICE_ALS: {
                als_data_t *als = (als_data_t *)buf.payload;
                UDATA_SHOW_UINT_1(buf.type, als->timestamp, als->lux);
                break;
            }

            case UDATA_SERVICE_PS: {
                proximity_data_t *ps = (proximity_data_t *)buf.payload;
                UDATA_SHOW_UINT_1(buf.type, ps->timestamp, ps->present);
                break;
            }

            case UDATA_SERVICE_BARO: {
                barometer_data_t *baro = (barometer_data_t *)buf.payload;
                UDATA_SHOW_UINT_1(buf.type, baro->timestamp, baro->p);
                break;
            }

            case UDATA_SERVICE_TEMP: {
                temperature_data_t *temp = (temperature_data_t *)buf.payload;
                UDATA_SHOW_UINT_1(buf.type, temp->timestamp, temp->t);
                break;
            }

            case UDATA_SERVICE_UV: {
                uv_data_t *uv = (uv_data_t *)buf.payload;
                UDATA_SHOW_UINT_1(buf.type, uv->timestamp, uv->uvi);
                break;
            }

            case UDATA_SERVICE_HUMI: {
                humidity_data_t *humi = (humidity_data_t *)buf.payload;
                UDATA_SHOW_UINT_1(buf.type, humi->timestamp, humi->h);
                break;
            }

            case UDATA_SERVICE_HALL: {
                hall_data_t *hall = (hall_data_t *)buf.payload;
                UDATA_SHOW_UINT_1(buf.type, hall->timestamp, hall->hall_level);
                break;
            }
            case UDATA_SERVICE_HR: {
                heart_rate_data_t *heart = (heart_rate_data_t *)buf.payload;
                UDATA_SHOW_UINT_1(buf.type, heart->timestamp, heart->hear_rate);
                break;
            }
            case UDATA_SERVICE_FORCE: {
                force_data_t *force = (force_data_t *)buf.payload;
                UDATA_SHOW_UINT_1(buf.type, force->timestamp, force->f);
                break;
            }

            case UDATA_SERVICE_GPS: {
                gps_data_t *gps = (gps_data_t *)buf.payload;
                UDATA_SHOW_FLOAT_3(buf.type, (uint32_t)gps->timestamp, gps->lat, gps->lon, gps->elv);
                break;
            }


            default:
                break;

        }

    }
}


int sensor_self_test(void)
{
    int ret = 0;
    dev_sensor_full_info_t test;

    (void)abs_data_dev_disable(TAG_DEV_ACC);
    memset(&test,0,sizeof(test));

    test.config.id =   SENSOR_IOCTL_SELF_TEST;
    ret = abs_data_ioctl(TAG_DEV_ACC,&test);
    if((ret < 0) || (0 == test.info.data[0])){
        printf("self test fail\n");
        ret = -1;
    }
    else{
        printf("self test success\n");
        ret = 0;
    }
    printf("abs_data_ioctl ret ============%d  %d %d %d\n",ret,test.info.data[0],test.info.data[1],test.info.data[2]);

    (void)abs_data_dev_enable(TAG_DEV_ACC);

    return ret;

}



int udata_sample(void)
{
    int ret = 0;

    aos_register_event_filter(EV_UDATA, uData_report_demo, NULL);

    ret = uData_subscribe(udata_sub_type);
    if (ret != 0) {
        LOG("%s %s %s %d\n", uDATA_STR, __func__, ERROR_LINE, __LINE__);
        return -1;
    }

    return 0;
}


#if (defined (CONFIG_AOS_CLI))

extern void  abs_sensor_read(sensor_tag_e tag);

static void handle_get_sensor_data_cmd(char *pwbuf, int blen, int argc, char **argv)
{
    //int tag = argc > 1 ? ((*(argv[1])) - 48): TAG_DEV_SENSOR_NUM_MAX;
	uint32_t tag = udata_sub_type;
    if(tag >= TAG_DEV_SENSOR_NUM_MAX){
       printf("tag = %d fail\n",tag);
       return;
    }

    if(tag == TAG_DEV_GYRO){
        gyro_data_read();
        return;
    }
    test_start_gyro++;

    abs_sensor_read(tag);
}
static struct cli_command sensorcmd = {
    .name = "sensor",
    .help = "get sensor data",
    .function = handle_get_sensor_data_cmd
};
void handle_sensor_clib_cmd(char *pwbuf, int blen, int argc, char **argv)
{
    int ret;
    int len = 0;
    char* cmd = argc > 1 ? argv[1] : "";
    
    if(udata_sub_type == UDATA_SERVICE_ACC){
        if (strcmp(cmd, "sample") == 0) {
            cali_test_sum++;
            abs_sensor_read(TAG_DEV_ACC);
        }
        else if (strcmp(cmd, "reset") == 0){
            ResetMatrix();
        }
        else if (strcmp(cmd, "start") == 0){
            len = sizeof(g_sensor_clib_para);
            ret = aos_kv_get(ACC_CLIB_PARA_NAME, &g_sensor_clib_para, &len);
            printf("kv get %s ret %d %d\n",ACC_CLIB_PARA_NAME,ret, len);
            g_acc_clib_flag = 1;
        }
    }
    
}
static struct cli_command clibcmd = {
    .name = "clib",
    .help = "clib cmd",
    .function = handle_sensor_clib_cmd
};
void handle_gyro_test_cmd(char *pwbuf, int blen, int argc, char **argv)
{
    char* cmd = argc > 1 ? argv[1] : "";
    if (strcmp(cmd, "clear") == 0) {
        gyro_calc_init();
    } 
    else if (strcmp(cmd, "start") == 0) {
        gyro_calc_init();
        gyro_flag = GYRO_CALC_START;
    }
    else if (strcmp(cmd, "stop") == 0) {
        gyro_flag = GYRO_CALC_STOP;
    }
    else if (strcmp(cmd, "show") == 0) {
        gyro_flag = GYRO_CALC_SHOW;
    }
    else {
        printf("bad cmd 'gyro %s'\n",cmd);
    }
}
static struct cli_command gyrocmd = {
    .name = "gyro",
    .help = "gyro test",
    .function = handle_gyro_test_cmd
};
#endif
int application_start(int argc, char **argv)
{
#ifdef DATA_TO_CLOUD
#ifdef CSP_LINUXHOST
    signal(SIGPIPE, SIG_IGN);
#endif
#if AOS_ATCMD
    at.set_mode(ASYN);
    at.init(AT_RECV_PREFIX, AT_RECV_SUCCESS_POSTFIX,
            AT_RECV_FAIL_POSTFIX, AT_SEND_DELIMITER, 1000);
#endif


#ifdef WITH_SAL
    sal_init();
#endif
    aos_set_log_level(AOS_LL_DEBUG);

    netmgr_init();
    aos_register_event_filter(EV_KEY, linkkit_key_process, NULL);
    aos_register_event_filter(EV_WIFI, wifi_service_event, NULL);
    aos_register_event_filter(EV_YUNIO, cloud_service_event, NULL);

#ifdef CONFIG_AOS_CLI
    aos_cli_register_command(&resetcmd);
    aos_cli_register_command(&ncmd);
#endif
    aos_task_new("netmgr", start_netmgr, NULL, 4096);

#endif
#ifdef CONFIG_AOS_CLI
    aos_cli_register_command(&gyrocmd);
    aos_cli_register_command(&sensorcmd);
    aos_cli_register_command(&clibcmd);
#endif
    ResetMatrix();
    udata_sample();


    aos_loop_run();

    return 0;
}


#if 1

     
  
  
double Abs(double a)
{
    return a<0 ? -a : a;
}
  
uint8_t Equal(double a,double b)
{
    return Abs(a-b) < 1e-6;
}
  
void ResetMatrix(void)
{
    int row , column;
     
    for(row = 0 ; row<m ; row++){
        for(column = 0 ; column<n ; column++){
            m_matrix[row][column] = 0.0f;
        }
    }
    memset(&g_sensor_clib_para,0,sizeof(g_sensor_clib_para));
    printf("m == %d\n",m);
}
         
void CalcData_Input(double x , double y , double z)
{
    double V[MATRIX_SIZE];
    int row , column;

    V[0] = x*x;
    V[1] = y*y;
    V[2] = z*z;
    V[3] = x;
    V[4] = y;
    V[5] = z;
    V[6] = 1.0;
         
    for(row = 0 ; row<MATRIX_SIZE ; row++){
        for(column = 0 ; column<MATRIX_SIZE ; column++){
            m_matrix[row][column] += V[row]*V[column];
        }
    }
}
  
void SwapRow(int row1 , int row2)
{
    int column;
    double tmp;
     
    for(column = 0 ; column<n ; column++){
        tmp = m_matrix[row1][column];
        m_matrix[row1][column] = m_matrix[row2][column];
        m_matrix[row2][column] = tmp;
    }
}
  
void MoveBiggestElement2Top(int s_row , int s_column)
{
    int row,column;
     
    for(row = s_row+1 ; row<m ; row++){
        if(Abs(m_matrix[s_row][s_column])<Abs(m_matrix[row][s_column])){
            SwapRow(s_row , row);
        }
    }
}
  
//高斯消元法，求行阶梯型矩阵
uint8_t Matrix_GaussElimination(void)
{
    int row,column,i,j;
    double tmp;
     
    for(row = 0,column=0 ; row<m-1 && column<n-1 ; row++,column++){

        MoveBiggestElement2Top(row , column);
          

        if(Equal(m_matrix[row][column],0.0f)){
            printf("qiyi matrix:%d %d\r\n" , row , column);

            row--;
            continue;
        }
          

        for(i = row+1 ; i<m ; i++){        
            if(Equal(m_matrix[i][column],0.0f))
                continue;        //为0，无需处理
              
            tmp = m_matrix[i][column]/m_matrix[row][column];
              
            for(j = column ; j<n ; j++){
                m_matrix[i][j] -= m_matrix[row][j]*tmp;
            }
        }

        DispMatrix();
        //printf("\r\n");
    }

    return 1;
}
  

int Matrix_RowSimplify(void)
{
    int c = n;

    int row,column,k,s,t;
    float tmp;

    for(row=0,column=0;row<m && column<n;row++,column++)
    {
        if(Equal(m_matrix[row][column],0))
        {
            row--;
            continue;
        }
      
        if(row == m-1){
            m_matrix[row][column] = 0.0f;
        }

        for(s=0;s<row;s++)
        {
            if(Equal(m_matrix[s][column],0)){
                
                continue;
            }
            
            tmp = m_matrix[s][column] / m_matrix[row][column];
            for(t=column;t<n;t++)
                m_matrix[s][t] -= m_matrix[row][t]*tmp;

        }
    }

    DispMatrix();
    //printf("\r\n");

    return c;
}
  
void Matrix_Solve(double* C , double* sol)
{
    int row,column,i;
    int any_sol[MATRIX_SIZE];

    memset(any_sol , 0 , MATRIX_SIZE);
    for(row=0,column=0 ; row<m && column<n-1 ; row++,column++){
        if(Equal(m_matrix[row][column] , 0.0f)){
            any_sol[column] = 1;
            row--;
        }
    }

    row = 0;
    for(column = 0 ; column<n-1 ; column++){
        if(any_sol[column] == 1){
            sol[column] = C[column];
        }else{
            sol[column] = m_matrix[row][n-1];

            for(i = column+1 ; i<n-1 ; i++){
                if(any_sol[i]==1 && !Equal(m_matrix[row][i],0.0f)){
                    sol[column] -= m_matrix[row][i]*C[i];
                }
            }

            sol[column] /= m_matrix[row][column];
            row++;
        }
    }
}
  
void DispMatrix(void)
{
    int row,column;
     
    for(row = 0 ; row<m ; row++){
        for(column = 0 ; column<n ; column++){
            //printf("%.3f        " , m_matrix[row][column]);
        }
        //printf("\r\n");
    }
}
  
void Calc_Process(double radius)
{
    double C[MATRIX_SIZE];
    double Res[MATRIX_SIZE];
    int i;
    double k;
    int ret = 0;

    Matrix_GaussElimination();
    Matrix_RowSimplify();

    for(i = 0 ; i<MATRIX_SIZE ; i++){
        C[i] = 1000.0f;
    }

    Matrix_Solve(C , Res);

    //printf("a:%.2f b:%.2f c:%.2f d:%.2f e:%.2f f:%.2f g:%.2f\r\n" , Res[0],Res[1],Res[2],Res[3],Res[4],Res[5],Res[6]);

    k = (Res[3]*Res[3]/Res[0]+Res[4]*Res[4]/Res[1]+Res[5]*Res[5]/Res[2] - 4*Res[6])/(4*radius*radius);

    m_result[0] = sqrt(Res[0] / k);
    m_result[1] = sqrt(Res[1] / k);
    m_result[2] = sqrt(Res[2] / k);
    m_result[3] = Res[3] / (2 * Res[0]);
    m_result[4] = Res[4] / (2 * Res[1]);
    m_result[5] = Res[5] / (2 * Res[2]);

    printf("Xo:%f Yo:%f Zo:%f Xg:%f Yg:%f Zg:%f C:%f\r\n" , m_result[3],m_result[4],m_result[5],m_result[0],m_result[1],m_result[2],k);

    g_sensor_clib_para.gain_x = m_result[0];
    g_sensor_clib_para.gain_y = m_result[1];
    g_sensor_clib_para.gain_z = m_result[2];

    g_sensor_clib_para.offset_x = m_result[3];
    g_sensor_clib_para.offset_y = m_result[4];
    g_sensor_clib_para.offset_z = m_result[5];

    ret = aos_kv_set(ACC_CLIB_PARA_NAME, &g_sensor_clib_para, sizeof(g_sensor_clib_para), 1);
    printf("kv set %s ret %d\n",ACC_CLIB_PARA_NAME,ret);
    
}
 
#endif




