#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include <time.h>

//control flow
#include "global_config.h"
#include "global_config.c"


// #include "Wire.h"
// #include <Adafruit_PN532.h>
// #include "Adafruit_PN532.h"


//components
#include "PN532.h"
#include "PN532.c"
#include "nfc_controller.h"
#include "nfc_controller.c"
#include "clock.h"
#include "clock.c"
#include <sim7000.h>
#include <sim7000.c>
#include <i2cdev.h>
#include <i2cdev.c>
#include "access_database.h"
#include "access_database.c"
#include "sd.h"
#include "sd.c"
#include "communication.h"
#include "communication.c"
#include "uarts.h"
#include "uarts.c"
#include "bluetooth.h"
#include "bluetooth.c"
#include "esp_bt.h"
//#include "ble.h"
#include "tft_controller.h"
#include "tft_controller.c"
#include "expander_controller.h"
#include "expander_controller.c"
#include <tftspi.h>
#include <tftspi.c>
#include "filecontrol.h"

#include "pulse_controller.h"
#include "pulse_controller.c"

#include "mcp7940.h"
#include "mcp7940.c"

#include "sim_controller.h"
#include "sim_controller.c"
//#include "wifi.h"
#include <wifi.h>
#include <wifi.c>
#include <tft.h>
#include <tft.c>

#define RESET_PIN gpio_num_t(25)

//#define fIRMWARE_VERSION "NFCReader_revB_18-07-19"
/*DIEGO GOMEZ
    RELAY CONFIGURATION 
 */

//#define fIRMWARE_VERSION "NFCReader_revB_14-03-20"
/*FRANCISCO ALVARADO
    GET_TRANSACTION_RECORDRS
 */

//#define fIRMWARE_VERSION "NFCReader_revB_15-03-20"
/*FRANCISCO ALVARADO
    42 transacciones
 */

//#define fIRMWARE_VERSION "NFCReader_revB_23-03-20"
/*FRANCISCO ALVARADO
    GET_TRANSACTION_RECORDRS check and pointer modification
DIEGO GOMEZ
    memory optimizacion
 */

//#define fIRMWARE_VERSION "NFCReader_revB_23-03-20"
/*FRANCISCO ALVARADO/DIEGO GOMEZ
    if (slot->list != NULL && !list_is_empty(slot->list))
                list_remove(slot->list, list_front(slot->list));

    Freq check update
    BT buffer -> 50

 */
//#define fIRMWARE_VERSION "NFCReader_revB_28-03-20"

/*
DIEGO GOMEZ (Oct 2020)
    RTC added
    SIM added
    CARD TYPE identity
    CLIEND ID added to memory map, checked in every transaction
    Date change from 23-10-20 to 20-11-20, to make sure that they are using the last one
*/
// #define fIRMWARE_VERSION "NFCReader_revBIN_02-12-20"
/*
DIEGO GOMEZ (Dic 2020)
    SIM Network manual attach added
    If APN is set, then wait until the modem response
*/
//#define fIRMWARE_VERSION "NFCReader_revBIN_09-12-20+"
/*
DIEGO GOMEZ (Mar 2021)
    POOLING TIME
    FUNCTIONALITY CHANGE MODEM7000
    POOLING add to web config end point
*/
/*
DIEGO GOMEZ (May 2021)
    Configuration:
        Game mode:
            option to enable or disable ticket redemption
            option to enable or disable Ads
*/
/*
DIEGO GOMEZ (Jul 2021)
    Memory Leak fixed (process_data_from_server)
    Memory test over night .. keep memory balance +- 4KB 
    #define fIRMWARE_VERSION "NFCReader_revBIN_13-05-21+"
*/
#define fIRMWARE_VERSION "NFCReader_rev_15-07-21"

typedef struct
{
    char *log;
    uint8_t type;
} report_message_t;

xQueueHandle report_message_queue;

int POOL_WAIT_TIME;

int READER_ID;
int OPERATION_MODE;
float OPERATION_VALUE;

uint8_t SHOW_COUNTDOWN = 0;
int16_t COUNT_TIME = 0;

typedef struct
{
    uint8_t uid[7];
    uint8_t uidLength;
} GAME_USER_t;

#define NUM_GAME_USERS 5
GAME_USER_t GAME_USERS[NUM_GAME_USERS];
uint8_t USER_COUNTER = 0;
// apr_size_t *input_length = reinterpret_cast<apr_size_t *>(apr_pcalloc(pool, sizeof(apr_size_t)));
void generate_report_message(void *mess)
{
    if (mess != NULL)
    {
        report_message_t *rep = reinterpret_cast<report_message_t *>(malloc(sizeof(report_message_t)));
        rep->log = (char*)malloc((size_t)strlen((char*)mess)+1);
        strcpy(rep->log, (char*)mess);
        rep->type = 0;
        free(mess);
        xQueueSend(report_message_queue, rep, 5);
    }
}

void generate_cash_report_message(uint16_t pulse)
{

    float cash = pulse * COST_PULSE;

    char *mess = get_cash_report(pulse, cash, READER_ID);

    restart_cash();

    if (mess != NULL)
    {
        ESP_LOGI("MAIN", "no era null");
        report_message_t *rep = (report_message_t *)malloc(sizeof(report_message_t));
        rep->log = (char*)malloc((size_t)strlen((char*)mess)+1);
        strcpy(rep->log, (char*)mess);
        rep->type = 1;
        free(mess);
        xQueueSend(report_message_queue, rep, 5);
    }
}
SemaphoreHandle_t lock;
bool is_slave;
uint8_t num;
int8_t sda;
int8_t scl;
//////////////////////////////////////////////////////////////////////////////////////////////
// static int wait(int ms)
// {
//     if (ms < 0)
//     {
//         ms *= -1;
//     }
//     vTaskDelay(10 / portTICK_RATE_MS);
//     for (int n = 10; n < ms; n += 10)
//     {
//         vTaskDelay(10 / portTICK_RATE_MS);
//     }
//     return 1;
// }

void reset_system()
{
    gpio_set_level(RESET_PIN, 1);
    gpio_set_level(RESET_PIN, 0);
    vTaskDelay(400 / portTICK_PERIOD_MS);
    gpio_set_level(RESET_PIN, 1);
    vTaskDelay(10 / portTICK_PERIOD_MS);
}

esp_err_t init()
{
    
    size_t tamA = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    //reset pin
    uint64_t pintBitMask = ((1ULL) << RESET_PIN);
    gpio_config_t io_conf;
    // io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = pintBitMask;
    io_conf.pull_down_en = (gpio_pulldown_t)0;
    io_conf.pull_up_en = (gpio_pullup_t)1;
    char buff[64];
    esp_err_t sts;
    esp_err_t sts_sim_init = ESP_FAIL;
    sts = gpio_config(&io_conf);
    if (sts != ESP_OK)
        ESP_LOGI("MAIN", "reset pin not set");
    //GPIO Expander
    sts = _init_gpio_i2c_configuration();
    wait(300);
    _MODEM_PWR.value = 0;
    write_value(&_MODEM_PWR);
    wait(1000);
    _MODEM_PWR.value = 1;
    write_value(&_MODEM_PWR);
    
    //wait(300);
    // mcp23017_free_desc(&DEV_EXPANDER);
    //display
    sts = _init_tft_config();
    
    if (sts == ESP_OK)
        _init_boot_init_screen();
   
    sprintf(buff, "FIRMWARE: %s", fIRMWARE_VERSION);
    ets_printf("yeshere");
    add_boot_line_info(buff);
    sprintf(buff, "initial mem: %d", heap_caps_get_free_size(MALLOC_CAP_8BIT));
    add_boot_line_info(buff);
    //sd
    //TODO add error if is not possible initialize the SD
    sts = initialize_SD();
    ets_printf("yeshere");
    if (sts == ESP_OK)
        add_boot_line_info("SD configured");
    else
    {
        add_boot_line_error("SD configuration fail!");
        return -1;
    }
  

    //load configuration params
    READER_ID = get_reader_id();
    sprintf(buff, "Reader ID: %d", READER_ID);
    add_boot_line_info(buff);

    int rs485_address = get_rs485_address();
    sprintf(buff, "RS$85 ADDRESS: %d", rs485_address);
    add_boot_line_info(buff);

    OPERATION_MODE = get_operation_mode();
    sprintf(buff, "Operation MODE: %d", OPERATION_MODE);
    add_boot_line_info(buff);
    OPERATION_VALUE = get_operation_value(OPERATION_MODE);
    sprintf(buff, "Operation Value: %f", OPERATION_VALUE);
    add_boot_line_info(buff);
    char *wifi_name = get_wifi_name();
    sprintf(buff, "wifi name: %s", wifi_name);
    add_boot_line_info(buff);
    char *wifi_password = get_wifi_password();
    sprintf(buff, "wifi password: ********");
    add_boot_line_info(buff);
    char *bluetooth_name = get_bluetooth_name();
    sprintf(buff, "bluetooth name: %s", bluetooth_name);
    add_boot_line_info(buff);
    char *sim_apn = get_sim_apn();
    sprintf(buff, "sim  apn: %s", sim_apn);
    add_boot_line_info(buff);
    char *sim_username = get_sim_username();
    sprintf(buff, "sim  username: %s", sim_username);
    add_boot_line_info(buff);
    char *sim_password = get_sim_password();
    sprintf(buff, "sim  password: %s", sim_password);
    add_boot_line_info(buff);
    uint16_t clientID = get_clientID();
    set_reader_clientID(clientID);
    sprintf(buff, "ClientID: %d", clientID);
    add_boot_line_info(buff);
    set_client_id_access_data(clientID);
    POOL_WAIT_TIME = get_pool_wait_time();
    sprintf(buff, "Pooling time: %d s", POOL_WAIT_TIME / 1000);
    add_boot_line_info(buff);
    //Wifi
    sts = initialize_wifi(wifi_name, wifi_password);
    if (sts == ESP_OK)
        add_boot_line_info("Wifi Initialized");
    else
        add_boot_line_error("Wifi Initialization fail!");

    if (strlen(sim_apn) > 0)
    {
        //Modem SIM
        add_boot_line_info("__wait__modem checking ... ");
        start_loading_animation();
        initialize_sim7000_uart();
        sts_sim_init = iniciar_sim(sim_apn, sim_username, sim_password);
        if (sts_sim_init == ESP_OK)
            add_boot_line_info("Modem Initialized");
        else
        {
            do
            {
                ESP_LOGI("MAIN INIT", "Rebooting modem\n");
                wait(300);
                _MODEM_PWR.value = 0;
                write_value(&_MODEM_PWR);
                wait(1500);
                _MODEM_PWR.value = 1;
                write_value(&_MODEM_PWR);
                wait(300);
                ESP_LOGI("MAIN INIT", "Modem Rebooted\n");
            } while (iniciar_sim(sim_apn, sim_username, sim_password) != ESP_OK);

            add_boot_line_error("Modem Initialization fail!");
        }
        stop_loading_animation();
    }
    else
        sts_sim_init = ESP_FAIL;
    //nfc
    sts = initilize_PN532();
    if (sts == ESP_OK)
        add_boot_line_info("initilize_PN532");
    else
        add_boot_line_error("initilize_PN532 fail!");
    sts = set_passive_activation_retries(0xFF);
    if (sts == ESP_OK)
        add_boot_line_info("set_passive_activation_retries");
    else
        add_boot_line_error("set_passive_activation_retries fail!");
    sts = SAM_Configuration();
    if (sts == ESP_OK)
        add_boot_line_info("SAM_Configuration");
    else
        add_boot_line_error("SAM_Configuration fail!");

    //bluetooth
    sts = initialize_bluetooth(bluetooth_name);
    if (sts == ESP_OK)
        add_boot_line_info("initialize_bluetooth");
    else
        add_boot_line_error("initialize_bluetooth fail!");

    // //ble
    // sts = initialize_ble(bluetooth_name);
    // if(sts == ESP_OK)
    //     add_boot_line_info("initialize_bluetooth");
    // else
    //     add_boot_line_error("initialize_bluetooth fail!");

    //uarts
    //TODO add error if is not possible initialize
    sts = initilize_rs485(rs485_address);
    if (sts == ESP_OK)
        add_boot_line_info("initilize_rs485");
    else
        add_boot_line_error("initilize_rs485 fail!");

    //TODO add error if is not possible initialize
    sts = initilize_usbc();
    if (sts == ESP_OK)
        add_boot_line_info("initilize_usbc");
    else
        add_boot_line_error("initilize_usbc fail!");

    //Read Modes params config
    sts = _read_modes_config();
    if (sts == ESP_OK)
        add_boot_line_info("_read_modes_config");
    else
        add_boot_line_error("_read_modes_config fail!");
    if (!heap_caps_check_integrity_all(1))
        ESP_LOGI("MAIN", "ERROR DESPUES DE _read_modes_config");
    //Read Screen params config

    set_cost_value(OPERATION_VALUE);
    set_global_operation_mode(OPERATION_MODE);

    sts = _read_screen_param();
    if (sts == ESP_OK)
        add_boot_line_info("_read_screen_param");
    else
        add_boot_line_error("_read_screen_param fail!");
    if (!heap_caps_check_integrity_all(1))
        ESP_LOGI("MAIN", "ERROR DESPUES DE _read_screen_param");

    /*
    El rst y pwr del MODEM esta conectado al expansor de puertos,
    voy a iniciarlo unicamente para que ponga en 1 la salida.
    */

    /*
        Sincronizacion de la hora:
            Verificar conexion WIFI...
                Si hay WIFI sincronizar con servidor...
                Ajustar RTC...
            Si no:
                Comprobar si hay SIM:
                    Sincronizar con servidor....
                    Ajustar RTC...
            
            Si no:
                Verificar si el RTC ya tiene valor (año mayor igual a 2020)
                    SI:
                        asignar hora del RTC
                    NO:
                        ¿¿(SUPONGO QUE PONER UN MENSAJE Y NO CONTINUAR)??

    */
    uint8_t rtc_animation = 1;
    uint8_t rtc_wifi_animation = 0;
    uint8_t rtc_sim_animation = 0;
    uint8_t rtc_clock_animation = 0;
    draw_connection_animation(&rtc_wifi_animation, &rtc_sim_animation, &rtc_clock_animation, &rtc_animation);
    tiempo_rtc rtc_time;
    set_pointer_dev_i2c_rtc(&DEV_EXPANDER);
    wait(500);
    uint8_t sts_rtc = deviceStatus();
    while (!sts_rtc)
    {
        sts_rtc = deviceStart();
        if (!sts_rtc)
        {
            ESP_LOGI("MAIN", "NO se inicializo RTC\n");
            wait(2000);
        }
        else
        {
            break;
        }
    }
    int start_time = esp_timer_get_time() / 1000;
    rtc_wifi_animation = 1;
    uint8_t wifi_sts = is_wifi_connected();
    while (!wifi_sts && (esp_timer_get_time() / 1000 - start_time) < 5000)
    {
        wait(1000);
        //Animacion en pantalla de wifi
        wifi_sts = is_wifi_connected();
    }
    rtc_wifi_animation = wifi_sts ? 2 : 3;
    rtc_sim_animation = 1;
    set_process_data_from_server_ptr(&process_data_from_server);
    if (sts_sim_init == ESP_OK && is_sim_connected(0))
        rtc_sim_animation = 2;
    else
        rtc_sim_animation = 3;
    if (wifi_sts || is_sim_connected(1))
    {
        rtc_clock_animation = 1;
        get_sever_time(READER_ID);
        // get_time_values(&rtc_time.y, &rtc_time.m, &rtc_time.d, &rtc_time.h, &rtc_time.M, &rtc_time.s);
        if (rtc_time.y >= 2020)
        {
            rtc_clock_animation = 2;
            adjust(rtc_time.y, rtc_time.m, rtc_time.d, 1, rtc_time.h, rtc_time.M, rtc_time.s);
        }
        else
        {
            rtc_clock_animation = 3;
        }
    }
    // get_time_values(&rtc_time.y, &rtc_time.m, &rtc_time.d, &rtc_time.h, &rtc_time.M, &rtc_time.s);
    if (rtc_time.y < 2020)
    {
        // No hay conexion...
        // se usara la info del RTC ... si esta actualizada.
        get_now_from_RTC(&rtc_time);
        if (rtc_time.y >= 2020)
        {
            set_time_manually(rtc_time.y, rtc_time.m, rtc_time.d, rtc_time.h, rtc_time.M, rtc_time.s);
            rtc_clock_animation = 2;
        }
        else
        {
            draw_msg_no_time();
            while (1) //NO se puede continuar
            {
                wait(1000);
            }
            //ERROR EN PANTALLA .... NECESITA LA HORA!!
        }
    }
    mcp23017_free_desc(&DEV_EXPANDER);
    wait(300);
    rtc_animation = 0;
    //GPIO Expander
    sts = _init_gpio_i2c_configuration();
    if (sts == ESP_OK)
        add_boot_line_info("_init_gpio_i2c_configuration");
    else
        add_boot_line_error("_init_gpio_i2c_configuration fail!");

    //LOAD IMAGES TO SPIRAM
    add_boot_line_info("        .... loading images to RAM ....");
    start_loading_animation();
    sts = load_images_from_sd();
    stop_loading_animation();
    add_boot_line_info("        .... images loaded ....");

    /* //PWM Leds timmer, keep at the end of the function    
    sts = _init_timer_config();
    if(sts == ESP_OK)
        add_boot_line_info("_init_timer_config");
    else
        add_boot_line_error("_init_timer_config fail!");*/

    ////--------- COLA DE MENSAJES
    report_message_queue = xQueueCreate(10, sizeof(report_message_t));
    return ESP_OK;
}

void display_images(void *arg)
{

    uint8_t id_im;
    uint64_t startW;
    set_cost_value(0);
    set_balance_value(0);
    while (_CURRENT_SCREEN == NULL)
    {
        wait(100);
    }
    screen_t *tempScreen = _CURRENT_SCREEN;
    while (1)
    {

        tempScreen = _CURRENT_SCREEN;

        if (tempScreen->ready && tempScreen->id >= 0)
        {
            config_text_charact(tempScreen->text);
            if (tempScreen->cant_img == 0)
            {

                disp_select();
                TFT_fillScreen((color_t){tempScreen->background.r, tempScreen->background.g, tempScreen->background.b});
                TFT_print(get_text(&(tempScreen->text)), tempScreen->text.x, tempScreen->text.y);
                disp_deselect();
                wait(500);
            }
            for (id_im = 0; id_im < tempScreen->cant_img; id_im++)
            {
                startW = esp_timer_get_time();
                if (tempScreen->images[id_im].pixels != NULL)
                {
                    //ESP_LOGI("MAIN", "PINTANDO %s #%d de %d\n", tempScreen->images[id_im].pixels->full_path, id_im, tempScreen->cant_img);
                    TFT_rgb_from_SPIRAM_image(tempScreen->images[id_im].pixels->color_table,
                                              tempScreen->images[id_im].pixels->pixels,
                                              tempScreen->images[id_im].pixels->width,
                                              tempScreen->images[id_im].pixels->height,
                                              get_text(&(tempScreen->text)), tempScreen->text.x, tempScreen->text.y);
                }
                else
                {
                    char *path = get_image_path(tempScreen->images[id_im], tempScreen);
                    if (path != NULL)
                    {
                        //ESP_LOGI("MAIN", "PINTANDO %s #%d de %d\n", path, id_im, tempScreen->cant_img);
                        TFT_rgb_from_flash_image(path, get_text(&(tempScreen->text)), tempScreen->text.x, tempScreen->text.y);
                        free(path);
                    }
                }
                if (SHOW_COUNTDOWN == 1 && COUNT_TIME > 0)
                {
                    if (COUNT_TIME > 1000)
                        COUNT_TIME = COUNT_TIME / 1000;
                    draw_timeout(COUNT_TIME--, 1000 - tempScreen->images[id_im].duration);
                }
                else
                {
                    SHOW_COUNTDOWN = 0;
                }
                while (tempScreen == _CURRENT_SCREEN && (esp_timer_get_time() - startW) / 1000 < tempScreen->images[id_im].duration)
                {
                    wait(10);
                }
                if (tempScreen != _CURRENT_SCREEN)
                    break;
            }
        }
    }
}

void leds(void *arg)
{
    uint8_t id_leds, id_cols, id_sec;
    while (_CURRENT_SCREEN == NULL)
    {
        wait(100);
    }
    screen_t *tempScreen = _CURRENT_SCREEN;
    while (1)
    {
        tempScreen = _CURRENT_SCREEN;
        led_animation_t *animation = tempScreen->animation;
        if (tempScreen->ready == 152535)
        {
            if (animation->cant_sec == 0)
            {
                wait(100);
            }
            for (id_sec = 0; id_sec < animation->cant_sec; id_sec++)
            {
                for (id_leds = 0; id_leds < NUM_LEDS; id_leds++)
                {
                    for (id_cols = 0; id_cols < NUM_COLS; id_cols++)
                    {
                        _LEDS[id_leds].color[id_cols].duty = animation->led_sec[id_sec].led[id_leds][id_cols];
                    }
                }
                if (tempScreen != _CURRENT_SCREEN)
                    break;
                wait(-1 * animation->led_sec[id_sec].duration);
            }
        }
    }
}

uint8_t nfc_process_targets(op_mode_t *mode, uint8_t add_or_substract, uint8_t transaction_type, uint8_t *uid, uint8_t *uidLength, uint8_t validate_master)
{

    uint8_t process = 0;
    uint8_t clk = 0;
    int8_t status = 0;
    ESP_LOGI("MAIN", "add_or_substract %d  transaction_type %d  \n", add_or_substract, transaction_type);
    //change_screen(mode->tap_screen);
    //do{
    if (check_client_ID() != 1)
    {
        set_message_value("No Valid ID Card");
        change_screen(mode->error_screen);
        return 0;
    }

    if (process == 0)
    {
        if (transaction_type == 0x03)
            status = retrieve_data(-1);
        if (transaction_type == 0x11)
            status = retrieve_ticket_data(mode->value);
        if (transaction_type == 0x12)
            status = retrieve_ticket_data(mode->value);
        else
            status = retrieve_data(mode->value);

        if (validate_master == 0 && (status == ESP_OK || status == 1))
            process++;
        else if (validate_master == 1 && status == ESP_OK)
            process++;
        else if (validate_master == 1 && status == 1)
            return 100; //mastert return 100
        else
            ESP_LOGE("MAIN", "cant retrieve data\n");
    }

    if (process == 1)
    {
        //change_screen(mode.tap_screen);
        if (transaction_type == 0x11)
        {
            status = check_ticket_data();
            if (status == ESP_OK || add_or_substract == 0)
                process++;
            else
            {
                set_message_value("No enough Tickets");
                change_screen(mode->error_screen);
                //break;
            }
        }

        else if (transaction_type != 0x03)
        {
            status = check_cash_data();
            if (status == ESP_OK || add_or_substract == 0)
                process++;
            else
            {
                set_message_value("No cash/credit");
                change_screen(mode->error_screen);
                //break;
            }
        }
        else
            process++;
    }

    if (process == 2)
    {
        //change_screen(mode.tap_screen);
        if (transaction_type == 0x11 || transaction_type == 0x12)
        {
            create_new_ticket_data(add_or_substract);
        }
        else
        {
            create_new_data(add_or_substract);
        }
        process++;
    }

    if (process == 3)
    {

        //change_screen(mode.tap_screen);
        if (transaction_type == 0x11 || transaction_type == 0x12)
        {
            status = in_out_tickets();
            if (status == ESP_OK)
            {
                set_balance_value(get_new_tickets());
                change_screen(mode->ticket_success_screen);
                process = 5;
                return 1;
            }
            else
                ESP_LOGI("MAIN", "cant set new ticket data\n");
        }
        else
        {
            status = in_out_cash();
            if (status == ESP_OK)
            {
                process++;
                set_balance_value(get_new_cash());
                change_screen(mode->success_screen);
            }
            else
                ESP_LOGI("MAIN", "cant set new cash data\n");
        }
    }

    if (process == 4)
    {
        status = Write_record(transaction_type, READER_ID);
        if (status == ESP_OK)
        {
            process = 5;
            //break;
        }
        else
            ESP_LOGI("MAIN", "cant set transaction record\n");
        return 1;
    }

    return 0;
}

uint8_t compare_users(uint8_t *uid, uint8_t uidLength)
{
    uint8_t id_u, id_p;
    uint8_t equal = 1;
    uint8_t equal_0 = 1;
    for (id_u = 0; id_u < NUM_GAME_USERS; id_u++)
    {
        equal = 1;
        for (id_p = 0; id_p < uidLength; id_p++)
        {
            if (uid[id_p] != GAME_USERS[id_u].uid[id_p])
            {
                equal = 0;
                break;
            }
        }
        if (equal)
            return equal;
        if (GAME_USERS[id_u].uidLength > 0)
        {
            equal_0 = 0;
        }
    }
    return equal_0;
}

void game_mode(op_mode_t *mode, uint8_t *uid, uint8_t *uidLength, int64_t *timer)
{
    if (get_tickets() > 0 && mode->enable_redemption)
    {
        set_ticket_value(get_tickets());
        change_screen(mode->ticket_screen);
    }
    else if ((esp_timer_get_time() - *timer) / 1000 > mode->ad_time && mode->enable_ads)
    {
        change_screen(mode->ad_screen);
    }
    else
    {
        change_screen(mode->normal_screen);
    }
    uint8_t status;

    if (mode->enable_redemption && get_tickets() > 0 && get_ticket_ready())
    { //CHECK READY
        set_ticket_value(get_tickets());
        change_screen(mode->ticket_screen);
        SHOW_COUNTDOWN = 1;
        COUNT_TIME = mode->timeout_ticket;
        status = 0;
        while (status == 0 && COUNT_TIME > 0)
        {
            status = read_passive_target_ID(PN532_MIFARE_ISO14443A, uid, uidLength);
            status = status && compare_users(uid, *uidLength);
            if (status)
            {
                //TRANSFERIR TIQUETES
                pair_uid(uid, *uidLength);
                float actual_mode_value = mode->value;
                mode->value = get_tickets();
                if (nfc_process_targets(mode, 0, 0x12, uid, uidLength, 0))
                {
                    change_screen(mode->ticket_success_screen);
                    char *report = get_ticket_report(100, READER_ID);
                    generate_report_message(report);
                    mode->value = actual_mode_value;
                    break;
                }
                else
                {
                    status = 0;
                }
                mode->value = actual_mode_value;
            }
        }
        memset(GAME_USERS, 0, sizeof(GAME_USER_t) * NUM_GAME_USERS);
        SHOW_COUNTDOWN = 0;
        if (COUNT_TIME == 0)
        {
            set_message_value("Time is up");
            change_screen(mode->error_screen);
            /*
            TIME OUT
            REPOR TIQUETES AL SERVIDOR
            */
            set_ticket_amount(get_tickets());
            char *report = get_ticket_report(101, READER_ID);
            generate_report_message(report);
        }
        restart_tickets();
        wait(mode->info_time);
        dispair_uid();
    }
    if (!heap_caps_check_integrity_all(1))
        ESP_LOGI("MAIN", "ERROR antes de procesar nfc");
    //uint8_t b = 1;
    if (read_passive_target_ID(PN532_MIFARE_ISO14443A, uid, uidLength))
    {
        *timer = esp_timer_get_time();
        pair_uid(uid, *uidLength);

        status = nfc_process_targets(mode, 1, 0x01, uid, uidLength, 0);
        if (status == 1)
        { //Payment ok
            memcpy(GAME_USERS[USER_COUNTER].uid, uid, *uidLength);
            GAME_USERS[USER_COUNTER].uidLength = *uidLength;
            USER_COUNTER = (USER_COUNTER + 1) % NUM_GAME_USERS;
            char *report = get_report(mode->type, READER_ID);
            generate_report_message(report);
            if (mode->relay > 0)
            {
                if (mode->relay >= 3)
                {
                    _RELAYS[(0) % NUM_RELA].value = 0;
                    _RELAYS[(1) % NUM_RELA].value = 0;
                    wait(mode->relay_time);
                    _RELAYS[(0) % NUM_RELA].value = 1;
                    _RELAYS[(1) % NUM_RELA].value = 1;
                }
                else
                {
                    _RELAYS[(mode->relay - 1) % NUM_RELA].value = 0;
                    wait(mode->relay_time);
                    _RELAYS[(mode->relay - 1) % NUM_RELA].value = 1;
                }
            }
            restart_tickets();
            // add to queue
        }

        wait(mode->info_time);
        dispair_uid();
    }
}

void ride_mode(op_mode_t *mode, uint8_t *uid, uint8_t *uidLength)
{
    change_screen(mode->normal_screen);
    if (read_passive_target_ID(PN532_MIFARE_ISO14443A, uid, uidLength))
    {
        pair_uid(uid, *uidLength);
        //

        uint8_t status = nfc_process_targets(mode, 1, 0x01, uid, uidLength, 1);
        if (status == 1)
        {
            char *report = get_report(mode->type, READER_ID);
            generate_report_message(report);
            // add to queue
        }

        else if (status == 100)
        {
            change_screen(mode->credit_dump_screen);
            uint8_t uid_dump[] = {0, 0, 0, 0, 0, 0, 0};
            SHOW_COUNTDOWN = 1;
            COUNT_TIME = mode->credit_dump_time;
            while (COUNT_TIME > 0)
            {
                if (read_passive_target_ID(PN532_MIFARE_ISO14443A, uid_dump, uidLength))
                {
                    if (memcmp(uid, uid_dump, 7))
                    {
                        pair_uid(uid_dump, *uidLength); //0x03 para dump
                        if (nfc_process_targets(mode, 1, 0x03, uid_dump, uidLength, 0) == 1)
                        {
                            char *report = get_report(255, READER_ID);
                            generate_report_message(report);
                            break;
                        }
                    }
                }
            }
            SHOW_COUNTDOWN = 0;
            if (COUNT_TIME == 0)
            {
                set_message_value("Time is up");
                change_screen(mode->error_screen);
            }
        }
        wait(mode->info_time);
        dispair_uid();
    }
}
void food_beverage_mode(op_mode_t *mode, uint8_t *uid, uint8_t *uidLength)
{
    // validate if it ready !!!!

    if (mode->value > 0)
    {
        change_screen(mode->amount_screen);
        if (read_passive_target_ID(PN532_MIFARE_ISO14443A, uid, uidLength))
        {
            pair_uid(uid, *uidLength);
            //change_screen(mode->tap_screen);

            if (nfc_process_targets(mode, 1, 0x01, uid, uidLength, 0))
            {
                char *report = get_report(mode->type, READER_ID);
                generate_report_message(report);
                mode->value = -1;
            }

            wait(mode->info_time);
            dispair_uid();
        }
    }
    else
    {
        change_screen(mode->normal_screen);
        wait(100);
    }
}

void value_transfer_mode(op_mode_t *mode, uint8_t *uid, uint8_t *uidLength)
{

    // validate if it ready !!!!
    if (mode->value > 0)
    {

        //ESP_LOGI("MAIN", "process cost : %f",mode->value);
        change_screen(mode->amount_screen);
        if (read_passive_target_ID(PN532_MIFARE_ISO14443A, uid, uidLength))
        {
            pair_uid(uid, *uidLength);
            //change_screen(mode->tap_screen);

            if (nfc_process_targets(mode, 0, 0x02, uid, uidLength, 0))
            {
                char *report = get_report(mode->type, READER_ID);
                generate_report_message(report);
                mode->value = -1;
            }

            wait(mode->info_time);
            dispair_uid();
        }
    }
    else
    {
        change_screen(mode->normal_screen);
        wait(100);
    }
}

void balance_mode(op_mode_t *mode, uint8_t *uid, uint8_t *uidLength)
{

    change_screen(mode->normal_screen);
    // validate if it ready !!!!
    if (read_passive_target_ID(PN532_MIFARE_ISO14443A, uid, uidLength))
    {
        pair_uid(uid, *uidLength);
        //change_screen(mode->tap_screen);
        float balance = get_actual_balance();
        if (balance >= 0)
        {
            set_balance_value(balance);
            change_screen(mode->success_screen);
        }
        else
        {
            ESP_LOGI("MAIN", "Can not validate card\n");
            set_message_value("No validation");
            change_screen(mode->error_screen);
        }

        wait(mode->info_time);
        dispair_uid();
    }
}

void redemption_mode(op_mode_t *mode, uint8_t *uid, uint8_t *uidLength)
{

    if (mode->value > 0)
    {
        set_ticket_value(mode->value);
        change_screen(mode->amount_screen);
        if (read_passive_target_ID(PN532_MIFARE_ISO14443A, uid, uidLength))
        {
            pair_uid(uid, *uidLength);
            //change_screen(mode->tap_screen);
            if (nfc_process_targets(mode, 1, 0x11, uid, uidLength, 0))
            {
                change_screen(mode->success_screen);
                char *report = get_ticket_report(mode->type, READER_ID);
                generate_report_message(report);
                mode->value = 0;
            }
            wait(mode->info_time);
            dispair_uid();
        }
    }
    else
    {
        change_screen(mode->normal_screen);
        // validate if it ready !!!!
        if (read_passive_target_ID(PN532_MIFARE_ISO14443A, uid, uidLength))
        {
            pair_uid(uid, *uidLength);
            int balance = get_actual_ticket_balance();
            if (balance >= 0)
            {
                set_ticket_value(balance);
                change_screen(mode->ticket_screen);
                int server_balance = 0;

                mode->ticket = -1;
                if (is_wifi_connected())
                {
                    if (check_available_tickets(uid))
                    {
                        uint8_t count = 0;
                        while (count < 20)
                        {
                            if (mode->ticket != -1)
                            {
                                server_balance = mode->ticket;
                                break;
                            }
                            wait(100);
                            count++;
                        }
                    }
                }
                //preguntar servidor
                //if(check_available_tickets(uid)){ wait hasta mode->ticket!=-1 y time out}
                //if (mode->ticket==-1) no se pudo acceder al dato
                //else if(mode->ticket>0) pongalo en pantalla
                //else ponga que no tiene

                if (server_balance > 0)
                {
                    set_ticket_value(server_balance);
                    change_screen(mode->tap_screen);
                    uint8_t uid_tickets[] = {0, 0, 0, 0, 0, 0, 0};
                    SHOW_COUNTDOWN = 1;
                    COUNT_TIME = mode->timeout_ticket;
                    int status = 0;
                    while (status == 0 && COUNT_TIME > 0)
                    {
                        status = read_passive_target_ID(PN532_MIFARE_ISO14443A, uid_tickets, uidLength);
                        status = status && !memcmp(uid, uid_tickets, 7);
                        if (status)
                        {
                            //TRANSFERIR TIQUETES
                            //pair_uid(uid, *uidLength);
                            mode->value = server_balance;
                            if (nfc_process_targets(mode, 0, 0x12, uid, uidLength, 0))
                            {
                                change_screen(mode->ticket_success_screen);
                                char *report = get_ticket_report(102, READER_ID);
                                generate_report_message(report);
                                mode->value = 0;
                                break;
                            }
                            else
                            {
                                status = 0;
                            }
                            mode->value = 0;
                        }
                    }
                    SHOW_COUNTDOWN = 0;
                    if (COUNT_TIME == 0)
                    {
                        set_message_value("TIME IS UP");
                        change_screen(mode->error_screen);
                    }
                }

                /*
                set_balance_value(balance);
                set_message_value("Cant Redem Available Credits");
                change_screen(mode->success_screen);
                */
            }
            else
            {
                ESP_LOGI("MAIN", "Can not validate card\n");
                set_message_value("Can not validate card");
                change_screen(mode->error_screen);
            }

            wait(mode->info_time);
            dispair_uid();
        }
    }
}

void flow_control(void *arg)
{
    int success;
    uint8_t uid[] = {0, 0, 0, 0, 0, 0, 0};
    uint8_t uidLength;
    op_mode_t *currentMode;
    int64_t time_counter = esp_timer_get_time();
    while (_CURRENT_MODE == NULL)
    {
        wait(100);
    }

    if (OPERATION_MODE == 0 || OPERATION_MODE == 1)
        _CURRENT_MODE->value = OPERATION_VALUE;

    //Communications
    initilize_communications(_CURRENT_MODE);
    initilize_wifi_communications(_CURRENT_MODE);
    initilize_bluetooth_communications(_CURRENT_MODE);

    while (1)
    {
        currentMode = _CURRENT_MODE;
        set_cost_value(currentMode->value);
        switch (currentMode->type)
        {
        case 0: //GAME MODE
        {
            game_mode(currentMode, uid, &uidLength, &time_counter);
            break;
        }
        case 1: //RIDE MODE
        {
            ride_mode(currentMode, uid, &uidLength);
            break;
        }
        case 2:
        {
            food_beverage_mode(currentMode, uid, &uidLength);
            break;
        }
        case 3:
        {
            value_transfer_mode(currentMode, uid, &uidLength);
            break;
        }
        case 4:
        {
            balance_mode(currentMode, uid, &uidLength);
            break;
        }
        case 5:
        {
            redemption_mode(currentMode, uid, &uidLength);
            break;
        }
        }
        wait(20);
    }
}

void read_usbc_devices(void *arg)
{

    read_usbc();
    vTaskDelete(NULL);
}
void read_rs485_devices(void *arg)
{

    read_rs485();
    vTaskDelete(NULL);
}

void send_server_data(void *arg)
{
    // el que sigue envia sino manda al arvhivo, tmabien debe revisar si hay algo en el arvhivo e intentar subir apenas tenga internet
    // char * report =
    //if (send_str_report(chareportr)!=1) uint8_t write_append_sd_file("log.txt",char);
    while (1)
    {
        report_message_t rep;
        xQueueReceive(report_message_queue, &rep, portMAX_DELAY);
        ESP_LOGI("MAIN", "MESAJE RECIBIDO EN COLA %s\n\n", rep.log);

        if (!heap_caps_check_integrity_all(1))
            ESP_LOGI("MAIN", "despues de enrtrar a server data ");

        if (is_wifi_connected() || is_sim_connected(0))
        {
            if (send_str_report(rep.log, rep.type) != ESP_OK)
                write_append_sd_file("/sdcard/log.txt", rep.log);
        }
        else
            write_append_sd_file("/sdcard/log.txt", rep.log);
        free(rep.log);
    }
}
size_t initialMem = 0;
void update_nfcreader(void *arg)
{
    while (1)
    {
        if (_CURRENT_MODE != NULL)
        {
            //wait_wifi_Connection();
            //size_t free_size_before = heap_caps_get_free_size(MALLOC_CAP_8BIT);
            if (is_wifi_connected() || is_sim_connected(0))
                check_updates(READER_ID);
            //size_t free_size_after = heap_caps_get_free_size(MALLOC_CAP_8BIT);
            //ESP_LOGW("UPDATE", "Memory available: before (%d) after (%d)  diff (%d)   initial diff (%d)", free_size_before, free_size_after, free_size_after - free_size_before, initialMem - free_size_after);
            wait(POOL_WAIT_TIME);
        }
    }
    vTaskDelete(NULL);
}
////////////////////////////////////////////////////////////////////////////////////////////////
extern "C" void app_main()
{
    // Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET);
    
    esp_log_level_set("*", ESP_LOG_VERBOSE);
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ret = init();
    if (!heap_caps_check_integrity_all(1))
        ESP_LOGI("MAIN", "ERROR DESPUES DE init()");
    set_virtual_card_prt(get_is_virtual_card_prt());
    //check_image_files();
    pool_wait_time_ptr = &POOL_WAIT_TIME;
    if (ret == ESP_OK)
    {
        xTaskCreatePinnedToCore(task_update_and_write_values, "task_update", 3 * 1024, NULL, 7, NULL, 0);
        xTaskCreatePinnedToCore(leds, "leds", 3 * 1024, NULL, 5, NULL, 0);
        xTaskCreatePinnedToCore(send_server_data, "send_server_data", 4 * 1024, NULL, 2, NULL, 0);
        xTaskCreatePinnedToCore(display_images, "display_images", 4 * 1024, NULL, 10, NULL, 1);
        xTaskCreatePinnedToCore(flow_control, "flow_control", 4 * 1024, NULL, 15, NULL, 1);
        xTaskCreate(read_usbc_devices, "read_usbc_devices", 3 * 1024, NULL, 7, NULL);
        xTaskCreate(read_rs485_devices, "read_rs485_devices", 3 * 1024, NULL, 7, NULL);
        xTaskCreate(update_nfcreader, "update_nfcreader", 4 * 1024, NULL, 5, NULL);
        if (OPERATION_MODE == GAME_MODE)
        {
            init_pulse_reader(_OPTOCUPLES + 0, (void (*)(uint16_t))set_ticket_value, generate_cash_report_message);
        }
        initialMem = heap_caps_get_free_size(MALLOC_CAP_8BIT);
        ESP_LOGI("MAIN", "\nAfter load mem: %d\n\n", initialMem);
    }
    /*
    while (1)
    {
        esp_bt_controller_status_t status_BT = esp_bt_controller_get_status();
        switch (status_BT)
        {
        case ESP_BT_CONTROLLER_STATUS_IDLE:
            ESP_LOGI("MAIN", "ESP_BT_CONTROLLER_STATUS_IDLE ");
            break;
        case ESP_BT_CONTROLLER_STATUS_INITED:
            ESP_LOGI("MAIN", "ESP_BT_CONTROLLER_STATUS_INITED ");
            break;
        case ESP_BT_CONTROLLER_STATUS_ENABLED:
            ESP_LOGI("MAIN", "ESP_BT_CONTROLLER_STATUS_ENABLED ");
            break;
        case ESP_BT_CONTROLLER_STATUS_NUM:
            ESP_LOGI("MAIN", "ESP_BT_CONTROLLER_STATUS_NUM ");
            break;

        default:
            ESP_LOGI("MAIN", "%d ", status_BT);
            break;
        }
        wait(2000);
    }
*/
    //wait_wifi_Connection();

    //get_sever_time(READER_ID);
}
