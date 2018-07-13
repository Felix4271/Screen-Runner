#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include <string.h>

#define MOSI_PIN 13
#define CLK_PIN 14
#define RST_PIN 23
#define CS_PIN 15
#define DC_PIN 22

typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} scrn_init_cmd_t;

DRAM_ATTR static const scrn_init_cmd_t scrn_init_cmds[]={
    {0xAE, {0}, 0}, // 0 disp off
    {0xD5, {0}, 0}, // 1 clk div
    {0x50, {0}, 0}, // 2 suggested ratio
    {0xA8, {0x3F}, 1}, // 3 set multiplex
    {0xD3,{0x0}, 1}, // 5 display offset
    {0x40, {0}, 0}, // 7 start line
    {0xAD,{0x8B}, 1}, // 8 enable charge pump
    {0xA1, {0}, 0}, // 10 seg remap 1, pin header at the top
    {0xC8, {0}, 0}, // 11 comscandec, pin header at the top
    {0xDA,{0x12}, 1}, // 12 set compins
    {0x81,{0x80}, 1}, // 14 set contrast
    {0xD9,{0x22}, 1}, // 16 set precharge
    {0xDB,{0x35}, 1}, // 18 set vcom detect
    {0xA6, {0}, 0}, // 20 display normal (non-inverted)
    {0xAF, {0}, 0}, // 21 disp on
    {0, {0}, 0xFF}
};

void scrn_cmd(spi_device_handle_t spi, const uint8_t cmd)
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=8;                     //Command is 8 bits
    t.tx_buffer=&cmd;               //The data is the cmd itself
    t.user=(void*)0;                //D/C needs to be set to 0
    ret=spi_device_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}

void scrn_data(spi_device_handle_t spi, const uint8_t *data, int len)
{
    esp_err_t ret;
    spi_transaction_t t;
    if (len==0) return;             //no need to send anything
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=len*8;                 //Len is in bytes, transaction length is in bits.
    t.tx_buffer=data;               //Data
    t.user=(void*)1;                //D/C needs to be set to 1
    ret=spi_device_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}

void scrn_spi_pre_transfer_callback(spi_transaction_t *t)
{
    int dc=(int)t->user;
    gpio_set_level(DC_PIN, dc);
}

//Initialize the display
void scrn_init(spi_device_handle_t spi)
{
    int cmd=0;
    const scrn_init_cmd_t* init_cmds = scrn_init_cmds;

    //Initialize non-SPI GPIOs
    gpio_set_direction(DC_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(RST_PIN, GPIO_MODE_OUTPUT);

    //Reset the display
    gpio_set_level(RST_PIN, 0);
    vTaskDelay(100 / portTICK_RATE_MS);
    gpio_set_level(RST_PIN, 1);
    vTaskDelay(100 / portTICK_RATE_MS);

    //Send all the commands
    while (init_cmds[cmd].databytes!=0xff) {
        scrn_cmd(spi, init_cmds[cmd].cmd);
        scrn_data(spi, init_cmds[cmd].data, init_cmds[cmd].databytes&0x1F);
        cmd++;
    }
}

//To send a set of lines we have to send a command, 2 data bytes, another command, 2 more data bytes and another command
//before sending the line data itself; a total of 6 transactions. (We can't put all of this in just one transaction
//because the D/C line needs to be toggled in the middle.)
//This routine queues these commands up so they get sent as quickly as possible.
static void send_lines(spi_device_handle_t spi, uint8_t *linedata)
{
    esp_err_t ret;
    //Transaction descriptors. Declared static so they're not allocated on the stack; we need this memory even when this
    //function is finished because the SPI driver needs access to it even while we're already calculating the next line.
    static spi_transaction_t trans[16];

    //In theory, it's better to initialize trans and data only once and hang on to the initialized
    //variables. We allocate them on the stack, so we need to re-init them each call.

    memset(&trans, 0, sizeof(spi_transaction_t));
    for (int i=0;i<16;i+=2) {
        trans[i].length=8*3;
        trans[i].user=(void*)0;
        trans[i].flags=SPI_TRANS_USE_TXDATA;
        trans[i].tx_data[0]=0xB0+(i/2);         //memory write
        printf("%u",trans[i].tx_data[0]);
        trans[i].tx_data[1]=0x02;
        trans[i].tx_data[2]=0x10;
        trans[i+1].length=1024;
        trans[i+1].user=(void*)1;  
        trans[i+1].tx_buffer=linedata+128*(i/2);       //Finally send the data
        trans[i+1].flags=0; //undo SPI_TRANS_USE_TXDATA flag
    }

    spi_transaction_t *rtrans;
    //Queue all transactions.
    for (int i=0;i<16;i+=1) {
        ret=spi_device_queue_trans(spi, &trans[i], portMAX_DELAY);
        assert(ret==ESP_OK);
        ret=spi_device_get_trans_result(spi, &rtrans, portMAX_DELAY);
        assert(ret==ESP_OK);
    }
    //When we are here, the SPI driver is busy (in the background) getting the transactions sent. That happens
    //mostly using DMA, so the CPU doesn't have much to do here. We're not going to wait for the transaction to
    //finish because we may as well spend the time calculating the next line. When that is done, we can call
    //send_line_finish, which will wait for the transfers to be done and check their status.
}

static void display_such_a_complicated_pattern(spi_device_handle_t spi)
{
    uint8_t *lines;
    //Allocate memory for the pixel buffers
    lines=heap_caps_malloc(1024*sizeof(uint8_t), MALLOC_CAP_DMA);
    assert(lines!=NULL);
    bool on=false;

    while (1) {
        on = !on;
        if (on) {
            for (int i=0;i<1024;i++) {
                lines[i] = 255;
            }
        } else {
            for (int i=0;i<1024;i++) {
                lines[i] = 0;
            }
        }
        send_lines(spi, lines);
        vTaskDelay(3000/portTICK_PERIOD_MS);
    }
}

void app_main()
{   
    esp_err_t ret;
    spi_device_handle_t spi;
    spi_bus_config_t buscfg={
        .mosi_io_num=MOSI_PIN,
        .sclk_io_num=CLK_PIN,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=256
    };
    spi_device_interface_config_t devcfg={
        .clock_speed_hz= 1000000,
        .mode=0,                                //SPI mode 0
        .spics_io_num=CS_PIN,                   //CS pin
        .queue_size=7,                          //We want to be able to queue 7 transactions at a time
        .pre_cb=scrn_spi_pre_transfer_callback, //Specify pre-transfer callback to handle D/C line
    };
    //Initialize the SPI bus
    ret=spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    ESP_ERROR_CHECK(ret);
    //Attach the scrn to the SPI bus
    ret=spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
    //Initialize the scrn
    scrn_init(spi);
    //Initialize the effect displayed
    display_such_a_complicated_pattern(spi);

    vTaskDelay(1000 / portTICK_RATE_MS);

    fflush(stdout);
}

