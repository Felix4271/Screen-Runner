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
#define U_PIN 16
#define L_PIN 17
#define D_PIN 18
#define R_PIN 19
#define C_PIN 5
#define MODE_PIN 2

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
    t.user=(void*)0;                //D/C needs to be set to 1
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
    gpio_set_direction(MODE_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(U_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(L_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(D_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(R_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(C_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(MODE_PIN, GPIO_PULLUP_ONLY);
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

static void send_lines(spi_device_handle_t spi, uint8_t *linedata)
{
    esp_err_t ret;
    static spi_transaction_t trans[16];

    memset(&trans, 0, sizeof(spi_transaction_t));
    for (int i=0;i<16;i+=2) {
        trans[i].length=8*3;
        trans[i].user=(void*)0;
        trans[i].flags=SPI_TRANS_USE_TXDATA;
        trans[i].tx_data[0]=0xB0+(i/2);
        trans[i].tx_data[1]=0x02;
        trans[i].tx_data[2]=0x10;
        trans[i+1].length=1024;
        trans[i+1].user=(void*)1;  
        trans[i+1].tx_buffer=linedata+128*(i/2);
        trans[i+1].flags=0; //undo SPI_TRANS_USE_TXDATA flag
    }

    spi_transaction_t *rtrans;
    for (int i=0;i<16;i+=1) {
        ret=spi_device_queue_trans(spi, &trans[i], portMAX_DELAY);
        assert(ret==ESP_OK);
    }
    for (int i=0;i<16;i++) {
        ret=spi_device_get_trans_result(spi, &rtrans, portMAX_DELAY);
        assert(ret==ESP_OK);
    }
}

void set_pixel(uint8_t x, uint8_t y, uint8_t value, uint8_t *lines) {
    if (value) {
        lines[x+128*(y/8)] |= 1<<y%8;
    } else {
        lines[x+128*(y/8)] &= ~(1<<y%8);
    }
}

bool get_pixel(uint8_t x, uint8_t y, uint8_t *lines) {
    return lines[x+128*(y/8)]&(1<<(y%8));
}

void set_rect(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t *lines) {
    if (height>8-(y%8)) {
        for (int i=0;i<width;i++) {
            lines[x+i+128*(y/8)] |= ((uint8_t) ~0)<<(y%8);
        }
        set_rect(x, y+8-(y%8), width, height-8+(y%8), lines);
    } else {
        for (int i=0;i<width;i++) {
            lines[x+i+128*(y/8)] |= ((uint8_t) (((uint8_t) ~0)<<height))>>(8-height-(y%8));
        }
    }
}

uint8_t count_neighbourghs(uint8_t x, uint8_t y, uint8_t *lines){
    uint8_t sets[8][2] = {{x-1, y-1}, {x, y-1}, {x+1, y-1}, {x-1, y}, {x+1, y}, {x-1, y+1}, {x, y+1}, {x+1, y+1}};
    uint8_t a=0;
    for (int i=0;i<8;i++) {
        sets[i][0]%=128;
        sets[i][1]%=64;
        a += get_pixel(sets[i][0], sets[i][1], lines);
    }
    return a;
}

void display_game_of_life(spi_device_handle_t spi, uint8_t *lines[2]) {
    bool adress = 0;                         //Records which memory buffer is being used for what
    uint8_t cursor[] = {49, 49};             //Records position of cursor in edit mode
    bool mode = 0;                           //Play mode or Edit mode
    //These three variables let the cursor flash
    bool cursor_on = 0;                      //Records state
    uint8_t cursor_switch_counter = 0;       //Switches state every 7 loops
    bool cursor_state = 0;                   //Remebers the state of the pixel before the cursor got there
    bool mode_reverted = true;               //Remembers whether cursor_state needs to be reset (after running, it may have changed)
    //These five allow commands to be recieved with little repitition
    bool prev_state[] = {0, 0, 0, 0, 0};
    bool new_state[5];
    uint8_t changed;
    uint8_t counters[] = {0, 0, 0, 0, 0};
    uint8_t pins[] = {U_PIN, L_PIN, D_PIN, R_PIN, C_PIN};
    /*
    //Draws a glider
    set_pixel(50, 50, 1, lines[adress]);
    set_pixel(51, 51, 1, lines[adress]);
    set_pixel(52, 49, 1, lines[adress]);
    set_pixel(52, 50, 1, lines[adress]);
    set_pixel(52, 51, 1, lines[adress]);*/
    //Draws a glider gun
    set_pixel(50, 50, 1, lines[adress]);
    set_pixel(50, 51, 1, lines[adress]);
    set_pixel(51, 50, 1, lines[adress]);
    set_pixel(51, 51, 1, lines[adress]);
    set_pixel(60, 50, 1, lines[adress]);
    set_pixel(60, 51, 1, lines[adress]);
    set_pixel(60, 52, 1, lines[adress]);
    set_pixel(61, 49, 1, lines[adress]);
    set_pixel(61, 53, 1, lines[adress]);
    set_pixel(62, 48, 1, lines[adress]);
    set_pixel(62, 54, 1, lines[adress]);
    set_pixel(63, 48, 1, lines[adress]);
    set_pixel(63, 54, 1, lines[adress]);
    set_pixel(64, 51, 1, lines[adress]);
    set_pixel(65, 49, 1, lines[adress]);
    set_pixel(65, 53, 1, lines[adress]);
    set_pixel(66, 50, 1, lines[adress]);
    set_pixel(66, 51, 1, lines[adress]);
    set_pixel(66, 52, 1, lines[adress]);
    set_pixel(67, 51, 1, lines[adress]);
    set_pixel(70, 48, 1, lines[adress]);
    set_pixel(70, 49, 1, lines[adress]);
    set_pixel(70, 50, 1, lines[adress]);
    set_pixel(71, 48, 1, lines[adress]);
    set_pixel(71, 49, 1, lines[adress]);
    set_pixel(71, 50, 1, lines[adress]);
    set_pixel(72, 47, 1, lines[adress]);
    set_pixel(72, 51, 1, lines[adress]);
    set_pixel(74, 46, 1, lines[adress]);
    set_pixel(74, 47, 1, lines[adress]);
    set_pixel(74, 51, 1, lines[adress]);
    set_pixel(74, 52, 1, lines[adress]);
    set_pixel(84, 48, 1, lines[adress]);
    set_pixel(84, 49, 1, lines[adress]);
    set_pixel(85, 48, 1, lines[adress]);
    set_pixel(85, 49, 1, lines[adress]);

    while (1) {
        if (mode) {
            set_pixel(cursor[0], cursor[1], cursor_state, lines[adress]);
            for (int i=0;i<128;i++) {
                for (int j=0;j<64;j++) {
                    set_pixel(i, j, 0, lines[1-adress]);
                    uint8_t b = count_neighbourghs(i, j, lines[adress]);
                    if (get_pixel(i, j, lines[adress])) {
                        if (b<2||b>3) {
                            set_pixel(i, j, 0, lines[1-adress]);
                        } else {
                            set_pixel(i, j, 1, lines[1-adress]);
                        }
                    } else {
                        if (b==3) {
                            set_pixel(i, j, 1, lines[1-adress]);
                        }
                    }
                }
            }
            adress = 1-adress;
            mode_reverted = true;
        } else {
            if (mode_reverted) {
                mode_reverted = false;
                cursor_state = get_pixel(cursor[0], cursor[1], lines[adress]);
            }
            cursor_switch_counter+=1;
            cursor_switch_counter%=16;
            if (!cursor_switch_counter) {
                cursor_on = !cursor_on;
            }
            changed=13;
            for (int i=0;i<4;i++) {
                new_state[i] = gpio_get_level(pins[i]);
                if (new_state[i]) {
                    if (prev_state[i]) {
                        counters[i] += 1;
                        if (counters[i] > 15) {
                            counters[i] = 12;
                            changed = i;
                        }
                    } else {
                        counters[i] = 0;
                        changed = i;
                    }
                }
                prev_state[i]=new_state[i];
            }
            new_state[4] = gpio_get_level(pins[4]);
            if (new_state[4]&&!prev_state[4]) {
                changed = 4;
                counters[4] = 0;
            }
            prev_state[4]=new_state[4];
            if (changed != 13) {
                if (changed != 4) {
                    set_pixel(cursor[0], cursor[1], cursor_state, lines[adress]);
                }
                if (changed == 0) {
                    cursor[1]--;
                } else if (changed == 1) {
                    cursor[0]--;
                } else if (changed == 2) {
                    cursor[1]++;
                } else if (changed == 3) {
                    cursor[0]++;
                } else if (changed == 4) {
                    set_pixel(cursor[0], cursor[1], !cursor_state, lines[adress]);
                }
                cursor[0] %= 128;
                cursor[1] %= 64;
                cursor_state = get_pixel(cursor[0], cursor[1], lines[adress]);
            }
            set_pixel(cursor[0], cursor[1], cursor_on, lines[adress]);
        }
        vTaskDelay(30/portTICK_PERIOD_MS);
        send_lines(spi, lines[adress]);
        bool current_mode = mode;
        if (gpio_get_level(MODE_PIN)==0) {
            for (int i=0;i<3;i++) {
                vTaskDelay(30/portTICK_PERIOD_MS);
                if (gpio_get_level(MODE_PIN)==0) {
                    mode = !current_mode;
                }
            }
        }
    }
}

static void display_such_a_complicated_pattern(spi_device_handle_t spi, uint8_t *lines)
{
    while (1) {
        for (int i = 0;i<128;i++) {
            for (int j=0;j<64;j++) {
                set_pixel(i, j, 0, lines);
            }
        }
        set_rect(10, 10, 100, 50, lines);
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
        .clock_speed_hz= 20000000,
        .mode=0,                                //SPI mode 0
        .spics_io_num=CS_PIN,                   //CS pin
        .queue_size=17,                          //We want to be able to queue 17 transactions at a time
        .pre_cb=scrn_spi_pre_transfer_callback //Specify pre-transfer callback to handle D/C line
    };
    uint8_t *lines[2];
    lines[0]=heap_caps_malloc(1024*sizeof(uint8_t), MALLOC_CAP_DMA);
    assert(lines[0]!=NULL);
    lines[1]=heap_caps_malloc(1024*sizeof(uint8_t), MALLOC_CAP_DMA);
    assert(lines[1]!=NULL);
    //Initialize the SPI bus
    ret=spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    ESP_ERROR_CHECK(ret);
    //Attach the scrn to the SPI bus
    ret=spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
    //Initialize the scrn
    scrn_init(spi);
    for (int i=0;i<2;i++) {
        for (int j=0;j<1024;j++) {
            lines[i][j] = 0x00;
        }
    }
    //Initialize the effect displayed
    display_game_of_life(spi, lines);
}