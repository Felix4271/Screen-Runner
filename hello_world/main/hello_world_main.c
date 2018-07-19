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
    uint8_t glider[] = {50, 50, 51, 51, 52, 49, 52, 50, 52, 51};
    uint8_t glider_gun[] = {50, 50, 50, 51, 51, 50, 51, 51, 60, 50, 60, 51, 60, 52, 
            61, 49, 61, 53, 62, 48, 62, 54, 63, 48, 63, 54, 64, 51, 65, 49, 65, 53, 
            66, 50, 66, 51, 66, 52, 67, 51, 70, 48, 70, 49, 70, 50, 71, 48, 71, 49,
            71, 50, 72, 47, 72, 51, 74, 46, 74, 47, 74, 51, 74, 52, 84, 48, 84, 49,
            85, 48, 85, 49};
    for (int i=0;i<72;i+=2) {
        set_pixel(glider_gun[0], glider_gun[1], 1, lines[adress]);
    }
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
        vTaskDelay(30/portTICK_RATE_MS);
        send_lines(spi, lines[adress]);
        bool current_mode = mode;
        if (gpio_get_level(MODE_PIN)==0) {
            for (int i=0;i<3;i++) {
                vTaskDelay(30/portTICK_RATE_MS);
                if (gpio_get_level(MODE_PIN)==0) {
                    mode = !current_mode;
                }
            }
        }
    }
}

void display_langtons_ant(spi_device_handle_t spi, uint8_t *lines) {
    uint8_t positions[2][2] = {{64, 32}, {62, 32}};
    uint8_t directions[] = {1, 1};
    bool a;
    while (1) {
        while (gpio_get_level(MODE_PIN)==1) {
            for (int i=0;i<2;i++) {
                if (i!=1||positions[0][0]!=positions[1][0]||positions[0][1]!=positions[1][1]) {
                    a = get_pixel(positions[i][0], positions[i][1], lines);
                }
                if (a) {
                    directions[i]++;
                } else {
                    directions[i]--;
                }
                set_pixel(positions[i][0], positions[i][1], !a, lines);
                directions[i]%=4;
                if (directions[i] == 0) {
                    positions[i][0]+=1;
                } else if (directions[i] == 1) {
                    positions[i][1]+=1;
                } else if (directions[i] == 2) {
                    positions[i][0]-=1;
                } else {
                    positions[i][1]-=1;
                }
                positions[i][0]%=128;
                positions[i][1]%=64;
                send_lines(spi, lines);
                vTaskDelay(8/portTICK_RATE_MS);
            }
        }
        while (gpio_get_level(MODE_PIN)==1) {
            vTaskDelay(100/portTICK_RATE_MS);
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
        vTaskDelay(3000/portTICK_RATE_MS);
    }
}

char font8x8_basic[128][8] = {
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+0000 (nul)
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+0001
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+0002
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+0003
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+0004
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+0005
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+0006
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+0007
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+0008
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+0009
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+000A
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+000B
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+000C
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+000D
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+000E
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+000F
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+0010
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+0011
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+0012
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+0013
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+0014
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+0015
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+0016
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+0017
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+0018
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+0019
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+001A
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+001B
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+001C
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+001D
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+001E
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+001F
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+0020 (space)
    { 0x18, 0x3C, 0x3C, 0x18, 0x18, 0x00, 0x18, 0x00},   // U+0021 (!)
    { 0x36, 0x36, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+0022 (")
    { 0x36, 0x36, 0x7F, 0x36, 0x7F, 0x36, 0x36, 0x00},   // U+0023 (#)
    { 0x0C, 0x3E, 0x03, 0x1E, 0x30, 0x1F, 0x0C, 0x00},   // U+0024 ($)
    { 0x00, 0x63, 0x33, 0x18, 0x0C, 0x66, 0x63, 0x00},   // U+0025 (%)
    { 0x1C, 0x36, 0x1C, 0x6E, 0x3B, 0x33, 0x6E, 0x00},   // U+0026 (&)
    { 0x06, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+0027 (')
    { 0x18, 0x0C, 0x06, 0x06, 0x06, 0x0C, 0x18, 0x00},   // U+0028 (()
    { 0x06, 0x0C, 0x18, 0x18, 0x18, 0x0C, 0x06, 0x00},   // U+0029 ())
    { 0x00, 0x66, 0x3C, 0xFF, 0x3C, 0x66, 0x00, 0x00},   // U+002A (*)
    { 0x00, 0x0C, 0x0C, 0x3F, 0x0C, 0x0C, 0x00, 0x00},   // U+002B (+)
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C, 0x06},   // U+002C (,)
    { 0x00, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x00, 0x00},   // U+002D (-)
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C, 0x00},   // U+002E (.)
    { 0x60, 0x30, 0x18, 0x0C, 0x06, 0x03, 0x01, 0x00},   // U+002F (/)
    { 0x3E, 0x63, 0x73, 0x7B, 0x6F, 0x67, 0x3E, 0x00},   // U+0030 (0)
    { 0x0C, 0x0E, 0x0C, 0x0C, 0x0C, 0x0C, 0x3F, 0x00},   // U+0031 (1)
    { 0x1E, 0x33, 0x30, 0x1C, 0x06, 0x33, 0x3F, 0x00},   // U+0032 (2)
    { 0x1E, 0x33, 0x30, 0x1C, 0x30, 0x33, 0x1E, 0x00},   // U+0033 (3)
    { 0x38, 0x3C, 0x36, 0x33, 0x7F, 0x30, 0x78, 0x00},   // U+0034 (4)
    { 0x3F, 0x03, 0x1F, 0x30, 0x30, 0x33, 0x1E, 0x00},   // U+0035 (5)
    { 0x1C, 0x06, 0x03, 0x1F, 0x33, 0x33, 0x1E, 0x00},   // U+0036 (6)
    { 0x3F, 0x33, 0x30, 0x18, 0x0C, 0x0C, 0x0C, 0x00},   // U+0037 (7)
    { 0x1E, 0x33, 0x33, 0x1E, 0x33, 0x33, 0x1E, 0x00},   // U+0038 (8)
    { 0x1E, 0x33, 0x33, 0x3E, 0x30, 0x18, 0x0E, 0x00},   // U+0039 (9)
    { 0x00, 0x0C, 0x0C, 0x00, 0x00, 0x0C, 0x0C, 0x00},   // U+003A (:)
    { 0x00, 0x0C, 0x0C, 0x00, 0x00, 0x0C, 0x0C, 0x06},   // U+003B (//)
    { 0x18, 0x0C, 0x06, 0x03, 0x06, 0x0C, 0x18, 0x00},   // U+003C (<)
    { 0x00, 0x00, 0x3F, 0x00, 0x00, 0x3F, 0x00, 0x00},   // U+003D (=)
    { 0x06, 0x0C, 0x18, 0x30, 0x18, 0x0C, 0x06, 0x00},   // U+003E (>)
    { 0x1E, 0x33, 0x30, 0x18, 0x0C, 0x00, 0x0C, 0x00},   // U+003F (?)
    { 0x3E, 0x63, 0x7B, 0x7B, 0x7B, 0x03, 0x1E, 0x00},   // U+0040 (@)
    { 0x0C, 0x1E, 0x33, 0x33, 0x3F, 0x33, 0x33, 0x00},   // U+0041 (A)
    { 0x3F, 0x66, 0x66, 0x3E, 0x66, 0x66, 0x3F, 0x00},   // U+0042 (B)
    { 0x3C, 0x66, 0x03, 0x03, 0x03, 0x66, 0x3C, 0x00},   // U+0043 (C)
    { 0x1F, 0x36, 0x66, 0x66, 0x66, 0x36, 0x1F, 0x00},   // U+0044 (D)
    { 0x7F, 0x46, 0x16, 0x1E, 0x16, 0x46, 0x7F, 0x00},   // U+0045 (E)
    { 0x7F, 0x46, 0x16, 0x1E, 0x16, 0x06, 0x0F, 0x00},   // U+0046 (F)
    { 0x3C, 0x66, 0x03, 0x03, 0x73, 0x66, 0x7C, 0x00},   // U+0047 (G)
    { 0x33, 0x33, 0x33, 0x3F, 0x33, 0x33, 0x33, 0x00},   // U+0048 (H)
    { 0x1E, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x1E, 0x00},   // U+0049 (I)
    { 0x78, 0x30, 0x30, 0x30, 0x33, 0x33, 0x1E, 0x00},   // U+004A (J)
    { 0x67, 0x66, 0x36, 0x1E, 0x36, 0x66, 0x67, 0x00},   // U+004B (K)
    { 0x0F, 0x06, 0x06, 0x06, 0x46, 0x66, 0x7F, 0x00},   // U+004C (L)
    { 0x63, 0x77, 0x7F, 0x7F, 0x6B, 0x63, 0x63, 0x00},   // U+004D (M)
    { 0x63, 0x67, 0x6F, 0x7B, 0x73, 0x63, 0x63, 0x00},   // U+004E (N)
    { 0x1C, 0x36, 0x63, 0x63, 0x63, 0x36, 0x1C, 0x00},   // U+004F (O)
    { 0x3F, 0x66, 0x66, 0x3E, 0x06, 0x06, 0x0F, 0x00},   // U+0050 (P)
    { 0x1E, 0x33, 0x33, 0x33, 0x3B, 0x1E, 0x38, 0x00},   // U+0051 (Q)
    { 0x3F, 0x66, 0x66, 0x3E, 0x36, 0x66, 0x67, 0x00},   // U+0052 (R)
    { 0x1E, 0x33, 0x07, 0x0E, 0x38, 0x33, 0x1E, 0x00},   // U+0053 (S)
    { 0x3F, 0x2D, 0x0C, 0x0C, 0x0C, 0x0C, 0x1E, 0x00},   // U+0054 (T)
    { 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x3F, 0x00},   // U+0055 (U)
    { 0x33, 0x33, 0x33, 0x33, 0x33, 0x1E, 0x0C, 0x00},   // U+0056 (V)
    { 0x63, 0x63, 0x63, 0x6B, 0x7F, 0x77, 0x63, 0x00},   // U+0057 (W)
    { 0x63, 0x63, 0x36, 0x1C, 0x1C, 0x36, 0x63, 0x00},   // U+0058 (X)
    { 0x33, 0x33, 0x33, 0x1E, 0x0C, 0x0C, 0x1E, 0x00},   // U+0059 (Y)
    { 0x7F, 0x63, 0x31, 0x18, 0x4C, 0x66, 0x7F, 0x00},   // U+005A (Z)
    { 0x1E, 0x06, 0x06, 0x06, 0x06, 0x06, 0x1E, 0x00},   // U+005B ([)
    { 0x03, 0x06, 0x0C, 0x18, 0x30, 0x60, 0x40, 0x00},   // U+005C (\)
    { 0x1E, 0x18, 0x18, 0x18, 0x18, 0x18, 0x1E, 0x00},   // U+005D (])
    { 0x08, 0x1C, 0x36, 0x63, 0x00, 0x00, 0x00, 0x00},   // U+005E (^)
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF},   // U+005F (_)
    { 0x0C, 0x0C, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+0060 (`)
    { 0x00, 0x00, 0x1E, 0x30, 0x3E, 0x33, 0x6E, 0x00},   // U+0061 (a)
    { 0x07, 0x06, 0x06, 0x3E, 0x66, 0x66, 0x3B, 0x00},   // U+0062 (b)
    { 0x00, 0x00, 0x1E, 0x33, 0x03, 0x33, 0x1E, 0x00},   // U+0063 (c)
    { 0x38, 0x30, 0x30, 0x3e, 0x33, 0x33, 0x6E, 0x00},   // U+0064 (d)
    { 0x00, 0x00, 0x1E, 0x33, 0x3f, 0x03, 0x1E, 0x00},   // U+0065 (e)
    { 0x1C, 0x36, 0x06, 0x0f, 0x06, 0x06, 0x0F, 0x00},   // U+0066 (f)
    { 0x00, 0x00, 0x6E, 0x33, 0x33, 0x3E, 0x30, 0x1F},   // U+0067 (g)
    { 0x07, 0x06, 0x36, 0x6E, 0x66, 0x66, 0x67, 0x00},   // U+0068 (h)
    { 0x0C, 0x00, 0x0E, 0x0C, 0x0C, 0x0C, 0x1E, 0x00},   // U+0069 (i)
    { 0x30, 0x00, 0x30, 0x30, 0x30, 0x33, 0x33, 0x1E},   // U+006A (j)
    { 0x07, 0x06, 0x66, 0x36, 0x1E, 0x36, 0x67, 0x00},   // U+006B (k)
    { 0x0E, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x1E, 0x00},   // U+006C (l)
    { 0x00, 0x00, 0x33, 0x7F, 0x7F, 0x6B, 0x63, 0x00},   // U+006D (m)
    { 0x00, 0x00, 0x1F, 0x33, 0x33, 0x33, 0x33, 0x00},   // U+006E (n)
    { 0x00, 0x00, 0x1E, 0x33, 0x33, 0x33, 0x1E, 0x00},   // U+006F (o)
    { 0x00, 0x00, 0x3B, 0x66, 0x66, 0x3E, 0x06, 0x0F},   // U+0070 (p)
    { 0x00, 0x00, 0x6E, 0x33, 0x33, 0x3E, 0x30, 0x78},   // U+0071 (q)
    { 0x00, 0x00, 0x3B, 0x6E, 0x66, 0x06, 0x0F, 0x00},   // U+0072 (r)
    { 0x00, 0x00, 0x3E, 0x03, 0x1E, 0x30, 0x1F, 0x00},   // U+0073 (s)
    { 0x08, 0x0C, 0x3E, 0x0C, 0x0C, 0x2C, 0x18, 0x00},   // U+0074 (t)
    { 0x00, 0x00, 0x33, 0x33, 0x33, 0x33, 0x6E, 0x00},   // U+0075 (u)
    { 0x00, 0x00, 0x33, 0x33, 0x33, 0x1E, 0x0C, 0x00},   // U+0076 (v)
    { 0x00, 0x00, 0x63, 0x6B, 0x7F, 0x7F, 0x36, 0x00},   // U+0077 (w)
    { 0x00, 0x00, 0x63, 0x36, 0x1C, 0x36, 0x63, 0x00},   // U+0078 (x)
    { 0x00, 0x00, 0x33, 0x33, 0x33, 0x3E, 0x30, 0x1F},   // U+0079 (y)
    { 0x00, 0x00, 0x3F, 0x19, 0x0C, 0x26, 0x3F, 0x00},   // U+007A (z)
    { 0x38, 0x0C, 0x0C, 0x07, 0x0C, 0x0C, 0x38, 0x00},   // U+007B ({)
    { 0x18, 0x18, 0x18, 0x00, 0x18, 0x18, 0x18, 0x00},   // U+007C (|)
    { 0x07, 0x0C, 0x0C, 0x38, 0x0C, 0x0C, 0x07, 0x00},   // U+007D (})
    { 0x6E, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // U+007E (~)
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}    // U+007F
};

static void draw_string(uint8_t *a,uint8_t length, uint8_t x, uint8_t y, uint8_t *lines) {
    for (int i=0;i<length;i++) {
        for (int j=0;j<8;j++) {
            lines[((y+j)/8)*128+x+8*i] = *a[i][j];
        }
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
    //display_langtons_ant(spi, lines[0]);
}