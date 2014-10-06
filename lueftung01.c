// ATtiny84A Pin map
//                                                      +-\/-+
//                                                VCC  1|o   |14  GND
//                                      UART TX - PB0  2|    |13  PA0 - SDA - 2.2kOhm - VCC
//                                                PB1  3|    |12  PA1 - SCL SHT21#1 - 2.2kOhm - VCC
//                                             |RESET  4|    |11  PA2 - SCL SHT21#2 - 2.2kOhm - VCC
// Fan - 16V-5V-Boost -  IRLI540N - 100Ohm - OC0A, PB2 5|    |10  PA3 - ST7735 RESet 
//                                 ST7735 RS/DC - PA7  6|    |9   PA4 - SCK - ST7735 SCL, Programmer
//                Programmer, ST7735 SDA - MOSI - PA6  7|    |8   PA5 - MISO - Programmer
//                                                      +----+
//
// SHT21 #-: Broken
// SHT21 #0 (Aussen): s30011846901C8000
// SHT21 #1 (Garage): s330118B7151C8000

// TODO
// Watchdog
// Serial protocol
// Async?
// Timeouts?
// Error handling -> Ignore value, flag old ones as old, switch off fan

#include <avr/io.h>
#include <stdlib.h>
//#define F_CPU 8000000 // 8 MHz
#include <util/delay.h>
#include "i2cmaster.h"
#include "i2c2master.h"
#include "ds.h"
#include "uTFT_ST7735.h"

#define FONTWIDTH 6
#define FONTHEIGHT 8

#define FANPINB (1<<PINB2)

enum {AUSSEN, GARAGE, SHTS, LUFTER=SHTS};
#define INVALID 0x8000

void ser_writes (char* txt) {
    while (*txt) {
        ser_write((uint8_t)(*txt++));
    }
}

#define hex(digit) ((digit)+((digit)>9?'A'-10:'0'))

char* hex8 (char* p, uint8_t val) {
    *p++ = hex(val>>4);
    *p++ = hex(val&0xF);
    *p = 0;
    return p;
}
char* hex16 (char* p, uint16_t val) {
    p = hex8(p,val>>8);
    p = hex8(p,val&0xFF);
    return p;
}

char* text (char* p, const char* txt) {
    while (*txt) {
        *p++ = *txt++;
    }
    *p = 0;
    return p;
}

char* dec (char* p, int16_t val, uint8_t factor) {
    if (val<0) {
        *p++ = '-';
        val = val;
    }
    int16_t digits = val/factor;
    itoa(digits,p,10);
    while (*p) p++;
    if (factor>1) {
        *p++ = '.';
        while (factor>1) {
            val -= digits*factor;
            digits = val*10/factor;
            *p++ = hex(digits);
            factor /= 10;
        }
    }    
    *p = 0;
    return p;
}

enum {
    SHT21_ADDR=0x80
};
enum {
    SHT21_TEMPHOLD=0xE3, SHT21_HUMIHOLD=0xE5, SHT21_TEMPPOLL=0xF3, SHT21_HUMIPOLL=0xF5,
};

// Checksum calculation, polynomial x^8+x^5+x^4+1. This is however not identical to _crc_ibutton_update!
uint8_t crcu (uint8_t crc, uint8_t data) {
    crc = crc ^ data;
    for (uint8_t i = 0; i<8; i++) {
        crc = crc&0x80? (crc<<1)^0x131: crc<<1;
    }
    return crc;
}

int16_t taupunkt (uint16_t relhumi, int16_t temp) {
    // http://www.mikrocontroller.net/topic/306226#3292459
    // http://wwwcaps.ou.edu/arpsbrowser/arps5.2.4browser/html_code/adas/mthermo.f90.html#DWPT
    //
    //    baker, schlatter  17-may-1982     original version.
    //
    //   this function returns the dew point (celsius) given the temperature
    //   (celsius) and relative humidity (%). the formula is used in the
    //   processing of u.s. rawinsonde data and is referenced in parry, h.
    //   dean, 1969: "the semiautomatic computation of rawinsondes,"
    //   technical memorandum wbtm edl 10, u.s. department of commerce,
    //   environmental science services administration, weather bureau,
    //   office of systems development, equipment development laboratory,
    //   silver spring, md (october), page 9 and page ii-4, line 460.
    
    float x = (1000-relhumi)/1000.0;
    float dpd = (14.55+0.0114*temp)*x + pow((2.5+0.0007*temp)*x,3) + (15.9+0.0117*temp)*pow(x,14);
    return temp-(int16_t)(dpd*10.0);
}

#define sht_read(sht,ack) ((sht)?i2c2_read:i2c_read)(ack)
#define sht_start(sht,adr) ((sht)?i2c2_start:i2c_start)(adr)
#define sht_write(sht,val) ((sht)?i2c2_write:i2c_write)(val)
#define sht_rep_start(sht,adr) ((sht)?i2c2_rep_start:i2c_rep_start)(adr)
#define sht_stop(sht) ((sht)?i2c2_stop:i2c_stop)()

uint8_t sht_read8crc (uint8_t sht, uint8_t* val8, uint8_t ack) {
    val8[0] = sht_read(sht,1);
    return crcu(0,val8[0])!=sht_read(sht,ack);
}

uint8_t sht_read16crc (uint8_t sht, uint16_t* val16, uint8_t ack) {
    uint8_t* val8 = (uint8_t*) val16;
    // AVR uses little endianness (lsb are first in storage, then msb)
    val8[1] = sht_read(sht,1);
    val8[0] = sht_read(sht,1);
    return crcu(crcu(0,val8[1]),val8[0])!=sht_read(sht,ack);
}

void writeval (int16_t val, uint8_t sign) {
    if (val==INVALID) {
        print(sign?"ERROR":"ERRO");
    } else {
        if (val<0) {
            write('-');
            val = -val;
        } else if (sign) {
            write('+');
        }
        uint8_t digit = val/100;
        write(digit+'0');
        val -= digit*100;
        digit = val/10;
        write(digit+'0');
        write('.');
        val -= digit*10;
        write(val+'0');
    }    
}

int main () {
    uint8_t sht, err, ser[8];
    uint16_t uval;
    char buffer[80], *p;
    int16_t val, temp[SHTS], humi[SHTS], taup[SHTS];
    const uint16_t BG=Color565(0x00,0x00,0x00), FG=Color565(0xFF,0xFF,0xFF), RED=Color565(0xFF,0x40,0x40), YLW=Color565(0xFF,0xFF,0x40), GRN=Color565(0x40,0xFF,0x40);

    // Go to full speed (8MHz internal OSC in our case)
    CLKPR = 1<<CLKPCE;
    CLKPR = 0;

    ser_init();
    ser_writes("init\n");

#define COL(col) ((col)==1?8:(col)==2?14:20)
#define ROW(sht) ((sht)==AUSSEN?4:(sht)==GARAGE?6:8)

    st7735_init();
    fillScreen(BG);
    setTextColor(FG,BG);
/*
    setCursor(0,0*FONTHEIGHT); print("SHT #1:"); // ° \367 ü \201
    setCursor(0,1*FONTHEIGHT); print("SHT #2:");
*/
    setCursor(0,ROW(AUSSEN)*FONTHEIGHT); print("Aussen: +##.# ##.#% +##.# ");
    setCursor(0,ROW(GARAGE)*FONTHEIGHT); print("Garage: +##.# ##.#% +##.# ");
    setCursor(0,ROW(LUFTER)*FONTHEIGHT); print("L\201fter: Aus");
    
    DDRB |= FANPINB;
    
    i2c_init();
    i2c2_init();
    _delay_ms(15);
    
    // TODO: SHT21 Soft Reset?
    
    for (sht=0; sht<SHTS; sht++) {
        err = sht_start(sht,SHT21_ADDR|I2C_WRITE);
        err |= sht_write(sht,0xFA);
        err |= sht_write(sht,0x0F);
        err |= sht_rep_start(sht,SHT21_ADDR|I2C_READ);
        err |= sht_read8crc(sht,ser+5,1);
        err |= sht_read8crc(sht,ser+4,1);
        err |= sht_read8crc(sht,ser+3,1);
        err |= sht_read8crc(sht,ser+2,0);
        err = sht_rep_start(sht,SHT21_ADDR|I2C_WRITE);
        err |= sht_write(sht,0xFC);
        err |= sht_write(sht,0xC9);
        err |= sht_rep_start(sht,SHT21_ADDR|I2C_READ);
        err |= sht_read16crc(sht,(uint16_t*)(ser+0),1);
        err |= sht_read16crc(sht,(uint16_t*)(ser+6),0);
        sht_stop(sht);
        
/*        
        p = text(buffer,"sht");
        p = hex8(p,sht);
        p = text(p," s");
        char* p2 = p;
        for (uint8_t i=0; i<8; i++) {
            p = hex8(p,ser[i]);
        }
        setTextColor(err?RED:FG,BG);
        setCursor(COL(1)*FONTWIDTH,sht*FONTHEIGHT);
        print(p2);
        p = text(p," e");
        p = hex8(p,err);
        p = text(p,"\n");
        ser_writes(buffer);
*/
    }    

   
    while (1) {
        for (sht=0; sht<SHTS; sht++) {
            err = sht_start(sht,SHT21_ADDR|I2C_WRITE);
            err |= sht_write(sht,SHT21_TEMPHOLD);
            err |= sht_rep_start(sht,SHT21_ADDR|I2C_READ);
            err |= sht_read16crc(sht,&uval,0);
            val = (int16_t)(-468.5 + 1757.2 / 65536.0 * (float)uval);
            temp[sht] = err||val<-600||val>999? INVALID: val; 
            err = sht_rep_start(sht,SHT21_ADDR|I2C_WRITE);
            err |= sht_write(sht,SHT21_HUMIHOLD);
            err |= sht_rep_start(sht,SHT21_ADDR|I2C_READ);
            err |= sht_read16crc(sht,&uval,0);
            sht_stop(sht);
            val = (int16_t)(-60.0 + 1250.0 / 65536.0 * (float)uval);
            humi[sht] = err||val<0||val>1000? INVALID: val;
            taup[sht] = temp[sht]!=INVALID&&humi[sht]!=INVALID? taupunkt(humi[sht],temp[sht]): INVALID;

            setTextColor(FG,BG);
            setCursor(COL(1)*FONTWIDTH,ROW(sht)*FONTHEIGHT);
            writeval(temp[sht],1);
            setCursor(COL(2)*FONTWIDTH,ROW(sht)*FONTHEIGHT);
            writeval((int16_t)humi[sht],0);
            setCursor(COL(3)*FONTWIDTH,ROW(sht)*FONTHEIGHT);
            writeval(taup[sht],1);
        }
        
        uint8_t fan = temp[GARAGE]!=INVALID && humi[GARAGE]!=INVALID && temp[AUSSEN]!=INVALID && humi[AUSSEN]!=INVALID
                && humi[GARAGE]>500 && taup[AUSSEN]+10<taup[GARAGE] && (temp[GARAGE]>100 || temp[AUSSEN]>temp[GARAGE]);
        p = text(buffer,"\t");
        for (sht=0; sht<SHTS; sht++) {
            if (temp[sht]!=INVALID) {
                p = text(p,"&t");
                *p++ = sht+'0';
                *p = 0;
                p = text(p,"t=");
                p = dec(p,temp[sht],10);
            }            
            if (humi[sht]!=INVALID) {
                p = text(p,"&t");
                *p++ = sht+'0';
                *p = 0;
                p = text(p,"f=");
                p = dec(p,humi[sht],10);
            }            
        }       
        p = text(p,"&fan=");
        *p++ = fan+'0';
        uint8_t crc = 0;
        for (char* p2=buffer+1; p2<p; p2++) {
            crc = crcu(crc,*p2);
        }
        *p++ = '\r';
        p = hex8(p,crc);
        *p++ = '\n';
        *p = 0;
        ser_writes(buffer);

        setCursor(COL(1)*FONTWIDTH,ROW(LUFTER)*FONTHEIGHT);
        if (fan && !(PORTB&FANPINB)) {
//            /*
            setTextColor(YLW,BG);
            print("Anlauf");
            for (uint16_t cyc=0;cyc<60000;cyc++) {
                PORTB |= FANPINB;
                _delay_us(2);
                PORTB &= ~FANPINB;
                _delay_us(8);
            }
            for (uint16_t cyc=0;cyc<60000;cyc++) {
                PORTB |= FANPINB;
                _delay_us(3);
                PORTB &= ~FANPINB;
                _delay_us(7);
            }
            for (uint16_t cyc=0;cyc<60000;cyc++) {
                PORTB |= FANPINB;
                _delay_us(4);
                PORTB &= ~FANPINB;
                _delay_us(6);
            }
            for (uint16_t cyc=0;cyc<60000;cyc++) {
                PORTB |= FANPINB;
                _delay_us(7);
                PORTB &= ~FANPINB;
                _delay_us(3);
            }
/*            
            for (uint16_t cyc=0;cyc<200;cyc++) {
                PORTB |= FANPINB;
                _delay_us(200);
                PORTB &= ~FANPINB;
                _delay_us(500);
            }
            for (uint16_t cyc=0;cyc<300;cyc++) {
                PORTB |= FANPINB;
                _delay_us(300);
                PORTB &= ~FANPINB;
                _delay_us(500);
            }
*/
//            */
            PORTB |= FANPINB;
            setTextColor(RED,BG);
            setCursor(COL(1)*FONTWIDTH,ROW(LUFTER)*FONTHEIGHT);
            print("An    ");
        } else if (!fan && (PORTB&FANPINB)){
            PORTB &= ~FANPINB;
            setTextColor(GRN,BG);
            print("Aus");
        }
        _delay_ms(1000);
    }    
}
