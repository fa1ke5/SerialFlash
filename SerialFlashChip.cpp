;/* SerialFlash Library - for filesystem-like access to SPI Serial Flash memory
 * https://github.com/PaulStoffregen/SerialFlash
 * Copyright (C) 2015, Paul Stoffregen, paul@pjrc.com
 *
 * Development of this library was funded by PJRC.COM, LLC by sales of Teensy.
 * Please support PJRC's efforts to develop open source software by purchasing
 * Teensy or other genuine PJRC products.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */


#include "SerialFlash.h"
#include "util/SerialFlash_directwrite.h"

#define CSASSERT()  DIRECT_WRITE_LOW(cspin_basereg, cspin_bitmask)
#define CSRELEASE() DIRECT_WRITE_HIGH(cspin_basereg, cspin_bitmask)
#define SPICONFIG   SPISettings(26000000, MSBFIRST, SPI_MODE0)

uint16_t SerialFlashChip::dirindex = 0;
uint8_t SerialFlashChip::flags = 0;
uint8_t SerialFlashChip::busy = 0;

static volatile IO_REG_TYPE *cspin_basereg;
static IO_REG_TYPE cspin_bitmask;

static SPIClass& SPIPORT = SPI;

#define FLAG_32BIT_ADDR		0x01	// larger than 16 MByte address
#define FLAG_STATUS_CMD70	0x02	// requires special busy flag check
#define FLAG_DIFF_SUSPEND	0x04	// uses 2 different suspend commands
#define FLAG_MULTI_DIE		0x08	// multiple die, don't read cross 32M barrier
#define FLAG_256K_BLOCKS	0x10	// has 256K erase blocks
#define FLAG_DIE_MASK		0xC0	// top 2 bits count during multi-die erase

#define CMD_PAGE_DATA_READ		0x13

#define CMD_READ					0x03
#define CMD_FAST_READ				0x0B
#define CMD_PROG_DATA_WRITE			0x02 //write to internal block buffer (2048 bytes + 64ECC)
#define CMD_RAND_PROG_DATA_WRITE	0x84
#define CMD_PROG_EXECUTE			0x10 //write internal block buffer to array
#define CMD_SOFT_DIE_SELECT   		0xC2
#define CMD_RESET_DEVICE   			0xFF  
#define CMD_BLOCK128K_ERASE  		0xD8
#define CMD_BADBLOCK_MANAGE  		0xA1
#define CMD_READ_BADBLOCK_TABLE 	0xA5


bool DIE_NAND = true;    // Set true or get detect function first if you use WINBOND SPI serial NAND W25M02GV
int Act_Die = 0;   //Current active DIE
uint16_t block = 0;
uint8_t page6 = 0;
uint8_t page16 = 0;
uint16_t column = 0;
uint16_t MaxColumn = 2048;
uint16_t Curr_Page = 0; //In buffer page address

bool PageRenew = true;
bool BuffRDReady = true;
bool BuffWRReady = false;

void SerialFlashChip::reset(void)
{ //Software esetting only for  W25M0xGV series NAND!!!!!
		SPI.beginTransaction(SPICONFIG);
		CSASSERT();
		SPIPORT.transfer(CMD_RESET_DEVICE); //up to 500uS wait to become ready after this
		CSRELEASE();
		SPIPORT.endTransaction();
}

void SerialFlashChip::wait(void)
{
	uint32_t status;
	//Serial.print("wait-");
	while (1) {
		SPIPORT.beginTransaction(SPICONFIG);
		CSASSERT();
		// all others work by simply reading the status reg
		SPIPORT.transfer(0x05);
		SPIPORT.transfer(0xC0); //Status register addr for NAND
        status = SPIPORT.transfer(0);
		CSRELEASE();
		SPIPORT.endTransaction();
     //   Serial.printf("b=%02x.", status & 0xFF);
		if (!(status & 1)) break;
	}
	busy = 0;
	
}

void SerialFlashChip::read(uint32_t addr, void *buf, uint32_t len)
{



  uint8_t *p = (uint8_t *)buf;
  uint8_t b, f, status, cmd;
uint32_t rdlen = 0;
  memset(p, 0, len);

    status = 1;
	SPIPORT.beginTransaction(SPICONFIG);
	CSASSERT();
	SPIPORT.transfer(0x05);
	SPIPORT.transfer(0xC0);
	
while (busy){
busy = SPIPORT.transfer(0) & 1;
}
CSRELEASE();
SPIPORT.endTransaction();

  do {
 //       if (DIE_NAND)
 //          { //W25M0XGV series NAND read

             column = addr & 0x7FF; 
             uint32_t maxlen = 2048 - column; 
             rdlen = (len <= maxlen) ? len : maxlen; 
             
             if ((addr >> 27) != Act_Die)
              { //Change active DIE acording to address
              
              Serial.print("Current DIE = ");Serial.println(Act_Die);
              Serial.print("need DIE = ");Serial.print(addr >> 27);Serial.print(" changing..");
              
              
              Act_Die = addr >> 27;
				SPIPORT.beginTransaction(SPICONFIG);
				CSASSERT();
				SPIPORT.transfer(CMD_SOFT_DIE_SELECT); 
				SPIPORT.transfer(Act_Die);   
				CSRELEASE();
				SPIPORT.endTransaction();
				
				delayMicroseconds(1);

              } else if ((addr >> 11) != Curr_Page || PageRenew)

            {   //Load page in internal buffer if need
					SPIPORT.beginTransaction(SPICONFIG);
                    CSASSERT();
                    SPIPORT.transfer(CMD_PAGE_DATA_READ);
                    SPIPORT.transfer(0); //Dummy byte
		            SPIPORT.transfer16(addr >> 11);
                    CSRELEASE();
                    SPIPORT.endTransaction();
                    
					Curr_Page = addr >> 11;
					PageRenew = false;
				//	delayMicroseconds(60);
					busy = 1; //???????????????????????????????????????????????
					if (busy) {
								wait();
							//	Serial.println("Wait...."); ///?????????????????????????????
								}
			} else { 
			
		//	SPIPORT.beginTransaction(SPICONFIG);

			
			
			}
 //   SPIPORT.beginTransaction(SPICONFIG); 
 SPIPORT.beginTransaction(SPICONFIG);
    CSASSERT();
   SPIPORT.transfer(CMD_READ);
  //  SPIPORT.transfer(CMD_FAST_READ);
    SPIPORT.transfer16(addr & 0x7FF);
    SPIPORT.transfer(0);//Dummy byte
        
//    }

    SPIPORT.transfer(p, rdlen);
    CSRELEASE();
    SPIPORT.endTransaction();
    p += rdlen;
    addr += rdlen;
    len -= rdlen;


  } while (len > 0);

  SPIPORT.endTransaction();

         					busy = 1; //???????????????????????????????????????????????
							if (busy) {
								wait();
							//	Serial.println("Wait...."); ///?????????????????????????????
								}   


}

void SerialFlashChip::write(uint32_t addr, const void *buf, uint32_t len)
{
  const uint8_t *p = (const uint8_t *)buf;
  uint32_t max, pagelen;
  //Serial.printf("WR: addr %08X, len %d\n", addr, len);

  do {
    if (busy) wait();

max = 2048 - (addr & 0x7FF); //248
pagelen = (len <= max) ? len : max; 

     //Serial.printf("WR: addr %08X, pagelen %d\n", addr, pagelen);

              if ((addr >> 27) != Act_Die)
                  { //Select DIE
                  
              Serial.print("Current DIE = ");Serial.println(Act_Die);
              Serial.print("need DIE = ");Serial.println(addr >> 27);

					Act_Die = addr >> 27;
					SPI.beginTransaction(SPICONFIG);
					CSASSERT();
					SPI.transfer(CMD_SOFT_DIE_SELECT); 
					SPI.transfer(Act_Die);   
					CSRELEASE();
					SPI.endTransaction();
					delayMicroseconds(1);
                  }
					SPI.beginTransaction(SPICONFIG);
					CSASSERT();
                    SPI.transfer(CMD_PAGE_DATA_READ);
                    SPI.transfer(0); //Dummy byte
                    SPI.transfer16(addr >> 11);
                    CSRELEASE();
		            SPI.endTransaction();
                    delayMicroseconds(50);
					Curr_Page = addr >> 11;
              //    }
                  
                  
			SPI.beginTransaction(SPICONFIG);
			
			CSASSERT(); 
			SPI.transfer(0x06); // write enable command
			CSRELEASE();
			
			SPI.endTransaction();
			delayMicroseconds(1);
			
			SPI.beginTransaction(SPICONFIG);
			
			CSASSERT();
			SPIPORT.transfer(CMD_RAND_PROG_DATA_WRITE);// random program buffer page command
            SPIPORT.transfer16(addr & 0x7FF);

    addr += pagelen;
    len -= pagelen;
    do {
      SPIPORT.transfer(*p++);
    } while (--pagelen > 0);
    
    
    CSRELEASE();

    
    
    //*********************
CSASSERT();
    SPIPORT.transfer(CMD_PROG_EXECUTE);
    SPIPORT.transfer(0); //Dummy byte
    SPIPORT.transfer16((addr - 1) >> 11);
    CSRELEASE();
    SPIPORT.endTransaction();
    delayMicroseconds(50);
//****************************
    PageRenew = true;
    busy = 4;
    
  } while (len > 0);




PageRenew = true;
}

void SerialFlashChip::eraseAll()
{
Serial.println("Init erase ALL prog....");
	if (busy) wait();
	uint8_t id[5];
	readID(id);
	//Serial.printf("ID: %02X %02X %02X\n", id[0], id[1], id[2]);
	if (id[0] == 0x20 && id[2] >= 0x20 && id[2] <= 0x22) {

		// Micron's multi-die chips require special die erase commands
		//  N25Q512A	20 BA 20  2 dies  32 Mbyte/die   65 nm transitors
		//  N25Q00AA	20 BA 21  4 dies  32 Mbyte/die   65 nm transitors
		//  MT25QL02GC	20 BA 22  2 dies  128 Mbyte/die  45 nm transitors
		uint8_t die_count = 2;
		if (id[2] == 0x21) die_count = 4;
		uint8_t die_index = flags >> 6;
		 //Serial.printf("Micron die erase %d\n", die_index);
		flags &= 0x3F;
		if (die_index >= die_count) return; // all dies erased :-)
		uint8_t die_size = 2;  // in 16 Mbyte units
		if (id[2] == 0x22) die_size = 8;
		SPIPORT.beginTransaction(SPICONFIG);
		CSASSERT();
		SPIPORT.transfer(0x06); // write enable command
		CSRELEASE();
		 delayMicroseconds(1);
		CSASSERT();
		// die erase command
		SPIPORT.transfer(0xC4);
		SPIPORT.transfer16((die_index * die_size) << 8);
		SPIPORT.transfer16(0x0000);
		CSRELEASE();
		 //Serial.printf("Micron erase begin\n");
		flags |= (die_index + 1) << 6;
	} else if (DIE_NAND){

for(uint16_t i = 0; i < 2048; i++){

uint32_t address = i*64;
eraseBlock(address);


}
        
        } else {

		// All other chips support the bulk erase command
		SPIPORT.beginTransaction(SPICONFIG);
		CSASSERT();
		// write enable command
		SPIPORT.transfer(0x06);
		CSRELEASE();
		 delayMicroseconds(1);
		CSASSERT();
		// bulk erase command
		SPIPORT.transfer(0xC7);
		CSRELEASE();
		SPIPORT.endTransaction();
	}
	busy = 3;
}


void SerialFlashChip::eraseBlock(uint32_t addr)
{
  uint8_t f = flags;
  if (busy) wait();

  SPI.beginTransaction(SPICONFIG);
  CSASSERT();  
  SPI.transfer(0x06); // write enable command
  CSRELEASE(); 
  SPI.endTransaction();

  if (DIE_NAND){
   	if (addr >> 16 != Act_Die)
		{ //Select DIE
		Serial.print("Current DIE = ");Serial.println(Act_Die);
              	Serial.print("need DIE = ");Serial.println(addr >> 16);
		Act_Die = addr >> 16;
	        SPI.beginTransaction(SPICONFIG);
		CSASSERT();  
		SPI.transfer(CMD_SOFT_DIE_SELECT); 
		SPI.transfer(Act_Die);   
		CSRELEASE();
		SPI.endTransaction();
        	//push write enable command
        	SPI.beginTransaction(SPICONFIG);
        	CSASSERT();
        	SPI.transfer(0x06); // write enable command
   		CSRELEASE();
      		SPI.endTransaction();
    		}

	SPI.beginTransaction(SPICONFIG);
    	CSASSERT();
    	SPI.transfer(CMD_BLOCK128K_ERASE);
    	SPI.transfer(0);//Dummy
    	SPI.transfer16(addr & 0xFFFF);

    
    	     }
 
  CSRELEASE();
  SPI.endTransaction();
  busy = 1;
  PageRenew = true;
}
/*
void SerialFlashChip::eraseBlock(uint32_t addr)
{
	uint8_t f = flags;
	if (busy) wait();
	SPIPORT.beginTransaction(SPICONFIG);
	CSASSERT();
	SPIPORT.transfer(0x06); // write enable command
	CSRELEASE();
	 delayMicroseconds(1);
	CSASSERT();
	if (f & FLAG_32BIT_ADDR) {
		SPIPORT.transfer(0xD8);
		SPIPORT.transfer16(addr >> 16);
		SPIPORT.transfer16(addr);
	} else {
		SPIPORT.transfer16(0xD800 | ((addr >> 16) & 255));
		SPIPORT.transfer16(addr);
	}
	CSRELEASE();
	SPIPORT.endTransaction();
	busy = 2;
}
*/


bool SerialFlashChip::ready()
{
	uint32_t status;
	if (!busy) return true;
	SPIPORT.beginTransaction(SPICONFIG);
	CSASSERT();
	if (flags & FLAG_STATUS_CMD70) {
		// some Micron chips require this different
		// command to detect program and erase completion
		SPIPORT.transfer(0x70);
		status = SPIPORT.transfer(0);
		CSRELEASE();
		SPIPORT.endTransaction();
		//Serial.printf("ready=%02x\n", status & 0xFF);
		if ((status & 0x80) == 0) return false;
	} else {
		// all others work by simply reading the status reg
		SPIPORT.transfer(0x05);
                if(DIE_NAND) SPIPORT.transfer(0xC0); //status register address C0 for NAND
		status = SPIPORT.transfer(0);
		CSRELEASE();
		SPIPORT.endTransaction();
                //Serial.printf("ready=%02x\n", status & 0xFF);
		if ((status & 1)) return false;
	}
	busy = 0;
	if (flags & 0xC0) {
		// continue a multi-die erase
Serial.println("Erase All function start.....");
		eraseAll();
		return false;
	}
	return true;
}


#define ID0_WINBOND	0xEF
#define ID0_SPANSION	0x01
#define ID0_MICRON	0x20
#define ID0_MACRONIX	0xC2
#define ID0_SST		0xBF

#define ID1_WIN_NAND	0xAB
#define ID2_WIN_NAND	0x21

//#define FLAG_32BIT_ADDR	0x01	// larger than 16 MByte address
//#define FLAG_STATUS_CMD70	0x02	// requires special busy flag check
//#define FLAG_DIFF_SUSPEND	0x04	// uses 2 different suspend commands
//#define FLAG_256K_BLOCKS	0x10	// has 256K erase blocks

bool SerialFlashChip::begin(SPIClass& device, uint8_t pin)
{
	SPIPORT = device;
	return begin(pin);
}

bool SerialFlashChip::begin(uint8_t pin)
{
	uint8_t id[5];
	uint8_t f;
	uint32_t size;

	cspin_basereg = PIN_TO_BASEREG(pin);
	cspin_bitmask = PIN_TO_BITMASK(pin);
	SPIPORT.begin();
	pinMode(pin, OUTPUT);
	CSRELEASE();
	readID(id);
//	f = 0;
//		f |= FLAG_32BIT_ADDR;
	//	SPIPORT.beginTransaction(SPICONFIG);
/*
	if (size > 16777216) {
		// more than 16 Mbyte requires 32 bit addresses
		f |= FLAG_32BIT_ADDR;
		SPIPORT.beginTransaction(SPICONFIG);
		if (id[0] == ID0_SPANSION) {
			// spansion uses MSB of bank register
			CSASSERT();
			SPIPORT.transfer16(0x1780); // bank register write
			CSRELEASE();
		} else {
			// micron & winbond & macronix use command
			CSASSERT();
			SPIPORT.transfer(0x06); // write enable
			CSRELEASE();
			delayMicroseconds(1);

			CSASSERT();
			SPIPORT.transfer(0xB7); // enter 4 byte addr mode
			CSRELEASE();

		} 

		SPIPORT.endTransaction();
		if (id[0] == ID0_MICRON) f |= FLAG_MULTI_DIE;
	}*/
	if (id[0] == ID0_SPANSION) {
		// Spansion has separate suspend commands
		f |= FLAG_DIFF_SUSPEND;
		if (!id[4]) {
			// Spansion chips with id[4] == 0 use 256K sectors
			f |= FLAG_256K_BLOCKS;
		}
	}
	if (id[0] == ID0_MICRON) {
		// Micron requires busy checks with a different command
		f |= FLAG_STATUS_CMD70; // TODO: all or just multi-die chips?
	}
	if (id[0] == ID0_WINBOND && id[1] == ID1_WIN_NAND && id[2] == ID2_WIN_NAND) {

//Change to first DIE
		    SPIPORT.beginTransaction(SPICONFIG);
		    CSASSERT();
                    SPI.transfer(CMD_SOFT_DIE_SELECT); 
              //     Serial.print(" Change DIE, sending SPI cmd CMD_SOFT_DIE_SELECT");
                    SPI.transfer(0x00);   
               //     Act_Die = 0;
                    CSRELEASE();
                    SPIPORT.endTransaction();
                    delayMicroseconds(50);
		//unlock for write
		Serial.println("Found Winbond NAND flash 1, unlocking ...");
		SPIPORT.beginTransaction(SPICONFIG);
  		CSASSERT();
  		SPIPORT.transfer(0x1F); //Write status reg
  		SPIPORT.transfer(0xA0);
  		SPIPORT.transfer(0x00);
  		CSRELEASE();
		SPIPORT.endTransaction();
		delayMicroseconds(50);

//Change to second DIE
		    SPIPORT.beginTransaction(SPICONFIG);
                    CSASSERT();
                    SPI.transfer(CMD_SOFT_DIE_SELECT); 
              //     Serial.print(" Change DIE, sending SPI cmd CMD_SOFT_DIE_SELECT");
                    SPI.transfer(0x01);   
              //      Act_Die = 1;
                    CSRELEASE();
                    SPIPORT.endTransaction();
                    delayMicroseconds(50);

		//unlock for write
		Serial.println("Found Winbond NAND flash 2, unlocking ...");
		SPIPORT.beginTransaction(SPICONFIG);
  		CSASSERT();
  		SPIPORT.transfer(0x1F);
  		SPIPORT.transfer(0xA0);
  		SPIPORT.transfer(0x00);
  		CSRELEASE();
		SPIPORT.endTransaction();
		delayMicroseconds(50);

//Change to first DIE
		    SPIPORT.beginTransaction(SPICONFIG);
		    CSASSERT();
                    SPI.transfer(CMD_SOFT_DIE_SELECT); 
              //     Serial.print(" Change DIE, sending SPI cmd CMD_SOFT_DIE_SELECT");
                    SPI.transfer(0x00);   
                    Act_Die = 0;
                    CSRELEASE();
                    SPIPORT.endTransaction();
                    delayMicroseconds(50);
	
	
	}
	flags = f;
	readID(id);
	return true;
}

// chips tested: https://github.com/PaulStoffregen/SerialFlash/pull/12#issuecomment-169596992
//
/*
void SerialFlashChip::sleep()
{
	if (busy) wait();
	SPIPORT.beginTransaction(SPICONFIG);
	CSASSERT();
	SPIPORT.transfer(0xB9); // Deep power down command
	CSRELEASE();
}

void SerialFlashChip::wakeup()
{
	SPIPORT.beginTransaction(SPICONFIG);
	CSASSERT();
	SPIPORT.transfer(0xAB); // Wake up from deep power down command
	CSRELEASE();
}
*/

void SerialFlashChip::readID(uint8_t *buf)
{
	if (busy) wait();
	SPI.beginTransaction(SPICONFIG);
	CSASSERT();
	SPI.transfer(0x9F);
	buf[0] = SPI.transfer(0); // manufacturer ID
	buf[1] = SPI.transfer(0); // memory type
if(!buf[0] && buf[1]){ //skip dummy byte for W25M02GVZEIG
buf[0] = buf [1];
buf[1] = SPI.transfer(0);
buf[2] = SPI.transfer(0);
DIE_NAND = true; 
}else {
	buf[2] = SPI.transfer(0); // capacity
	}


	CSRELEASE();
	SPI.endTransaction();
	//Serial.printf("ID: %02X %02X %02X\n", buf[0], buf[1], buf[2]);
}

void SerialFlashChip::readSerialNumber(uint8_t *buf) //needs room for 8 bytes
{
	if (busy) wait();
	SPIPORT.beginTransaction(SPICONFIG);
	CSASSERT();
	SPIPORT.transfer(0x4B);			
	SPIPORT.transfer16(0);	
	SPIPORT.transfer16(0);
	for (int i=0; i<8; i++) {		
		buf[i] = SPIPORT.transfer(0);
	}
	CSRELEASE();
	SPIPORT.endTransaction();
//	Serial.printf("Serial Number: %02X %02X %02X %02X %02X %02X %02X %02X\n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
}

uint32_t SerialFlashChip::capacity(const uint8_t *id)
{
	uint32_t n = 1048576; // unknown chips, default to 1 MByte

	if (id[2] >= 16 && id[2] <= 31) {
		n = 1ul << id[2];
	} else
	if (id[2] >= 32 && id[2] <= 37) {
		n = 1ul << (id[2] - 6);
	} else
	if ((id[0]==0 && id[1]==0 && id[2]==0) || 
		(id[0]==255 && id[1]==255 && id[2]==255)) {
		n = 0;
       	}

        if (id[0] == 0xEF && id[1] == 0xAB && id[2] == 0x21) {
		n = n*2; 
               
        }
	//Serial.printf("capacity %lu\n", n);
	return n;
}

uint32_t SerialFlashChip::blockSize()
{
	// Spansion chips >= 512 mbit use 256K sectors
	if (flags & FLAG_256K_BLOCKS) return 262144;
	// everything else seems to have 64K sectors
           if (DIE_NAND) return 2*65536;
	return 65536;
}



/*
Chip		Uniform Sector Erase
		20/21	52	D8/DC
		-----	--	-----
W25Q64CV	4	32	64
W25Q128FV	4	32	64
S25FL127S			64
N25Q512A	4		64
N25Q00AA	4		64
S25FL512S			256
SST26VF032	4
W25M02GVZEIG		128
*/



//			size	sector			busy	pgm/erase	chip
// Part			Mbyte	kbyte	ID bytes	cmd	suspend		erase
// ----			----	-----	--------	---	-------		-----
// Winbond W25Q64CV	8	64	EF 40 17
// Winbond W25Q128FV	16	64	EF 40 18	05	single		60 & C7
// Winbond W25Q256FV	32	64	EF 40 19

// Winbond W25M02GVZEIG	2x128	128	EF AB 21

	
// Spansion S25FL064A	8	?	01 02 16
// Spansion S25FL127S	16	64	01 20 18	05
// Spansion S25FL128P	16	64	01 20 18
// Spansion S25FL256S	32	64	01 02 19	05			60 & C7
// Spansion S25FL512S	64	256	01 02 20
// Macronix MX25L12805D 16	?	C2 20 18
// Macronix MX66L51235F	64		C2 20 1A
// Numonyx M25P128	16	?	20 20 18
// Micron M25P80	1	?	20 20 14
// Micron N25Q128A	16	64	20 BA 18
// Micron N25Q512A	64	?	20 BA 20	70	single		C4 x2
// Micron N25Q00AA	128	64	20 BA 21		single		C4 x4
// Micron MT25QL02GC	256	64	20 BA 22	70			C4 x2
// SST SST25WF010	1/8	?	BF 25 02
// SST SST25WF020	1/4	?	BF 25 03
// SST SST25WF040	1/2	?	BF 25 04
// SST SST25VF016B	1	?	BF 25 41
// SST26VF016			?	BF 26 01
// SST26VF032			?	BF 26 02
// SST25VF032		4	64	BF 25 4A
// SST26VF064		8	?	BF 26 43
// LE25U40CMC		1/2	64	62 06 13

SerialFlashChip SerialFlash;
