#ifndef __INC_FASTSPI_XTENSA_ESP8266_H
#define __INC_FASTSPI_XTENSA_ESP8266_H

FASTLED_NAMESPACE_BEGIN

// ESP8266 SPI code was copied/modified from implementation by David Ogilvy (MetalPhreak)
// spi frequence function from esp8266_peri.h writte by Hristo Gochkov for esp8266-Arduino

//Expansion Macros
#define spi_busy(spi_no) READ_PERI_REG(SPI_CMD(spi_no))&SPI_USR
#define ESP8266_REG(addr) *((volatile uint32_t *)(0x60000000+(addr)))
#define SPI1CLK 	ESP8266_REG(0x118)
#define GPMUX  ESP8266_REG(0x800)

#define spi_txd(spi_no, bits, data) spi_transaction(spi_no, 0, 0, 0, 0, bits, (uint32) data, 0, 0)
#define spi_tx8(spi_no, data)       spi_transaction(spi_no, 0, 0, 0, 0, 8,    (uint32) data, 0, 0)
#define spi_tx16(spi_no, data)      spi_transaction(spi_no, 0, 0, 0, 0, 16,   (uint32) data, 0, 0)
#define spi_tx32(spi_no, data)      spi_transaction((uint8 )spi_no, (uint8 )0, (uint16) 0, (uint32) 0, (uint32)0, (uint32)32,   (uint32) data, (uint32)0, (uint32)0)

#define spi_rxd(spi_no, bits) spi_transaction(spi_no, 0, 0, 0, 0, 0, 0, bits, 0)
#define spi_rx8(spi_no)       spi_transaction(spi_no, 0, 0, 0, 0, 0, 0, 8,    0)
#define spi_rx16(spi_no)      spi_transaction(spi_no, 0, 0, 0, 0, 0, 0, 16,   0)
#define spi_rx32(spi_no)      spi_transaction(spi_no, 0, 0, 0, 0, 0, 0, 32,   0)

typedef union {
        uint32_t regValue;
        struct {
                unsigned regL :6;
                unsigned regH :6;
                unsigned regN :6;
                unsigned regPre :13;
                unsigned regEQU :1;
        };
} spiClk_t;

// A skeletal implementation of hardware SPI support.  Fill in the necessary code for init, waiting, and writing.  The rest of
// the method implementations should provide a starting point, even if not hte most efficient to start with
// _SPI_CLOCK_DIVIDER should be uint32_t so we can input the desired frequence here...
template <uint8_t _DATA_PIN, uint8_t _CLOCK_PIN, uint8_t _SPI_CLOCK_DIVIDER>
class XTENSAHardwareSPIOutput {
	Selectable *m_pSelect;
	
	template<int BITS> static inline void writeBits(uint16_t w) {
		writeWord(w);
	}

public:
	XTENSAHardwareSPIOutput() { m_pSelect = NULL; }
	XTENSAHardwareSPIOutput(Selectable *pSelect) { m_pSelect = pSelect; }

	// set the object representing the selectable
	void setSelect(Selectable *pSelect) { /* TODO */ }

	// initialize the SPI subssytem
	void init() { 		
		// init		
		WRITE_PERI_REG(PERIPHS_IO_MUX, 0x105); // |(XTENSA_SPI_CLK_DIV <<9) Set bit 9 if 80MHz sysclock required
		PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, 2); //GPIO12 is HSPI MISO pin (Master Data In)
		PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, 2); //GPIO13 is HSPI MOSI pin (Master Data Out)
		PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTMS_U, 2); //GPIO14 is HSPI CLK pin (Clock)
		PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U, 2); //GPIO15 is HSPI CS pin (Chip Select / Slave Select)

		// setclock
		// Assuming macro DATA_RATE_MHZ(x) was used, DATA_RATE_KHZ should overflow uint8_t? bug in FL?
		// calculate backward to get the original frequence X in DATA_RATE_MHZ(x)
		uint32_t spiSpeed = CPU_CLK_FREQ/_SPI_CLOCK_DIVIDER;
//		os_printf("setFrequency(%d)\n", spiSpeed);

		setFrequency(spiSpeed);
		/*
#if SPI_CLK_PREDIV == 0	&& SPI_CLK_CNTDIV == 0
		WRITE_PERI_REG(SPI_CLK(XTENSA_SPI_TYPE), SPI_CLK_EQU_SYSCLK);
#else
		WRITE_PERI_REG(SPI_CLK(XTENSA_SPI_TYPE), 
					(((SPI_CLK_PREDIV-1)&SPI_CLKDIV_PRE)<<SPI_CLKDIV_PRE_S)|
					(((SPI_CLK_CNTDIV-1)&SPI_CLKCNT_N)<<SPI_CLKCNT_N_S)|
					(((SPI_CLK_CNTDIV>>1)&SPI_CLKCNT_H)<<SPI_CLKCNT_H_S)|
					((0&SPI_CLKCNT_L)<<SPI_CLKCNT_L_S));
#endif
*/

		// set tx byteorder
#if SPI_BYTE_ORDER == 1
		SET_PERI_REG_MASK(SPI_USER(XTENSA_SPI_TYPE), SPI_WR_BYTE_ORDER);
#else
		CLEAR_PERI_REG_MASK(SPI_USER(XTENSA_SPI_TYPE), SPI_WR_BYTE_ORDER);
#endif		

		// set rx byteorder
#if SPI_BYTE_ORDER == 1
		SET_PERI_REG_MASK(SPI_USER(XTENSA_SPI_TYPE), SPI_RD_BYTE_ORDER);
#else		
		CLEAR_PERI_REG_MASK(SPI_USER(XTENSA_SPI_TYPE), SPI_RD_BYTE_ORDER);
#endif

		SET_PERI_REG_MASK(SPI_USER(XTENSA_SPI_TYPE), SPI_CS_SETUP|SPI_CS_HOLD);
		CLEAR_PERI_REG_MASK(SPI_USER(XTENSA_SPI_TYPE), SPI_FLASH_MODE);
	}

	// latch the CS select
	void inline select() __attribute__((always_inline)) { if(m_pSelect != NULL) { m_pSelect->select(); } }

	// release the CS select
	void inline release() __attribute__((always_inline)) { if(m_pSelect != NULL) { m_pSelect->release(); } }

	// wait until all queued up data has been written
	static void waitFully() { 
		while(spi_busy(XTENSA_SPI_TYPE));	//wait for SPI transaction to complete	
	}

	// write a byte out via SPI (returns immediately on writing register)
	static void writeByte(uint8_t b) { 
		uint32_t dout_data = (uint32_t) b;
//		spi_transaction(spi_no, 0, 0, 0, 0, 8,    (uint32) data, 0, 0)
		//code for custom Chip Select as GPIO PIN here
	
		while(spi_busy(XTENSA_SPI_TYPE)); //wait for SPI to be ready	
	
		//########## Enable SPI Functions ##########//
		//disable MOSI, MISO, ADDR, COMMAND, DUMMY in case previously set.
		CLEAR_PERI_REG_MASK(SPI_USER(XTENSA_SPI_TYPE), SPI_USR_MOSI|SPI_USR_MISO|SPI_USR_COMMAND|SPI_USR_ADDR|SPI_USR_DUMMY);
	
		//########## END SECTION ##########//
	
		//########## Setup Bitlengths ##########//
		WRITE_PERI_REG(SPI_USER1(XTENSA_SPI_TYPE), ((-1)&SPI_USR_ADDR_BITLEN)<<SPI_USR_ADDR_BITLEN_S | //Number of bits in Address
										  ((8-1)&SPI_USR_MOSI_BITLEN)<<SPI_USR_MOSI_BITLEN_S | //Number of bits to Send
										  ((-1)&SPI_USR_MISO_BITLEN)<<SPI_USR_MISO_BITLEN_S |  //Number of bits to receive
										  ((-1)&SPI_USR_DUMMY_CYCLELEN)<<SPI_USR_DUMMY_CYCLELEN_S); //Number of Dummy bits to insert
		//########## END SECTION ##########//
	
			
	
	
		//########## Setup DOUT data ##########//
		SET_PERI_REG_MASK(SPI_USER(XTENSA_SPI_TYPE), SPI_USR_MOSI); //enable MOSI function in SPI module
		//copy data to W0
		WRITE_PERI_REG(SPI_W0(XTENSA_SPI_TYPE), dout_data <<(24));
		//########## END SECTION ##########//
	
		//########## Begin SPI Transaction ##########//
		SET_PERI_REG_MASK(SPI_CMD(XTENSA_SPI_TYPE), SPI_USR);
		//########## END SECTION ##########//
		
		//Transaction completed
	}

	// write a word out via SPI (returns immediately on writing register)
	static void writeWord(uint16_t w) { 
		uint32_t dout_data = (uint32_t) w;	
		//code for custom Chip Select as GPIO PIN here
	
		while(spi_busy(XTENSA_SPI_TYPE)); //wait for SPI to be ready	
	
	//########## Enable SPI Functions ##########//
		//disable MOSI, MISO, ADDR, COMMAND, DUMMY in case previously set.
		CLEAR_PERI_REG_MASK(SPI_USER(XTENSA_SPI_TYPE), SPI_USR_MOSI|SPI_USR_MISO|SPI_USR_COMMAND|SPI_USR_ADDR|SPI_USR_DUMMY);	
	//########## END SECTION ##########//
	
	//########## Setup Bitlengths ##########//
		WRITE_PERI_REG(SPI_USER1(XTENSA_SPI_TYPE), ((-1)&SPI_USR_ADDR_BITLEN)<<SPI_USR_ADDR_BITLEN_S | //Number of bits in Address
										  ((16-1)&SPI_USR_MOSI_BITLEN)<<SPI_USR_MOSI_BITLEN_S | //Number of bits to Send
										  ((0-1)&SPI_USR_MISO_BITLEN)<<SPI_USR_MISO_BITLEN_S |  //Number of bits to receive
										  ((-1)&SPI_USR_DUMMY_CYCLELEN)<<SPI_USR_DUMMY_CYCLELEN_S); //Number of Dummy bits to insert
	//########## END SECTION ##########//
	
		
	//########## Setup DOUT data ##########//
		SET_PERI_REG_MASK(SPI_USER(XTENSA_SPI_TYPE), SPI_USR_MOSI); //enable MOSI function in SPI module
		//copy data to W0
		WRITE_PERI_REG(SPI_W0(XTENSA_SPI_TYPE), dout_data <<(16));
	//########## END SECTION ##########//
	
	//########## Begin SPI Transaction ##########//
		SET_PERI_REG_MASK(SPI_CMD(XTENSA_SPI_TYPE), SPI_USR);
	//########## END SECTION ##########//
	}

	// A raw set of writing byte values, assumes setup/init/waiting done elsewhere
	static void writeBytesValueRaw(uint8_t value, int len) {
		while(len--) { writeByte(value); }
	}

	// A full cycle of writing a value for len bytes, including select, release, and waiting
	void writeBytesValue(uint8_t value, int len) {
		select(); writeBytesValueRaw(value, len); release();
	}

	// A full cycle of writing a value for len bytes, including select, release, and waiting
	template <class D> void writeBytes(register uint8_t *data, int len) {
		uint8_t *end = data + len;
		select();
		// could be optimized to write 16bit words out instead of 8bit bytes
		while(data != end) {
			writeByte(D::adjust(*data++));
		}
		D::postBlock(len);
		waitFully();
		release();
	}

	// A full cycle of writing a value for len bytes, including select, release, and waiting
	void writeBytes(register uint8_t *data, int len) { writeBytes<DATA_NOP>(data, len); }

	// write a single bit out, which bit from the passed in byte is determined by template parameter
	template <uint8_t BIT> inline static void writeBit(uint8_t b) { 
		// not implemented for now, only used by smd16716??
	}
	
	// write a block of uint8_ts out in groups of three.  len is the total number of uint8_ts to write out.  The template
	// parameters indicate how many uint8_ts to skip at the beginning and/or end of each grouping
	template <uint8_t FLAGS, class D, EOrder RGB_ORDER> void writePixels(PixelController<RGB_ORDER> pixels) {
		select();
		int len = pixels.mLen;

		if(FLAGS & FLAG_START_BIT) {
			while(pixels.has(1)) {
				writeBits<9>((1<<8) | D::adjust(pixels.loadAndScale0()));
				writeByte(D::adjust(pixels.loadAndScale1()));
				writeByte(D::adjust(pixels.loadAndScale2()));
				pixels.advanceData();
				pixels.stepDithering();
			}
		} else {
			while(pixels.has(1)) {
				writeByte(D::adjust(pixels.loadAndScale0()));
				writeByte(D::adjust(pixels.loadAndScale1()));
				writeByte(D::adjust(pixels.loadAndScale2()));
				pixels.advanceData();
				pixels.stepDithering();
			}
		}
		D::postBlock(len);
		release();
	}

	/**
	 * calculate the Frequency based on the register value
	 * @param reg
	 * @return
	 */
	static uint32_t ClkRegToFreq(spiClk_t * reg) {
	    return (CPU_CLK_FREQ / ((reg->regPre + 1) * (reg->regN + 1)));
	}

	void setFrequency(uint32_t freq) {
//		os_printf("setFrequency(uint32_t freq)\n");
	    static uint32_t lastSetFrequency = 0;
	    static uint32_t lastSetRegister = 0;
	
	    if(freq >= CPU_CLK_FREQ) {
	        setClockDivider(0x80000000);
	        return;
	    }
	
	    if(lastSetFrequency == freq && lastSetRegister == SPI1CLK) {
	        // do nothing (speed optimization)
	        return;
	    }
//		os_printf("setFrequency 1\n");
	
	    const spiClk_t minFreqReg = { 0x7FFFF000 };
	    uint32_t minFreq = ClkRegToFreq((spiClk_t*) &minFreqReg);
	    if(freq < minFreq) {
	        // use minimum possible clock
	        setClockDivider(minFreqReg.regValue);
	        lastSetRegister = SPI1CLK;
	        lastSetFrequency = freq;
	        return;
	    }
//		os_printf("setFrequency 2\n");
	
	    uint8_t calN = 1;
	
	    spiClk_t bestReg = { 0 };
	    int32_t bestFreq = 0;
	
	    // find the best match
	    while(calN <= 0x3F) { // 0x3F max for N
	
//		os_printf("setFrequency 3\n");
	        spiClk_t reg = { 0 };
	        int32_t calFreq;
	        int32_t calPre;
	        int8_t calPreVari = -2;
	
	        reg.regN = calN;
	
	        while(calPreVari++ <= 1) { // test different variants for Pre (we calculate in int so we miss the decimals, testing is the easyest and fastest way)
//	        		os_printf("setFrequency 4\n");

	            calPre = (((CPU_CLK_FREQ / (reg.regN + 1)) / freq) - 1) + calPreVari;
	            if(calPre > 0x1FFF) {
	                reg.regPre = 0x1FFF; // 8191
	            } else if(calPre <= 0) {
	                reg.regPre = 0;
	            } else {
	                reg.regPre = calPre;
	            }
	
	            reg.regL = ((reg.regN + 1) / 2);
	            // reg.regH = (reg.regN - reg.regL);
	
	            // test calculation
	            calFreq = ClkRegToFreq(&reg);
//	            os_printf("-----[0x%08X][%d]\t EQU: %d\t Pre: %d\t N: %d\t H: %d\t L: %d = %d\n", reg.regValue, freq, reg.regEQU, reg.regPre, reg.regN, reg.regH, reg.regL, calFreq);
	
	            if(calFreq == (int32_t) freq) {
	                // accurate match use it!
	                memcpy(&bestReg, &reg, sizeof(bestReg));
	                break;
	            } else if(calFreq < (int32_t) freq) {
	                // never go over the requested frequency
	                if(abs(freq - calFreq) < abs(freq - bestFreq)) {
	                    bestFreq = calFreq;
	                    memcpy(&bestReg, &reg, sizeof(bestReg));
	                }
	            }
	        }
	        if(calFreq == (int32_t) freq) {
	            // accurate match use it!
	            break;
	        }
	        calN++;
	    }
	
//	    os_printf("[0x%08X][%d]\t EQU: %d\t Pre: %d\t N: %d\t H: %d\t L: %d\t - Real Frequency: %d\n", bestReg.regValue, freq, bestReg.regEQU, bestReg.regPre, bestReg.regN, bestReg.regH, bestReg.regL, ClkRegToFreq(&bestReg));
	
	    setClockDivider(bestReg.regValue);
	    lastSetRegister = SPI1CLK;
	    lastSetFrequency = freq;
	
	}

	void setClockDivider(uint32_t clockDiv) {
	    if(clockDiv == 0x80000000) {
	        GPMUX |= (1 << 9); // Set bit 9 if sysclock required
	    } else {
	        GPMUX &= ~(1 << 9);
	    }
	    SPI1CLK = clockDiv;
	}
};

FASTLED_NAMESPACE_END

#endif
