/*
  This is a library written for the AMS TMF-8801 Time-of-flight sensor
  SparkFun sells these at its website:
  https://www.sparkfun.com/products/17716

  Do you like this library? Help support open source hardware. Buy a board!

  Written by Ricardo Ramos  @ SparkFun Electronics, February 15th, 2021
  This file is the core of the TMF-8801 ToF sensor library.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include "SparkFun_TMF8801_Arduino_Library.h"

bool TMF8801::begin(byte address, TwoWire& wirePort)
{
	// Initialize the selected I2C interface 
	bool ready = tmf8801_io.begin(address, wirePort);
	
	// If the interface is not ready or TMF8801 is unreacheable return false
	if (ready == false)
	{
		lastError = ERROR_I2C_COMM_ERROR;
		return false;
	}

	// Reset TMF8801. Since it clears itself, we don't need to clear it
	tmf8801_io.setRegisterBit(REGISTER_ENABLE_REG, CPU_RESET);

	ready = cpuReady();
	if (ready == false)
	{
		lastError = ERROR_CPU_RESET_TIMEOUT;
		return false;
	}

	// Are we really talking to a TMF8801 ?
	byte value = tmf8801_io.readSingleByte(REGISTER_ID);
	if (value != CHIP_ID_NUMBER)
	{
		lastError = ERROR_WRONG_CHIP_ID;
		return false;
	}

	// Load the measurement application and wait until it's ready
	tmf8801_io.writeSingleByte(REGISTER_APPREQID, APPLICATION);
	ready = applicationReady();
	if (ready == false)
	{
		lastError = ERROR_CPU_LOAD_APPLICATION_ERROR;
		return false;
	}

	// Set calibration data
	tmf8801_io.writeSingleByte(REGISTER_COMMAND, COMMAND_CALIBRATION);
	tmf8801_io.writeMultipleBytes(REGISTER_FACTORY_CALIB_0, calibrationData, sizeof(calibrationData));
	tmf8801_io.writeMultipleBytes(REGISTER_STATE_DATA_WR_0, ALGO_STATE, sizeof(ALGO_STATE));

	// Configure the application - values were taken from AN0597, pp. 22
	updateCommandData8();	

	// Start the application
	tmf8801_io.writeSingleByte(REGISTER_COMMAND, COMMAND_MEASURE);

	delay(10);

	// Set lastError no NONE
	lastError = ERROR_NONE;
	return true;
}

bool TMF8801::cpuReady()
{
	short counter = 0;

	// Wait for CPU_READY_TIMEOUT mSec until TMF8801 is ready
	do
	{
		bool ready = tmf8801_io.isBitSet(REGISTER_ENABLE_REG, CPU_READY);
		if (ready == false)
		{
			counter++;
			delay(100);
		}
		else
		{
			return true;
		}
	} while (counter < CPU_READY_TIMEOUT);

	// If TMF8801 CPU is not ready, return false
	return false;
}

bool TMF8801::dataAvailable()
{
	// Returns true if REGISTER_CONTENTS is 0x55
	byte result = tmf8801_io.readSingleByte(REGISTER_REGISTER_CONTENTS);
	return result == COMMAND_RESULT;
}

bool TMF8801::isConnected()
{
	// Polls I2C interface
	bool twiConnected = tmf8801_io.isConnected();
	if (!twiConnected)
		return false;

	// Returns true if TMF8801 ID returned id is 0x07
	return (tmf8801_io.readSingleByte(REGISTER_ID) == CHIP_ID_NUMBER);
}

bool TMF8801::applicationReady()
{
	short counter = 0;

	// Wait for APPLICATION_READY_TIMEOUT mSec until TMF8801 is ready
	do
	{
		bool ready = (tmf8801_io.readSingleByte(REGISTER_APPID) == APPLICATION);
		if (ready == false)
		{
			counter++;
			delay(100);
		}
		else
		{
			return true;
		}
	} while (counter < APPLICATION_READY_TIMEOUT);

	// If application is not ready, return false
	return false;
}

byte TMF8801::getLastError()
{
	return lastError;
}

bool TMF8801::getCalibrationData(byte* calibrationResults)
{
	tmf8801_io.writeSingleByte(REGISTER_COMMAND, 0xff);
	delay(50);

	// Returns device's calibration data values (14 bytes)
	lastError = ERROR_NONE;
	uint32_t calibrationStart = millis();

	byte value;
	do
	{
		tmf8801_io.writeSingleByte(REGISTER_COMMAND, COMMAND_FACTORY_CALIBRATION);
		delay(10);
		value = tmf8801_io.readSingleByte(REGISTER_REGISTER_CONTENTS);
		if (value == CONTENT_CALIBRATION)
		{
			delay(10);
			tmf8801_io.readMultipleBytes(REGISTER_FACTORY_CALIB_0, calibrationResults, CALIBRATION_DATA_LENGTH);
			return true;
		}
		delay(50);
	} while (millis() - calibrationStart < 30000);
	
	// returns false and writes the lastError if TMF8801 calibration data read operation fails
	lastError = ERROR_FACTORY_CALIBRATION_ERROR;
	return false;
}

void TMF8801::setCalibrationData(const byte* newCalibrationData)
{
	// Copies passed array into calibrationData
	memcpy(calibrationData, newCalibrationData, CALIBRATION_DATA_LENGTH);

	// Reset device with updated values
	resetDevice();
}

byte TMF8801::getApplicationVersionMajor()
{
	return tmf8801_io.readSingleByte(REGISTER_APPREV_MAJOR);
}

byte TMF8801::getApplicationVersionMinor()
{
	return tmf8801_io.readSingleByte(REGISTER_APPREV_MINOR);
}

byte TMF8801::getHardwareVersion()
{
	return tmf8801_io.readSingleByte(REGISTER_REVID);
}

short TMF8801::getSerialNumber()
{
	short serial = 0;
	byte value[2];
	byte result;
	// Request serial number to device
	do
	{	tmf8801_io.writeSingleByte(REGISTER_COMMAND, COMMAND_SERIAL);
		delay(50);
		result = tmf8801_io.readSingleByte(REGISTER_REGISTER_CONTENTS);
		delay(10);
	} while (result != COMMAND_SERIAL);

	// Read two bytes and combine them as a single int
	tmf8801_io.readMultipleBytes(REGISTER_STATE_DATA_0, value, 2);
	serial = value[1];
	serial = serial << 8;
	serial |= value[0];
	return serial;
}

byte TMF8801::getMeasurementReliability()
{
	// Returns result info without measurement status bits
	return (resultInfo & 0x3f);
}

byte TMF8801::getMeasurementStatus()
{
	// returns resultInfo without measurement reliability bits
	return (resultInfo >> 6);
}

byte TMF8801::getMeasurementNumber()
{
	return resultNumber;
}

void TMF8801::resetDevice()
{
	// Applies newly updated array into main application
	tmf8801_io.setRegisterBit(REGISTER_ENABLE_REG, CPU_RESET);

	// Checks if CPU is ready
	bool ready = false;
	do
	{
		ready = cpuReady();
	} while (!ready);

	// Loads measurement application
	tmf8801_io.writeSingleByte(REGISTER_APPREQID, APPLICATION);
	ready = false;
	do
	{
		ready = applicationReady();
	} while (!ready);

	// Write calibration data and algorithm state into device
	tmf8801_io.writeSingleByte(REGISTER_COMMAND, COMMAND_CALIBRATION);
	tmf8801_io.writeMultipleBytes(REGISTER_FACTORY_CALIB_0, calibrationData, sizeof(calibrationData));
	tmf8801_io.writeMultipleBytes(REGISTER_STATE_DATA_WR_0, ALGO_STATE, sizeof(ALGO_STATE));

	// Updates CMD_DATA_7 to CMD_DATA_0
	updateCommandData8();

	// Start measurements application
	tmf8801_io.writeSingleByte(REGISTER_COMMAND, COMMAND_MEASURE);

	// Wait 50 msec then return
	delay(50);
}

void TMF8801::wakeUpDevice()
{
	byte result;
	// Write ENABLE_REG to bring device back to operation and wait until it's back
	do
	{
		tmf8801_io.writeSingleByte(REGISTER_ENABLE_REG, 0x01);
		result = tmf8801_io.readSingleByte(REGISTER_ENABLE_REG);
		delay(100);
	} while (result != 0x41);
}

byte TMF8801::getStatus()
{
	return tmf8801_io.readSingleByte(REGISTER_STATUS);
}

void TMF8801::doMeasurement()
{
	byte buffer[4];
	tmf8801_io.readMultipleBytes(REGISTER_RESULT_NUMBER, buffer, sizeof(buffer));
	resultNumber = buffer[0];
	resultInfo = buffer[1];
	distancePeak = buffer[3];
	distancePeak = distancePeak << 8;
	distancePeak += buffer[2];
}

int TMF8801::getDistance()
{
	// Returns interrupt pin to open drain
	clearInterruptFlag();
	// Reads measurement data
	doMeasurement();
	return distancePeak;
}

void TMF8801::enableInterrupt()
{
	byte registerValue = tmf8801_io.readSingleByte(REGISTER_INT_ENAB);
	registerValue |= INTERRUPT_MASK;
	tmf8801_io.writeSingleByte(REGISTER_INT_ENAB, registerValue);
	delay(10);
	doMeasurement();
}

void TMF8801::disableInterrupt()
{
	byte registerValue = tmf8801_io.readSingleByte(REGISTER_INT_ENAB);
	registerValue &= ~INTERRUPT_MASK;
	tmf8801_io.writeSingleByte(REGISTER_INT_ENAB, registerValue);
}

void TMF8801::clearInterruptFlag()
{
	byte registerValue = tmf8801_io.readSingleByte(REGISTER_INT_STATUS);
	registerValue |= INTERRUPT_MASK;
	tmf8801_io.writeSingleByte(REGISTER_INT_STATUS, registerValue);
}

void TMF8801::updateCommandData8()
{
	// Writes commandDataValues array into CMD_DATA_7 to CMD_DATA_0 registers
	tmf8801_io.writeMultipleBytes(REGISTER_CMD_DATA7, commandDataValues, sizeof(commandDataValues));
}

bool TMF8801::measurementEnabled()
{
	// Returns true if resultInfo 7:6 are both zeroed
	byte result = resultInfo;
	result = result >> 6;
	return result == 0;
}

void TMF8801::setGPIO0Mode(byte gpioMode)
{
	// Does not allow invalid values to be set into register
	if (gpioMode > MODE_HIGH_OUTPUT)
		return;

	byte currentRegisterValue;

	// Read current value and change only GPIO0 values
	currentRegisterValue = tmf8801_io.readSingleByte(REGISTER_CMD_DATA0);
	currentRegisterValue &= 0xf0;
	currentRegisterValue += gpioMode;
	commandDataValues[CMD_DATA_5] = currentRegisterValue;

	// Send command to device
	byte buffer[2];
	buffer[0] = currentRegisterValue;
	buffer[1] = 0x0f;
	tmf8801_io.writeMultipleBytes(REGISTER_CMD_DATA0, buffer, 2);
}

byte TMF8801::getGPIO0Mode()
{
	// Read REGISTER_CMD_DATA0 and mask accordingly
	byte currentRegisterValue;
	currentRegisterValue = tmf8801_io.readSingleByte(REGISTER_CMD_DATA0);
	return (currentRegisterValue & 0x0f);
}

void TMF8801::setGPIO1Mode(byte gpioMode)
{	
	// Does not allow invalid values to be set into register
	if (gpioMode > MODE_HIGH_OUTPUT)
		return;

	byte currentRegisterValue;

	// Read current value and change only GPIO1 values
	currentRegisterValue = tmf8801_io.readSingleByte(REGISTER_CMD_DATA0);
	currentRegisterValue &= 0x0f;
	currentRegisterValue += (gpioMode << 4);
	commandDataValues[CMD_DATA_5] = currentRegisterValue;

	// Send command to device
	byte buffer[2];
	buffer[0] = currentRegisterValue;
	buffer[1] = 0x0f;
	tmf8801_io.writeMultipleBytes(REGISTER_CMD_DATA0, buffer, 2);
}

byte TMF8801::getGPIO1Mode()
{
	// Read REGISTER_CMD_DATA0 and shift accordingly
	byte currentRegisterValue;
	currentRegisterValue = tmf8801_io.readSingleByte(REGISTER_CMD_DATA0);
	return (currentRegisterValue >> 4);
}

byte TMF8801::getRegisterValue(byte reg)
{
	return tmf8801_io.readSingleByte(reg);
}

void TMF8801::setRegisterValue(byte reg, byte value)
{
	tmf8801_io.writeSingleByte(reg, value);
}

void TMF8801::getRegisterMultipleValues(byte reg, byte* buffer, byte length)
{
	tmf8801_io.readMultipleBytes(reg, buffer, length);
}

void TMF8801::setRegisterMultipleValues(byte reg, const byte* buffer, byte length)
{
	tmf8801_io.writeMultipleBytes(reg, buffer, length);
}
byte TMF8801::status_read()
{
	    uint8_t sta_buf[3] = {0};

   
   tmf8801_io.writeMultipleBytes(BOOTLOADER, sta_buf, 3);
    
    delay(5);
    if (sta_buf[0] != 0x00 ||
        sta_buf[1] != 0x00 ||
        sta_buf[2] != 0xFF)
    {

     
        return TMF8x01_NK;
    }
    
    return TMF8x01_OK;
}


void TMF8801::remap_rst_RAM()
{
	
	uint8_t cmd_buf[3]={0x11,0x00,0xEE};
	    int y=0;
		bool ready = false;
	    tmf8801_io.writeMultipleBytes(BOOTLOADER, cmd_buf, sizeof(cmd_buf));
		delay(5);

		//TMF8801_reset();
		while(1){
			y++;
			ready = cpuReady();
		if (ready == false && y%10==0)
		{
		 lastError = ERROR_CPU_RESET_TIMEOUT;
		}
		else if(y>50)
		{	printf("Ram reset fail\n\r");
			for(;;);}
		else if(ready ==true)
		{printf("Ram reset ok \n\r");
		return;}
		}
}
void TMF8801::download_INT_RAM()
{
	uint8_t  cmd_buf[] = {0x14, 0x01, 0x29,0xC1},ready=-1;
	//uint8_t checksum=0;
	  /*
	           * cmd_buf [0]: Initialization Download HW Command (0x14)
	           * cmd_buf [1]: Data length (0x01)
	           * cmd_buf [3]: 1 byte data
	           * cmd_buf [4]: ?heck bytes
	     */
	tmf8801_io.writeMultipleBytes(BOOTLOADER, cmd_buf, 4);
    delay(5);

	for(int i=0;i<5;i++){
			ready=status_read();
			
			if(ready==TMF8x01_OK)
			{printf("download_INT_RAM ok\n\r");
			break;}
			}
			if(ready==TMF8x01_NK)
			{for(;;);}

}

void TMF8801::set_ram_addr(uint16_t addr)
{
	uint8_t  cmd_buf[5] = {0};
    uint32_t tick       =  0;

    cmd_buf[0] = 0x43;
    cmd_buf[1] = 0x02;
    cmd_buf[2] = (uint8_t)addr;
    cmd_buf[3] = (uint8_t)(addr >> 8);
    cmd_buf[4] = (uint8_t)~(cmd_buf[0]+cmd_buf[1]+cmd_buf[2]+cmd_buf[3]);

  //  write_register(0x08,cmd_buf[0]);
  //  write_register(0x09,cmd_buf[1]);//size 2
  //  write_register(0x0A,cmd_buf[2]);//data0
  //  write_register(0x0B,cmd_buf[3]);//data1
   //write_register(0x8B,cmd_buf[4]);//CSUM
   	tmf8801_io.writeMultipleBytes(BOOTLOADER, cmd_buf, 5);
   delay(5);
   
    while (1)
    {
    	
        if ( status_read()== TMF8x01_OK)
        {
            break;
        }
        tick ++;
        if (tick >= 10)
        {

            printf("tmf8801_set_ram_addr fial!\r\n");
            return ;
        }
    }
    return ;
	
}
void TMF8801::write_ram()
{
	
	uint8_t  cmd_buf[19] = {0};

    uint16_t buf_length  =  0;
    uint16_t buf_line    =  0;
    uint32_t tick        =  0;
    int i           =  0;
    uint16_t j           =  0;
    uint16_t address_pointer  =  0;
   // uint8_t data=0x0A;
    uint8_t sum=0;
    int count=0;
    buf_length = sizeof(app_buf) / sizeof(app_buf[0]);
    buf_line   = buf_length / 16;

    cmd_buf[0] = 0x41;
    cmd_buf[1] = 0x10;
    //printf("total count=%d \r\n",buf_line);
    //tmf8801_set_ram_addr (0x00);
    for (i = 0; i <buf_line; i++) {
    	//data=0x0A;//reset data
    	sum=0;//reset sum
    	address_pointer=i*16;
    	set_ram_addr(address_pointer);
    	

        for (j = 0; j < 16; j++) {

            cmd_buf[2+j] = app_buf[16 * i + j];
          //  data=data+j;
           // write_register(data,cmd_buf[2+j]);//data0~XXX
        }

        cmd_buf[18] = 0;
        for (j = 0; j < 18; j++) {
        	sum+= cmd_buf[j];

        }
        cmd_buf[18] =~(uint8_t)sum;

   		tmf8801_io.writeMultipleBytes(BOOTLOADER, cmd_buf, 19);
  		 delay(5);


       while(1)
        {
            if (status_read() == TMF8x01_OK)
            {
            	count++;
            	if(count%100==0)
            		printf("Patch download=%d(until over 690 will finish) \r\n",count);
                break;
            }
          tick++;
            if (tick >= 10)
            {

                printf("__tmf8801_write_ram fial!\r\n");
                return ;
            }
        }
        }
   // printf("fail address=%#x \r\n",address_pointer);
    return ;
}

void TMF8801::reset_SW()
{
	uint8_t status=0,x=0,cmd_buf[3]={0x10,0x00,0xEF},ready=-1;
	

		tmf8801_io.writeSingleByte(REGISTER_APPREQID, BOOTLOADER);
	      
		tmf8801_io.writeSingleByte(REGISTER_ENABLE_REG, CPU_RESET);
       

		
		if (!cpuReady())
			{
				 x=tmf8801_io.readSingleByte(REGISTER_ENABLE_REG);
				 printf("CPU staute= %#x \n\r",x);
			    return ;}

		while(1)
		{
			status=tmf8801_io.readSingleByte(REGISTER_APPID);
		if(status==BOOTLOADER)//Bootloader
		   {
		   	   	tmf8801_io.writeMultipleBytes(BOOTLOADER, cmd_buf, 3);
   				delay(5);
			//remap_rst_RAM();
			ready=status_read();
			if(ready==TMF8x01_OK)
			{printf("reset SW ok\r\n");
			return;}
			else if(ready==TMF8x01_NK)
			{printf("reset SW fail\r\n");
			for(;;);}
			}
			}
}



