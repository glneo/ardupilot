#include <AP_HAL.h>
#include "Storage.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX && LINUX_STORAGE_USE_FRAM
#include <assert.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>

using namespace Linux;

/*
  This stores 'eeprom' data on the FRAM, with a 4k size, and a
  in-memory buffer. This keeps the latency down.
 */

// name the storage file after the sketch so you can use the same board
// card for ArduCopter and ArduPlane

extern const AP_HAL::HAL& hal;
LinuxStorage::LinuxStorage():
_fd(-1),
_dirty_mask(0),
_spi(NULL),
_spi_sem(NULL),
_initialised(false)
{}


void LinuxStorage::_storage_create(void)
{

    int fd = open();
    hal.console->println("Storage: FRAM is getting reset to default values");
    
    if (fd == -1) {
        hal.scheduler->panic("Failed to load FRAM");
    }
    for (uint16_t loc=0; loc<sizeof(_buffer); loc += LINUX_STORAGE_MAX_WRITE) {
        if (write(fd, &_buffer[loc], LINUX_STORAGE_MAX_WRITE) != LINUX_STORAGE_MAX_WRITE) {
            hal.scheduler->panic("Error filling FRAM");            
        }
    }

    // ensure the directory is updated with the new size

    fsync(fd);
    close(fd);
}

void LinuxStorage::_storage_open(void)
{
    if (_initialised) {
        return;
    }

    _dirty_mask = 0;
    int fd = open();
    if (fd == -1) {
        _storage_create();
        fd = open();
        if (fd == -1) {
            hal.scheduler->panic("Failed to access FRAM");
        }
    }
    
    if (read(fd, _buffer, sizeof(_buffer)) != sizeof(_buffer)) {
        _storage_create();
        fd = open();
        if (fd == -1) {
            hal.scheduler->panic("Failed to access FRAM");
        }
        if (read(fd, _buffer, sizeof(_buffer)) != sizeof(_buffer)) {
            hal.scheduler->panic("Failed to read FRAM");
        }
    }
    _initialised = true;
}

/*
  mark some lines as dirty. Note that there is no attempt to avoid
  the race condition between this code and the _timer_tick() code
  below, which both update _dirty_mask. If we lose the race then the
  result is that a line is written more than once, but it won't result
  in a line not being written.
 */
void LinuxStorage::_mark_dirty(uint16_t loc, uint16_t length)
{
    uint16_t end = loc + length;
    for (uint8_t line=loc>>LINUX_STORAGE_LINE_SHIFT;
         line <= end>>LINUX_STORAGE_LINE_SHIFT;
         line++) {
        _dirty_mask |= 1U << line;
    }
}

uint8_t LinuxStorage::read_byte(uint16_t loc) 
{
    if (loc >= sizeof(_buffer)) {
        return 0;
    }
    _storage_open();
    return _buffer[loc];
}

uint16_t LinuxStorage::read_word(uint16_t loc) 
{
    uint16_t value;
    if (loc >= sizeof(_buffer)-(sizeof(value)-1)) {
        return 0;
    }
    _storage_open();
    memcpy(&value, &_buffer[loc], sizeof(value));
    return value;
}

uint32_t LinuxStorage::read_dword(uint16_t loc) 
{
    uint32_t value;
    if (loc >= sizeof(_buffer)-(sizeof(value)-1)) {
        return 0;
    }
    _storage_open();
    memcpy(&value, &_buffer[loc], sizeof(value));
    return value;
}

void LinuxStorage::read_block(void *dst, uint16_t loc, size_t n) 
{
    if (loc >= sizeof(_buffer)-(n-1)) {
        return;
    }
    _storage_open();
    memcpy(dst, &_buffer[loc], n);
}

void LinuxStorage::write_byte(uint16_t loc, uint8_t value) 
{
    if (loc >= sizeof(_buffer)) {
        return;
    }
    if (_buffer[loc] != value) {
        _storage_open();
        _buffer[loc] = value;
        _mark_dirty(loc, sizeof(value));
    }
}

void LinuxStorage::write_word(uint16_t loc, uint16_t value) 
{
    if (loc >= sizeof(_buffer)-(sizeof(value)-1)) {
        return;
    }
    if (memcmp(&value, &_buffer[loc], sizeof(value)) != 0) {
        _storage_open();
        memcpy(&_buffer[loc], &value, sizeof(value));
        _mark_dirty(loc, sizeof(value));
    }
}

void LinuxStorage::write_dword(uint16_t loc, uint32_t value) 
{
    if (loc >= sizeof(_buffer)-(sizeof(value)-1)) {
        return;
    }
    if (memcmp(&value, &_buffer[loc], sizeof(value)) != 0) {
        _storage_open();
        memcpy(&_buffer[loc], &value, sizeof(value));
        _mark_dirty(loc, sizeof(value));
    }
}

void LinuxStorage::write_block(uint16_t loc, const void *src, size_t n) 
{
    if (loc >= sizeof(_buffer)-(n-1)) {
        return;
    }
    if (memcmp(src, &_buffer[loc], n) != 0) {
        _storage_open();
        memcpy(&_buffer[loc], src, n);
        _mark_dirty(loc, n);
    }
}

void LinuxStorage::_timer_tick(void)
{
    if (!_initialised || _dirty_mask == 0) {
        return;
    }

    if (_fd == -1) {
        _fd = open();
        if (_fd == -1) {
            return;
		}
	}

	// write out the first dirty set of lines. We don't write more
	// than one to keep the latency of this call to a minimum
	uint8_t i, n;
	for (i=0; i<LINUX_STORAGE_NUM_LINES; i++) {
		if (_dirty_mask & (1<<i)) {
			break;
		}
	}
	if (i == LINUX_STORAGE_NUM_LINES) {
		// this shouldn't be possible
		return;
	}
	uint32_t write_mask = (1U<<i);
	// see how many lines to write
	for (n=1; (i+n) < LINUX_STORAGE_NUM_LINES && 
		     n < (LINUX_STORAGE_MAX_WRITE>>LINUX_STORAGE_LINE_SHIFT); n++) {
		if (!(_dirty_mask & (1<<(n+i)))) {
			break;
		}		
		// mark that line clean
		write_mask |= (1<<(n+i));
	}

	/*
	  write the lines. This also updates _dirty_mask. Note that
	  because this is a SCHED_FIFO thread it will not be preempted
	  by the main task except during blocking calls. This means we
	  don't need a semaphore around the _dirty_mask updates.
	 */
	if (lseek(_fd, i<<LINUX_STORAGE_LINE_SHIFT, SEEK_SET) == (i<<LINUX_STORAGE_LINE_SHIFT)) {
		_dirty_mask &= ~write_mask;
		if (write(_fd, &_buffer[i<<LINUX_STORAGE_LINE_SHIFT], n<<LINUX_STORAGE_LINE_SHIFT) != n<<LINUX_STORAGE_LINE_SHIFT) {
			// write error - likely EINTR
			_dirty_mask |= write_mask;
			_fd = -1;
		}
	}
}

//File control function overloads

int8_t LinuxStorage::open()
{
    if(_initialised){
        return 0;
    }
    uint8_t manufacturerID;
    _spi = hal.spi->device(AP_HAL::SPIDevice_Dataflash);
    uint8_t signature[4] = {0x00,0xaf,0xf0,0x0f};
    uint8_t j = 0;
    for(int i=0;true;i++){
        manufacturerID = _register_read(0,OPCODE_RDID);

        if(manufacturerID == 0x7F){
            break;
        }
        else{
            hal.scheduler->delay(1000);
        }
        if(i==4){
            hal.scheduler->panic(PSTR("FRAM: Failed to receive Manufacturer ID 5 times"));
        }
    }
    
    while(j<4){
        if(_register_read(j+4100,OPCODE_READ) == -1){
            continue;
        }
        else if((uint8_t)_register_read(j+4100,OPCODE_READ) != signature[j]){
            while(_register_write(signature,4100,4) == -1);
            return -1;
        }
        else{
            j++;
        }
    }
    _initialised = true;
    hal.console->println("FRAM: Online");
    return 0;
}

int32_t LinuxStorage::write(uint16_t fd,uint8_t *Buff, uint16_t NumBytes){
	if( _register_write(Buff,fptr,NumBytes) == -1){
		return -1;
	}
	return NumBytes;
}
int32_t LinuxStorage::read(uint16_t fd, uint8_t *Buff, uint16_t NumBytes){
    for(uint16_t i=fptr;i<(fptr+NumBytes);i++){
        Buff[i-fptr]= _register_read(i,OPCODE_READ);

		if(Buff[i-fptr]==-1){
			return -1;
		}
    }
    fptr+=NumBytes; 
    return NumBytes; 
}
uint32_t LinuxStorage::lseek(uint16_t fd,uint32_t offset,uint16_t whence){
    fptr = offset;
    return offset; 
}

//FRAM I/O functions

int8_t LinuxStorage::_register_write( uint8_t* src, uint16_t addr, uint16_t len ){

    uint8_t *tx;
    uint8_t *rx;
    uint16_t i;
	tx = new uint8_t[len+3];
	rx = new uint8_t[len+3];
	_write_enable(true);
    
    tx[0] = OPCODE_WRITE;
    tx[1] = addr>>8;
    tx[2] = addr;
	
    for(i=0;i<len;i++){
		tx[i+3] = src[i];
	}
	if(transaction(tx, rx, len+3) == -1){
		return -1;
	}
	if(_write_enable(false) == -1){
		return -1;
	}
	return len;
}

int8_t LinuxStorage::_write_enable(bool enable)
{
    uint8_t tx[2];
    uint8_t rx[2];
	if(enable){
	    tx[0] = OPCODE_WREN;
	    tx[1] = 0;
	    return transaction(tx, rx, 2);
	}
	else{
	    tx[0] = OPCODE_WRDI;
	    tx[1] = 0;
	    return transaction(tx, rx, 2);
	}
	
}

int8_t LinuxStorage::_register_read( uint16_t addr, uint8_t opcode )
{
    uint8_t tx[4] = {opcode, addr>>8, addr, 0};
    uint8_t rx[4];
    
    if(transaction(tx, rx, 4) == -1){
    	return -1;
    }
    return rx[3];
}

int8_t LinuxStorage::transaction(uint8_t* tx, uint8_t* rx, uint16_t len){
	_spi_sem = _spi->get_semaphore();
    if (!_spi_sem->take(100)) {
       // FRAM: Unable to get semaphore
		return -1;
    }
    _spi->transaction(tx, rx, len);
	_spi_sem->give();
	return 0;
}

#endif // CONFIG_HAL_BOARD
