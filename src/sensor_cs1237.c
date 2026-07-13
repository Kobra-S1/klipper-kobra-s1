// NOTE:
// The following CS1237 command handlers are intentionally missing in this port.
// These commands were not open-sourced by Anycubic:
//   - "cs1237_calibration_phase oid=%c cali_state=%c speed_state=%c"
//   - "cs1237_calibration_DataProcess oid=%c"
//
#include <string.h> // memcpy
#include "autoconf.h" // CONFIG_MACH_AVR
#include "board/gpio.h" // gpio_out_write
#include "board/irq.h" // irq_poll
#include "board/misc.h" // timer_read_time
#include "basecmd.h" // oid_alloc
#include "command.h" // DECL_COMMAND
#include "sched.h" // sched_shutdown
#include "stm32/internal.h" // gpio_peripheral
#include "sensor_cs1237.h" // cs1237_query
#include "generic/armcm_timer.h"
#include "generic/io.h"
#include "generic/armcm_boot.h"
#include "stm32/gpio.h"
#include "stm32/flash.h"
#include "generic/Debug_serial_irq.h" // Debug_sendf

typedef unsigned int cs1237_time_t;
struct task_wake cs1237_wake,report_wake;
struct gpio_out cs_sclk,cs_dat,level_out;
struct gpio_in cs_dat_in;

static cs1237_time_t
nsecs_to_ticks(uint32_t ns)
{
    return timer_from_us(ns * 1000) / 1000000;
}

static inline int
cs1237_check_elapsed(cs1237_time_t t1, cs1237_time_t t2
                       , cs1237_time_t ticks)
{
    return t2 - t1 >= ticks;
}

static cs1237_time_t
cs1237_get_time(void)
{
    return timer_read_time();
}

static inline void
cs1237_delay_no_irq(cs1237_time_t start, cs1237_time_t ticks)
{
    while (!cs1237_check_elapsed(start, cs1237_get_time(), ticks))
        ;
}

static inline void
cs1237_delay(cs1237_time_t start, cs1237_time_t ticks)
{
    while (!cs1237_check_elapsed(start, cs1237_get_time(), ticks))
        irq_poll();
}

/****************************************************************
 * CS1237 Support
 ****************************************************************/
#define MIN_PULSE_TIME  nsecs_to_ticks(300)
#define MAX_READ_TIME timer_from_us(50)

// Out-of-range (railed / disconnected load-cell ADC) detection.
// When the raw 24-bit signed reading
// stays within 0x4000 counts of either +/- full-scale rail (i.e.
// |adc| > 0x7FC000) for OOR_DEBOUNCE consecutive data-ready samples, the
// CS1237 is saturated/faulty -> raise shutdown "cs1237 adc out of range".
// Debounce compare "9 < count"
#define CS1237_ADC_OUT_OF_RANGE 0x7FC000
#define CS1237_OOR_DEBOUNCE     10

static inline int32_t
cs1237_abs(int32_t y)
{
    return y < 0 ? -y : y;
}

void cs1237_exti_interrupt_disable(void)
{
    EXTI_INTEN &= ~((uint32_t)1<<6); //PA6
}

void cs1237_exti_interrupt_enable(void)
{
    EXTI_INTEN |= (uint32_t)1<<6; //PA6
}
uint8_t cs1237_exti_flag_get(void)
{
    if( RESET != ((*(volatile uint32_t *)(uint32_t)(0x40010414)) & ((uint32_t)1 << 6)))
        return SET;
    else
        return RESET;
}

void cs1237_exti_flag_clear(void)
{
    (*(volatile uint32_t *)(uint32_t)(0x40010414)) = (uint32_t)1<<6;
}

void CS1237_EXTI_IRQHandler(void)
{
    
    if(RESET != cs1237_exti_flag_get())
    {
        sched_wake_task(&cs1237_wake);
        cs1237_exti_flag_clear();
    }
}

uint_fast8_t cs1237_report_event(struct timer *timer)
{
    struct cs1237_sensor *cs = container_of(timer, struct cs1237_sensor, timer);
    sched_wake_task(&report_wake);
    cs->timer.waketime +=cs->report_tick;
    return SF_RESCHEDULE;
}

void cs1237_exti_init(void)
{
    EXTI_RTEN &= ~((uint32_t)1<<6);//EXTI rising edge trigger
    EXTI_FTEN |= (uint32_t)1<<6;//EXTI falling edge trigger
    *(volatile uint32_t *)(uint32_t)(0x4001040C) = (uint32_t)1<<6;
    cs1237_exti_flag_clear();//clean flag 清标志
    armcm_enable_irq(CS1237_EXTI_IRQHandler, EXTI9_5_IRQn, 5);
    
}
DECL_INIT(cs1237_exti_init);

/*one clk*/
void cs1237_oneclk(void)
{  
    irq_disable();
    gpio_out_write(cs_sclk,1);
    cs1237_delay_no_irq(cs1237_get_time(), MIN_PULSE_TIME);
    gpio_out_write(cs_sclk,0);
    irq_enable();
    cs1237_delay(cs1237_get_time(), MIN_PULSE_TIME);
}
/*read reg*/
uint8_t cs1237_read_reg(void)
{
    
    for(uint8_t i = 0;i<29;i++) //1-29th CLK
    {
        cs1237_oneclk();
    }  

    gpio_out_reset(cs_dat,0);
    uint8_t read_cmd = 0xAC;    //0X56:read cmd;Because there are only 7 clocks, move one digit to the left
    for(uint8_t i = 0;i<7;i++) //30th ~36th
    {
        if(read_cmd & 0x80)
            gpio_out_write(cs_dat,1);
        else
            gpio_out_write(cs_dat,0);
        read_cmd <<=1;
        cs1237_oneclk();
    }
    cs1237_oneclk();//37th
    gpio_in_reset(cs_dat_in,-1);//
    uint8_t data = 0;
    for(uint8_t i=0;i<8;i++) //38th ~45th
    {
        cs1237_oneclk();
        data <<=1;
        if(gpio_in_read(cs_dat_in) == 1)
            data++;
    }
    cs1237_oneclk();//46th
    gpio_out_reset(cs_dat,0);
    gpio_out_write(cs_dat,1);

    return data;
}
/*write reg*/
void cs1237_write_reg(struct cs1237_sensor *cs)
{
    uint8_t write_cmd = 0;
    uint8_t data = cs->speed;
    gpio_out_reset(cs_dat,0);
    udelay(100);
    gpio_in_reset(cs_dat_in,-1);
    // 1th ~ 29th脉冲
    for(uint8_t i = 0;i<29;i++) //1-29th CLK
    {
        cs1237_oneclk();
    }

    gpio_out_reset(cs_dat,0);
    write_cmd = 0xCA;    //0X65:write cmd。Because there are only 7 clocks, move one digit to the left
    //30th ~36th
    for(uint8_t i = 0;i<7;i++) 
    {
        if(write_cmd & 0x80)
            gpio_out_write(cs_dat,1);
        else
            gpio_out_write(cs_dat,0);
        write_cmd <<=1;
        cs1237_oneclk();
    }
    //37th
    cs1237_oneclk();
    // write data 38th ~ 45th
    for(uint8_t i=0;i<8;i++) //38th ~45th
    {        
        if(data & 0x80)
            gpio_out_write(cs_dat,1);
        else
            gpio_out_write(cs_dat,0);
        data <<=1;
        cs1237_oneclk();
    }
    //46th
    gpio_out_write(cs_dat,1);
    cs1237_oneclk();

    gpio_in_reset(cs_dat_in,-1);
    // EXTI state is managed by caller (command_config/enable_cs1237)
}

/*read adc*/
int32_t cs1237_read_adc(void)
{
    // data is ready
    long counts = 0x00;

    cs1237_exti_interrupt_disable();
    gpio_in_reset(cs_dat_in,-1);

    for(uint8_t i = 0;i<24;i++)
    {
        cs1237_oneclk();
        // read 2's compliment int bits
        counts = (counts << 1) | gpio_in_read(cs_dat_in);
    }

    for(uint8_t i = 0;i<3;i++) //25th ~27th
    {       
        cs1237_oneclk();
    }
    irq_disable();
    gpio_out_reset(cs_dat,0);
    gpio_out_write(cs_dat,1);//pull up

    irq_enable();
    gpio_in_reset(cs_dat_in,-1);

    cs1237_exti_interrupt_enable();//
    counts <<= 8;
    counts = (int32_t)counts;
    counts /= 256;
    return counts;
}

// Create an cs1237 sensor
void
command_config_cs1237(uint32_t *args)
{
    struct cs1237_sensor *cs = oid_alloc(args[0]
                             , command_config_cs1237, sizeof(*cs));

    level_out = gpio_out_setup(args[1],0);      //level out pin
    cs_dat = gpio_out_setup(args[2], -1);       //cs1237 wrtie pin
    cs_dat_in = gpio_in_setup(args[2], -1);     //cs1237 read pin   
    cs_sclk = gpio_out_setup(args[3], 0);       //cs1237 clk pin
 
    cs->speed = args[4];             //cs1237 config(adc speed)
    cs->sensitivity = args[5];       //cs1237 trigger threshold
    cs1237_write_reg(cs);
    cs->check_enable = 0;
    cs->timer.func = cs1237_report_event;
    // Sensor starts disabled — host must send enable_cs1237 to activate
    cs1237_exti_interrupt_disable();
    Debug_sendf("config_cs1237 oid=%d sensitivity=%d\n", args[0], cs->sensitivity);
}
DECL_COMMAND(command_config_cs1237, "config_cs1237 oid=%c level_pin=%u dout_pin=%u" \
                                    " sclk_pin=%u register=%u sensitivity=%i");

void command_checkself_cs1237(uint32_t *args)
{

   #define CHECKSELF_FLAG_ADDR 0x08007800
   #define CHECKSELF_PAGE_SIZE 0x800
   int32_t checkself_flag = -1;
   uint32_t checkself_data = -1;
   struct flash_config fc = flash_setup(0);
   struct flash_config *info = &fc;
   uint32_t write_addr;
   uint32_t read_addr;
   uint32_t find_pos = 0;//offset write addr
   write_addr = CHECKSELF_FLAG_ADDR;

   // Scan for writable address (read-only, IRQs stay enabled)
   for(uint16_t i = 0; i<CHECKSELF_PAGE_SIZE/4; i++){
        if(flash_read_byte(info,write_addr)==0xFFFFFFFF)
        {
           write_addr =  CHECKSELF_FLAG_ADDR + 4*i;
           find_pos = i;
           break;
        }
        write_addr +=4;
   }

   if(find_pos == 0 || find_pos == 1)
     read_addr = CHECKSELF_FLAG_ADDR;
   else
     read_addr = CHECKSELF_FLAG_ADDR + 4*(find_pos -1);

   if(args[1]&0xFF)  //write flag
   {
        // Disable IRQs only for flash modification to minimize watchdog risk
        irq_disable();
        if(write_addr>=CONFIG_FLASH_APP1_ADDRESS)//page full,need erase
        {
            flash_erase(info,CHECKSELF_FLAG_ADDR);
            write_addr = CHECKSELF_FLAG_ADDR;
        }
        flash_write_byte(info,write_addr,args[1]&0x0F);
        irq_enable();
        read_addr = write_addr;
   }

   checkself_data = flash_read_byte(info,read_addr);
   if(checkself_data != 0xFFFFFFFF) //not selfcheck
       checkself_flag = checkself_data;

   sendf("cs1237_checkself_flag oid=%c flag=%c",args[0],checkself_flag);
}
DECL_COMMAND(command_checkself_cs1237, "checkself_cs1237 oid=%c write=%c");

void
command_reset_cs1237(uint32_t *args)
{
    // Note: args[1] (count) is declared in protocol but currently unused
    struct cs1237_sensor *cs = oid_lookup(args[0],command_config_cs1237);
    cs->homing_flag = 1;
    cs->homing_adc = cs->adc32;
}
DECL_COMMAND(command_reset_cs1237, "reset_cs1237 oid=%c count=%c");

void 
command_query_cs1237_diff(uint32_t *args)
{
    struct cs1237_sensor *cs = oid_lookup(args[0],command_config_cs1237);
    sendf("cs1237_diff oid=%c diff=%i raw=%i",args[0],cs1237_abs(cs->adc_diff),cs->avr_adc32);
}
DECL_COMMAND(command_query_cs1237_diff, "query_cs1237_diff oid=%c");

void command_query_cs1237_adc(uint32_t *args)
{
    struct cs1237_sensor *cs = oid_lookup(args[0],command_config_cs1237);
    sendf("cs1237_state oid=%c adc=%i raw=%i state=%c",
          args[0], cs->avr_adc32, cs->adc32, 0);
}
DECL_COMMAND(command_query_cs1237_adc, "query_cs1237_adc oid=%c");

void command_enable_cs1237(uint32_t *args)
{
    struct cs1237_sensor *cs = oid_lookup(args[0],command_config_cs1237);
    cs->check_enable = args[1];
    if (cs->check_enable) {
        // Clear level_pin to prevent stale trigger state
        gpio_out_reset(level_out, 0);
        // Mark for seeding: first ADC reading will initialize EMA filters
        // to the actual baseline value instead of 0 (starting from 0 causes
        // a false trigger when |baseline| > sensitivity*128/3 ≈ 107K)
        cs->seeding = 1;
        cs->slow = 0;
        cs->fast = 0;
        cs->last_trig = 0;
        cs->oor_count = 0;
        // Clear pending EXTI flag before enabling to avoid stale data-ready
        cs1237_exti_flag_clear();
        // Re-enable EXTI so data-ready pulses wake cs1237_task
        cs1237_exti_interrupt_enable();
        if (cs->report_tick) {
            // Only enable timer if report_tick has been initialized
            irq_disable();
            cs->timer.waketime = timer_read_time() + cs->report_tick;
            sched_add_timer(&cs->timer);
            irq_enable();
        }
    } else {
        // Disable EXTI to stop 1280 Hz interrupt overhead
        cs1237_exti_interrupt_disable();
        sched_del_timer(&cs->timer);
        // Clear level_pin so stale trigger doesn't persist
        gpio_out_reset(level_out, 0);
        cs->last_trig = 0;
    }
}
DECL_COMMAND(command_enable_cs1237, "enable_cs1237 oid=%c state=%c");

void command_start_cs1237_report(uint32_t *args)
{
    struct cs1237_sensor *cs = oid_lookup(args[0],command_config_cs1237);
    sched_del_timer(&cs->timer);
    uint8_t enable = args[1];
    cs->report_tick = args[2];
    // Rebase-experiment behavior: report command also updates sensitivity.
    cs->sensitivity = args[4];

    if(enable && cs->report_tick){
        irq_disable();
        cs->timer.waketime = timer_read_time() + cs->report_tick;
        sched_add_timer(&cs->timer);
        irq_enable();
    }

}
DECL_COMMAND(command_start_cs1237_report, "start_cs1237_report oid=%c enable=%c ticks=%i print_state=%c sensitivity=%i");

void average_filter(struct cs1237_sensor *cs)
{
     #define  NUM  50  
     int32_t  sum_adc = 0;
     static   int32_t  avr_buf[NUM]; 

     for(uint8_t i = 0;i<NUM-1;i++)
     {
         avr_buf[i] = avr_buf[i+1];
         sum_adc += avr_buf[i];
     }
     avr_buf[NUM-1] = (int32_t)cs->adc32; 
     sum_adc += avr_buf[NUM-1]; 
     cs->avr_adc32 = sum_adc/NUM;

}
void cs1237_report_task(void)
{
   if (!sched_check_wake(&report_wake))
        return;

    uint8_t oid;
    struct cs1237_sensor *cs;
    foreach_oid(oid, cs, command_config_cs1237)
    {
       sendf("cs1237_state oid=%c adc=%i raw=%i state=%c",oid,cs->avr_adc32,cs->adc32,0); 
    }    
}
DECL_TASK(cs1237_report_task);

void cs1237_task(void)
{
   if (!sched_check_wake(&cs1237_wake))
        return;

    uint8_t oid;
    struct cs1237_sensor *cs;
    foreach_oid(oid, cs, command_config_cs1237)
    {
        // Skip expensive ADC read when sensor is disabled
        if (!cs->check_enable)
            continue;

        cs->adc32 = cs1237_read_adc();

        // Railed / disconnected sensor detection (stock fw 1.4.3 parity):
        // debounce OOR_DEBOUNCE consecutive samples pinned near the +/- rail,
        // then disable the data-ready interrupt and shut down.
        if (cs1237_abs(cs->adc32) > CS1237_ADC_OUT_OF_RANGE) {
            if (++cs->oor_count >= CS1237_OOR_DEBOUNCE) {
                cs1237_exti_interrupt_disable();
                shutdown("cs1237 adc out of range");
            }
        } else {
            cs->oor_count = 0;
        }

        average_filter(cs);

        // Seed EMA filters from first reading so rate starts at 0
        if (cs->seeding) {
            cs->slow = cs->adc32;
            cs->fast = cs->adc32;
            cs->seeding = 0;
            continue; // skip trigger logic until filters are seeded
        }

        cs->slow = cs->slow + (cs->adc32 - cs->slow) / 128;
        cs->fast = cs->fast + (cs->adc32 - cs->fast) / 32;
        int32_t rate = cs->fast - cs->slow;

        int32_t sensitivity = cs->sensitivity;
        if(rate < sensitivity || rate > -sensitivity) {
            if(!cs->last_trig) {
                gpio_out_reset(level_out,1);
                cs->last_trig = 1;
                if(cs->homing_flag){
                    cs->homing_flag = 0;
                    cs->adc_diff = cs->homing_adc - cs->adc32;
                }
            }       
        } else{
            if(cs->last_trig) {
                gpio_out_reset(level_out,0);
                cs->last_trig = 0;
            }
        }
    }
 
}   
DECL_TASK(cs1237_task);
