/**
 * @file LQG_Controller.c
 *
 */

#include <stdio.h>
#include <getopt.h>
#include <signal.h>
#include <stdlib.h> // for atoi() and exit()
#include <rc/mpu.h>
#include <rc/bmp.h>
#include <rc/time.h>

// change these for your platform
#define I2C_BUS 2
#define GPIO_INT_PIN_CHIP 3
#define GPIO_INT_PIN_PIN  21

// Global Variables
static int running = 0;

static rc_mpu_data_t mpu_data;
static rc_bmp_data_t bmp_data;

// local functions
static void __print_data(void);
static void __print_header(void);
static int __calibrate_sensors();

/**
 * @brief      interrupt handler to catch ctrl-c
 */
static void __signal_handler(__attribute__ ((unused)) int dummy)
{
    running=0;
    return;
}

/**
 * This is the IMU interrupt function.
 */
static void __print_data(void)
{
	printf("\r");
    printf(" ");
    
	rc_bmp_read(&bmp_data);
    // print fused TaitBryan Angles
    printf("%9.2fm %6.1f %6.1f %6.1f |",bmp_data.alt_m,
    									mpu_data.fused_TaitBryan[TB_PITCH_X]*RAD_TO_DEG,\
                                    	mpu_data.fused_TaitBryan[TB_ROLL_Y]*RAD_TO_DEG,\
                                    	mpu_data.fused_TaitBryan[TB_YAW_Z]*RAD_TO_DEG);
   
    fflush(stdout);
    return;
}

/**
 * Based on which data is marked to be printed, print the correct labels. this
 * is printed only once and the actual data is updated on the next line.
 */
static void __print_header(void)
{
    printf(" ");
    printf(" Altitude, FusedTaitBryan(deg) |");
    printf("\n");
}

static int __calibrate_sensors()
{
	rc_mpu_config_t config = rc_mpu_default_config();
    config.i2c_bus = I2C_BUS;
        
    if(rc_mpu_calibrate_accel_routine(config)<0)
    {
        printf("Failed to complete accelerometer calibration\n");
        return -1;
    }
    printf("\nacceleometer calibration file written\n");
    printf("run rc_test_mpu to check performance\n");
        
    printf("\nThis program will generate a new gyro calibration file\n");
    printf("keep your board very still for this procedure.\n");
    printf("Press any key to continue\n");
    getchar();
    printf("Starting calibration routine\n");
        
    if(rc_mpu_calibrate_gyro_routine(config)<0)
    {
       printf("Failed to complete gyro calibration\n");
       return -1;
    }
    printf("\ngyro calibration file written\n");
    printf("run rc_test_mpu to check performance\n");
        
    printf("\n");
    printf("This will sample the magnetometer for the next 15 seconds\n");
    printf("Rotate the board around in the air through as many orientations\n");
    printf("as possible to collect sufficient data for calibration\n");
    printf("Press any key to continue\n");
    getchar();
    printf("spin spin spin!!!\n\n");
        
    // wait for the user to actually start
    rc_usleep(2000000);
    if(rc_mpu_calibrate_mag_routine(config)<0)
    {
        printf("Failed to complete magnetometer calibration\n");
        return -1;
    }
    printf("\nmagnetometer calibration file written\n");
    printf("run rc_test_mpu to check performance\n");
        
    return 0;
}

/**
 * main() serves to parse user options, initialize the imu and interrupt
 * handler, and wait for the rc_get_state()==EXITING condition before exiting
 * cleanly. The imu_interrupt function print_data() is what actually prints new
 * imu data to the screen after being set with rc_mpu_set_dmp_callback().
 *
 * @return     0 on success -1 on failure
 */
int main()
{
        //__calibrate_sensors();
        rc_mpu_config_t conf = rc_mpu_default_config();
        conf.i2c_bus = I2C_BUS;
        conf.gpio_interrupt_pin_chip = GPIO_INT_PIN_CHIP;
        conf.gpio_interrupt_pin = GPIO_INT_PIN_PIN;
        conf.dmp_sample_rate = 100;
        conf.enable_magnetometer = 1;
        
        // set signal handler so the loop can exit cleanly to ^C cmd
        signal(SIGINT, __signal_handler);
        running = 1;
        
        // init barometer sample at 87Hz, no internal filter
        if(rc_bmp_init(BMP_OVERSAMPLE_4, BMP_FILTER_OFF)) 
        {
        	printf("rc_bmp_init_failed\n");
        	return -1;
        }
        
        // set up the imu for dmp interrupt operation
        if(rc_mpu_initialize_dmp(&mpu_data, conf))
        {
            printf("rc_mpu_initialize_failed\n");
            return -1;
        }
        
        // write labels for what data will be printed and associate the interrupt
        // function to print data immediately after the header.
        __print_header();
        rc_mpu_set_dmp_callback(&__print_data);
        
        //now just wait, print_data() will be called by the interrupt
        while(running)  rc_usleep(100000);
        // shut things down
        rc_bmp_power_off();
        rc_mpu_power_off();
        printf("\n");
        fflush(stdout);
        return 0;
}