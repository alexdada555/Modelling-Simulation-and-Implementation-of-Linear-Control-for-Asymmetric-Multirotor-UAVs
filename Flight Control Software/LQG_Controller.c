/**
 * @file LQG_Controller.c
 *
 */

#include <stdio.h>
#include <getopt.h>
#include <signal.h>
#include <stdlib.h> // for atoi() and exit()
#include <rc/time.h>


#include <rc/mpu.h>
#include <rc/bmp.h>


#include <math.h> // for M_PI
#include <rc/math/kalman.h>
#include <rc/math/filter.h>
#include <rc/math/quaternion.h>


// I2C Pins
#define I2C_BUS 2
#define GPIO_INT_PIN_CHIP 3
#define GPIO_INT_PIN_PIN  21

// Sensor Data Structures
static rc_mpu_data_t mpu_data;
static rc_bmp_data_t bmp_data;

// System Dimensions
#define n 14
#define m 6
#define p 4
#define T 0.01

double A[n][n];
double B[n][m];
double C[p][n];
double D[p][m];

rc_kalman_t kf  = RC_KALMAN_INITIALIZER;
    
rc_matrix_t F   = RC_MATRIX_INITIALIZER;
rc_matrix_t G   = RC_MATRIX_INITIALIZER;
rc_matrix_t H   = RC_MATRIX_INITIALIZER;
rc_matrix_t Q   = RC_MATRIX_INITIALIZER;
rc_matrix_t R   = RC_MATRIX_INITIALIZER;
rc_matrix_t Pi  = RC_MATRIX_INITIALIZER;
    
rc_vector_t u   = RC_VECTOR_INITIALIZER;
rc_vector_t y   = RC_VECTOR_INITIALIZER;
    
rc_matrix_t K   = RC_MATRIX_INITIALIZER;
rc_matrix_t Ki  = RC_MATRIX_INITIALIZER;

static int running = 0;
float w1 = 1600;
float w4 = 1600;
    
// Local Functions
static void __print_data(void);
static void __print_header(void);
static int __calibrate_sensors(void);

/**
 * @brief      interrupt handler to catch ctrl-c
 */
static void __signal_handler(__attribute__ ((unused)) int dummy)
{
    running=0;
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
    printf(" \t z | phi | theta | psi |");
    printf("\n");
}

/**
 * This is the IMU interrupt function.
 */
static void __print_data(void)
{
    printf("\r");
    
	rc_bmp_read(&bmp_data);
    
    y.d[0] = bmp_data.alt_m;
    y.d[1] = mpu_data.fused_TaitBryan[TB_ROLL_Y]*RAD_TO_DEG;
    y.d[2] = mpu_data.fused_TaitBryan[TB_PITCH_X]*RAD_TO_DEG;
    y.d[3] = mpu_data.fused_TaitBryan[TB_YAW_Z]*RAD_TO_DEG;
    y.d[4] = w1;
    y.d[5] = w4;
    
    // print fused TaitBryan Angles and Altitude
    //printf("%9.2fm %6.1f %6.1f %6.1f |",y.d[0],y.d[1],y.d[2],y.d[3]);
                                    	
    rc_kalman_update_lin(&kf, u, y);
    
    // print result
    printf("\rpos: %5.2f vel: %5.2f u: %5.2f y: %5.2f    ", kf.x_est.d[0], kf.x_est.d[1], u.d[0], y.d[0]);
    
    fflush(stdout);
    return;
}

static int __calibrate_sensors(void)
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
    // setup sensors
    //__calibrate_sensors();
    rc_mpu_config_t conf = rc_mpu_default_config();
    conf.i2c_bus = I2C_BUS;
    conf.gpio_interrupt_pin_chip = GPIO_INT_PIN_CHIP;
    conf.gpio_interrupt_pin = GPIO_INT_PIN_PIN;
    conf.dmp_sample_rate = 100;
    conf.enable_magnetometer = 1;
    
    // allocate appropriate memory for system
    rc_matrix_zeros(&F, n, n);
    rc_matrix_zeros(&G, n, m);
    rc_matrix_zeros(&H, p+2, n);
    rc_matrix_zeros(&Q, n, n);
    rc_matrix_zeros(&R, p+2, p+2);
    rc_matrix_zeros(&Pi, n, n);
    
    rc_vector_zeros(&u, m);
    rc_vector_zeros(&y, p+2);
    
    rc_matrix_zeros(&K, n, m);
    rc_matrix_zeros(&Ki, p, m);
    
    // define system
    F.d[0][0] = 1;
    F.d[0][1] = T;
    F.d[1][0] = 0;
    F.d[1][1] = 1;
    
    G.d[0][0] = 0.5*T*T;
    G.d[0][1] = T;
    
    H.d[0][0] = 1;
    H.d[0][1] = 0;
    
    // noise covariance matrices
    Q.d[0][0] = 0.1;
    Q.d[1][1] = 0.0001;
    
    R.d[0][0] = 1.0;
    
    // initial P
    Pi.d[0][0] = 0.0001;
    Pi.d[1][1] = 0.0001;
        
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
    
    // set up the kalman filter    
    if(rc_kalman_alloc_lin(&kf,F,G,H,Q,R,Pi)) 
    {   
         printf("rc_kalman_alloc_lin_failed\n");
        return -1;
    }
        
    // write labels for what data will be printed and associate the interrupt
    // function to print data immediately after the header.
    __print_header();
    rc_mpu_set_dmp_callback(&__print_data);
        
    // now just wait, print_data() will be called by the interrupt
    while(running)  rc_usleep(10000);
    
    // shut things down
    rc_bmp_power_off();
    rc_mpu_power_off();
    rc_kalman_free(&kf);
    printf("\n");
    fflush(stdout);
        
    return 0;
}
