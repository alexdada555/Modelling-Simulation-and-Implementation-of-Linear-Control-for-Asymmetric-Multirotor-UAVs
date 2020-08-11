/**
 * @file LQG_Controller.c
 *
 */

#include <stdio.h>
#include <getopt.h>
#include <signal.h>
#include <stdlib.h> // for atoi() and exit()
#include <math.h> // for M_PI

#include <rc/mpu.h>
#include <rc/bmp.h>

#include <rc/time.h>
#include <rc/math/kalman.h>
#include <rc/math/filter.h>
#include <rc/math/quaternion.h>

#include "Model_Parameters.h"

// I2C Pins
#define I2C_BUS 2
#define GPIO_INT_PIN_CHIP 3
#define GPIO_INT_PIN_PIN  21

// Filter Parameters
#define SAMPLE_RATE     100     // hz
#define DT              (1.0/SAMPLE_RATE)
#define PRINT_HZ        100
#define BMP_RATE_DIV    100      // optionally sample bmp less frequently than mpu

// Sensor Data Structures
static rc_mpu_config_t conf;
static rc_mpu_data_t mpu_data;
static rc_bmp_data_t bmp_data;
double innit_alt;

// Kalman Data Structures
static rc_kalman_t kf  = RC_KALMAN_INITIALIZER;
    
static rc_matrix_t F   = RC_MATRIX_INITIALIZER;
static rc_matrix_t G   = RC_MATRIX_INITIALIZER;
static rc_matrix_t H   = RC_MATRIX_INITIALIZER;
static rc_matrix_t Q   = RC_MATRIX_INITIALIZER;
static rc_matrix_t R   = RC_MATRIX_INITIALIZER;
static rc_matrix_t Pi  = RC_MATRIX_INITIALIZER;

// Reference-Error-Input-Output Data Structures    
static rc_vector_t r   = RC_VECTOR_INITIALIZER;
static rc_vector_t xe   = RC_VECTOR_INITIALIZER;
static rc_vector_t u   = RC_VECTOR_INITIALIZER;
static rc_vector_t u1   = RC_VECTOR_INITIALIZER;
static rc_vector_t u2   = RC_VECTOR_INITIALIZER;
static rc_vector_t y   = RC_VECTOR_INITIALIZER;
static rc_vector_t ye   = RC_VECTOR_INITIALIZER;

// System Gains Data Structures 
static rc_matrix_t K   = RC_MATRIX_INITIALIZER;
static rc_matrix_t Ki  = RC_MATRIX_INITIALIZER;

static int running = 0;
    
// Local Functions
static int __calibrate_sensors(void);
static int __sensor_setup(void);
static int __kalman_setup(void);
static void __state_estimate(void);

/**
 * @brief      interrupt handler to catch ctrl-c
 */
static void __signal_handler(__attribute__ ((unused)) int dummy)
{
    running=0;
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
 * Setup and Configure sensors
 */
static int __sensor_setup(void)
{
    conf = rc_mpu_default_config();
    conf.i2c_bus = I2C_BUS;
    conf.gpio_interrupt_pin_chip = GPIO_INT_PIN_CHIP;
    conf.gpio_interrupt_pin = GPIO_INT_PIN_PIN;
    conf.dmp_sample_rate = SAMPLE_RATE;
    //conf.dmp_fetch_accel_gyro = 1;
    conf.enable_magnetometer = 1;
    
    printf("hellosen\n");
    return 0;
}

/**
 * Allocate appropriate memory for system
 * Read in system matrices from Model_Parameters.h
 */
static int __kalman_setup(void)
{
    rc_matrix_zeros(&F, n, n);
    rc_matrix_zeros(&G, n, m);
    rc_matrix_zeros(&H, p+2, n);
    rc_matrix_zeros(&Q, n, n);
    rc_matrix_zeros(&R, p+2, p+2);
    rc_matrix_zeros(&Pi, n, n);
    
    rc_vector_zeros(&r, p);
    rc_vector_zeros(&xe, p);
    rc_vector_zeros(&u, m);
    rc_vector_zeros(&u1, m);
    rc_vector_zeros(&u2, m);
    rc_vector_zeros(&y, p+2);
    rc_vector_zeros(&ye, p);
    
    rc_matrix_zeros(&K, m, n);
    rc_matrix_zeros(&Ki, m, p);
    
    // define system
    for(int i = 0; i<n; i++)
        for(int j = 0; j<n; j++)
            F.d[i][j] = A[i][j];
    
    for(int i = 0; i<n; i++)
        for(int j = 0; j<m; j++)
            G.d[i][j] = B[i][j];
    
    for(int i = 0; i<p+2; i++)
        for(int j = 0; j<n; j++)
            H.d[i][j] = C[i][j];
    
    // noise covariance matrices
    for(int i = 0; i<n; i++)
        for(int j = 0; j<n; j++)
            Q.d[i][j] = Rw[i][j];
    
    for(int i = 0; i<p+2; i++)
        for(int j = 0; j<p+2; j++)
            R.d[i][j] = Rv[i][j];
    
    for(int i = 0; i<m; i++)
        for(int j = 0; j<n; j++)
            K.d[i][j] = Kdt[i][j];
    
    for(int i = 0; i<m; i++)
        for(int j = 0; j<p; j++)
            Ki.d[i][j] = Kidt[i][j];
    
    Pi.d[0][0] = 1258.69;
    Pi.d[0][1] = 158.6114;
        
    printf("hellokf\n");
    return 0;
}

/**
 * This is the IMU interrupt function.
 * It will read in the sensor values
 * Perform sensor fusion to produce y = [z,phi,theta,psi]
 * Implement a kalman filter to estimate the full state Xest
 */
static void __state_estimate(void)
{
    static int bmp_sample_counter = 0;
    
    // do first-run filter setup
    if(kf.step==0)
    {
        kf.x_est.d[0] = bmp_data.alt_m;
    }
    
    
    y.d[0] = bmp_data.alt_m - innit_alt;
    y.d[1] = mpu_data.fused_TaitBryan[TB_ROLL_Y];
    y.d[2] = mpu_data.fused_TaitBryan[TB_PITCH_X];
    y.d[3] = mpu_data.fused_TaitBryan[TB_YAW_Z];
    y.d[4] = 0;//w1;
    y.d[5] = 0;//w4;
    
    // print fused TaitBryan Angles and Altitude
    //printf("%9.2fm %6.1f %6.1f %6.1f |",y.d[0],y.d[1],y.d[2],y.d[3]);
                                   	
    rc_kalman_update_lin(&kf, u, y);
    
    // print result
    //printf("\rpos: %5.2f vel: %5.2f u: %5.2f y: %5.2f   \n ", kf.x_est.d[0], kf.x_est.d[1], u.d[0], y.d[0]);
    rc_vector_print(kf.x_est);
    printf("\n");
    
    fflush(stdout);
    bmp_sample_counter++;
    if(bmp_sample_counter>=BMP_RATE_DIV)
    {
        // perform the i2c reads to the sensor, on bad read just try later
        if(rc_bmp_read(&bmp_data)) return;
        bmp_sample_counter=0;
    }
    return;
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
    __sensor_setup();
    __kalman_setup();
    
    // set signal handler so the loop can exit cleanly to ^C cmd
    signal(SIGINT, __signal_handler);
    running = 1;
    
    // init barometer sample at 87Hz, no internal filter
    if(rc_bmp_init(BMP_OVERSAMPLE_16, BMP_FILTER_16)) 
    {
        printf("rc_bmp_init_failed\n");
        return -1;
    }
    
    if(rc_bmp_read(&bmp_data)) 
    {
        return -1;
    }
    
    printf("waiting for sensors to settle\n");
    fflush(stdout);
    rc_usleep(10000000);
    
    // calculate altitude offset
    for (int i = 0; i < 10;i++)
    {
        rc_bmp_read(&bmp_data);
        innit_alt += bmp_data.alt_m;
        rc_usleep(100000);
    }
    innit_alt = innit_alt/10;
    printf("helloinit:%5.2f \n",innit_alt);
    
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
    
    // Interrupt function to call state estimator.
    rc_mpu_set_dmp_callback(&__state_estimate);
        
    // main program loop
    while(running)
    {
        /*
        ye.d[0] = r.d[0] - kf.x_est.d[0];
        ye.d[1] = r.d[0] - kf.x_est.d[3];
        ye.d[2] = r.d[0] - kf.x_est.d[5];
        ye.d[3] = r.d[0] - kf.x_est.d[7];
        
        xe.d[0] = xe.d[0] + T*ye.d[0];
        xe.d[1] = xe.d[1] + T*ye.d[1];
        xe.d[2] = xe.d[2] + T*ye.d[2];
        xe.d[3] = xe.d[3] + T*ye.d[3];
        
        rc_matrix_times_col_vec(K,kf.x_est,&u1);
        rc_matrix_times_col_vec(Ki,xe,&u2);
        
        u.d[0] = u1.d[0] + u2.d[0] + u_e[0];
        u.d[1] = u1.d[1] + u2.d[1] + u_e[1];
        u.d[2] = u1.d[2] + u2.d[2] + u_e[2];
        u.d[3] = u1.d[3] + u2.d[3] + u_e[3];
        u.d[4] = u1.d[4] + u2.d[4] + u_e[4];
        u.d[5] = u1.d[5] + u2.d[5] + u_e[5];
        
        rc_vector_print(u);
        */
        rc_usleep(10000); // run at 100Hz
    }
    
    // shut things down
    rc_bmp_power_off();
    rc_mpu_power_off();
    rc_kalman_free(&kf);
    printf("\n");
    fflush(stdout);
        
    return 0;
}
