#ifndef __TMC4671_WRAPPER_H__
#define __TMC4671_WRAPPER_H__

#include <mbed.h>

extern "C"
{
#include "TMC4671.h"
}

enum Mode {Open_loop, Abn_encoder, Digital_hall};

class Motor_tmc4671
{
  private:
    uint16_t _icID;
    Mode _mode;

  public:
    /**
     * @brief Get the icID of the motor
     * 
     * @return uint16_t The icID of the motor
     */
    uint16_t get_icID();

    /**
     * @brief Get the mode of the motor (Open_loop, Abn_encoder or Digital_hall)
     * 
     * @return Mode The current mode of the motor
     */
    Mode get_mode();

    /**
     * @brief Set the mode of the motor
     * 
     * @param mode The mode to set
     */
    void set_mode(Mode mode);

    Motor_tmc4671(uint16_t icID, Mode mode);

    /**
     * @brief Configure initial registers of the motor and initialize motor ABN encoder.
     *        Must be called before motor usage in Abn_encoder mode
     * 
     * @param debug Value to test the initial configuration by executing small movements
     */
    void init_config(int debug);

    /**
     * @brief Configure the initial SPI object for communication with all motors
     *        Must be called before using any Trinamic motor.
     */
    static void init_SPI();

    /**
     * @brief Read a value from a TMC4671 register
     * 
     * @param address Register address
     * @return int32_t Value read from the register
     */
    int32_t read_SPI(uint8_t address);

    /**
     * @brief Write a value to a TMC4671 register
     * 
     * @param address Register address
     * @param value Value to write
     */
    void write_SPI(uint8_t address, int32_t value);

    /**
     * @brief Set the motor speed in rad/s
     * 
     * @param speed Desired speed in rad/s
     */
    void set_speed(float speed);

    /**
     * @brief Get the current motor speed in rad/s
     * 
     * @return float Current speed in rad/s
     */
    float get_speed();

    /**
     * @brief Set the torque of the motor
     * 
     * @param speed Desired torque
     */
    void set_torque_raw(int32_t torque);

    /**
     * @brief Get the torque of the motor
     * 
     * @return float The current torque of the motor
     */
    int32_t get_torque_raw();


    /**
     * @brief Switch the mode of the motor
     * 
     * @param mode Mode on which the motor should be switched
     */
    void switch_mode(Mode mode);
};

#endif //__TMC4671_WRAPPER_H__