#include "tmc4671_wrapper.h"
#include <cmath>

extern "C"
{
  void tmc4671_readWriteSPI(uint16_t icID, uint8_t *data, size_t data_length);
}

#define RADIUS 0.025

#define MODE_ERROR -1

#define NUM_OF_SLAVES 5
static DigitalOut chip_select[5] = {SPI_CS_DRV1, SPI_CS_DRV2, SPI_CS_DRV3, SPI_CS_DRV4, SPI_CS_DRV5};

SPI device(SPI_MOSI_DRV, SPI_MISO_DRV, SPI_SCK_DRV);

// icID is used to identify the chip to be selected
void tmc4671_readWriteSPI(uint16_t icID, uint8_t *data, size_t data_length)
{
  // the _peripheral attribute is a spi_peripheral_s type, and correspond to the slave

  // acquire exclusive access to this SPI bus
  device.lock();

  //print_hexa_format(data, "Command: ", data_length);

  // operator = has been redefined
  // is it related to the pin we want, or the selection of the chip (slave select ?)
  chip_select[icID] = 0;

  int nb_bits_used = 0;

  // data[0] represents the write_bit
  nb_bits_used = device.write((char *)data, data_length, (char *)data, data_length);

  //print_hexa_format(data, "Response: ", data_length);

  // we can check if nb_bits_used == data_length

  chip_select[icID] = 1;

  // release exclusive access to this SPI bus
  device.unlock();
}

void Motor_tmc4671::write_SPI(uint8_t address, int32_t value)
{
  tmc4671_writeRegister(this->get_icID(), address, value);
}

int32_t Motor_tmc4671::read_SPI(uint8_t address)
{
  return tmc4671_readRegister(this->get_icID(), address);
}

Motor_tmc4671::Motor_tmc4671(uint16_t icID, Mode mode):_icID(icID),_mode(mode)
{

}

void Motor_tmc4671::init_SPI()
{
  // change the frequency depending of the datasheet
  device.frequency(5000000);

  // configure the format according to the datasheet
  // the mode should be =3 according to the phase and polarity
  // 8 corresponds to the length of each datagram
  device.format(8, 3);

  // assure that none of the slaves are selected
  for (int i = 0; i < NUM_OF_SLAVES; i++)
  {
    chip_select[i] = 1;
  }
}

uint16_t Motor_tmc4671::get_icID()
{
  return this->_icID;
}

Mode Motor_tmc4671::get_mode()
{
  return this->_mode;
}

void Motor_tmc4671::set_mode(Mode mode)
{
  this->_mode = mode;
}

void Motor_tmc4671::init_config(int debug)
{
  // ===== General settings =====
  uint16_t icID = this->get_icID();

  // Limits
  tmc4671_writeRegister(icID, TMC4671_PID_TORQUE_FLUX_LIMITS, 0x000003E8);

  // PI settings
  tmc4671_writeRegister(icID, TMC4671_PID_TORQUE_P_TORQUE_I, 0x01000100);
  tmc4671_writeRegister(icID, TMC4671_PID_FLUX_P_FLUX_I, 0x01000100);
  tmc4671_writeRegister(icID, TMC4671_PID_VELOCITY_P_VELOCITY_I, 0x01000100);
  tmc4671_writeRegister(icID, TMC4671_PID_POSITION_P_POSITION_I, 0x00400040);

  // Motor type &  PWM configuration
  tmc4671_writeRegister(icID, TMC4671_MOTOR_TYPE_N_POLE_PAIRS, 0x00030008);
  tmc4671_writeRegister(icID, TMC4671_PWM_POLARITIES, 0x00000000);
  tmc4671_writeRegister(icID, TMC4671_PWM_MAXCNT, 0x00000F9F);
  tmc4671_writeRegister(icID, TMC4671_PWM_BBM_H_BBM_L, 0x00001919);
  tmc4671_writeRegister(icID, TMC4671_PWM_SV_CHOP, 0x00000007);

  // ADC configuration
  tmc4671_writeRegister(icID, TMC4671_ADC_I_SELECT, 0x24000100);
  tmc4671_writeRegister(icID, TMC4671_DSADC_MCFG_B_MCFG_A, 0x00100010);
  tmc4671_writeRegister(icID, TMC4671_DSADC_MCLK_A, 0x20000000);
  tmc4671_writeRegister(icID, TMC4671_DSADC_MCLK_B, 0x20000000);
  tmc4671_writeRegister(icID, TMC4671_DSADC_MDEC_B_MDEC_A, 0x014E014E);
  tmc4671_writeRegister(icID, TMC4671_ADC_I0_SCALE_OFFSET, 0x00328394);
  tmc4671_writeRegister(icID, TMC4671_ADC_I1_SCALE_OFFSET, 0x003283FE);

  // ===== Open loop configuration =====

  switch (this->get_mode()) {
    case Open_loop:
      // Open loop settings
      tmc4671_writeRegister(icID, TMC4671_OPENLOOP_MODE, 0x00000000);
      tmc4671_writeRegister(icID, TMC4671_OPENLOOP_ACCELERATION, 0x0000003C);
      tmc4671_writeRegister(icID, TMC4671_OPENLOOP_VELOCITY_TARGET, 0x00000000);
      // Note: the IDE gave this value to use: 0x0000000A

      // Feedback selection
      tmc4671_writeRegister(icID, TMC4671_PHI_E_SELECTION, 0x00000002);
      tmc4671_writeRegister(icID, TMC4671_VELOCITY_SELECTION, 0x00000000);
      tmc4671_writeRegister(icID, TMC4671_POSITION_SELECTION, 0x00000000);

      // UQ_UD extern (only mode available for open loop)
      tmc4671_writeRegister(icID, TMC4671_MODE_RAMP_MODE_MOTION, 0x00000008);

      // ===== Open loop test drive =====

      if(debug) {
          tmc4671_writeRegister(icID, TMC4671_UQ_UD_EXT, 0x00000913);
          // Switch to open loop velocity mode
          tmc4671_writeRegister(icID, TMC4671_MODE_RAMP_MODE_MOTION, 0x00000008);

          // Rotate right
          tmc4671_writeRegister(icID, TMC4671_OPENLOOP_VELOCITY_TARGET, 0x0000003C);
          wait_us(2000000);

          // Rotate left
          tmc4671_writeRegister(icID, TMC4671_OPENLOOP_VELOCITY_TARGET, 0xFFFFFFC4);
          wait_us(2000000);

          // Stop
          tmc4671_writeRegister(icID, TMC4671_OPENLOOP_VELOCITY_TARGET, 0x00000000);
          wait_us(2000000);
          tmc4671_writeRegister(icID, TMC4671_UQ_UD_EXT, 0x00000000);
      } 
    break;
    case Digital_hall:
      // =====  Digital hall configuration =====

      // Digital hall settings
      tmc4671_writeRegister(icID, TMC4671_HALL_MODE, 0x00000000);
      tmc4671_writeRegister(icID, TMC4671_HALL_PHI_E_PHI_M_OFFSET, 0x00000000);
      // Note: the IDE gave this value to use: 0x0FA00000
      
      // Feedback selection
      tmc4671_writeRegister(icID, TMC4671_PHI_E_SELECTION, 0x00000005);
      tmc4671_writeRegister(icID, TMC4671_VELOCITY_SELECTION, 0x00000000);
      tmc4671_writeRegister(icID, TMC4671_POSITION_SELECTION, 0x00000000);

      // Velocity mode (default)
      tmc4671_writeRegister(icID, TMC4671_MODE_RAMP_MODE_MOTION, 0x00000002);

      // ===== Digital hall test drive =====

      if(debug) {
          // Switch to torque mode
          tmc4671_writeRegister(icID, TMC4671_MODE_RAMP_MODE_MOTION, 0x00000001);

          // Rotate right
          tmc4671_writeRegister(icID, TMC4671_PID_TORQUE_FLUX_TARGET, 0x03E80000);
          wait_us(2000000);

          // Rotate left
          tmc4671_writeRegister(icID, TMC4671_PID_TORQUE_FLUX_TARGET, 0xFC180000);
          wait_us(2000000);

          // Stop
          tmc4671_writeRegister(icID, TMC4671_PID_TORQUE_FLUX_TARGET, 0x00000000);
          wait_us(2000000);
      }
    break;
    case Abn_encoder:
      // ===== ABN encoder configuration =====

      // ABN encoder settings
      tmc4671_writeRegister(icID, TMC4671_ABN_DECODER_MODE, 0x00000000);
      tmc4671_writeRegister(icID, TMC4671_ABN_DECODER_PPR, 0x000007D0);

      // Init encoder (mode 0)
      tmc4671_writeRegister(icID, TMC4671_MODE_RAMP_MODE_MOTION, 0x00000008);
      tmc4671_writeRegister(icID, TMC4671_ABN_DECODER_PHI_E_PHI_M_OFFSET, 0x00000000);
      tmc4671_writeRegister(icID, TMC4671_PHI_E_SELECTION, 0x00000001);
      tmc4671_writeRegister(icID, TMC4671_PHI_E_EXT, 0x00000000);
      tmc4671_writeRegister(icID, TMC4671_UQ_UD_EXT, 0x00000FA0);
        
      wait_us(1000000);
      tmc4671_writeRegister(icID, TMC4671_ABN_DECODER_COUNT, 0x00000000);

      // Feedback selection
      tmc4671_writeRegister(icID, TMC4671_PHI_E_SELECTION, 0x00000003);
      tmc4671_writeRegister(icID, TMC4671_VELOCITY_SELECTION, 0x00000000);
      tmc4671_writeRegister(icID, TMC4671_POSITION_SELECTION, 0x00000000);

      // Velocity mode (default)
      tmc4671_writeRegister(icID, TMC4671_MODE_RAMP_MODE_MOTION, 0x00000002);

      // ===== ABN encoder test drive =====

      if(debug) {
          // Feedback selection
          tmc4671_writeRegister(icID, TMC4671_PHI_E_SELECTION, 0x00000003);
          tmc4671_writeRegister(icID, TMC4671_VELOCITY_SELECTION, 0x00000009);

          // Switch to torque mode
          tmc4671_writeRegister(icID, TMC4671_MODE_RAMP_MODE_MOTION, 0x00000001);

          // Rotate right
          tmc4671_writeRegister(icID, TMC4671_PID_TORQUE_FLUX_TARGET, 0x03E80000);
          wait_us(2000000);

          // Rotate left
          tmc4671_writeRegister(icID, TMC4671_PID_TORQUE_FLUX_TARGET, 0xFC180000);
          wait_us(2000000);

          // Stop
          tmc4671_writeRegister(icID, TMC4671_PID_TORQUE_FLUX_TARGET, 0x00000000);
      }
    break;
  }
}

void Motor_tmc4671::switch_mode(Mode mode)
{
  this->set_mode(mode);
  if (mode == Open_loop)
  {
    // Switch to open loop velocity mode
    tmc4671_writeRegister(this->get_icID(), TMC4671_MODE_RAMP_MODE_MOTION, 0x00000008);
    // TODO: also change PHI_E_SELECTION (see initialization)
  }
  //else if (mode == Abn_encoder || mode == Digital_hall)
  //{
  //  // Switch to torque mode
  //  tmc4671_writeRegister(this->get_icID(), TMC4671_MODE_RAMP_MODE_MOTION, 0x00000001);
  //}
}

void Motor_tmc4671::set_speed(float speed)
{
  int32_t rpm = (speed*60)/(2*M_PI*RADIUS);
  int32_t rpe = rpm * 8;
  if (this->get_mode() == Open_loop)
    this->write_SPI(TMC4671_OPENLOOP_VELOCITY_TARGET, rpe);
  else
    tmc4671_setTargetVelocity(this->get_icID(), rpe);
}

float Motor_tmc4671::get_speed()

{
  int32_t rpm_elec ; //rpm*8
  if (this->get_mode() == Open_loop)
    rpm_elec = this->read_SPI(TMC4671_OPENLOOP_VELOCITY_ACTUAL);
  else
    rpm_elec = tmc4671_getActualVelocity(this->get_icID());

  float result = (rpm_elec/8) * (2*M_PI*RADIUS)/60;

  return result;
}

void Motor_tmc4671::set_torque_raw(int32_t torque)
{
  if (this->get_mode() == Abn_encoder || this->get_mode() == Digital_hall)
  {
    tmc4671_setTargetTorque_raw(this->get_icID(), torque);
  }
}

int32_t Motor_tmc4671::get_torque_raw()
{
  if (this->get_mode() == Abn_encoder || this->get_mode() == Digital_hall)
  {
    return tmc4671_getActualTorque_raw(this->get_icID());
  }
  return MODE_ERROR;
}