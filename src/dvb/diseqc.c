#include <time.h>
#include <linux/dvb/frontend.h>
#include <sys/ioctl.h>

#include "tvheadend.h"
#include "diseqc.h"

/*--------------------------------------------------------------------------*/

static inline void
msleep(uint32_t msec)
{
  struct timespec req = { msec / 1000, 1000000 * (msec % 1000) };

  while (nanosleep(&req, &req))
    ;
}

int
diseqc_send_msg(int fe_fd, __u8 framing_byte, __u8 address, __u8 cmd,
                    __u8 data_1,  __u8 data_2, __u8 data_3, __u8 msg_len)
{
  int err;
  struct dvb_diseqc_master_cmd message;

  tvhtrace("diseqc", "sending %X %X %X %X %X %X",
           framing_byte, address, cmd, data_1, data_2, data_3);
  
  message.msg[0] = framing_byte;
  message.msg[1] = address;
  message.msg[2] = cmd;
  message.msg[3] = data_1;
  message.msg[4] = data_2;
  message.msg[5] = data_3;
  message.msg_len = msg_len;
  if ((err = ioctl(fe_fd, FE_DISEQC_SEND_MASTER_CMD, &message))) {
	tvhlog(LOG_ERR, "diseqc", "error sending diseqc command");
    return err;
  }
  return 0;
}

int
diseqc_setup(int fe_fd, int lnb_num, int voltage, int band,
              uint32_t version, uint32_t repeats)
{
  int i = (lnb_num % 4) * 4 + voltage * 2 + (band ? 1 : 0);
  int j = lnb_num / 4;
  int k, err;

  tvhtrace("diseqc",
           "fe_fd=%i, lnb_num=%i, voltage=%i, band=%i, version=%i, repeats=%i",
           fe_fd, lnb_num, voltage, band, version, repeats);

  /* verify lnb number and diseqc data */
  if(lnb_num < 0 || lnb_num >=64 || i < 0 || i >= 16 || j < 0 || j >= 16)
    return -1;

  /* turn off continuous tone */
  tvhtrace("diseqc", "disabling continuous tone");
  if ((err = ioctl(fe_fd, FE_SET_TONE, SEC_TONE_OFF))) {
	tvhlog(LOG_ERR, "diseqc", "error trying to turn off continuous tone");
    return err;
  }

  /* set lnb voltage */
  tvhtrace("diseqc", "setting lnb voltage to %iV", (i/2) % 2 ? 18 : 13);
  if ((err = ioctl(fe_fd, FE_SET_VOLTAGE, (i/2) % 2 ? SEC_VOLTAGE_18 : SEC_VOLTAGE_13))) {
	tvhlog(LOG_ERR, "diseqc", "error setting lnb voltage");
    return err;
  }
  msleep(15);

  if (repeats == 0) { /* uncommited msg, wait 15ms, commited msg */
    if ((err = diseqc_send_msg(fe_fd, 0xE0, 0x10, 0x39, 0xF0 | j, 0, 0, 4)))
      return err;
    msleep(15);
    if ((err = diseqc_send_msg(fe_fd, 0xE0, 0x10, 0x38, 0xF0 | i, 0, 0, 4)))
      return err;
  } else { /* commited msg, 25ms, uncommited msg, 25ms, commited msg, etc */
    if ((err = diseqc_send_msg(fe_fd, 0xE0, 0x10, 0x38, 0xF0 | i, 0, 0, 4)))
      return err;
    for (k = 0; k < repeats; k++) {
      msleep(25);
      if ((err = diseqc_send_msg(fe_fd, 0xE0, 0x10, 0x39, 0xF0 | j, 0, 0, 4)))
        return err;
      msleep(25);
      if ((err = diseqc_send_msg(fe_fd, 0xE1, 0x10, 0x38, 0xF0 | i, 0, 0, 4)))
        return err;
    }
  }
  msleep(15);

  /* set toneburst */
  tvhtrace("diseqc", (i/4) % 2 ? "sending mini diseqc B" : "sending mini diseqc A");
  if ((err = ioctl(fe_fd, FE_DISEQC_SEND_BURST, (i/4) % 2 ? SEC_MINI_B : SEC_MINI_A))) {
	tvhlog(LOG_ERR, "diseqc", "error sending mini diseqc command");
    return err;
  }
  msleep(15);

  /* set continuous tone */
  tvhtrace("diseqc", i % 2 ? "enabling continous tone" : "disabling continuous tone");
  if ((err = ioctl(fe_fd, FE_SET_TONE, i % 2 ? SEC_TONE_ON : SEC_TONE_OFF))) {
	tvhlog(LOG_ERR, "diseqc", "error setting continuous tone");
    return err;
  }
  return 0;
}

int
diseqc_rotor_gotox(int fe_fd, int pos) {
  int err;

  /* turn off continuous tone */
  tvhtrace("diseqc", "disabling continuous tone");
  if ((err = ioctl(fe_fd, FE_SET_TONE, SEC_TONE_OFF))) {
	tvhlog(LOG_ERR, "diseqc", "error trying to turn off continuous tone");
    return err;
  }

  /* set lnb voltage */
  tvhtrace("diseqc", "setting lnb voltage to 18V");
  if ((err = ioctl(fe_fd, FE_SET_VOLTAGE, SEC_VOLTAGE_18))) {
	tvhlog(LOG_ERR, "diseqc", "error setting lnb voltage");
    return err;
  }
  msleep(15);

  tvhtrace("diseqc", "GOTOX goto stored position %d", pos);

  if ((err = diseqc_send_msg(fe_fd, 0xE0, 0x31, 0x6B, 0x00 | pos, 0, 0, 4))) {
	tvhlog(LOG_ERR, "diseqc", "error sending USALS goto diseqc command");
    return err;
  }
  msleep(15);

  return 0;
}

int
diseqc_rotor_usals(int fe_fd, double lat, double lng, double pos) {

  /*
     USALS rotor logic adapted from tune-s2 
     http://updatelee.blogspot.com/2010/09/tune-s2.html 
    
     Antenna Alignment message data format: 
     http://www.dvb.org/technology/standards/A155-3_DVB-RCS2_Higher_layer_satellite_spec.pdf 
  */

  int err;

  /* turn off continuous tone */
  tvhtrace("diseqc", "disabling continuous tone");
  if ((err = ioctl(fe_fd, FE_SET_TONE, SEC_TONE_OFF))) {
	tvhlog(LOG_ERR, "diseqc", "error trying to turn off continuous tone");
    return err;
  }

  /* set lnb voltage */
  tvhtrace("diseqc", "setting lnb voltage to 18V");
  if ((err = ioctl(fe_fd, FE_SET_VOLTAGE, SEC_VOLTAGE_18))) {
	tvhlog(LOG_ERR, "diseqc", "error setting lnb voltage");
    return err;
  }
  msleep(15);

  double r_eq = 6378.14;
  double r_sat = 42164.57;
    
  double site_lat  = (lat * TO_RADS);
  double site_lng = (lng * TO_RADS);
  double sat_lng  = (pos * TO_RADS);
        
  double dishVector[3] = {
    (r_eq * cos(site_lat)),
    0,
    (r_eq * sin(site_lat))
  };
    
  double satVector[3] = {
    (r_sat * cos(site_lng - sat_lng)),
    (r_sat * sin(site_lng - sat_lng)),
    0
  };
    
  double satPointing[3] = {
    (satVector[0] - dishVector[0]),
    (satVector[1] - dishVector[1]),
    (satVector[2] - dishVector[2])
  };
        
  double motor_angle = ((atan(satPointing[1] / satPointing[0])) * TO_DEC);

  tvhtrace("diseqc", "USALS goto %.1f%c (motor angle is %.2f %s)", fabs(pos), 
           (pos > 0.0) ? 'E' : 'W', motor_angle, (motor_angle > 0.0) ? "counterclockwise" : "clockwise");
        
  int sixteenths = ((fabs(motor_angle) * 16.0) + 0.5);
    
  int angle_1 = (((motor_angle > 0.0) ? 0xd0 : 0xe0) | (sixteenths >> 8));
  int angle_2 = (sixteenths & 0xff);

  if ((err = diseqc_send_msg(fe_fd, 0xE0, 0x31, 0x6E, angle_1, angle_2, 0, 5))) {
	tvhlog(LOG_ERR, "diseqc", "error sending USALS goto diseqc command");
    return err;
  }
  msleep(15);

  return 0;
}

int
diseqc_voltage_off(int fe_fd)
{
  int err;
  
  tvhtrace("diseqc", "sending diseqc voltage off command");
  if ((err = ioctl(fe_fd, FE_SET_VOLTAGE, SEC_VOLTAGE_OFF))) {
    tvhlog(LOG_ERR, "diseqc", "error sending diseqc voltage off command");
    return err;
  }
  return 0;
}
