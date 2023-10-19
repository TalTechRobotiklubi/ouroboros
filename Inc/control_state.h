#ifndef CONTROL_STATE_H_
#define CONTROL_STATE_H_

#include <stdbool.h>

#define REC_SW_UP 1000
#define REC_SW_DOWN 2000
#define REC_SW_MID 1500

typedef enum
{
  ReceiverSwitchUp,
  ReceiverSwitchMiddle,
  ReceiverSwitchDown
} ReceiverSwitchState_e;

typedef struct
{
  uint16_t right_horizontal;
  uint16_t right_vertical;
  uint16_t left_vertical;
  uint16_t left_horizontal;

  ReceiverSwitchState_e switch_a;
  ReceiverSwitchState_e switch_b;
  ReceiverSwitchState_e switch_c;
  ReceiverSwitchState_e switch_d;
} ReceiverState_t;

typedef struct
{
  bool robot_active;
  bool engines_active;
  bool weapon_active;
  
  bool commands_recent;
  ReceiverState_t curr_commands;

  uint32_t last_connection_check;
  bool connection_lost;
} ControlState_t;

#define JOIN_TO_WORD(arr) ((*(arr)) | (*((arr) + 1) << 8))

ReceiverState_t DecodeReceiverState(uint8_t* buff)
{
  ReceiverState_t rec = { 0 };

  uint16_t ch1 = JOIN_TO_WORD(buff + 2);
  uint16_t ch2 = JOIN_TO_WORD(buff + 4);
  uint16_t ch3 = JOIN_TO_WORD(buff + 6);
  uint16_t ch4 = JOIN_TO_WORD(buff + 8);
  uint16_t ch5 = JOIN_TO_WORD(buff + 10);
  uint16_t ch6 = JOIN_TO_WORD(buff + 12);
  uint16_t ch7 = JOIN_TO_WORD(buff + 14);
  uint16_t ch8 = JOIN_TO_WORD(buff + 16);

  rec.right_horizontal = ch1;
  rec.right_vertical = ch2;
  rec.left_vertical = ch3;
  rec.left_horizontal = ch4;

  if (ch5 == REC_SW_UP)
    rec.switch_a = ReceiverSwitchUp;
  else
    rec.switch_a = ReceiverSwitchDown;

  if (ch6 == REC_SW_UP)
    rec.switch_b = ReceiverSwitchUp;
  else
    rec.switch_b = ReceiverSwitchDown;

  if (ch7 == REC_SW_UP)
    rec.switch_c = ReceiverSwitchUp;
  else if (ch7 == REC_SW_MID)
    rec.switch_c = ReceiverSwitchMiddle;
  else
    rec.switch_c = ReceiverSwitchDown;

  if (ch8 == REC_SW_UP)
    rec.switch_d = ReceiverSwitchUp;
  else
    rec.switch_d = ReceiverSwitchDown;

  return rec;
}

#undef JOIN_TO_WORD

ControlState_t InitializeControlState()
{
  ControlState_t ret = {
    .robot_active = false,
    .engines_active = false,
    .weapon_active = false,
    
    .commands_recent = false,
    .curr_commands = { 0 },
    .last_connection_check = 0,
    .connection_lost = false
  };
  
  return ret;
}

bool RequiresConnectionCheck(ControlState_t* ctrl_state)
{
  if (ctrl_state->connection_lost)
    return true;

  if (HAL_GetTick() - ctrl_state->last_connection_check > 1000)
    return true;

  return false;
}

bool DoConnectionCheck(ControlState_t* ctrl_state)
{
  uint32_t ppm_line = ADC_Read(&hadc2, 1);
  bool has_ppm = true;

  ctrl_state->last_connection_check = HAL_GetTick();

  if (ppm_line < 100)
  {
    int failed = 0;
    for (int i = 0; i < 5; i++)
    {
      if (ADC_Read(&hadc2, 1) < 100)
        failed++;

      HAL_Delay(10);
    }

    if (failed == 5)
      has_ppm = false;
  }

  if (!has_ppm)
  {
    ctrl_state->connection_lost = true;
    return false;
  }
  else
  {
    if (ctrl_state->connection_lost)
      ctrl_state->connection_lost = false;

    return true;
  }
}

void ApplyControlState(ControlState_t* ctrl_state, ReceiverState_t* rec_state)
{
  if (rec_state->switch_c == ReceiverSwitchUp)
    ctrl_state->robot_active = false;
  else if (rec_state->switch_c == ReceiverSwitchDown)
    ctrl_state->robot_active = true;
  
  if (ctrl_state->robot_active)
  {
    if (rec_state->switch_a == ReceiverSwitchUp)
      ctrl_state->weapon_active = false;
    else
      ctrl_state->weapon_active = true;
    
    if (rec_state->switch_d == ReceiverSwitchUp)
      ctrl_state->engines_active = false;
    else
      ctrl_state->engines_active = true;
  }
  else
  {
    ctrl_state->engines_active = false;
    ctrl_state->weapon_active = false;
  }
  
  ctrl_state->curr_commands = *rec_state;
  ctrl_state->commands_recent = true;
}

#endif // CONTROL_STATE_H_
