#include "weapon.h"

#define ACTIVATION_TIMEOUT 3000

PER_Weapon_t PER_Weapon_Init(PER_IOPair_t io)
{
  PER_Weapon_t wpn = {
      .io = io,
      .is_active = false,
      .activation_time = 0,
      .is_timed_out = false
  };

  return wpn;
}

void PER_Weapon_Process(PER_Weapon_t* wpn)
{
  if (wpn->is_active)
  {
    volatile uint32_t time_now = HAL_GetTick();

    if (!wpn->is_timed_out && (time_now - wpn->activation_time) >= ACTIVATION_TIMEOUT)
    {
      PER_UtilResetIO(&wpn->io);
      wpn->is_timed_out = true;
    }
  }
}

void PER_Weapon_Enable(PER_Weapon_t* wpn)
{
  wpn->activation_time = HAL_GetTick();
  wpn->is_active = true;

  PER_UtilSetIO(&wpn->io);
}

void PER_Weapon_Disable(PER_Weapon_t* wpn)
{
  wpn->is_active = false;
  wpn->is_timed_out = false;

  PER_UtilResetIO(&wpn->io);
}
