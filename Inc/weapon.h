#ifndef PER_WEAPON_H_
#define PER_WEAPON_H_

#include <stdbool.h>

#include "utility.h"

typedef struct
{
  PER_IOPair_t io;
  
  bool is_active;
  uint32_t activation_time;
  bool is_timed_out;
} PER_Weapon_t;

PER_Weapon_t PER_Weapon_Init(PER_IOPair_t io);
void PER_Weapon_Process(PER_Weapon_t* wpn);
void PER_Weapon_Enable(PER_Weapon_t* wpn);
void PER_Weapon_Disable(PER_Weapon_t* wpn);

#endif // PER_WEAPON_H_