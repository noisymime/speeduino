#pragma once

#include "statuses.h"

/** \enum LoadSource
 * @brief The load source for various tables
 * */
enum LoadSource {
  /** Manifold Absolute Pressure (MAP). Aka Intake MAP (IMAP). 
   * I.e. a pressure sensor that reads the pressure (positive or negative) 
   * in the intake manifold */
  LOAD_SOURCE_MAP,
  /** Throttle Position Sensor (TPS)*/
  LOAD_SOURCE_TPS,
  /** Ratio of intake pressure to exhaust pressure.
   * A variation of the standard MAP that uses the ratio of inlet manifold pressure to exhaust manifold pressure,
   * something that is particularly useful on turbo engines. */
  LOAD_SOURCE_IMAPEMAP
};

/**
 * @brief Get the load value, based the supplied algorithm
 * 
 * @param algorithm The load algorithm
 * @param current The current system state, from which the load is computed.
 * @return The load.
 */
static inline int16_t getLoad(LoadSource algorithm, const statuses &current) {
  if (algorithm == LOAD_SOURCE_TPS)
  {
    //Alpha-N
    return current.TPS * 2U;
  }
  else if (algorithm == LOAD_SOURCE_IMAPEMAP)
  {
    //IMAP / EMAP
    return ((int16_t)current.MAP * 100) / current.EMAP;
  } else {
    // LOAD_SOURCE_MAP (the default). Aka Speed Density
    return current.MAP;
  }
}