/*
 * cetiConfig.h
 *
 *      Author: Kelly Ostrom
 */

#ifndef INC_CETICONFIG_H_
#define INC_CETICONFIG_H_

/** CETI Tag Configuration
*
*------------------------------------------------------------------------------
* Depth Thresholds P1 and P2
* If pressure < P1, device is considered at the surface
* If pressure > P2, device is considered submerged
* Between P1 and P2 is a hysteresis band
* The state machine uses this information to turn the Recovery
* board on and off depending on the phase of the deployment
* Units are bar absolute with reference value of 1 bar (absolute) abs
* See Keller 4LD...9LD data sheet
*/
#define P1 0.04
#define P2 0.10

/*------------------------------------------------------------------------------
* Battery Thresholds V1 and V2
* This is used as a barebones method to determine state of charge
* The Tag's gas gauge has more sophisticated features available
* This is just to get started with
* V_LOW - battery is low, time to release from the whale!
* V_CRIT - battery is critical, time to put the system to sleep
* Units are volts
*/
#define V_LOW 6.4
#define V_CRIT 6.2

/*------------------------------------------------------------------------------
* Deployment Timeout
* Indepedent of battery state of charge, if deployment
* time reaches this value, the tag will release
* The battery thresholds take precedence
* Units are minutes, use integers only
* 240
*/
#define T0 240
//------------------------------------------------------------------------------

#endif /* INC_CETICONFIG_H_ */
