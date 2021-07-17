#include "globals.h"
#include "errors.h"

void createLog(uint8_t *logBuffer)
{
    currentStatus.spark ^= (-currentStatus.hasSync ^ currentStatus.spark) & (1U << BIT_SPARK_SYNC); //Set the sync bit of the Spark variable to match the hasSync variable

    logBuffer[0] = currentStatus.secl; //secl is simply a counter that increments each second. Used to track unexpected resets (Which will reset this count to 0)
    logBuffer[1] = currentStatus.status1; //status1 Bitfield
    logBuffer[2] = currentStatus.engine; //Engine Status Bitfield
    logBuffer[3] = currentStatus.syncLossCounter;
    logBuffer[4] = lowByte(currentStatus.MAP); //2 bytes for MAP
    logBuffer[5] = highByte(currentStatus.MAP);
    logBuffer[6] = (byte)(currentStatus.IAT + CALIBRATION_TEMPERATURE_OFFSET); //mat
    logBuffer[7] = (byte)(currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET); //Coolant ADC
    logBuffer[8] = currentStatus.batCorrection; //Battery voltage correction (%)
    logBuffer[9] = currentStatus.battery10; //battery voltage
    logBuffer[10] = currentStatus.O2; //O2
    logBuffer[11] = currentStatus.egoCorrection; //Exhaust gas correction (%)
    logBuffer[12] = currentStatus.iatCorrection; //Air temperature Correction (%)
    logBuffer[13] = currentStatus.wueCorrection; //Warmup enrichment (%)
    logBuffer[14] = lowByte(currentStatus.RPM); //rpm HB
    logBuffer[15] = highByte(currentStatus.RPM); //rpm LB
    logBuffer[16] = (byte)(currentStatus.AEamount >> 1); //TPS acceleration enrichment (%) divided by 2 (Can exceed 255)
    logBuffer[17] = currentStatus.corrections; //Total GammaE (%)
    logBuffer[18] = currentStatus.VE; //Current VE (%). Can be equal to VE1 or VE2 or a calculated value from both of them
    logBuffer[19] = currentStatus.VE1; //VE 1 (%)
    logBuffer[20] = currentStatus.VE2; //VE 2 (%)
    logBuffer[21] = currentStatus.afrTarget;
    logBuffer[22] = currentStatus.tpsDOT; //TPS DOT
    logBuffer[23] = currentStatus.advance;
    logBuffer[24] = currentStatus.TPS; // TPS (0% to 100%)
    //Need to split the int loopsPerSecond value into 2 bytes
    if(currentStatus.loopsPerSecond > 60000) { currentStatus.loopsPerSecond = 60000;}
    logBuffer[25] = lowByte(currentStatus.loopsPerSecond);
    logBuffer[26] = highByte(currentStatus.loopsPerSecond);

    //The following can be used to show the amount of free memory
    currentStatus.freeRAM = freeRam();
    logBuffer[27] = lowByte(currentStatus.freeRAM); //(byte)((currentStatus.loopsPerSecond >> 8) & 0xFF);
    logBuffer[28] = highByte(currentStatus.freeRAM);

    logBuffer[29] = (byte)(currentStatus.boostTarget >> 1); //Divide boost target by 2 to fit in a byte
    logBuffer[30] = (byte)(currentStatus.boostDuty / 100);
    logBuffer[31] = currentStatus.spark; //Spark related bitfield

    //rpmDOT must be sent as a signed integer
    logBuffer[32] = lowByte(currentStatus.rpmDOT);
    logBuffer[33] = highByte(currentStatus.rpmDOT);

    logBuffer[34] = currentStatus.ethanolPct; //Flex sensor value (or 0 if not used)
    logBuffer[35] = currentStatus.flexCorrection; //Flex fuel correction (% above or below 100)
    logBuffer[36] = currentStatus.flexIgnCorrection; //Ignition correction (Increased degrees of advance) for flex fuel

    logBuffer[37] = currentStatus.idleLoad;
    logBuffer[38] = currentStatus.testOutputs;

    logBuffer[39] = currentStatus.O2_2; //O2
    logBuffer[40] = currentStatus.baro; //Barometer value

    logBuffer[41] = lowByte(currentStatus.canin[0]);
    logBuffer[42] = highByte(currentStatus.canin[0]);
    logBuffer[43] = lowByte(currentStatus.canin[1]);
    logBuffer[44] = highByte(currentStatus.canin[1]);
    logBuffer[45] = lowByte(currentStatus.canin[2]);
    logBuffer[46] = highByte(currentStatus.canin[2]);
    logBuffer[47] = lowByte(currentStatus.canin[3]);
    logBuffer[48] = highByte(currentStatus.canin[3]);
    logBuffer[49] = lowByte(currentStatus.canin[4]);
    logBuffer[50] = highByte(currentStatus.canin[4]);
    logBuffer[51] = lowByte(currentStatus.canin[5]);
    logBuffer[52] = highByte(currentStatus.canin[5]);
    logBuffer[53] = lowByte(currentStatus.canin[6]);
    logBuffer[54] = highByte(currentStatus.canin[6]);
    logBuffer[55] = lowByte(currentStatus.canin[7]);
    logBuffer[56] = highByte(currentStatus.canin[7]);
    logBuffer[57] = lowByte(currentStatus.canin[8]);
    logBuffer[58] = highByte(currentStatus.canin[8]);
    logBuffer[59] = lowByte(currentStatus.canin[9]);
    logBuffer[60] = highByte(currentStatus.canin[9]);
    logBuffer[61] = lowByte(currentStatus.canin[10]);
    logBuffer[62] = highByte(currentStatus.canin[10]);
    logBuffer[63] = lowByte(currentStatus.canin[11]);
    logBuffer[64] = highByte(currentStatus.canin[11]);
    logBuffer[65] = lowByte(currentStatus.canin[12]);
    logBuffer[66] = highByte(currentStatus.canin[12]);
    logBuffer[67] = lowByte(currentStatus.canin[13]);
    logBuffer[68] = highByte(currentStatus.canin[13]);
    logBuffer[69] = lowByte(currentStatus.canin[14]);
    logBuffer[70] = highByte(currentStatus.canin[14]);
    logBuffer[71] = lowByte(currentStatus.canin[15]);
    logBuffer[72] = highByte(currentStatus.canin[15]);

    logBuffer[73] = currentStatus.tpsADC;
    logBuffer[74] = getNextError();

    logBuffer[75] = lowByte(currentStatus.PW1); //Pulsewidth 1 multiplied by 10 in ms. Have to convert from uS to mS.
    logBuffer[76] = highByte(currentStatus.PW1); //Pulsewidth 1 multiplied by 10 in ms. Have to convert from uS to mS.
    logBuffer[77] = lowByte(currentStatus.PW2); //Pulsewidth 2 multiplied by 10 in ms. Have to convert from uS to mS.
    logBuffer[78] = highByte(currentStatus.PW2); //Pulsewidth 2 multiplied by 10 in ms. Have to convert from uS to mS.
    logBuffer[79] = lowByte(currentStatus.PW3); //Pulsewidth 3 multiplied by 10 in ms. Have to convert from uS to mS.
    logBuffer[80] = highByte(currentStatus.PW3); //Pulsewidth 3 multiplied by 10 in ms. Have to convert from uS to mS.
    logBuffer[81] = lowByte(currentStatus.PW4); //Pulsewidth 4 multiplied by 10 in ms. Have to convert from uS to mS.
    logBuffer[82] = highByte(currentStatus.PW4); //Pulsewidth 4 multiplied by 10 in ms. Have to convert from uS to mS.

    logBuffer[83] = currentStatus.status3;

    logBuffer[84] = currentStatus.nChannels;
    logBuffer[85] = lowByte(currentStatus.fuelLoad);
    logBuffer[86] = highByte(currentStatus.fuelLoad);
    logBuffer[87] = lowByte(currentStatus.ignLoad);
    logBuffer[88] = highByte(currentStatus.ignLoad);
    logBuffer[89] = lowByte(currentStatus.dwell);
    logBuffer[90] = highByte(currentStatus.dwell);
    logBuffer[91] = currentStatus.CLIdleTarget;
    logBuffer[92] = currentStatus.mapDOT;
    logBuffer[93] = scaleCrankAngleDown(currentStatus.vvt1Angle);
    logBuffer[94] = scaleCrankAngleDown(currentStatus.vvt1TargetAngle);
    logBuffer[95] = (byte) currentStatus.vvt1Duty;
    logBuffer[96] = lowByte(currentStatus.flexBoostCorrection);
    logBuffer[97] = highByte(currentStatus.flexBoostCorrection);
    logBuffer[98] = currentStatus.baroCorrection;
    logBuffer[99] = currentStatus.VE; //Current VE (%). Can be equal to VE1 or VE2 or a calculated value from both of them
    logBuffer[100] = currentStatus.ASEValue; //Current ASE (%)
    logBuffer[101] = lowByte(currentStatus.vss);
    logBuffer[102] = highByte(currentStatus.vss);
    logBuffer[103] = currentStatus.gear;
    logBuffer[104] = currentStatus.fuelPressure;
    logBuffer[105] = currentStatus.oilPressure;
    logBuffer[106] = currentStatus.wmiPW;
    logBuffer[107] = currentStatus.status4;
    logBuffer[108] = scaleCrankAngleDown(currentStatus.vvt2Angle);
    logBuffer[109] = scaleCrankAngleDown(currentStatus.vvt2TargetAngle);
    logBuffer[110] = (byte) currentStatus.vvt2Duty;
    logBuffer[111] = currentStatus.advance1;
    logBuffer[112] = currentStatus.advance2;

}