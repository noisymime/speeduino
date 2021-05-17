/*
DO NOT EDIT THIS FILE.

It is auto generated and your edits will be overwritten
*/

static void printPage1(Print &target) {
	target.println(F("\nPg 1 Cfg"));
	target.println(configPage1.aseTaperTime);
	target.println(configPage1.aeColdPct);
	target.println(configPage1.aeColdTaperMin);
	target.println(configPage1.aeMode);
	target.println(configPage1.battVCorMode);
	target.println(configPage1.SoftLimitMode);
	target.println(configPage1.useTachoSweep);
	target.println(configPage1.aeApplyMode);
	target.println(configPage1.multiplyMAP);
	print_array(target, configPage1.wueValues);
	target.println(configPage1.crankingPct);
	target.println(configPage1.pinMapping);
	target.println(configPage1.tachoPin);
	target.println(configPage1.tachoDiv);
	target.println(configPage1.tachoDuration);
	target.println(configPage1.maeThresh);
	target.println(configPage1.taeThresh);
	target.println(configPage1.aeTime);
	target.println(configPage1.displayType);
	target.println(configPage1.display1);
	target.println(configPage1.display2);
	target.println(configPage1.display3);
	target.println(configPage1.display4);
	target.println(configPage1.display5);
	target.println(configPage1.displayB1);
	target.println(configPage1.displayB2);
	target.println(configPage1.reqFuel);
	target.println(configPage1.divider);
	target.println(configPage1.alternate);
	target.println(configPage1.multiplyMAP_old);
	target.println(configPage1.includeAFR);
	target.println(configPage1.hardCutType);
	target.println(configPage1.ignAlgorithm);
	target.println(configPage1.indInjAng);
	target.println(configPage1.injOpen);
	print_array(target, configPage1.injAng);
	target.println(configPage1.mapSample);
	target.println(configPage1.twoStroke);
	target.println(configPage1.injType);
	target.println(configPage1.nCylinders);
	target.println(configPage1.algorithm);
	target.println(configPage1.fixAngEnable);
	target.println(configPage1.nInjectors);
	target.println(configPage1.engineType);
	target.println(configPage1.flexEnabled);
	target.println(configPage1.legacyMAP);
	target.println(configPage1.baroCorr);
	target.println(configPage1.injLayout);
	target.println(configPage1.perToothIgn);
	target.println(configPage1.dfcoEnabled);
	target.println(configPage1.aeColdTaperMax);
	target.println(configPage1.dutyLim);
	target.println(configPage1.flexFreqLow);
	target.println(configPage1.flexFreqHigh);
	target.println(configPage1.boostMaxDuty);
	target.println(configPage1.tpsMin);
	target.println(configPage1.tpsMax);
	target.println(configPage1.mapMin);
	target.println(configPage1.mapMax);
	target.println(configPage1.fpPrime);
	target.println(configPage1.stoich);
	target.println(configPage1.oddfire2);
	target.println(configPage1.oddfire3);
	target.println(configPage1.oddfire4);
	target.println(configPage1.idleUpPin);
	target.println(configPage1.idleUpPolarity);
	target.println(configPage1.idleUpEnabled);
	target.println(configPage1.idleUpAdder);
	target.println(configPage1.aeTaperMin);
	target.println(configPage1.aeTaperMax);
	target.println(configPage1.iacCLminDuty);
	target.println(configPage1.iacCLmaxDuty);
	target.println(configPage1.boostMinDuty);
	target.println(configPage1.baroMin);
	target.println(configPage1.baroMax);
	target.println(configPage1.EMAPMin);
	target.println(configPage1.EMAPMax);
	target.println(configPage1.fanWhenOff);
	target.println(configPage1.fanWhenCranking);
	target.println(configPage1.useDwellMap);
	target.println(configPage1.fanUnused);
	target.println(configPage1.rtc_mode);
	target.println(configPage1.incorporateAFR);
	print_array(target, configPage1.asePct);
	print_array(target, configPage1.aseCount);
	print_array(target, configPage1.aseBins);
	print_array(target, configPage1.primePulse);
	print_array(target, configPage1.primeBins);
	target.println(configPage1.CTPSPin);
	target.println(configPage1.CTPSPolarity);
	target.println(configPage1.CTPSEnabled);
	target.println(configPage1.idleAdvEnabled);
	target.println(configPage1.idleAdvAlgorithm);
	target.println(configPage1.idleAdvDelay);
	target.println(configPage1.idleAdvRPM);
	target.println(configPage1.idleAdvTPS);
	print_array(target, configPage1.injAngRPM);
	target.println(configPage1.idleTaperTime);
	target.println(configPage1.dfcoDelay);
	target.println(configPage1.dfcoMinCLT);
	target.println(configPage1.vssMode);
	target.println(configPage1.vssPin);
	target.println(configPage1.vssPulsesPerKm);
	target.println(configPage1.vssSmoothing);
	target.println(configPage1.vssRatio1);
	target.println(configPage1.vssRatio2);
	target.println(configPage1.vssRatio3);
	target.println(configPage1.vssRatio4);
	target.println(configPage1.vssRatio5);
	target.println(configPage1.vssRatio6);
	target.println(configPage1.idleUpOutputEnabled);
	target.println(configPage1.idleUpOutputInv);
	target.println(configPage1.idleUpOutputPin);
	target.println(configPage1.tachoSweepMaxRPM);
	target.println(configPage1.primingDelay);
	target.println(configPage1.iacTPSlimit);
	target.println(configPage1.iacRPMlimitHysteresis);
	target.println(configPage1.rtc_trim);
	target.println(configPage1.idleAdvVss);
	target.println(configPage1.mapSwitchPoint);
	print_array(target, configPage1.unused2_95);
}

static void printPage2(Print &target) {
	target.println(F("\nPg 2 Cfg"));
	target.println(F("\nVE Table"));
	serial_print_3dtable(target, fuelTable);
}

static void printPage3(Print &target) {
	target.println(F("\nPg 3 Cfg"));
	target.println(F("\nIgnition Advance Table"));
	serial_print_3dtable(target, ignitionTable);
}

static void printPage4(Print &target) {
	target.println(F("\nPg 4 Cfg"));
	target.println(configPage4.triggerAngle);
	target.println(configPage4.FixAng);
	target.println(configPage4.CrankAng);
	target.println(configPage4.TrigAngMul);
	target.println(configPage4.TrigEdge);
	target.println(configPage4.TrigSpeed);
	target.println(configPage4.IgInv);
	target.println(configPage4.TrigPattern);
	target.println(configPage4.TrigEdgeSec);
	target.println(configPage4.fuelPumpPin);
	target.println(configPage4.useResync);
	target.println(configPage4.sparkDur);
	target.println(configPage4.trigPatternSec);
	target.println(configPage4.PollLevelPol);
	target.println(configPage4.bootloaderCaps);
	target.println(configPage4.resetControlConfig);
	target.println(configPage4.resetControlPin);
	target.println(configPage4.SkipCycles);
	target.println(configPage4.boostType);
	target.println(configPage4.useDwellLim);
	target.println(configPage4.sparkMode);
	target.println(configPage4.triggerFilter);
	target.println(configPage4.ignCranklock);
	target.println(configPage4.dwellCrank);
	target.println(configPage4.dwellRun);
	target.println(configPage4.triggerTeeth);
	target.println(configPage4.triggerMissingTeeth);
	target.println(configPage4.crankRPM);
	target.println(configPage4.floodClear);
	target.println(configPage4.SoftRevLim);
	target.println(configPage4.SoftLimRetard);
	target.println(configPage4.SoftLimMax);
	target.println(configPage4.HardRevLim);
	print_array(target, configPage4.taeBins);
	print_array(target, configPage4.taeValues);
	print_array(target, configPage4.wueBins);
	target.println(configPage4.dwellLimit);
	print_array(target, configPage4.dwellCorrectionValues);
	print_array(target, configPage4.iatRetBins);
	print_array(target, configPage4.iatRetValues);
	target.println(configPage4.dfcoRPM);
	target.println(configPage4.dfcoHyster);
	target.println(configPage4.dfcoTPSThresh);
	target.println(configPage4.ignBypassEnabled);
	target.println(configPage4.ignBypassPin);
	target.println(configPage4.ignBypassHiLo);
	target.println(configPage4.ADCFILTER_TPS);
	target.println(configPage4.ADCFILTER_CLT);
	target.println(configPage4.ADCFILTER_IAT);
	target.println(configPage4.ADCFILTER_O2);
	target.println(configPage4.ADCFILTER_BAT);
	target.println(configPage4.ADCFILTER_MAP);
	target.println(configPage4.ADCFILTER_BARO);
	print_array(target, configPage4.cltAdvBins);
	print_array(target, configPage4.cltAdvValues);
	print_array(target, configPage4.maeBins);
	print_array(target, configPage4.maeRates);
	target.println(configPage4.batVoltCorrect);
	print_array(target, configPage4.baroFuelBins);
	print_array(target, configPage4.baroFuelValues);
	print_array(target, configPage4.idleAdvBins);
	print_array(target, configPage4.idleAdvValues);
	target.println(configPage4.engineProtectMaxRPM);
	target.println(configPage4.vvt2CL0DutyAng);
	target.println(configPage4.vvt2PWMdir);
	target.println(configPage4.unusedBits4_123);
	target.println(configPage4.ANGLEFILTER_VVT);
	target.println(configPage4.FILTER_FLEX);
	target.println(configPage4.vvtMinClt);
	target.println(configPage4.vvtDelay);
}

static void printPage5(Print &target) {
	target.println(F("\nPg 5 Cfg"));
	target.println(F("\nAFR Table"));
	serial_print_3dtable(target, afrTable);
}

static void printPage6(Print &target) {
	target.println(F("\nPg 6 Cfg"));
	target.println(configPage6.egoAlgorithm);
	target.println(configPage6.egoType);
	target.println(configPage6.boostEnabled);
	target.println(configPage6.vvtEnabled);
	target.println(configPage6.engineProtectType);
	target.println(configPage6.egoKP);
	target.println(configPage6.egoKI);
	target.println(configPage6.egoKD);
	target.println(configPage6.egoTemp);
	target.println(configPage6.egoCount);
	target.println(configPage6.vvtMode);
	target.println(configPage6.vvtLoadSource);
	target.println(configPage6.vvtPWMdir);
	target.println(configPage6.vvtCLUseHold);
	target.println(configPage6.vvtCLAlterFuelTiming);
	target.println(configPage6.boostCutEnabled);
	target.println(configPage6.egoLimit);
	target.println(configPage6.ego_min);
	target.println(configPage6.ego_max);
	target.println(configPage6.ego_sdelay);
	target.println(configPage6.egoRPM);
	target.println(configPage6.egoTPSMax);
	target.println(configPage6.vvt1Pin);
	target.println(configPage6.useExtBaro);
	target.println(configPage6.boostMode);
	target.println(configPage6.boostPin);
	target.println(configPage6.unused_bit);
	target.println(configPage6.useEMAP);
	print_array(target, configPage6.voltageCorrectionBins);
	print_array(target, configPage6.injVoltageCorrectionValues);
	print_array(target, configPage6.airDenBins);
	print_array(target, configPage6.airDenRates);
	target.println(configPage6.boostFreq);
	target.println(configPage6.vvtFreq);
	target.println(configPage6.idleFreq);
	target.println(configPage6.launchPin);
	target.println(configPage6.launchEnabled);
	target.println(configPage6.launchHiLo);
	target.println(configPage6.lnchSoftLim);
	target.println(configPage6.lnchRetard);
	target.println(configPage6.lnchHardLim);
	target.println(configPage6.lnchFuelAdd);
	target.println(configPage6.idleKP);
	target.println(configPage6.idleKI);
	target.println(configPage6.idleKD);
	target.println(configPage6.boostLimit);
	target.println(configPage6.boostKP);
	target.println(configPage6.boostKI);
	target.println(configPage6.boostKD);
	target.println(configPage6.lnchPullRes);
	target.println(configPage6.iacPWMrun);
	target.println(configPage6.fuelTrimEnabled);
	target.println(configPage6.flatSEnable);
	target.println(configPage6.baroPin);
	target.println(configPage6.flatSSoftWin);
	target.println(configPage6.flatSRetard);
	target.println(configPage6.flatSArm);
	print_array(target, configPage6.iacCLValues);
	print_array(target, configPage6.iacOLStepVal);
	print_array(target, configPage6.iacOLPWMVal);
	print_array(target, configPage6.iacBins);
	print_array(target, configPage6.iacCrankSteps);
	print_array(target, configPage6.iacCrankDuty);
	print_array(target, configPage6.iacCrankBins);
	target.println(configPage6.iacAlgorithm);
	target.println(configPage6.iacStepTime);
	target.println(configPage6.iacChannels);
	target.println(configPage6.iacPWMdir);
	target.println(configPage6.iacFastTemp);
	target.println(configPage6.iacStepHome);
	target.println(configPage6.iacStepHyster);
	target.println(configPage6.fanInv);
	target.println(configPage6.fanEnable);
	target.println(configPage6.fanPin);
	target.println(configPage6.fanSP);
	target.println(configPage6.fanHyster);
	target.println(configPage6.fanFreq);
	print_array(target, configPage6.fanPWMBins);
}

static void printPage7(Print &target) {
	target.println(F("\nPg 7 Cfg"));
	target.println(F("\nBoost Duty / Target"));
	serial_print_3dtable(target, boostTbl);
	target.println(F("\nVVT control Table"));
	serial_print_3dtable(target, vvtTbl);
	target.println(F("\nFuel Staging Table"));
	serial_print_3dtable(target, stagingTbl);
}

static void printPage8(Print &target) {
	target.println(F("\nPg 8 Cfg"));
	target.println(F("\nFuel trim Table 1"));
	serial_print_3dtable(target, fuelTrimTable1Tbl);
	target.println(F("\nFuel trim Table 2"));
	serial_print_3dtable(target, fuelTrimTable2Tbl);
	target.println(F("\nFuel trim Table 3"));
	serial_print_3dtable(target, fuelTrimTable3Tbl);
	target.println(F("\nFuel trim Table 4"));
	serial_print_3dtable(target, fuelTrimTable4Tbl);
	target.println(F("\nFuel trim Table 5"));
	serial_print_3dtable(target, fuelTrimTable5Tbl);
	target.println(F("\nFuel trim Table 6"));
	serial_print_3dtable(target, fuelTrimTable6Tbl);
	target.println(F("\nFuel trim Table 7"));
	serial_print_3dtable(target, fuelTrimTable7Tbl);
	target.println(F("\nFuel trim Table 8"));
	serial_print_3dtable(target, fuelTrimTable8Tbl);
}

static void printPage9(Print &target) {
	target.println(F("\nPg 9 Cfg"));
	target.println(configPage9.enable_secondarySerial);
	target.println(configPage9.intcan_available);
	target.println(configPage9.enable_intcan);
	print_array(target, configPage9.caninput_sel);
	print_array(target, configPage9.caninput_source_can_address);
	print_array(target, configPage9.caninput_source_start_byte);
	target.println(configPage9.caninput_source_num_bytes);
	target.println(configPage9.unused10_67);
	target.println(configPage9.unused10_68);
	target.println(configPage9.enable_candata_out);
	print_array(target, configPage9.canoutput_sel);
	print_array(target, configPage9.canoutput_param_group);
	print_array(target, configPage9.canoutput_param_start_byte);
	print_array(target, configPage9.canoutput_param_num_bytes);
	target.println(configPage9.unused10_110);
	target.println(configPage9.unused10_111);
	target.println(configPage9.unused10_112);
	target.println(configPage9.unused10_113);
	target.println(configPage9.speeduino_tsCanId);
	target.println(configPage9.true_address);
	target.println(configPage9.realtime_base_address);
	target.println(configPage9.obd_address);
	print_array(target, configPage9.Auxinpina);
	print_array(target, configPage9.Auxinpinb);
	target.println(configPage9.iacStepperInv);
	target.println(configPage9.iacCoolTime);
	target.println(configPage9.boostByGearEnabled);
	target.println(configPage9.blankfield);
	target.println(configPage9.unused10_153);
	target.println(configPage9.iacMaxSteps);
	target.println(configPage9.idleAdvStartDelay);
	target.println(configPage9.boostByGear1);
	target.println(configPage9.boostByGear2);
	target.println(configPage9.boostByGear3);
	target.println(configPage9.boostByGear4);
	target.println(configPage9.boostByGear5);
	target.println(configPage9.boostByGear6);
	print_array(target, configPage9.unused10_160);
}

static void printPage10(Print &target) {
	target.println(F("\nPg 10 Cfg"));
	print_array(target, configPage10.crankingEnrichBins);
	print_array(target, configPage10.crankingEnrichValues);
	target.println(configPage10.rotaryType);
	target.println(configPage10.stagingEnabled);
	target.println(configPage10.stagingMode);
	target.println(configPage10.EMAPPin);
	print_array(target, configPage10.rotarySplitValues);
	print_array(target, configPage10.rotarySplitBins);
	target.println(configPage10.boostSens);
	target.println(configPage10.boostIntv);
	target.println(configPage10.stagedInjSizePri);
	target.println(configPage10.stagedInjSizeSec);
	target.println(configPage10.lnchCtrlTPS);
	print_array(target, configPage10.flexBoostBins);
	print_array(target, configPage10.flexBoostAdj);
	print_array(target, configPage10.flexFuelBins);
	print_array(target, configPage10.flexFuelAdj);
	print_array(target, configPage10.flexAdvBins);
	print_array(target, configPage10.flexAdvAdj);
	target.println(configPage10.n2o_enable);
	target.println(configPage10.n2o_arming_pin);
	target.println(configPage10.n2o_minCLT);
	target.println(configPage10.n2o_maxMAP);
	target.println(configPage10.n2o_minTPS);
	target.println(configPage10.n2o_maxAFR);
	target.println(configPage10.n2o_stage1_pin);
	target.println(configPage10.n2o_pin_polarity);
	target.println(configPage10.n2o_stage1_unused);
	target.println(configPage10.n2o_stage1_minRPM);
	target.println(configPage10.n2o_stage1_maxRPM);
	target.println(configPage10.n2o_stage1_adderMin);
	target.println(configPage10.n2o_stage1_adderMax);
	target.println(configPage10.n2o_stage1_retard);
	target.println(configPage10.n2o_stage2_pin);
	target.println(configPage10.n2o_stage2_unused);
	target.println(configPage10.n2o_stage2_minRPM);
	target.println(configPage10.n2o_stage2_maxRPM);
	target.println(configPage10.n2o_stage2_adderMin);
	target.println(configPage10.n2o_stage2_adderMax);
	target.println(configPage10.n2o_stage2_retard);
	target.println(configPage10.knock_mode);
	target.println(configPage10.knock_pin);
	target.println(configPage10.knock_trigger);
	target.println(configPage10.knock_pullup);
	target.println(configPage10.knock_limiterDisable);
	target.println(configPage10.knock_unused);
	target.println(configPage10.knock_count);
	target.println(configPage10.knock_threshold);
	target.println(configPage10.knock_maxMAP);
	target.println(configPage10.knock_maxRPM);
	print_array(target, configPage10.knock_window_rpms);
	print_array(target, configPage10.knock_window_angle);
	print_array(target, configPage10.knock_window_dur);
	target.println(configPage10.knock_maxRetard);
	target.println(configPage10.knock_firstStep);
	target.println(configPage10.knock_stepSize);
	target.println(configPage10.knock_stepTime);
	target.println(configPage10.knock_duration);
	target.println(configPage10.knock_recoveryStepTime);
	target.println(configPage10.knock_recoveryStep);
	target.println(configPage10.fuel2Algorithm);
	target.println(configPage10.fuel2Mode);
	target.println(configPage10.fuel2SwitchVariable);
	target.println(configPage10.fuel2SwitchValue);
	target.println(configPage10.fuel2InputPin);
	target.println(configPage10.fuel2InputPolarity);
	target.println(configPage10.fuel2InputPullup);
	target.println(configPage10.vvtCLholdDuty);
	target.println(configPage10.vvtCLKP);
	target.println(configPage10.vvtCLKI);
	target.println(configPage10.vvtCLKD);
	target.println(configPage10.vvtCL0DutyAng);
	target.println(configPage10.vvtCLMinAng);
	target.println(configPage10.vvtCLMaxAng);
	target.println(configPage10.crankingEnrichTaper);
	target.println(configPage10.fuelPressureEnable);
	target.println(configPage10.oilPressureEnable);
	target.println(configPage10.oilPressureProtEnbl);
	target.println(configPage10.oilPressurePin);
	target.println(configPage10.fuelPressurePin);
	target.println(configPage10.fuelPressureMin);
	target.println(configPage10.fuelPressureMax);
	target.println(configPage10.oilPressureMin);
	target.println(configPage10.oilPressureMax);
	print_array(target, configPage10.oilPressureProtRPM);
	print_array(target, configPage10.oilPressureProtMins);
	target.println(configPage10.wmiEnabled);
	target.println(configPage10.wmiMode);
	target.println(configPage10.wmiAdvEnabled);
	target.println(configPage10.wmiTPS);
	target.println(configPage10.wmiRPM);
	target.println(configPage10.wmiMAP);
	target.println(configPage10.wmiMAP2);
	target.println(configPage10.wmiIAT);
	target.println(configPage10.wmiOffset);
	target.println(configPage10.wmiIndicatorEnabled);
	target.println(configPage10.wmiIndicatorPin);
	target.println(configPage10.wmiIndicatorPolarity);
	target.println(configPage10.wmiEmptyEnabled);
	target.println(configPage10.wmiEmptyPin);
	target.println(configPage10.wmiEmptyPolarity);
	target.println(configPage10.wmiEnabledPin);
	print_array(target, configPage10.wmiAdvBins);
	print_array(target, configPage10.wmiAdvAdj);
	target.println(configPage10.vvtCLminDuty);
	target.println(configPage10.vvtCLmaxDuty);
	target.println(configPage10.vvt2Pin);
	target.println(configPage10.vvt2Enabled);
	target.println(configPage10.TrigEdgeThrd);
	print_array(target, configPage10.fuelTempBins);
	print_array(target, configPage10.fuelTempValues);
	target.println(configPage10.spark2Algorithm);
	target.println(configPage10.spark2Mode);
	target.println(configPage10.spark2SwitchVariable);
	target.println(configPage10.spark2SwitchValue);
	target.println(configPage10.spark2InputPin);
	target.println(configPage10.spark2InputPolarity);
	target.println(configPage10.spark2InputPullup);
	print_array(target, configPage10.unused11_187_191);
}

static void printPage11(Print &target) {
	target.println(F("\nPg 11 Cfg"));
	target.println(F("\nFuel Table 2"));
	serial_print_3dtable(target, fuelTable2);
}

static void printPage12(Print &target) {
	target.println(F("\nPg 12 Cfg"));
	target.println(F("\nWMI control Table"));
	serial_print_3dtable(target, wmiTbl);
	target.println(F("\nVVT2 control Table"));
	serial_print_3dtable(target, vvt2Tbl);
	target.println(F("\nDwell map"));
	serial_print_3dtable(target, dwell_map);
}

static void printPage13(Print &target) {
	target.println(F("\nPg 13 Cfg"));
	target.println(configPage13.outputInverted);
	target.println(configPage13.kindOfLimiting0);
	target.println(configPage13.kindOfLimiting1);
	target.println(configPage13.kindOfLimiting2);
	target.println(configPage13.kindOfLimiting3);
	target.println(configPage13.kindOfLimiting4);
	target.println(configPage13.kindOfLimiting5);
	target.println(configPage13.kindOfLimiting6);
	target.println(configPage13.kindOfLimiting7);
	print_array(target, configPage13.outputPin);
	print_array(target, configPage13.outputDelay);
	print_array(target, configPage13.firstDataIn);
	print_array(target, configPage13.secondDataIn);
	print_array(target, configPage13.outputTimeLimit);
	print_array(target, configPage13.unused13_35_49);
	print_array(target, configPage13.firstTarget);
	print_array(target, configPage13.secondTarget);
	target.println(configPage13.operation[0].firstCompType);
	target.println(configPage13.operation[0].secondCompType);
	target.println(configPage13.operation[0].bitwise);
	target.println(configPage13.operation[1].firstCompType);
	target.println(configPage13.operation[1].secondCompType);
	target.println(configPage13.operation[1].bitwise);
	target.println(configPage13.operation[2].firstCompType);
	target.println(configPage13.operation[2].secondCompType);
	target.println(configPage13.operation[2].bitwise);
	target.println(configPage13.operation[3].firstCompType);
	target.println(configPage13.operation[3].secondCompType);
	target.println(configPage13.operation[3].bitwise);
	target.println(configPage13.operation[4].firstCompType);
	target.println(configPage13.operation[4].secondCompType);
	target.println(configPage13.operation[4].bitwise);
	target.println(configPage13.operation[5].firstCompType);
	target.println(configPage13.operation[5].secondCompType);
	target.println(configPage13.operation[5].bitwise);
	target.println(configPage13.operation[6].firstCompType);
	target.println(configPage13.operation[6].secondCompType);
	target.println(configPage13.operation[6].bitwise);
	target.println(configPage13.operation[7].firstCompType);
	target.println(configPage13.operation[7].secondCompType);
	target.println(configPage13.operation[7].bitwise);
	print_array(target, configPage13.candID);
	print_array(target, configPage13.unused12_106_115);
	target.println(configPage13.onboard_log_csv_separator);
	target.println(configPage13.onboard_log_file_style);
	target.println(configPage13.onboard_log_file_rate);
	target.println(configPage13.onboard_log_filenaming);
	target.println(configPage13.onboard_log_storage);
	target.println(configPage13.onboard_log_trigger_boot);
	target.println(configPage13.onboard_log_trigger_RPM);
	target.println(configPage13.onboard_log_trigger_prot);
	target.println(configPage13.onboard_log_trigger_Vbat);
	target.println(configPage13.onboard_log_trigger_Epin);
	target.println(configPage13.onboard_log_tr1_duration);
	target.println(configPage13.onboard_log_tr2_thr_on);
	target.println(configPage13.onboard_log_tr2_thr_off);
	target.println(configPage13.onboard_log_tr3_thr_RPM);
	target.println(configPage13.onboard_log_tr3_thr_MAP);
	target.println(configPage13.onboard_log_tr3_thr_Oil);
	target.println(configPage13.onboard_log_tr3_thr_AFR);
	target.println(configPage13.onboard_log_tr4_thr_on);
	target.println(configPage13.onboard_log_tr4_thr_off);
	target.println(configPage13.onboard_log_tr5_thr_on);
	print_array(target, configPage13.unused12_125_127);
}

static void printPage14(Print &target) {
	target.println(F("\nPg 14 Cfg"));
	target.println(F("\nSecond Ignition Advance Table"));
	serial_print_3dtable(target, ignitionTable2);
}

void printPageAscii(byte pageNum, Print &target) {
	switch(pageNum) {
		case 1:
		printPage1(target);
		break;
		case 2:
		printPage2(target);
		break;
		case 3:
		printPage3(target);
		break;
		case 4:
		printPage4(target);
		break;
		case 5:
		printPage5(target);
		break;
		case 6:
		printPage6(target);
		break;
		case 7:
		printPage7(target);
		break;
		case 8:
		printPage8(target);
		break;
		case 9:
		printPage9(target);
		break;
		case 10:
		printPage10(target);
		break;
		case 11:
		printPage11(target);
		break;
		case 12:
		printPage12(target);
		break;
		case 13:
		printPage13(target);
		break;
		case 14:
		printPage14(target);
		break;
	}
}