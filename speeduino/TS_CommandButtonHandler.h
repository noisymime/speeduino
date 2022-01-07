
/** \file
 * Header file for the TunerStudio command handler
 * The command handler manages all the inputs FROM TS which are issued when a command button is clicked by the user
 */

#define TS_CMD_TEST_DSBL  256
#define TS_CMD_TEST_ENBL  257

#define TS_CMD_INJ1_ON    513
#define TS_CMD_INJ1_OFF   514
#define TS_CMD_INJ1_50PC  515
#define TS_CMD_INJ2_ON    516
#define TS_CMD_INJ2_OFF   517
#define TS_CMD_INJ2_50PC  518
#define TS_CMD_INJ3_ON    519
#define TS_CMD_INJ3_OFF   520
#define TS_CMD_INJ3_50PC  521
#define TS_CMD_INJ4_ON    522
#define TS_CMD_INJ4_OFF   523
#define TS_CMD_INJ4_50PC  524
#define TS_CMD_INJ5_ON    525
#define TS_CMD_INJ5_OFF   526
#define TS_CMD_INJ5_50PC  527
#define TS_CMD_INJ6_ON    528
#define TS_CMD_INJ6_OFF   529
#define TS_CMD_INJ6_50PC  530
#define TS_CMD_INJ7_ON    531
#define TS_CMD_INJ7_OFF   532
#define TS_CMD_INJ7_50PC  533
#define TS_CMD_INJ8_ON    534
#define TS_CMD_INJ8_OFF   535
#define TS_CMD_INJ8_50PC  536

#define TS_CMD_IGN1_ON    769
#define TS_CMD_IGN1_OFF   770
#define TS_CMD_IGN1_50PC  771
#define TS_CMD_IGN2_ON    772
#define TS_CMD_IGN2_OFF   773
#define TS_CMD_IGN2_50PC  774
#define TS_CMD_IGN3_ON    775
#define TS_CMD_IGN3_OFF   776
#define TS_CMD_IGN3_50PC  777
#define TS_CMD_IGN4_ON    778
#define TS_CMD_IGN4_OFF   779
#define TS_CMD_IGN4_50PC  780
#define TS_CMD_IGN5_ON    781
#define TS_CMD_IGN5_OFF   782
#define TS_CMD_IGN5_50PC  783
#define TS_CMD_IGN6_ON    784
#define TS_CMD_IGN6_OFF   785
#define TS_CMD_IGN6_50PC  786
#define TS_CMD_IGN7_ON    787
#define TS_CMD_IGN7_OFF   788
#define TS_CMD_IGN7_50PC  789
#define TS_CMD_IGN8_ON    790
#define TS_CMD_IGN8_OFF   791
#define TS_CMD_IGN8_50PC  792

#define TS_CMD_STM32_REBOOT     12800
#define TS_CMD_STM32_BOOTLOADER 12801

#define TS_CMD_SD_FORMAT  13057

#define TS_CMD_VSS_60KMH  39168 //0x99x00
#define TS_CMD_VSS_RATIO1 39169
#define TS_CMD_VSS_RATIO2 39170
#define TS_CMD_VSS_RATIO3 39171
#define TS_CMD_VSS_RATIO4 39172
#define TS_CMD_VSS_RATIO5 39173
#define TS_CMD_VSS_RATIO6 39174

/* the maximum id number is 65,535 */
void TS_CommandButtonsHandler(uint16_t);
