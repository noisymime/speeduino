EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr User 8268 5846
encoding utf-8
Sheet 1 1
Title "Dual Variable Reluctance MAX9926 mode A2"
Date "2021-02-24"
Rev "3.5"
Comp "(c)2021 Phill Rogers @ TechColab.co.je"
Comment1 "Optional onboard regulator or 5V direct"
Comment2 "Optional power resistors"
Comment3 "Optional edge header for breadboard etc."
Comment4 "M2 mounting holes"
$EndDescr
Text Notes 6450 1000 0    60   ~ 0
Furniture
$Comp
L Mechanical:Fiducial FID1
U 1 1 60331E91
P 6600 600
F 0 "FID1" H 6668 642 50  0000 L CNN
F 1 "Fiducial" H 6668 567 50  0000 L CNN
F 2 "Fiducial:Fiducial_0.5mm_Mask1mm" H 6600 600 50  0001 C CNN
F 3 "~" H 6600 600 50  0001 C CNN
	1    6600 600 
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:Fiducial FID2
U 1 1 6034DBD2
P 6600 750
F 0 "FID2" H 6668 792 50  0000 L CNN
F 1 "Fiducial" H 6668 717 50  0000 L CNN
F 2 "Fiducial:Fiducial_0.5mm_Mask1mm" H 6600 750 50  0001 C CNN
F 3 "~" H 6600 750 50  0001 C CNN
	1    6600 750 
	1    0    0    -1  
$EndComp
$Comp
L 2021-02-17_17-37-40:MAX9926UAEE+ U1
U 1 1 602DAA0C
P 1300 2500
F 0 "U1" H 1850 2750 60  0000 C CNN
F 1 "MAX9926UAEE+" H 3450 2750 60  0000 C CNN
F 2 "MAX9926UAEE+:MAX9926UAEE&plus_" H 2800 2740 60  0001 C CNN
F 3 "https://datasheet.octopart.com/MAX9926UAEE%2B-Maxim-Integrated-datasheet-11046290.pdf" H 1300 2500 60  0001 C CNN
F 4 "Maxim Integrated" H 1300 2500 50  0001 C CNN "MFG Name"
F 5 "MAX9926UAEE+" H 1300 2500 50  0001 C CNN "MFG Part Num"
F 6 "MAX9926 Series 10 mA Differential Input Logic Output Sensor Interface -SSOP-16" H 1300 2500 50  0001 C CNN "Description"
F 7 "SSOP-16" H 1300 2500 50  0001 C CNN "Package JEDEC ID"
F 8 "Yes" H 1300 2500 50  0001 C CNN "Critical"
	1    1300 2500
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 602DCA31
P 4550 2850
F 0 "C1" H 4550 2950 50  0000 L CNN
F 1 "10n" H 4550 2750 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4588 2700 50  0001 C CNN
F 3 "~" H 4550 2850 50  0001 C CNN
F 4 "10 V ±5% Tolerance U2J Multilayer Ceramic Capacitor" H 4550 2850 50  0001 C CNN "Description"
F 5 "0603_1608Metric" H 4550 2850 50  0001 C CNN "Package JEDEC ID"
F 6 "MurataMurata" H 4550 2850 50  0001 C CNN "MFG Name"
F 7 "GRM1857U1A103JA44D" H 4550 2850 50  0001 C CNN "MFG Part Num"
	1    4550 2850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0101
U 1 1 602DDA11
P 4400 3500
F 0 "#PWR0101" H 4400 3250 50  0001 C CNN
F 1 "GND" H 4200 3450 50  0000 C CNN
F 2 "" H 4400 3500 50  0001 C CNN
F 3 "" H 4400 3500 50  0001 C CNN
	1    4400 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	1300 2500 1200 2500
Wire Wire Line
	1200 2500 1200 2700
Wire Wire Line
	1200 2700 1300 2700
Wire Wire Line
	1300 3200 1200 3200
Wire Wire Line
	1200 3200 1200 3000
Wire Wire Line
	1200 3000 1300 3000
Connection ~ 1200 3000
Wire Wire Line
	1200 3200 1200 3500
Connection ~ 1200 3200
Wire Wire Line
	4400 2800 4300 2800
$Comp
L Device:C C2
U 1 1 602EB266
P 4750 2850
F 0 "C2" H 4750 2950 50  0000 L CNN
F 1 "100n" H 4750 2750 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4788 2700 50  0001 C CNN
F 3 "~" H 4750 2850 50  0001 C CNN
F 4 "Ceramic 0.1uF 16V X7R 10% Pad SMD 0603 Soft Termination 125C Automotive T/R" H 4750 2850 50  0001 C CNN "Description"
F 5 "0603_1608Metric" H 4750 2850 50  0001 C CNN "Package JEDEC ID"
F 6 "Murata" H 4750 2850 50  0001 C CNN "MFG Name"
F 7 "GCJ188R71C104KA01D" H 4750 2850 50  0001 C CNN "MFG Part Num"
	1    4750 2850
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 602EBADD
P 4950 2850
F 0 "C3" H 4950 2950 50  0000 L CNN
F 1 "1u" H 4950 2750 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4988 2700 50  0001 C CNN
F 3 "~" H 4950 2850 50  0001 C CNN
F 4 "16V ±10% Tolerance X7R Surface Mount Multilayer Ceramic Capacitor" H 4950 2850 50  0001 C CNN "Description"
F 5 "0603_1608Metric" H 4950 2850 50  0001 C CNN "Package JEDEC ID"
F 6 "Murata" H 4950 2850 50  0001 C CNN "MFG Name"
F 7 "GCM188R71C105KA64D" H 4950 2850 50  0001 C CNN "MFG Part Num"
	1    4950 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	4550 2700 4750 2700
Connection ~ 4550 2700
Wire Wire Line
	4750 2700 4950 2700
Connection ~ 4750 2700
Wire Wire Line
	4550 3000 4750 3000
Wire Wire Line
	4750 3000 4950 3000
Connection ~ 4750 3000
Wire Wire Line
	4400 2800 4400 3000
Wire Wire Line
	4550 3000 4400 3000
Connection ~ 4550 3000
Connection ~ 4400 3000
Wire Wire Line
	4400 3000 4400 3500
NoConn ~ 4300 2900
NoConn ~ 1300 2600
NoConn ~ 1300 3100
Wire Wire Line
	4400 2200 4400 2700
$Comp
L power:VCC #PWR0102
U 1 1 602F5E11
P 4400 2200
F 0 "#PWR0102" H 4400 2050 50  0001 C CNN
F 1 "VCC" H 4250 2200 50  0000 C CNN
F 2 "" H 4400 2200 50  0001 C CNN
F 3 "" H 4400 2200 50  0001 C CNN
	1    4400 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	1200 2700 1200 3000
Connection ~ 1200 2700
Wire Wire Line
	4400 2700 4550 2700
Wire Wire Line
	4300 3000 4400 3000
Wire Wire Line
	4300 2700 4400 2700
Connection ~ 4400 2700
Connection ~ 4400 3500
Connection ~ 4400 2200
$Comp
L Device:C C20
U 1 1 60314324
P 4550 3250
F 0 "C20" H 4400 3350 50  0000 L CNN
F 1 "1n" H 4450 3150 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4588 3100 50  0001 C CNN
F 3 "~" H 4550 3250 50  0001 C CNN
F 4 "1000 pF 50 V ±5 % Tolerance C0G SMT Multilayer Ceramic Capacitor" H 4550 3250 50  0001 C CNN "Description"
F 5 "0603_1608Metric" H 4550 3250 50  0001 C CNN "Package JEDEC ID"
F 6 "Murata" H 4550 3250 50  0001 C CNN "MFG Name"
F 7 "GRM1885C1H102JA01D" H 4550 3250 50  0001 C CNN "MFG Part Num"
	1    4550 3250
	1    0    0    -1  
$EndComp
$Comp
L Device:C C10
U 1 1 603156D1
P 4550 2450
F 0 "C10" H 4400 2550 50  0000 L CNN
F 1 "1n" H 4450 2350 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4588 2300 50  0001 C CNN
F 3 "~" H 4550 2450 50  0001 C CNN
F 4 "1000 pF 50 V ±5 % Tolerance C0G SMT Multilayer Ceramic Capacitor" H 4550 2450 50  0001 C CNN "Description"
F 5 "0603_1608Metric" H 4550 2450 50  0001 C CNN "Package JEDEC ID"
F 6 "Murata" H 4550 2450 50  0001 C CNN "MFG Name"
F 7 "GRM1885C1H102JA01D" H 4550 2450 50  0001 C CNN "MFG Part Num"
	1    4550 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	4300 3100 4550 3100
Wire Wire Line
	4300 2600 4550 2600
Wire Wire Line
	4300 3200 4300 3400
Wire Wire Line
	4300 3400 4550 3400
Wire Wire Line
	4300 2500 4300 2300
Wire Wire Line
	4300 2300 4550 2300
$Comp
L Device:R R12
U 1 1 6032192F
P 4800 2300
F 0 "R12" V 4800 2250 50  0000 L CNN
F 1 "10k" V 4700 2250 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 4730 2300 50  0001 C CNN
F 3 "~" H 4800 2300 50  0001 C CNN
F 4 "" H 4800 2300 50  0001 C CNN "Description"
F 5 "" H 4800 2300 50  0001 C CNN "Package JEDEC ID"
F 6 "Yageo" H 4800 2300 50  0001 C CNN "MFG Name"
F 7 "RC1206FR-7W10KL" H 4800 2300 50  0001 C CNN "MFG Part Num"
	1    4800 2300
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4550 3400 4650 3400
Connection ~ 4550 3400
Wire Wire Line
	4550 3100 4650 3100
Connection ~ 4550 3100
Wire Wire Line
	4550 2300 4650 2300
Connection ~ 4550 2300
Wire Wire Line
	4550 2600 4650 2600
Connection ~ 4550 2600
$Comp
L Device:R R13
U 1 1 6032AC19
P 5050 2450
F 0 "R13" H 5100 2400 50  0000 L CNN
F 1 "4.7k" H 5100 2500 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 4980 2450 50  0001 C CNN
F 3 "~" H 5050 2450 50  0001 C CNN
F 4 "DoNoPlace. User option" H 5050 2450 50  0001 C CNN "Critical"
F 5 "3W, 5%" H 5050 2450 50  0001 C CNN "Description"
	1    5050 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	4950 2300 5050 2300
Wire Wire Line
	4950 2600 5050 2600
Connection ~ 5050 2300
Wire Wire Line
	5050 2600 5250 2600
Connection ~ 5050 2600
$Comp
L Connector_Generic:Conn_02x04_Counter_Clockwise J1
U 1 1 6033C0E4
P 5600 2800
F 0 "J1" H 5650 3000 50  0000 C CNN
F 1 "8-pin header" H 5650 2450 50  0000 C CNN
F 2 "Package_DIP:DIP-8_W7.62mm" H 5600 2800 50  0001 C CNN
F 3 "~" H 5600 2800 50  0001 C CNN
	1    5600 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	5250 3100 5250 3000
Wire Wire Line
	5250 2600 5250 2800
Wire Wire Line
	5300 2700 5300 2300
Wire Wire Line
	5050 2300 5300 2300
$Comp
L Device:R R20
U 1 1 60351FB1
P 6000 2350
F 0 "R20" H 6050 2400 50  0000 L CNN
F 1 "1k" H 6050 2300 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 5930 2350 50  0001 C CNN
F 3 "~" H 6000 2350 50  0001 C CNN
F 4 "Thick Film; +/-1% 0.2W" H 6000 2350 50  0001 C CNN "Description"
F 5 "0603_1608Metric" H 6000 2350 50  0001 C CNN "Package JEDEC ID"
F 6 "Vishay" H 6000 2350 50  0001 C CNN "MFG Name"
F 7 "CRCW06031K00FKEAHP" H 6000 2350 50  0001 C CNN "MFG Part Num"
	1    6000 2350
	1    0    0    -1  
$EndComp
$Comp
L Device:R R10
U 1 1 60351FB7
P 5900 2350
F 0 "R10" H 5700 2400 50  0000 L CNN
F 1 "1k" H 5750 2300 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 5830 2350 50  0001 C CNN
F 3 "~" H 5900 2350 50  0001 C CNN
F 4 "Thick Film; +/-1% 0.2W" H 5900 2350 50  0001 C CNN "Description"
F 5 "0603_1608Metric" H 5900 2350 50  0001 C CNN "Package JEDEC ID"
F 6 "Vishay" H 5900 2350 50  0001 C CNN "MFG Name"
F 7 "CRCW06031K00FKEAHP" H 5900 2350 50  0001 C CNN "MFG Part Num"
	1    5900 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	5900 2700 5900 2500
Wire Wire Line
	1300 2900 1250 2900
Wire Wire Line
	1250 2900 1250 3650
Wire Wire Line
	1300 2800 1250 2800
Wire Wire Line
	1250 2800 1250 2050
Wire Wire Line
	5900 3000 5900 3100
Wire Wire Line
	5900 3100 5450 3100
Wire Wire Line
	5450 3100 5450 2200
Wire Wire Line
	5250 2800 5400 2800
Wire Wire Line
	5300 2700 5400 2700
Wire Wire Line
	4400 2200 5450 2200
Wire Wire Line
	1200 3500 4400 3500
Wire Wire Line
	1250 3650 6000 3650
Wire Wire Line
	4950 3100 5050 3100
Wire Wire Line
	4950 3400 5050 3400
Connection ~ 5050 3100
Wire Wire Line
	5050 3100 5250 3100
Connection ~ 5050 3400
Wire Wire Line
	5050 3400 5300 3400
Wire Wire Line
	5300 2900 5400 2900
Wire Wire Line
	5300 2900 5300 3400
Wire Wire Line
	5250 3000 5400 3000
$Comp
L Mechanical:MountingHole H1
U 1 1 6037799D
P 7100 600
F 0 "H1" H 7200 646 50  0000 L CNN
F 1 "MountingHole" H 7200 555 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.2mm_M2" H 7100 600 50  0001 C CNN
F 3 "~" H 7100 600 50  0001 C CNN
	1    7100 600 
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H2
U 1 1 603785C5
P 7100 800
F 0 "H2" H 7200 846 50  0000 L CNN
F 1 "MountingHole" H 7200 755 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.2mm_M2" H 7100 800 50  0001 C CNN
F 3 "~" H 7100 800 50  0001 C CNN
	1    7100 800 
	1    0    0    -1  
$EndComp
Text Notes 4450 2150 0    50   ~ 0
VCC = 5V
Wire Notes Line
	6450 1000 6450 500 
Wire Notes Line
	6450 1000 7750 1000
Wire Wire Line
	6050 2800 6000 2800
Wire Wire Line
	5900 2700 6050 2700
Connection ~ 5900 2700
Wire Wire Line
	5900 2200 6000 2200
Wire Wire Line
	6000 2500 6000 2800
Connection ~ 6000 2800
Wire Wire Line
	6000 2800 5900 2800
Wire Wire Line
	5450 2200 5900 2200
Connection ~ 5450 2200
Connection ~ 5900 2200
Wire Wire Line
	6050 2700 6050 2050
Wire Wire Line
	1250 2050 6050 2050
$Comp
L Device:CP C4
U 1 1 6037FBE4
P 5200 950
F 0 "C4" H 5200 1050 50  0000 L CNN
F 1 "10u" H 5200 850 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 5238 800 50  0001 C CNN
F 3 "~" H 5200 950 50  0001 C CNN
F 4 "25 V 10 uF ±20% Tolerance X5R Multilayer Ceramic Capacitor" H 5200 950 50  0001 C CNN "Description"
F 5 "0805_2012Metric" H 5200 950 50  0001 C CNN "Package JEDEC ID"
F 6 "Murata" H 5200 950 50  0001 C CNN "MFG Name"
F 7 "GRM21BR61E106MA73L" H 5200 950 50  0001 C CNN "MFG Part Num"
	1    5200 950 
	1    0    0    -1  
$EndComp
Wire Wire Line
	5900 2900 5950 2900
Wire Wire Line
	5950 2900 5950 3500
Wire Wire Line
	5950 3500 4400 3500
Wire Wire Line
	6000 2800 6000 3650
Text Label 6050 2700 0    50   ~ 0
COUT1
Text Label 6050 2800 0    50   ~ 0
COUT2
Text Label 6050 2900 0    50   ~ 0
GND
Text Label 6050 3000 0    50   ~ 0
VCC
Text Label 5850 800  0    50   ~ 0
VCC
Text Label 4800 1100 0    50   ~ 0
COUT1
Text Label 4800 1000 0    50   ~ 0
COUT2
Text Label 4800 900  0    50   ~ 0
GND
Text Label 4800 800  0    50   ~ 0
Vin
Wire Notes Line
	6100 550  4400 550 
Text Notes 4450 1350 0    60   ~ 0
Optional, but SMT capacitor should\n always be fitted for convenience
Wire Wire Line
	4800 900  5050 900 
Wire Wire Line
	5050 900  5050 1100
Wire Wire Line
	5050 1100 5200 1100
Wire Wire Line
	4800 800  5050 800 
Connection ~ 5200 1100
Wire Wire Line
	5200 1100 5550 1100
Wire Notes Line
	4400 1400 6100 1400
$Comp
L Device:R R23
U 1 1 6032659E
P 5050 3250
F 0 "R23" H 4850 3300 50  0000 L CNN
F 1 "4.7k" H 4850 3200 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 4980 3250 50  0001 C CNN
F 3 "~" H 5050 3250 50  0001 C CNN
F 4 "DoNoPlace. User option" H 5050 3250 50  0001 C CNN "Critical"
F 5 "" H 5050 3250 50  0001 C CNN "Characteristics"
F 6 "3W, 5%" H 5050 3250 50  0001 C CNN "Description"
	1    5050 3250
	-1   0    0    1   
$EndComp
$Comp
L Regulator_Linear:AMS1117CD-5.0 U2
U 1 1 60325A8A
P 5550 800
F 0 "U2" H 5650 550 50  0000 C CNN
F 1 "AMS1117CD-5.0" H 5550 951 50  0000 C CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline_Wide" H 5550 1000 50  0001 C CNN
F 3 "http://www.advanced-monolithic.com/pdf/ds1117.pdf" H 5650 550 50  0001 C CNN
F 4 "Advanced Monolithic Systems" H 5550 800 50  0001 C CNN "MFG Name"
F 5 "AMS1117-5.0" H 5550 800 50  0001 C CNN "MFG Part Num"
F 6 "1A Low Dropout Voltage Regulator" H 5550 800 50  0001 C CNN "Description"
F 7 "TO92 inline wide" H 5550 800 50  0001 C CNN "Package JEDEC ID"
F 8 "DoNoPlace. User option" H 5550 800 50  0001 C CNN "Critical"
	1    5550 800 
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x04 J2
U 1 1 603276E3
P 4600 900
F 0 "J2" H 4550 1100 50  0000 L CNN
F 1 "Edge" V 4700 800 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Horizontal" H 4600 900 50  0001 C CNN
F 3 "~" H 4600 900 50  0001 C CNN
F 4 "DoNoPlace. User option" H 4600 900 50  0001 C CNN "Critical"
	1    4600 900 
	-1   0    0    -1  
$EndComp
$Comp
L power:+12V #PWR0103
U 1 1 60369B79
P 5050 800
F 0 "#PWR0103" H 5050 650 50  0001 C CNN
F 1 "+12V" H 5065 973 50  0000 C CNN
F 2 "" H 5050 800 50  0001 C CNN
F 3 "" H 5050 800 50  0001 C CNN
	1    5050 800 
	1    0    0    -1  
$EndComp
Connection ~ 5050 800 
Wire Wire Line
	5050 800  5200 800 
Wire Notes Line
	4400 550  4400 1400
Wire Notes Line
	6100 550  6100 1400
Wire Wire Line
	5200 800  5250 800 
Connection ~ 5200 800 
Wire Wire Line
	5950 2900 6050 2900
Connection ~ 5950 2900
Wire Wire Line
	6050 3000 5900 3000
Connection ~ 5900 3000
Text Notes 600  1050 0    50   ~ 0
SoFar:\n+ under 20x20mm & single sided for ‘Letter’ postage.\n+ M2 mounting holes\n+ Optional power resisotrs\n+ Optional regulator for typical automotive power\n+ Optional pin header for breadboard etc. \n+ Could cut at pin header for direct PCB mount by castellation
Text Label 6850 2200 2    50   ~ 0
COUT1
Text Label 6850 2300 2    50   ~ 0
COUT2
Text Label 6850 2400 2    50   ~ 0
GND
Text Label 6850 2500 2    50   ~ 0
Vin
Text HLabel 6900 2200 2    50   Input ~ 0
COUT1
Text HLabel 6900 2300 2    50   Input ~ 0
COUT2
Text HLabel 6900 2400 2    50   Input ~ 0
GND
Text HLabel 6900 2500 2    50   Input ~ 0
Vin
Wire Wire Line
	6850 2200 6900 2200
Wire Wire Line
	6850 2300 6900 2300
Wire Wire Line
	6850 2400 6900 2400
Wire Wire Line
	6850 2500 6900 2500
$Comp
L Device:R R3
U 1 1 60812326
P 4800 3400
F 0 "R3" V 4800 3350 50  0000 L CNN
F 1 "10k" V 4700 3200 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 4730 3400 50  0001 C CNN
F 3 "~" H 4800 3400 50  0001 C CNN
F 4 "" H 4800 3400 50  0001 C CNN "Description"
F 5 "" H 4800 3400 50  0001 C CNN "Package JEDEC ID"
F 6 "Yageo" H 4800 3400 50  0001 C CNN "MFG Name"
F 7 "RC1206FR-7W10KL" H 4800 3400 50  0001 C CNN "MFG Part Num"
	1    4800 3400
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R2
U 1 1 6081295A
P 4800 3100
F 0 "R2" V 4800 3050 50  0000 L CNN
F 1 "10k" V 4700 2900 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 4730 3100 50  0001 C CNN
F 3 "~" H 4800 3100 50  0001 C CNN
F 4 "" H 4800 3100 50  0001 C CNN "Description"
F 5 "" H 4800 3100 50  0001 C CNN "Package JEDEC ID"
F 6 "Yageo" H 4800 3100 50  0001 C CNN "MFG Name"
F 7 "RC1206FR-7W10KL" H 4800 3100 50  0001 C CNN "MFG Part Num"
	1    4800 3100
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R1
U 1 1 60812D66
P 4800 2600
F 0 "R1" V 4800 2550 50  0000 L CNN
F 1 "10k" V 4900 2550 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 4730 2600 50  0001 C CNN
F 3 "~" H 4800 2600 50  0001 C CNN
F 4 "" H 4800 2600 50  0001 C CNN "Description"
F 5 "" H 4800 2600 50  0001 C CNN "Package JEDEC ID"
F 6 "Yageo" H 4800 2600 50  0001 C CNN "MFG Name"
F 7 "RC1206FR-7W10KL" H 4800 2600 50  0001 C CNN "MFG Part Num"
	1    4800 2600
	0    -1   -1   0   
$EndComp
$EndSCHEMATC
