EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Device:Battery_Cell BT1
U 1 1 6251FDAE
P 5400 2800
F 0 "BT1" V 5500 2650 50  0000 L CNN
F 1 "Battery_Cell" V 5600 2650 50  0000 L CNN
F 2 "projekt_zespolowy:bat" V 5400 2860 50  0001 C CNN
F 3 "~" V 5400 2860 50  0001 C CNN
	1    5400 2800
	1    0    0    -1  
$EndComp
$Comp
L projekt~zespolowy:TB6612FNG U2
U 1 1 6252088A
P 8150 2850
F 0 "U2" H 8175 3375 50  0000 C CNN
F 1 "TB6612FNG" H 8175 3284 50  0000 C CNN
F 2 "projekt_zespolowy:TB6612FNG" H 8100 3250 50  0001 C CNN
F 3 "" H 8100 3250 50  0001 C CNN
	1    8150 2850
	-1   0    0    1   
$EndComp
$Comp
L projekt~zespolowy:Encoder E1
U 1 1 625211D9
P 9650 2350
F 0 "E1" H 9675 2825 50  0000 C CNN
F 1 "Encoder" H 9675 2734 50  0000 C CNN
F 2 "" H 9650 2700 50  0001 C CNN
F 3 "" H 9650 2700 50  0001 C CNN
	1    9650 2350
	1    0    0    -1  
$EndComp
$Comp
L projekt~zespolowy:Encoder E2
U 1 1 62521B1B
P 9650 3250
F 0 "E2" H 9675 3725 50  0000 C CNN
F 1 "Encoder" H 9675 3634 50  0000 C CNN
F 2 "" H 9650 3600 50  0001 C CNN
F 3 "" H 9650 3600 50  0001 C CNN
	1    9650 3250
	1    0    0    -1  
$EndComp
$Comp
L Motor:Motor_DC M1
U 1 1 6252586F
P 10400 2300
F 0 "M1" H 10558 2296 50  0000 L CNN
F 1 "Motor_DC" H 10558 2205 50  0000 L CNN
F 2 "" H 10400 2210 50  0001 C CNN
F 3 "~" H 10400 2210 50  0001 C CNN
	1    10400 2300
	1    0    0    -1  
$EndComp
$Comp
L Motor:Motor_DC M2
U 1 1 6252632F
P 10400 3200
F 0 "M2" H 10558 3196 50  0000 L CNN
F 1 "Motor_DC" H 10558 3105 50  0000 L CNN
F 2 "" H 10400 3110 50  0001 C CNN
F 3 "~" H 10400 3110 50  0001 C CNN
	1    10400 3200
	1    0    0    -1  
$EndComp
$Comp
L projekt~zespolowy:HC-06_ZS-040 B1
U 1 1 625293F3
P 6550 5150
F 0 "B1" V 7015 5392 50  0000 C CNN
F 1 "HC-06_ZS-040" V 6924 5392 50  0000 C CNN
F 2 "projekt_zespolowy:HC-06_ZS-040" H 6500 6000 50  0001 C CNN
F 3 "" H 6500 6000 50  0001 C CNN
	1    6550 5150
	0    -1   -1   0   
$EndComp
$Comp
L projekt~zespolowy:OV2640 OV1
U 1 1 62529CA3
P 4000 5200
F 0 "OV1" H 3975 6565 50  0000 C CNN
F 1 "OV2640" H 3975 6474 50  0000 C CNN
F 2 "" H 4000 6450 50  0001 C CNN
F 3 "" H 4000 6450 50  0001 C CNN
	1    4000 5200
	0    1    1    0   
$EndComp
$Comp
L Regulator_Linear:LM7808_TO220 U4
U 1 1 6252D06C
P 8850 5100
F 0 "U4" H 8800 5250 50  0000 L CNN
F 1 "LM7808_TO220" H 8550 5350 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 8850 5325 50  0001 C CIN
F 3 "https://www.onsemi.cn/PowerSolutions/document/MC7800-D.PDF" H 8850 5050 50  0001 C CNN
	1    8850 5100
	0    1    1    0   
$EndComp
$Comp
L Regulator_Linear:LM7808_TO220 U5
U 1 1 62532B9D
P 9600 5100
F 0 "U5" H 9550 5250 50  0000 L CNN
F 1 "LM7808_TO220" H 9300 5350 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 9600 5325 50  0001 C CIN
F 3 "https://www.onsemi.cn/PowerSolutions/document/MC7800-D.PDF" H 9600 5050 50  0001 C CNN
	1    9600 5100
	0    1    1    0   
$EndComp
$Comp
L Regulator_Linear:LM7808_TO220 U3
U 1 1 62533A6E
P 8100 5100
F 0 "U3" H 8050 5250 50  0000 L CNN
F 1 "LM7808_TO220" H 7800 5350 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 8100 5325 50  0001 C CIN
F 3 "https://www.onsemi.cn/PowerSolutions/document/MC7800-D.PDF" H 8100 5050 50  0001 C CNN
	1    8100 5100
	0    1    1    0   
$EndComp
$Comp
L Device:R R2
U 1 1 625424B0
P 5000 3250
F 0 "R2" H 5070 3296 50  0000 L CNN
F 1 "100" H 5070 3205 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P15.24mm_Horizontal" V 4930 3250 50  0001 C CNN
F 3 "~" H 5000 3250 50  0001 C CNN
	1    5000 3250
	1    0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 62542ACC
P 8500 5750
F 0 "C2" H 8615 5796 50  0000 L CNN
F 1 "100n" H 8615 5705 50  0000 L CNN
F 2 "projekt_zespolowy:C" H 8538 5600 50  0001 C CNN
F 3 "~" H 8500 5750 50  0001 C CNN
	1    8500 5750
	0    1    1    0   
$EndComp
$Comp
L Device:C C1
U 1 1 62548C21
P 8450 4450
F 0 "C1" H 8565 4496 50  0000 L CNN
F 1 "100n" H 8565 4405 50  0000 L CNN
F 2 "projekt_zespolowy:C" H 8488 4300 50  0001 C CNN
F 3 "~" H 8450 4450 50  0001 C CNN
	1    8450 4450
	0    -1   -1   0   
$EndComp
$Comp
L pspice:0 #GND03
U 1 1 6254AFF5
P 5400 3700
F 0 "#GND03" H 5400 3600 50  0001 C CNN
F 1 "0" H 5400 3789 50  0000 C CNN
F 2 "" H 5400 3700 50  0001 C CNN
F 3 "~" H 5400 3700 50  0001 C CNN
	1    5400 3700
	1    0    0    -1  
$EndComp
$Comp
L pspice:0 #GND05
U 1 1 62553190
P 8750 3750
F 0 "#GND05" H 8750 3650 50  0001 C CNN
F 1 "0" H 8750 3839 50  0000 C CNN
F 2 "" H 8750 3750 50  0001 C CNN
F 3 "~" H 8750 3750 50  0001 C CNN
	1    8750 3750
	1    0    0    -1  
$EndComp
$Comp
L pspice:0 #GND02
U 1 1 62553BD8
P 5100 5850
F 0 "#GND02" H 5100 5750 50  0001 C CNN
F 1 "0" H 5100 5939 50  0000 C CNN
F 2 "" H 5100 5850 50  0001 C CNN
F 3 "~" H 5100 5850 50  0001 C CNN
	1    5100 5850
	1    0    0    -1  
$EndComp
$Comp
L MCU_ST_STM32F3:STM32F334R8Tx U1
U 1 1 62555389
P 2250 3750
F 0 "U1" H 2950 5850 50  0000 C CNN
F 1 "STM32F334R8Tx" H 3150 5750 50  0000 C CNN
F 2 "projekt_zespolowy:stm32f334R8" H 1650 2050 50  0001 R CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00097745.pdf" H 2250 3750 50  0001 C CNN
	1    2250 3750
	1    0    0    -1  
$EndComp
$Comp
L pspice:0 #GND01
U 1 1 6256F892
P 2150 5850
F 0 "#GND01" H 2150 5750 50  0001 C CNN
F 1 "0" H 2150 5939 50  0000 C CNN
F 2 "" H 2150 5850 50  0001 C CNN
F 3 "~" H 2150 5850 50  0001 C CNN
	1    2150 5850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2150 5850 2150 5550
Wire Wire Line
	2150 1950 2150 1600
Wire Wire Line
	9950 2200 10050 2200
Wire Wire Line
	10050 2200 10050 2600
Wire Wire Line
	10050 2600 10400 2600
Wire Wire Line
	9950 2100 10400 2100
Wire Wire Line
	10400 3000 9950 3000
Wire Wire Line
	9950 3100 10050 3100
Wire Wire Line
	10050 3100 10050 3500
Wire Wire Line
	10050 3500 10400 3500
Wire Wire Line
	9400 2600 8750 2600
Wire Wire Line
	8750 3500 9400 3500
Wire Wire Line
	8750 3500 8750 3750
Connection ~ 8750 3500
Wire Wire Line
	8750 2600 8750 3150
Wire Wire Line
	8500 3150 8750 3150
Connection ~ 8750 3150
Wire Wire Line
	8750 3150 8750 3500
Wire Wire Line
	8500 3050 8900 3050
Wire Wire Line
	8900 3050 8900 2300
Wire Wire Line
	9400 2300 8900 2300
Connection ~ 8900 2300
Wire Wire Line
	8900 2300 8900 2200
Wire Wire Line
	9400 3200 8900 3200
Wire Wire Line
	8900 3200 8900 3050
Connection ~ 8900 3050
Wire Wire Line
	8500 2950 9400 2950
Wire Wire Line
	9400 2950 9400 3000
Wire Wire Line
	8500 2850 9350 2850
Wire Wire Line
	9350 2850 9350 3100
Wire Wire Line
	9350 3100 9400 3100
Wire Wire Line
	8500 2750 9150 2750
Wire Wire Line
	9150 2750 9150 2200
Wire Wire Line
	8500 2650 9050 2650
Wire Wire Line
	9050 2650 9050 2100
Wire Wire Line
	9050 2100 9400 2100
Wire Wire Line
	9150 2200 9400 2200
Wire Wire Line
	8500 2550 8700 2550
Wire Wire Line
	8500 2450 8750 2450
Wire Wire Line
	8750 2450 8750 2600
Connection ~ 8750 2600
Wire Wire Line
	7750 2850 7300 2850
Wire Wire Line
	7300 2850 7300 2200
Wire Wire Line
	7300 2200 8900 2200
Wire Wire Line
	7750 2450 7650 2450
Wire Wire Line
	7650 2450 7650 2350
Wire Wire Line
	7650 2350 8750 2350
Wire Wire Line
	8750 2350 8750 2450
Connection ~ 8750 2450
Text GLabel 7750 3150 0    50   Input ~ 0
PWMA
Text GLabel 7750 3050 0    50   Input ~ 0
AIN2
Text GLabel 7750 2950 0    50   Input ~ 0
AIN1
Text GLabel 7750 2750 0    50   Input ~ 0
BIN1
Text GLabel 7750 2650 0    50   Input ~ 0
BIN2
Text GLabel 7750 2550 0    50   Input ~ 0
PWMB
Text GLabel 9400 2400 0    50   Input ~ 0
E1OA
Text GLabel 9400 2500 0    50   Input ~ 0
E1OB
Text GLabel 9400 3300 0    50   Input ~ 0
E2OA
Text GLabel 9400 3400 0    50   Input ~ 0
E2OB
Text GLabel 8700 1850 1    50   Input ~ 0
VMOTS
$Comp
L Device:R R1
U 1 1 6253404C
P 5000 2750
F 0 "R1" H 5070 2796 50  0000 L CNN
F 1 "236" H 5070 2705 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P15.24mm_Horizontal" V 4930 2750 50  0001 C CNN
F 3 "~" H 5000 2750 50  0001 C CNN
	1    5000 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	5000 2600 5000 2450
Wire Wire Line
	5000 3500 5000 3400
Connection ~ 5400 3500
Wire Wire Line
	5000 3100 5000 3000
Wire Wire Line
	5400 2900 5400 3500
Wire Wire Line
	5400 3500 5400 3700
Text GLabel 5800 2450 2    50   Input ~ 0
VMOT
Text GLabel 5800 3500 2    50   Input ~ 0
GND
Text GLabel 4800 3500 0    50   Input ~ 0
GND
Text GLabel 4800 3000 0    50   Input ~ 0
VCC
Wire Wire Line
	4800 3000 5000 3000
Connection ~ 5000 3000
Wire Wire Line
	5000 3000 5000 2900
Wire Wire Line
	5000 3500 4800 3500
Wire Wire Line
	8850 5400 8850 5500
Wire Wire Line
	8650 5750 8850 5750
Wire Wire Line
	8350 5750 7650 5750
Wire Wire Line
	7650 4450 8300 4450
Wire Wire Line
	8600 4450 8850 4450
Wire Wire Line
	8850 4450 8850 4650
Wire Wire Line
	9600 4800 9600 4650
Connection ~ 8850 4650
Wire Wire Line
	8850 4650 8850 4800
Wire Wire Line
	8850 4650 8100 4650
Wire Wire Line
	8100 4650 8100 4800
Wire Wire Line
	8100 5400 8100 5500
Wire Wire Line
	8100 5500 8850 5500
Connection ~ 8850 5500
Wire Wire Line
	8850 5500 8850 5750
Wire Wire Line
	9600 5500 8850 5500
Wire Wire Line
	7650 5750 7650 5600
Wire Wire Line
	9300 5100 9300 5600
Wire Wire Line
	9300 5600 8550 5600
Connection ~ 7650 5600
Wire Wire Line
	7650 5600 7650 4450
Wire Wire Line
	7800 5100 7800 5600
Connection ~ 7800 5600
Wire Wire Line
	7800 5600 7650 5600
Wire Wire Line
	8550 5100 8550 5600
Connection ~ 8550 5600
Wire Wire Line
	8550 5600 7800 5600
Wire Wire Line
	8850 5750 9300 5750
Connection ~ 8850 5750
Wire Wire Line
	8850 4450 9300 4450
Connection ~ 8850 4450
Text GLabel 9300 4450 2    50   Input ~ 0
VMOT
Text GLabel 9300 5750 2    50   Input ~ 0
VMOTS
Wire Wire Line
	6800 4950 7200 4950
Wire Wire Line
	7200 4950 7200 4550
Wire Wire Line
	6800 5400 6850 5400
Wire Wire Line
	6850 5400 6850 5450
Wire Wire Line
	6800 5250 6950 5250
Wire Wire Line
	6950 5250 6950 5450
Wire Wire Line
	6800 5100 7200 5100
Wire Wire Line
	7200 5100 7200 5450
$Comp
L pspice:0 #GND04
U 1 1 6255394F
P 7200 5450
F 0 "#GND04" H 7200 5350 50  0001 C CNN
F 1 "0" H 7200 5539 50  0000 C CNN
F 2 "" H 7200 5450 50  0001 C CNN
F 3 "~" H 7200 5450 50  0001 C CNN
	1    7200 5450
	1    0    0    -1  
$EndComp
Text GLabel 6950 5450 3    50   Input ~ 0
TX
Text GLabel 6850 5450 3    50   Input ~ 0
RX
Text GLabel 2950 2350 2    50   Input ~ 0
TX
Text GLabel 2950 2450 2    50   Input ~ 0
RX
Wire Wire Line
	5100 4500 5100 4750
Wire Wire Line
	5100 5600 5100 5850
Wire Wire Line
	8700 1850 8700 2550
Text GLabel 4950 4750 1    50   Input ~ 0
SIOC
Text GLabel 4650 4750 1    50   Input ~ 0
PCLK
Text GLabel 4450 4750 1    50   Input ~ 0
D7
Text GLabel 4300 4750 1    50   Input ~ 0
D5
Text GLabel 4150 4750 1    50   Input ~ 0
D3
Text GLabel 4000 4750 1    50   Input ~ 0
D1
Text GLabel 3800 4750 1    50   Input ~ 0
RESET
Text GLabel 3800 5600 3    50   Input ~ 0
PWDN
Text GLabel 4000 5600 3    50   Input ~ 0
D0
Text GLabel 4150 5600 3    50   Input ~ 0
D2
Text GLabel 4300 5600 3    50   Input ~ 0
D4
Text GLabel 4450 5600 3    50   Input ~ 0
D6
Text GLabel 4650 5600 3    50   Input ~ 0
XCLK
Text GLabel 4950 5600 3    50   Input ~ 0
SIOD
Text GLabel 1550 5350 0    50   Input ~ 0
AIN2
Text GLabel 1550 5250 0    50   Input ~ 0
AIN1
Text GLabel 1550 5150 0    50   Input ~ 0
BIN1
Text GLabel 1550 5050 0    50   Input ~ 0
BIN2
Text GLabel 1550 4650 0    50   Input ~ 0
E2OA
Text GLabel 1550 4750 0    50   Input ~ 0
E2OB
Text GLabel 1550 4850 0    50   Input ~ 0
E1OA
Text GLabel 1550 4950 0    50   Input ~ 0
E1OB
Text GLabel 2950 2650 2    50   Input ~ 0
SIOC
Text GLabel 2950 2950 2    50   Input ~ 0
PCLK
Text GLabel 2950 5350 2    50   Input ~ 0
D7
Text GLabel 2950 5150 2    50   Input ~ 0
D5
Text GLabel 2950 4950 2    50   Input ~ 0
D3
Text GLabel 2950 3550 2    50   Input ~ 0
D1
Text GLabel 1550 3450 0    50   Input ~ 0
RESET
Text GLabel 1550 3350 0    50   Input ~ 0
PWDN
Text GLabel 2950 3450 2    50   Input ~ 0
D0
Text GLabel 2950 3650 2    50   Input ~ 0
D2
Text GLabel 2950 5050 2    50   Input ~ 0
D4
Text GLabel 2950 5250 2    50   Input ~ 0
D6
Text GLabel 2950 2850 2    50   Input ~ 0
XCLK
Text GLabel 2950 2750 2    50   Input ~ 0
SIOD
Text GLabel 1550 4550 0    50   Input ~ 0
PWMA
Text GLabel 1550 4450 0    50   Input ~ 0
PWMB
Wire Wire Line
	5000 2450 5400 2450
Wire Wire Line
	5400 3500 5000 3500
Connection ~ 5000 3500
Connection ~ 8900 2200
Wire Wire Line
	8900 2200 8900 1850
Text GLabel 5100 4500 1    50   Input ~ 0
VCC
Text GLabel 7200 4550 1    50   Input ~ 0
VCC
Text GLabel 8900 1850 1    50   Input ~ 0
VCC
Text GLabel 2150 1600 1    50   Input ~ 0
VCC
$Comp
L Regulator_Linear:LM7808_TO220 U6
U 1 1 625487B6
P 10250 5100
F 0 "U6" H 10200 5250 50  0000 L CNN
F 1 "LM7808_TO220" H 9950 5350 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 10250 5325 50  0001 C CIN
F 3 "https://www.onsemi.cn/PowerSolutions/document/MC7800-D.PDF" H 10250 5050 50  0001 C CNN
	1    10250 5100
	0    1    1    0   
$EndComp
Wire Wire Line
	10250 5400 10250 5500
Wire Wire Line
	9600 5400 9600 5500
Wire Wire Line
	10250 5500 9600 5500
Connection ~ 9600 5500
Wire Wire Line
	10250 4800 10250 4650
Wire Wire Line
	8850 4650 9600 4650
Connection ~ 9600 4650
Wire Wire Line
	9600 4650 10250 4650
Wire Wire Line
	9950 5100 9950 5600
Wire Wire Line
	9950 5600 9300 5600
Connection ~ 9300 5600
Wire Wire Line
	5400 2450 5400 2600
Wire Wire Line
	5800 3500 5400 3500
Wire Wire Line
	5800 2450 5400 2450
Connection ~ 5400 2450
$EndSCHEMATC
