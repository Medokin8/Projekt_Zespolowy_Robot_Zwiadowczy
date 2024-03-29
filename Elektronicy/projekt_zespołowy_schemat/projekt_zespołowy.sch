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
L pspice:0 #GND05
U 1 1 62553190
P 9000 3750
F 0 "#GND05" H 9000 3650 50  0001 C CNN
F 1 "0" H 9000 3839 50  0000 C CNN
F 2 "" H 9000 3750 50  0001 C CNN
F 3 "~" H 9000 3750 50  0001 C CNN
	1    9000 3750
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
F 2 "projekt_zespołowy:stm32f334R8" H 1650 2050 50  0001 R CNN
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
	9000 3500 9000 3750
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
VMOT
Wire Wire Line
	6800 4950 7200 4950
Wire Wire Line
	6800 5400 6850 5400
Wire Wire Line
	6850 5400 6850 5450
Wire Wire Line
	6800 5250 6950 5250
Wire Wire Line
	6950 5250 6950 5450
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
Text GLabel 2950 2450 2    50   Input ~ 0
TX
Wire Wire Line
	5100 4500 5100 4750
Wire Wire Line
	5100 5600 5100 5850
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
$Comp
L projekt_zespołowy-rescue:OV2640-projekt_zespolowy OV1
U 1 1 6253B020
P 4000 5200
F 0 "OV1" H 3975 6565 50  0000 C CNN
F 1 "OV2640-projekt_zespolowy" H 3975 6474 50  0000 C CNN
F 2 "" H 4000 6450 50  0001 C CNN
F 3 "" H 4000 6450 50  0001 C CNN
	1    4000 5200
	0    1    1    0   
$EndComp
$Comp
L projekt_zespołowy-rescue:TB6612FNG-projekt_zespolowy U8
U 1 1 6255B122
P 8150 2850
F 0 "U8" H 8175 2235 50  0000 C CNN
F 1 "TB6612FNG-projekt_zespolowy" H 8175 2326 50  0000 C CNN
F 2 "projekt_zespołowy:TB6612FNG" H 8100 3250 50  0001 C CNN
F 3 "" H 8100 3250 50  0001 C CNN
	1    8150 2850
	-1   0    0    1   
$EndComp
$Comp
L projekt_zespołowy-rescue:HC-06_ZS-040-projekt_zespolowy B1
U 1 1 6255C17F
P 6550 5150
F 0 "B1" V 7015 5392 50  0000 C CNN
F 1 "HC-06_ZS-040-projekt_zespolowy" V 6924 5392 50  0000 C CNN
F 2 "projekt_zespołowy:HC-06_ZS-040" H 6500 6000 50  0001 C CNN
F 3 "" H 6500 6000 50  0001 C CNN
	1    6550 5150
	0    -1   -1   0   
$EndComp
$Comp
L projekt_zespołowy-rescue:Encoder-projekt_zespolowy E1
U 1 1 62561183
P 9650 2350
F 0 "E1" H 9675 2825 50  0000 C CNN
F 1 "Encoder-projekt_zespolowy" H 9675 2734 50  0000 C CNN
F 2 "" H 9650 2700 50  0001 C CNN
F 3 "" H 9650 2700 50  0001 C CNN
	1    9650 2350
	1    0    0    -1  
$EndComp
$Comp
L projekt_zespołowy-rescue:Encoder-projekt_zespolowy E2
U 1 1 62561CF1
P 9650 3250
F 0 "E2" H 9675 3725 50  0000 C CNN
F 1 "Encoder-projekt_zespolowy" H 9675 3634 50  0000 C CNN
F 2 "" H 9650 3600 50  0001 C CNN
F 3 "" H 9650 3600 50  0001 C CNN
	1    9650 3250
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR04
U 1 1 625F874D
P 8900 1850
F 0 "#PWR04" H 8900 1700 50  0001 C CNN
F 1 "+3V3" H 8915 2023 50  0000 C CNN
F 2 "" H 8900 1850 50  0001 C CNN
F 3 "" H 8900 1850 50  0001 C CNN
	1    8900 1850
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR03
U 1 1 6280003F
P 5100 4500
F 0 "#PWR03" H 5100 4350 50  0001 C CNN
F 1 "+3V3" H 5115 4673 50  0000 C CNN
F 2 "" H 5100 4500 50  0001 C CNN
F 3 "" H 5100 4500 50  0001 C CNN
	1    5100 4500
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR01
U 1 1 6282018A
P 2150 1600
F 0 "#PWR01" H 2150 1450 50  0001 C CNN
F 1 "+3V3" H 2165 1773 50  0000 C CNN
F 2 "" H 2150 1600 50  0001 C CNN
F 3 "" H 2150 1600 50  0001 C CNN
	1    2150 1600
	1    0    0    -1  
$EndComp
$Comp
L Regulator_Linear:LM7808_TO220 U9
U 1 1 6260DECE
P 9700 5000
F 0 "U9" H 9650 5150 50  0000 L CNN
F 1 "LM7808_TO220" H 9400 5250 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 9700 5225 50  0001 C CIN
F 3 "https://www.onsemi.cn/PowerSolutions/document/MC7800-D.PDF" H 9700 4950 50  0001 C CNN
	1    9700 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	9400 5000 9150 5000
Wire Wire Line
	9700 5750 10250 5750
Wire Wire Line
	9700 5750 9150 5750
Connection ~ 9700 5750
$Comp
L Device:C C3
U 1 1 62671222
P 9150 5400
F 0 "C3" H 9265 5446 50  0000 L CNN
F 1 "330n" H 9265 5355 50  0000 L CNN
F 2 "projekt_zespołowy:C" H 9188 5250 50  0001 C CNN
F 3 "~" H 9150 5400 50  0001 C CNN
	1    9150 5400
	1    0    0    -1  
$EndComp
$Comp
L Device:C C4
U 1 1 62672C38
P 10250 5400
F 0 "C4" H 10365 5446 50  0000 L CNN
F 1 "100n" H 10365 5355 50  0000 L CNN
F 2 "projekt_zespołowy:C" H 10288 5250 50  0001 C CNN
F 3 "~" H 10250 5400 50  0001 C CNN
	1    10250 5400
	1    0    0    -1  
$EndComp
Wire Wire Line
	8750 5000 9150 5000
Connection ~ 9150 5000
Wire Wire Line
	9150 5550 9150 5750
Wire Wire Line
	10250 5750 10250 5550
Wire Wire Line
	10000 5000 10250 5000
Wire Wire Line
	9150 5000 9150 5250
Wire Wire Line
	10250 5000 10250 5250
Wire Wire Line
	9700 5300 9700 5750
Wire Wire Line
	8200 5750 9150 5750
Connection ~ 9150 5750
Text GLabel 10400 5750 2    50   Input ~ 0
GND
Text GLabel 10300 5000 2    50   Input ~ 0
VMOT
Wire Wire Line
	10400 5750 10250 5750
Connection ~ 10250 5750
Text GLabel 2950 2350 2    50   Input ~ 0
RX
Wire Wire Line
	8200 5000 8450 5000
Wire Wire Line
	10250 5000 10300 5000
Connection ~ 10250 5000
Wire Wire Line
	2350 1600 2350 1950
$Comp
L power:+5V #PWR?
U 1 1 62A41660
P 2350 1600
F 0 "#PWR?" H 2350 1450 50  0001 C CNN
F 1 "+5V" H 2365 1773 50  0000 C CNN
F 2 "" H 2350 1600 50  0001 C CNN
F 3 "" H 2350 1600 50  0001 C CNN
	1    2350 1600
	1    0    0    -1  
$EndComp
$Comp
L Connector:pin P?
U 1 1 62A4A3D1
P 8650 2200
F 0 "P?" V 8850 2550 50  0000 C CNN
F 1 "bat_lt_9V+" V 8750 2450 50  0000 C CNN
F 2 "" V 8650 2450 50  0001 C CNN
F 3 "" V 8650 2450 50  0001 C CNN
	1    8650 2200
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 62A52854
P 7300 1850
F 0 "#PWR?" H 7300 1700 50  0001 C CNN
F 1 "+5V" H 7315 2023 50  0000 C CNN
F 2 "" H 7300 1850 50  0001 C CNN
F 3 "" H 7300 1850 50  0001 C CNN
	1    7300 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	8900 1850 8900 2300
Wire Wire Line
	7300 1850 7300 2850
Wire Wire Line
	9000 3500 9400 3500
Connection ~ 9000 3500
Wire Wire Line
	8750 3500 9000 3500
$Comp
L pspice:0 #GND?
U 1 1 62A65346
P 7350 3750
F 0 "#GND?" H 7350 3650 50  0001 C CNN
F 1 "0" H 7350 3839 50  0000 C CNN
F 2 "" H 7350 3750 50  0001 C CNN
F 3 "~" H 7350 3750 50  0001 C CNN
	1    7350 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	7350 2450 7350 3750
$Comp
L Connector:pin P?
U 1 1 62A72187
P 7450 2400
F 0 "P?" V 7350 2500 50  0000 L CNN
F 1 "bat_lt_9V-" V 7450 2500 50  0000 L CNN
F 2 "" V 7450 2650 50  0001 C CNN
F 3 "" V 7450 2650 50  0001 C CNN
	1    7450 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	7350 2450 7450 2450
Connection ~ 7450 2450
Wire Wire Line
	7450 2450 7750 2450
Connection ~ 8700 2200
Wire Wire Line
	8700 2200 8700 2550
Wire Wire Line
	8700 1850 8700 2200
$Comp
L Device:Polyfuse F?
U 1 1 62A7CAEE
P 8600 5000
F 0 "F?" V 8375 5000 50  0000 C CNN
F 1 "Polyfuse" V 8466 5000 50  0000 C CNN
F 2 "" H 8650 4800 50  0001 L CNN
F 3 "~" H 8600 5000 50  0001 C CNN
	1    8600 5000
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 62A8B142
P 7200 4500
F 0 "#PWR?" H 7200 4350 50  0001 C CNN
F 1 "+5V" H 7215 4673 50  0000 C CNN
F 2 "" H 7200 4500 50  0001 C CNN
F 3 "" H 7200 4500 50  0001 C CNN
	1    7200 4500
	1    0    0    -1  
$EndComp
$Comp
L Connector:pin P?
U 1 1 62A9C399
P 8150 5750
F 0 "P?" V 8050 5850 50  0000 L CNN
F 1 "bat_gt_9V-" V 8150 5850 50  0000 L CNN
F 2 "" V 8150 6000 50  0001 C CNN
F 3 "" V 8150 6000 50  0001 C CNN
	1    8150 5750
	0    -1   -1   0   
$EndComp
$Comp
L Connector:pin P?
U 1 1 62A9FD3A
P 8150 5000
F 0 "P?" V 8050 5100 50  0000 L CNN
F 1 "bat_gt_9V+" V 8150 5100 50  0000 L CNN
F 2 "" V 8150 5250 50  0001 C CNN
F 3 "" V 8150 5250 50  0001 C CNN
	1    8150 5000
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6800 5100 7200 5100
Wire Wire Line
	7200 4950 7200 4500
Wire Wire Line
	7200 5100 7200 5450
$EndSCHEMATC
