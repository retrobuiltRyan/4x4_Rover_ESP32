# 4x4_Rover_ESP32

*video coming eventually*

Generic platform for 1:10 scale robot rover (skid steer style) with ESP32 and PS4 controller.  Hardware [mostly] from scratch. The 1:10 scale (primarily wheel span and chassis width) mimics some common RC and toy-scale bodies. Makes a uniquie sleeper build that can share a body from off-the-shelf kids toys. 
![4x4_skidsteer_rover_real](https://github.com/user-attachments/assets/ce0b91fb-4464-43e5-ac15-fb2367ae0194)

<img width="1233" height="671" alt="barbie_rover_chassis_assy" src="https://github.com/user-attachments/assets/3f10b477-7590-4c3a-a1b9-9fe1fcea4d42" />

### 🔩 Mechanical Hardware/Fasteners for Assembly (BoM)
| Item | Assembly     | Description                                | Qty | Cost   | Line Total | Vendor Link / McMaster Part#         |
|------|--------------|--------------------------------------------|-----|--------|------------|--------------------------------------------------------------------------------------|
| 1    | Wheel        | Lego "Compatible" Tire 94x38 (or similar)  | 1   | $17.65 | $17.65     | [Link](https://www.aliexpress.us/item/3256808684598937.html)                         |
| 2    | Drive        | Shaft Coupler, 4mm dia                     | 4   | $1.65  | $6.60      | [Link](https://www.aliexpress.us/item/3256805863207590.html)                         |
| 3    | Drive        | Motor 12V, 82 RPM                          | 4   | $5.74  | $22.96     | [Link](https://www.aliexpress.us/item/3256807500148966.html) or [Link](https://www.aliexpress.us/item/3256807350691232.html)                       |
| alt3 | Drive        | Motor 12V, 82 RPM + RC Wheel               | 4   | $6.46  | $25.84     | [Link](https://www.aliexpress.us/item/3256806026937992.html)                         |
| 5    | Chassis      | #6-32 x 0.5 in Round Head Screw            | 28  | $0.03  | $0.94      | 90279A148                                                                            |
| 6    | Chassis      | #6-32 x 0.75 in Round Head Screw           | 20  | $0.04  | $0.83      | 90279A151                                                                            |
| 7    | Chassis      | #6-32 nut                                  | 20  | $0.02  | $0.40      | 90480A007                                                                            |
| 8    | Chassis      | #6-32 nylon lock nut                       | 20  | $0.04  | $0.80      | 99397A622                                                                            |
| 9    | Chassis      | #6-32 x 1.0 in F/F Hex Standoff (optional) | 4   | $0.47  | $1.88      | 90480A005                                                                            |
| 10   | Wheel        | #4-40 x 0.5 in Round Head Screw            | 16  | $0.02  | $0.32      | 90272A110                                                                            |
| 11   | Wheel        | #4-40 nut                                  | 16  | $0.01  | $0.16      | 90480A005                                                                            |
| 12   | Wheel        | #4-40 lock nut (optional)                  | 16  | $0.03  | $0.48      | 90631A005                                                                            |
| 13   | On/Off switch | Switch, SPST                              | 1   | $0.71  | $0.71      | [Link](https://www.digikey.com/en/products/detail/e-switch/RA11131121/2720267)       |
|14    | 3D Print Parts| PLA Filament (or whatever)                | ~650g |$0.02/g | $13      | any                                                                                  | 
|      |              |                                            |     |        | **$66.74** | Alternate hardware not included in total cost                                        |


### 🪫 Electronics (Control and Drive PCB)
PS4 Controller (Renewed) https://www.amazon.com/dp/B07QD2HK7B/ref=dp_cr_wdg_tit_rfb
| Reference | Value | Qty | DigiKey P/N | Adafruit P/N | AliExpress |
|------------|--------|-----|--------------|---------------|-------------|
| BZ1 | Buzzer_5V | 1 | 445-2525-1-ND |  |  |
| C1,C3,C4,C5,C7,C8,C9 | 0.1uF 50V | 7 | 1276-1068-1-ND |  |  |
| C6,C10,C11 | 0.22uF 50V CER | 3 | 445-2283-1-ND |  |  |
| C12,C13,C14 | 22uF 50V | 3 | 399-11438-1-ND |  |  |
| D1,D5,D10,D11 | LED | 4 | 1080-1419-1-ND |  |  |
| D6,D7,D8,D9 | WS2812B | 4 |  | 3094 |  |
| D12 | SMF15A | 1 | SMF15A-E3-08CT-ND |  |  |
| J1 | PINHD_1x2_Male | 1 | S1012EC-40-ND |  |  |
| J2,J12,J14,J16 | Screw_Terminal_4_P3.50mm | 4 | 732-691243110004-ND |  |  |
| J3,J15 | ServoPort_04 | 2 | S1012EC-40-ND |  |  |
| J4,J5,J8,J13 | Screw_Terminal_2_P3.50mm | 4 | 732-2747-ND |  |  |
| J9 | Barrel_Jack | 1 | EJ508A-ND |  |  |
| J17 | PINHD_1x3_Male | 1 |  |  |  |
| Q1 | MOSFET P-CH 30V 25A TO252 | 1 | 785-1106-1-ND |  |  |
| Q2 | MMBT2222A | 1 | MMBT2222ATPMSCT-ND |  |  |
| Q3 | AO3400A | 1 | 785-1000-1-ND |  |  |
| R1,R2,R4,R7,R8,R9,R10,R18,R22,R36,R37,R38 | 10K | 12 | 311-10.0KFRCT-ND |  |  |
| R3,R6 | 100 | 2 | 311-100FRCT-ND |  |  |
| R5,R19,R20,R32 | 1K | 4 | 311-1.00KFRCT-ND |  |  |
| R16 | 220 | 1 | 311-220FRCT-ND |  |  |
| R17,R21 | 330 | 2 | 311-330FRCT-ND |  |  |
| R33 | 100k | 1 | 311-100KFRCT-ND |  |  |
| RN1,RN2,RN3 | 220 array | 3 | CAY16-201J4LFCT-ND |  |  |
| U1 | INA260 | 1 | 296-47777-1-ND |  |  |
| U2 | ESP32 | 1 | 1965-ESP32-DEVKITC-32UE-ND |  |  |
| U3 | PCA9685PW | 1 | 568-11925-1-ND |  |  |
| U4,U6,U9 | DRV8833 | 3 |  |  | [Link](https://www.aliexpress.us/item/3256806096142480.html) |
| U5 | UCC27511ADBV | 1 | 296-49474-1-ND |  |  |
| U7 | L7812 | 1 | 296-44522-1-ND |  |  |
| U8 | IMU LSM6DS3TR-C | 1 |  | 4503 |  |
| U14 | MAX40200AUK | 1 | 175-MAX40203AUK+TCT-ND |  |  |
| U15 | L7805 | 1 | 497-7255-1-ND |  |  |
| U18,U19 | MP1584 Buck Module | 2 |  |  | [Link](https://www.aliexpress.us/item/3256806890547813.html) |
*electronics BoM cost = TBD*
