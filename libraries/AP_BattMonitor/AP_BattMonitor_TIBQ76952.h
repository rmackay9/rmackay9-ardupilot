#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/SPIDevice.h>
#include "AP_BattMonitor_Backend.h"
#include <utility>

#if AP_BATTERY_TIBQ76952_ENABLED

//Data	Memory	registers	Name in TRM
#define TIBQ769x2_Cell1Gain 0x9180      //Calibration:Voltage:Cell 1 Gain			
#define TIBQ769x2_Cell2Gain 0x9182      //Calibration:Voltage:Cell 2 Gain			
#define TIBQ769x2_Cell3Gain 0x9184      //Calibration:Voltage:Cell 3 Gain			
#define TIBQ769x2_Cell4Gain 0x9186      //Calibration:Voltage:Cell 4 Gain			
#define TIBQ769x2_Cell5Gain 0x9188      //Calibration:Voltage:Cell 5 Gain			
#define TIBQ769x2_Cell6Gain 0x918A      //Calibration:Voltage:Cell 6 Gain			
#define TIBQ769x2_Cell7Gain 0x918C      //Calibration:Voltage:Cell 7 Gain			
#define TIBQ769x2_Cell8Gain 0x918E      //Calibration:Voltage:Cell 8 Gain			
#define TIBQ769x2_Cell9Gain 0x9190      //Calibration:Voltage:Cell 9 Gain			
#define TIBQ769x2_Cell10Gain 0x9192      //Calibration:Voltage:Cell 10 Gain			
#define TIBQ769x2_Cell11Gain 0x9194      //Calibration:Voltage:Cell 11 Gain			
#define TIBQ769x2_Cell12Gain 0x9196      //Calibration:Voltage:Cell 12 Gain			
#define TIBQ769x2_Cell13Gain 0x9198      //Calibration:Voltage:Cell 13 Gain			
#define TIBQ769x2_Cell14Gain 0x919A      //Calibration:Voltage:Cell 14 Gain			
#define TIBQ769x2_Cell15Gain 0x919C      //Calibration:Voltage:Cell 15 Gain			
#define TIBQ769x2_Cell16Gain 0x919E      //Calibration:Voltage:Cell 16 Gain			
#define TIBQ769x2_PackGain 0x91A0      //Calibration:Voltage:Pack Gain			
#define TIBQ769x2_TOSGain 0x91A2      //Calibration:Voltage:TOS Gain			
#define TIBQ769x2_LDGain 0x91A4      //Calibration:Voltage:LD Gain			
#define TIBQ769x2_ADCGain 0x91A6      //Calibration:Voltage:ADC Gain			
#define TIBQ769x2_CCGain 0x91A8      //Calibration:Current:CC Gain			
#define TIBQ769x2_CapacityGain 0x91AC      //Calibration:Current:Capacity Gain			
#define TIBQ769x2_VcellOffset 0x91B0      //Calibration:Vcell Offset:Vcell Offset			
#define TIBQ769x2_VdivOffset 0x91B2      //Calibration:V Divider Offset:Vdiv Offset			
#define TIBQ769x2_CoulombCounterOffsetSamples 0x91C6      //Calibration:Current Offset:Coulomb Counter Offset Samples			
#define TIBQ769x2_BoardOffset 0x91C8      //Calibration:Current Offset:Board Offset			
#define TIBQ769x2_InternalTempOffset 0x91CA      //Calibration:Temperature:Internal Temp Offset			
#define TIBQ769x2_CFETOFFTempOffset 0x91CB      //Calibration:Temperature:CFETOFF Temp Offset			
#define TIBQ769x2_DFETOFFTempOffset 0x91CC      //Calibration:Temperature:DFETOFF Temp Offset			
#define TIBQ769x2_ALERTTempOffset 0x91CD      //Calibration:Temperature:ALERT Temp Offset			
#define TIBQ769x2_TS1TempOffset 0x91CE      //Calibration:Temperature:TS1 Temp Offset			
#define TIBQ769x2_TS2TempOffset 0x91CF      //Calibration:Temperature:TS2 Temp Offset			
#define TIBQ769x2_TS3TempOffset 0x91D0      //Calibration:Temperature:TS3 Temp Offset			
#define TIBQ769x2_HDQTempOffset 0x91D1      //Calibration:Temperature:HDQ Temp Offset			
#define TIBQ769x2_DCHGTempOffset 0x91D2      //Calibration:Temperature:DCHG Temp Offset			
#define TIBQ769x2_DDSGTempOffset 0x91D3      //Calibration:Temperature:DDSG Temp Offset			
#define TIBQ769x2_IntGain 0x91E2      //Calibration:Internal Temp Model:Int Gain			
#define TIBQ769x2_Intbaseoffset 0x91E4      //Calibration:Internal Temp Model:Int base offset			
#define TIBQ769x2_IntMaximumAD 0x91E6      //Calibration:Internal Temp Model:Int Maximum AD			
#define TIBQ769x2_IntMaximumTemp 0x91E8      //Calibration:Internal Temp Model:Int Maximum Temp			
#define TIBQ769x2_T18kCoeffa1 0x91EA      //Calibration:18K Temperature Model:Coeff a1			
#define TIBQ769x2_T18kCoeffa2 0x91EC      //Calibration:18K Temperature Model:Coeff a2			
#define TIBQ769x2_T18kCoeffa3 0x91EE      //Calibration:18K Temperature Model:Coeff a3			
#define TIBQ769x2_T18kCoeffa4 0x91F0      //Calibration:18K Temperature Model:Coeff a4			
#define TIBQ769x2_T18kCoeffa5 0x91F2      //Calibration:18K Temperature Model:Coeff a5			
#define TIBQ769x2_T18kCoeffb1 0x91F4      //Calibration:18K Temperature Model:Coeff b1			
#define TIBQ769x2_T18kCoeffb2 0x91F6      //Calibration:18K Temperature Model:Coeff b2			
#define TIBQ769x2_T18kCoeffb3 0x91F8      //Calibration:18K Temperature Model:Coeff b3			
#define TIBQ769x2_T18kCoeffb4 0x91FA      //Calibration:18K Temperature Model:Coeff b4			
#define TIBQ769x2_T18kAdc0 0x91FE      //Calibration:18K Temperature Model:Adc0			
#define TIBQ769x2_T180kCoeffa1 0x9200      //Calibration:180K Temperature Model:Coeff a1			
#define TIBQ769x2_T180kCoeffa2 0x9202      //Calibration:180K Temperature Model:Coeff a2			
#define TIBQ769x2_T180kCoeffa3 0x9204      //Calibration:180K Temperature Model:Coeff a3			
#define TIBQ769x2_T180kCoeffa4 0x9206      //Calibration:180K Temperature Model:Coeff a4			
#define TIBQ769x2_T180kCoeffa5 0x9208      //Calibration:180K Temperature Model:Coeff a5			
#define TIBQ769x2_T180kCoeffb1 0x920A      //Calibration:180K Temperature Model:Coeff b1			
#define TIBQ769x2_T180kCoeffb2 0x920C      //Calibration:180K Temperature Model:Coeff b2			
#define TIBQ769x2_T180kCoeffb3 0x920E      //Calibration:180K Temperature Model:Coeff b3			
#define TIBQ769x2_T180kCoeffb4 0x9210      //Calibration:180K Temperature Model:Coeff b4			
#define TIBQ769x2_T180kAdc0 0x9214      //Calibration:180K Temperature Model:Adc0			
#define TIBQ769x2_CustomCoeffa1 0x9216      //Calibration:Custom Temperature Model:Coeff a1			
#define TIBQ769x2_CustomCoeffa2 0x9218      //Calibration:Custom Temperature Model:Coeff a2			
#define TIBQ769x2_CustomCoeffa3 0x921A      //Calibration:Custom Temperature Model:Coeff a3			
#define TIBQ769x2_CustomCoeffa4 0x921C      //Calibration:Custom Temperature Model:Coeff a4			
#define TIBQ769x2_CustomCoeffa5 0x921E      //Calibration:Custom Temperature Model:Coeff a5			
#define TIBQ769x2_CustomCoeffb1 0x9220      //Calibration:Custom Temperature Model:Coeff b1			
#define TIBQ769x2_CustomCoeffb2 0x9222      //Calibration:Custom Temperature Model:Coeff b2			
#define TIBQ769x2_CustomCoeffb3 0x9224      //Calibration:Custom Temperature Model:Coeff b3			
#define TIBQ769x2_CustomCoeffb4 0x9226      //Calibration:Custom Temperature Model:Coeff b4			
#define TIBQ769x2_CustomRc0 0x9228      //Calibration:Custom Temperature Model:Rc0			
#define TIBQ769x2_CustomAdc0 0x922A      //Calibration:Custom Temperature Model:Adc0			
#define TIBQ769x2_CoulombCounterDeadband 0x922D      //Calibration:Current Deadband:Coulomb Counter Deadband			
#define TIBQ769x2_CUVThresholdOverride 0x91D4      //Calibration:CUV:CUV Threshold Override			
#define TIBQ769x2_COVThresholdOverride 0x91D6      //Calibration:COV:COV Threshold Override			
#define TIBQ769x2_MinBlowFuseVoltage 0x9231      //Settings:Fuse:Min Blow Fuse Voltage			
#define TIBQ769x2_FuseBlowTimeout 0x9233      //Settings:Fuse:Fuse Blow Timeout			
#define TIBQ769x2_PowerConfig 0x9234      //Settings:Configuration:Power Config			
#define TIBQ769x2_REG12Config 0x9236      //Settings:Configuration:REG12 Config			
#define TIBQ769x2_REG0Config 0x9237      //Settings:Configuration:REG0 Config			
#define TIBQ769x2_HWDRegulatorOptions 0x9238      //Settings:Configuration:HWD Regulator Options			
#define TIBQ769x2_CommType 0x9239      //Settings:Configuration:Comm Type			
#define TIBQ769x2_I2CAddress 0x923A      //Settings:Configuration:I2C Address			
#define TIBQ769x2_SPIConfiguration 0x923C      //Settings:Configuration:SPI Configuration			
#define TIBQ769x2_CommIdleTime 0x923D      //Settings:Configuration:Comm Idle Time			
#define TIBQ769x2_CFETOFFPinConfig 0x92FA      //Settings:Configuration:CFETOFF Pin Config			
#define TIBQ769x2_DFETOFFPinConfig 0x92FB      //Settings:Configuration:DFETOFF Pin Config			
#define TIBQ769x2_ALERTPinConfig 0x92FC      //Settings:Configuration:ALERT Pin Config			
#define TIBQ769x2_TS1Config 0x92FD      //Settings:Configuration:TS1 Config			
#define TIBQ769x2_TS2Config 0x92FE      //Settings:Configuration:TS2 Config			
#define TIBQ769x2_TS3Config 0x92FF      //Settings:Configuration:TS3 Config			
#define TIBQ769x2_HDQPinConfig 0x9300      //Settings:Configuration:HDQ Pin Config			
#define TIBQ769x2_DCHGPinConfig 0x9301      //Settings:Configuration:DCHG Pin Config			
#define TIBQ769x2_DDSGPinConfig 0x9302      //Settings:Configuration:DDSG Pin Config			
#define TIBQ769x2_DAConfiguration 0x9303      //Settings:Configuration:DA Configuration			
#define TIBQ769x2_VCellMode 0x9304      //Settings:Configuration:Vcell Mode			
#define TIBQ769x2_CC3Samples 0x9307      //Settings:Configuration:CC3 Samples			
#define TIBQ769x2_ProtectionConfiguration 0x925F      //Settings:Protection:Protection Configuration			
#define TIBQ769x2_EnabledProtectionsA 0x9261      //Settings:Protection:Enabled Protections A			
#define TIBQ769x2_EnabledProtectionsB 0x9262      //Settings:Protection:Enabled Protections B			
#define TIBQ769x2_EnabledProtectionsC 0x9263      //Settings:Protection:Enabled Protections C			
#define TIBQ769x2_CHGFETProtectionsA 0x9265      //Settings:Protection:CHG FET Protections A			
#define TIBQ769x2_CHGFETProtectionsB 0x9266      //Settings:Protection:CHG FET Protections B			
#define TIBQ769x2_CHGFETProtectionsC 0x9267      //Settings:Protection:CHG FET Protections C			
#define TIBQ769x2_DSGFETProtectionsA 0x9269      //Settings:Protection:DSG FET Protections A			
#define TIBQ769x2_DSGFETProtectionsB 0x926A      //Settings:Protection:DSG FET Protections B			
#define TIBQ769x2_DSGFETProtectionsC 0x926B      //Settings:Protection:DSG FET Protections C			
#define TIBQ769x2_BodyDiodeThreshold 0x9273      //Settings:Protection:Body Diode Threshold			
#define TIBQ769x2_DefaultAlarmMask 0x926D      //Settings:Alarm:Default Alarm Mask			
#define TIBQ769x2_SFAlertMaskA 0x926F      //Settings:Alarm:SF Alert Mask A			
#define TIBQ769x2_SFAlertMaskB 0x9270      //Settings:Alarm:SF Alert Mask B			
#define TIBQ769x2_SFAlertMaskC 0x9271      //Settings:Alarm:SF Alert Mask C			
#define TIBQ769x2_PFAlertMaskA 0x92C4      //Settings:Alarm:PF Alert Mask A			
#define TIBQ769x2_PFAlertMaskB 0x92C5      //Settings:Alarm:PF Alert Mask B			
#define TIBQ769x2_PFAlertMaskC 0x92C6      //Settings:Alarm:PF Alert Mask C			
#define TIBQ769x2_PFAlertMaskD 0x92C7      //Settings:Alarm:PF Alert Mask D			
#define TIBQ769x2_EnabledPFA 0x92C0      //Settings:Permanent Failure:Enabled PF A			
#define TIBQ769x2_EnabledPFB 0x92C1      //Settings:Permanent Failure:Enabled PF B			
#define TIBQ769x2_EnabledPFC 0x92C2      //Settings:Permanent Failure:Enabled PF C			
#define TIBQ769x2_EnabledPFD 0x92C3      //Settings:Permanent Failure:Enabled PF D			
#define TIBQ769x2_FETOptions 0x9308      //Settings:FET:FET Options			
#define TIBQ769x2_ChgPumpControl 0x9309      //Settings:FET:Chg Pump Control			
#define TIBQ769x2_PrechargeStartVoltage 0x930A      //Settings:FET:Precharge Start Voltage			
#define TIBQ769x2_PrechargeStopVoltage 0x930C      //Settings:FET:Precharge Stop Voltage			
#define TIBQ769x2_PredischargeTimeout 0x930E      //Settings:FET:Predischarge Timeout			
#define TIBQ769x2_PredischargeStopDelta 0x930F      //Settings:FET:Predischarge Stop Delta			
#define TIBQ769x2_DsgCurrentThreshold 0x9310      //Settings:Current Thresholds:Dsg Current Threshold			
#define TIBQ769x2_ChgCurrentThreshold 0x9312      //Settings:Current Thresholds:Chg Current Threshold			
#define TIBQ769x2_CheckTime 0x9314      //Settings:Cell Open-Wire:Check Time			
#define TIBQ769x2_Cell1Interconnect 0x9315      //Settings:Interconnect Resistances:Cell 1 Interconnect			
#define TIBQ769x2_Cell2Interconnect 0x9317      //Settings:Interconnect Resistances:Cell 2 Interconnect			
#define TIBQ769x2_Cell3Interconnect 0x9319      //Settings:Interconnect Resistances:Cell 3 Interconnect			
#define TIBQ769x2_Cell4Interconnect 0x931B      //Settings:Interconnect Resistances:Cell 4 Interconnect			
#define TIBQ769x2_Cell5Interconnect 0x931D      //Settings:Interconnect Resistances:Cell 5 Interconnect			
#define TIBQ769x2_Cell6Interconnect 0x931F      //Settings:Interconnect Resistances:Cell 6 Interconnect			
#define TIBQ769x2_Cell7Interconnect 0x9321      //Settings:Interconnect Resistances:Cell 7 Interconnect			
#define TIBQ769x2_Cell8Interconnect 0x9323      //Settings:Interconnect Resistances:Cell 8 Interconnect			
#define TIBQ769x2_Cell9Interconnect 0x9325      //Settings:Interconnect Resistances:Cell 9 Interconnect			
#define TIBQ769x2_Cell10Interconnect 0x9327      //Settings:Interconnect Resistances:Cell 10 Interconnect			
#define TIBQ769x2_Cell11Interconnect 0x9329      //Settings:Interconnect Resistances:Cell 11 Interconnect			
#define TIBQ769x2_Cell12Interconnect 0x932B      //Settings:Interconnect Resistances:Cell 12 Interconnect			
#define TIBQ769x2_Cell13Interconnect 0x932D      //Settings:Interconnect Resistances:Cell 13 Interconnect			
#define TIBQ769x2_Cell14Interconnect 0x932F      //Settings:Interconnect Resistances:Cell 14 Interconnect			
#define TIBQ769x2_Cell15Interconnect 0x9331      //Settings:Interconnect Resistances:Cell 15 Interconnect			
#define TIBQ769x2_Cell16Interconnect 0x9333      //Settings:Interconnect Resistances:Cell 16 Interconnect			
#define TIBQ769x2_MfgStatusInit 0x9343      //Settings:Manufacturing:Mfg Status Init			
#define TIBQ769x2_BalancingConfiguration 0x9335      //Settings:Cell Balancing Config:Balancing Configuration			
#define TIBQ769x2_MinCellTemp 0x9336      //Settings:Cell Balancing Config:Min Cell Temp			
#define TIBQ769x2_MaxCellTemp 0x9337      //Settings:Cell Balancing Config:Max Cell Temp			
#define TIBQ769x2_MaxInternalTemp 0x9338      //Settings:Cell Balancing Config:Max Internal Temp			
#define TIBQ769x2_CellBalanceInterval 0x9339      //Settings:Cell Balancing Config:Cell Balance Interval			
#define TIBQ769x2_CellBalanceMaxCells 0x933A      //Settings:Cell Balancing Config:Cell Balance Max Cells			
#define TIBQ769x2_CellBalanceMinCellVCharge 0x933B      //Settings:Cell Balancing Config:Cell Balance Min Cell V (Charge)			
#define TIBQ769x2_CellBalanceMinDeltaCharge 0x933D      //Settings:Cell Balancing Config:Cell Balance Min Delta (Charge)			
#define TIBQ769x2_CellBalanceStopDeltaCharge 0x933E      //Settings:Cell Balancing Config:Cell Balance Stop Delta (Charge)			
#define TIBQ769x2_CellBalanceMinCellVRelax 0x933F      //Settings:Cell Balancing Config:Cell Balance Min Cell V (Relax)			
#define TIBQ769x2_CellBalanceMinDeltaRelax 0x9341      //Settings:Cell Balancing Config:Cell Balance Min Delta (Relax)			
#define TIBQ769x2_CellBalanceStopDeltaRelax 0x9342      //Settings:Cell Balancing Config:Cell Balance Stop Delta (Relax)			
#define TIBQ769x2_ShutdownCellVoltage 0x923F      //Power:Shutdown:Shutdown Cell Voltage			
#define TIBQ769x2_ShutdownStackVoltage 0x9241      //Power:Shutdown:Shutdown Stack Voltage			
#define TIBQ769x2_LowVShutdownDelay 0x9243      //Power:Shutdown:Low V Shutdown Delay			
#define TIBQ769x2_ShutdownTemperature 0x9244      //Power:Shutdown:Shutdown Temperature			
#define TIBQ769x2_ShutdownTemperatureDelay 0x9245      //Power:Shutdown:Shutdown Temperature Delay			
#define TIBQ769x2_FETOffDelay 0x9252      //Power:Shutdown:FET Off Delay			
#define TIBQ769x2_ShutdownCommandDelay 0x9253      //Power:Shutdown:Shutdown Command Delay			
#define TIBQ769x2_AutoShutdownTime 0x9254      //Power:Shutdown:Auto Shutdown Time			
#define TIBQ769x2_RAMFailShutdownTime 0x9255      //Power:Shutdown:RAM Fail Shutdown Time			
#define TIBQ769x2_SleepCurrent 0x9248      //Power:Sleep:Sleep Current			
#define TIBQ769x2_VoltageTime 0x924A      //Power:Sleep:Voltage Time			
#define TIBQ769x2_WakeComparatorCurrent 0x924B      //Power:Sleep:Wake Comparator Current			
#define TIBQ769x2_SleepHysteresisTime 0x924D      //Power:Sleep:Sleep Hysteresis Time			
#define TIBQ769x2_SleepChargerVoltageThreshold 0x924E      //Power:Sleep:Sleep Charger Voltage Threshold			
#define TIBQ769x2_SleepChargerPACKTOSDelta 0x9250      //Power:Sleep:Sleep Charger PACK-TOS Delta			
#define TIBQ769x2_ConfigRAMSignature 0x91E0      //System Data:Integrity:Config RAM Signature			
#define TIBQ769x2_CUVThreshold 0x9275      //Protections:CUV:Threshold			
#define TIBQ769x2_CUVDelay 0x9276      //Protections:CUV:Delay			
#define TIBQ769x2_CUVRecoveryHysteresis 0x927B      //Protections:CUV:Recovery Hysteresis			
#define TIBQ769x2_COVThreshold 0x9278      //Protections:COV:Threshold			
#define TIBQ769x2_COVDelay 0x9279      //Protections:COV:Delay			
#define TIBQ769x2_COVRecoveryHysteresis 0x927C      //Protections:COV:Recovery Hysteresis			
#define TIBQ769x2_COVLLatchLimit 0x927D      //Protections:COVL:Latch Limit			
#define TIBQ769x2_COVLCounterDecDelay 0x927E      //Protections:COVL:Counter Dec Delay			
#define TIBQ769x2_COVLRecoveryTime 0x927F      //Protections:COVL:Recovery Time			
#define TIBQ769x2_OCCThreshold 0x9280      //Protections:OCC:Threshold			
#define TIBQ769x2_OCCDelay 0x9281      //Protections:OCC:Delay			
#define TIBQ769x2_OCCRecoveryThreshold 0x9288      //Protections:OCC:Recovery Threshold			
#define TIBQ769x2_OCCPACKTOSDelta 0x92B0      //Protections:OCC:PACK-TOS Delta			
#define TIBQ769x2_OCD1Threshold 0x9282      //Protections:OCD1:Threshold			
#define TIBQ769x2_OCD1Delay 0x9283      //Protections:OCD1:Delay			
#define TIBQ769x2_OCD2Threshold 0x9284      //Protections:OCD2:Threshold			
#define TIBQ769x2_OCD2Delay 0x9285      //Protections:OCD2:Delay			
#define TIBQ769x2_SCDThreshold 0x9286      //Protections:SCD:Threshold			
#define TIBQ769x2_SCDDelay 0x9287      //Protections:SCD:Delay			
#define TIBQ769x2_SCDRecoveryTime 0x9294      //Protections:SCD:Recovery Time			
#define TIBQ769x2_OCD3Threshold 0x928A      //Protections:OCD3:Threshold			
#define TIBQ769x2_OCD3Delay 0x928C      //Protections:OCD3:Delay			
#define TIBQ769x2_OCDRecoveryThreshold 0x928D      //Protections:OCD:Recovery Threshold			
#define TIBQ769x2_OCDLLatchLimit 0x928F      //Protections:OCDL:Latch Limit			
#define TIBQ769x2_OCDLCounterDecDelay 0x9290      //Protections:OCDL:Counter Dec Delay			
#define TIBQ769x2_OCDLRecoveryTime 0x9291      //Protections:OCDL:Recovery Time			
#define TIBQ769x2_OCDLRecoveryThreshold 0x9292      //Protections:OCDL:Recovery Threshold			
#define TIBQ769x2_SCDLLatchLimit 0x9295      //Protections:SCDL:Latch Limit			
#define TIBQ769x2_SCDLCounterDecDelay 0x9296      //Protections:SCDL:Counter Dec Delay			
#define TIBQ769x2_SCDLRecoveryTime 0x9297      //Protections:SCDL:Recovery Time			
#define TIBQ769x2_SCDLRecoveryThreshold 0x9298      //Protections:SCDL:Recovery Threshold			
#define TIBQ769x2_OTCThreshold 0x929A      //Protections:OTC:Threshold			
#define TIBQ769x2_OTCDelay 0x920B      //Protections:OTC:Delay			
#define TIBQ769x2_OTCRecovery 0x929C      //Protections:OTC:Recovery			
#define TIBQ769x2_OTDThreshold 0x929D      //Protections:OTD:Threshold			
#define TIBQ769x2_OTDDelay 0x929E      //Protections:OTD:Delay			
#define TIBQ769x2_OTDRecovery 0x929F      //Protections:OTD:Recovery			
#define TIBQ769x2_OTFThreshold 0x92A0      //Protections:OTF:Threshold			
#define TIBQ769x2_OTFDelay 0x92A1      //Protections:OTF:Delay			
#define TIBQ769x2_OTFRecovery 0x92A2      //Protections:OTF:Recovery			
#define TIBQ769x2_OTINTThreshold 0x92A3      //Protections:OTINT:Threshold			
#define TIBQ769x2_OTINTDelay 0x92A4      //Protections:OTINT:Delay			
#define TIBQ769x2_OTINTRecovery 0x92A5      //Protections:OTINT:Recovery			
#define TIBQ769x2_UTCThreshold 0x92A6      //Protections:UTC:Threshold			
#define TIBQ769x2_UTCDelay 0x92A7      //Protections:UTC:Delay			
#define TIBQ769x2_UTCRecovery 0x92A8      //Protections:UTC:Recovery			
#define TIBQ769x2_UTDThreshold 0x92A9      //Protections:UTD:Threshold			
#define TIBQ769x2_UTDDelay 0x92AA      //Protections:UTD:Delay			
#define TIBQ769x2_UTDRecovery 0x92AB      //Protections:UTD:Recovery			
#define TIBQ769x2_UTINTThreshold 0x92AC      //Protections:UTINT:Threshold			
#define TIBQ769x2_UTINTDelay 0x92AD      //Protections:UTINT:Delay			
#define TIBQ769x2_UTINTRecovery 0x92AE      //Protections:UTINT:Recovery			
#define TIBQ769x2_ProtectionsRecoveryTime 0x92AF      //Protections:Recovery:Time			
#define TIBQ769x2_HWDDelay 0x92B2      //Protections:HWD:Delay			
#define TIBQ769x2_LoadDetectActiveTime 0x92B4      //Protections:Load Detect:Active Time			
#define TIBQ769x2_LoadDetectRetryDelay 0x92B5      //Protections:Load Detect:Retry Delay			
#define TIBQ769x2_LoadDetectTimeout 0x92B6      //Protections:Load Detect:Timeout			
#define TIBQ769x2_PTOChargeThreshold 0x92BA      //Protections:PTO:Charge Threshold			
#define TIBQ769x2_PTODelay 0x92BC      //Protections:PTO:Delay			
#define TIBQ769x2_PTOReset 0x92BE      //Protections:PTO:Reset			
#define TIBQ769x2_CUDEPThreshold 0x92C8      //Permanent Fail:CUDEP:Threshold			
#define TIBQ769x2_CUDEPDelay 0x92CA      //Permanent Fail:CUDEP:Delay			
#define TIBQ769x2_SUVThreshold 0x92CB      //Permanent Fail:SUV:Threshold			
#define TIBQ769x2_SUVDelay 0x92CD      //Permanent Fail:SUV:Delay			
#define TIBQ769x2_SOVThreshold 0x92CE      //Permanent Fail:SOV:Threshold			
#define TIBQ769x2_SOVDelay 0x92D0      //Permanent Fail:SOV:Delay			
#define TIBQ769x2_TOSSThreshold 0x92D1      //Permanent Fail:TOS:Threshold			
#define TIBQ769x2_TOSSDelay 0x92D3      //Permanent Fail:TOS:Delay			
#define TIBQ769x2_SOCCThreshold 0x92D4      //Permanent Fail:SOCC:Threshold			
#define TIBQ769x2_SOCCDelay 0x92D6      //Permanent Fail:SOCC:Delay			
#define TIBQ769x2_SOCDThreshold 0x92D7      //Permanent Fail:SOCD:Threshold			
#define TIBQ769x2_SOCDDelay 0x92D9      //Permanent Fail:SOCD:Delay			
#define TIBQ769x2_SOTThreshold 0x92DA      //Permanent Fail:SOT:Threshold			
#define TIBQ769x2_SOTDelay 0x92DB      //Permanent Fail:SOT:Delay			
#define TIBQ769x2_SOTFThreshold 0x92DC      //Permanent Fail:SOTF:Threshold			
#define TIBQ769x2_SOTFDelay 0x92DD      //Permanent Fail:SOTF:Delay			
#define TIBQ769x2_VIMRCheckVoltage 0x92DE      //Permanent Fail:VIMR:Check Voltage			
#define TIBQ769x2_VIMRMaxRelaxCurrent 0x92E0      //Permanent Fail:VIMR:Max Relax Current			
#define TIBQ769x2_VIMRThreshold 0x92E2      //Permanent Fail:VIMR:Threshold			
#define TIBQ769x2_VIMRDelay 0x92E4      //Permanent Fail:VIMR:Delay			
#define TIBQ769x2_VIMRRelaxMinDuration 0x92E5      //Permanent Fail:VIMR:Relax Min Duration			
#define TIBQ769x2_VIMACheckVoltage 0x92E7      //Permanent Fail:VIMA:Check Voltage			
#define TIBQ769x2_VIMAMinActiveCurrent 0x92E9      //Permanent Fail:VIMA:Min Active Current			
#define TIBQ769x2_VIMAThreshold 0x92EB      //Permanent Fail:VIMA:Threshold			
#define TIBQ769x2_VIMADelay 0x92ED      //Permanent Fail:VIMA:Delay			
#define TIBQ769x2_CFETFOFFThreshold 0x92EE      //Permanent Fail:CFETF:OFF Threshold			
#define TIBQ769x2_CFETFOFFDelay 0x92F0      //Permanent Fail:CFETF:OFF Delay			
#define TIBQ769x2_DFETFOFFThreshold 0x92F1      //Permanent Fail:DFETF:OFF Threshold			
#define TIBQ769x2_DFETFOFFDelay 0x92F3      //Permanent Fail:DFETF:OFF Delay			
#define TIBQ769x2_VSSFFailThreshold 0x92F4      //Permanent Fail:VSSF:Fail Threshold			
#define TIBQ769x2_VSSFDelay 0x92F6      //Permanent Fail:VSSF:Delay			
#define TIBQ769x2_PF2LVLDelay 0x92F7      //Permanent Fail:2LVL:Delay			
#define TIBQ769x2_LFOFDelay 0x92F8      //Permanent Fail:LFOF:Delay			
#define TIBQ769x2_HWMXDelay 0x92F9      //Permanent Fail:HWMX:Delay			
#define TIBQ769x2_SecuritySettings 0x9256      //Security:Settings:Security Settings			
#define TIBQ769x2_UnsealKeyStep1 0x9257      //Security:Keys:Unseal Key Step 1			
#define TIBQ769x2_UnsealKeyStep2 0x9259      //Security:Keys:Unseal Key Step 2			
#define TIBQ769x2_FullAccessKeyStep1 0x925B      //Security:Keys:Full Access Key Step 1			
#define TIBQ769x2_FullAccessKeyStep2 0x925D      //Security:Keys:Full Access Key Step 2			

//Direct Commands 
#define TIBQ769x2_ControlStatus 0x00
#define TIBQ769x2_SafetyAlertA 0x02
#define TIBQ769x2_SafetyStatusA 0x03
#define TIBQ769x2_SafetyAlertB 0x04
#define TIBQ769x2_SafetyStatusB 0x05
#define TIBQ769x2_SafetyAlertC 0x06
#define TIBQ769x2_SafetyStatusC 0x07
#define TIBQ769x2_PFAlertA 0x0A
#define TIBQ769x2_PFStatusA 0x0B
#define TIBQ769x2_PFAlertB 0x0C
#define TIBQ769x2_PFStatusB 0x0D
#define TIBQ769x2_PFAlertC 0x0E
#define TIBQ769x2_PFStatusC 0x0F
#define TIBQ769x2_PFAlertD 0x10
#define TIBQ769x2_PFStatusD 0x11
#define TIBQ769x2_BatteryStatus 0x12
#define TIBQ769x2_Cell1Voltage 0x14
#define TIBQ769x2_Cell2Voltage 0x16
#define TIBQ769x2_Cell3Voltage 0x18
#define TIBQ769x2_Cell4Voltage 0x1A
#define TIBQ769x2_Cell5Voltage 0x1C
#define TIBQ769x2_Cell6Voltage 0x1E
#define TIBQ769x2_Cell7Voltage 0x20
#define TIBQ769x2_Cell8Voltage 0x22
#define TIBQ769x2_Cell9Voltage 0x24
#define TIBQ769x2_Cell10Voltage 0x26
#define TIBQ769x2_Cell11Voltage 0x28
#define TIBQ769x2_Cell12Voltage 0x2A
#define TIBQ769x2_Cell13Voltage 0x2C
#define TIBQ769x2_Cell14Voltage 0x2E
#define TIBQ769x2_Cell15Voltage 0x30
#define TIBQ769x2_Cell16Voltage 0x32
#define TIBQ769x2_StackVoltage 0x34
#define TIBQ769x2_PACKPinVoltage 0x36
#define TIBQ769x2_LDPinVoltage 0x38
#define TIBQ769x2_CC2Current 0x3A
#define TIBQ769x2_AlarmStatus 0x62
#define TIBQ769x2_AlarmRawStatus 0x64
#define TIBQ769x2_AlarmEnable 0x66
#define TIBQ769x2_IntTemperature 0x68
#define TIBQ769x2_CFETOFFTemperature 0x6A
#define TIBQ769x2_DFETOFFTemperature 0x6C
#define TIBQ769x2_ALERTTemperature 0x6E
#define TIBQ769x2_TS1Temperature 0x70
#define TIBQ769x2_TS2Temperature 0x72
#define TIBQ769x2_TS3Temperature 0x74
#define TIBQ769x2_HDQTemperature 0x76
#define TIBQ769x2_DCHGTemperature 0x78
#define TIBQ769x2_DDSGTemperature 0x7A
#define TIBQ769x2_FETStatus 0x7F

//Subcommands 
#define TIBQ769x2_DEVICE_NUMBER 0x0001
#define TIBQ769x2_FW_VERSION 0x0002
#define TIBQ769x2_HW_VERSION 0x0003
#define TIBQ769x2_IROM_SIG 0x0004
#define TIBQ769x2_STATIC_CFG_SIG 0x0005
#define TIBQ769x2_PREV_MACWRITE 0x0007
#define TIBQ769x2_DROM_SIG 0x0009
#define TIBQ769x2_SECURITY_KEYS 0x0035
#define TIBQ769x2_SAVED_PF_STATUS 0x0053
#define TIBQ769x2_MANUFACTURINGSTATUS 0x0057
#define TIBQ769x2_MANU_DATA 0x0070
#define TIBQ769x2_DASTATUS1 0x0071
#define TIBQ769x2_DASTATUS2 0x0072
#define TIBQ769x2_DASTATUS3 0x0073
#define TIBQ769x2_DASTATUS4 0x0074
#define TIBQ769x2_DASTATUS5 0x0075
#define TIBQ769x2_DASTATUS6 0x0076
#define TIBQ769x2_DASTATUS7 0x0077
#define TIBQ769x2_CUV_SNAPSHOT 0x0080
#define TIBQ769x2_COV_SNAPSHOT 0X0081
#define TIBQ769x2_CB_ACTIVE_CELLS 0x0083
#define TIBQ769x2_CB_SET_LVL 0x0084
#define TIBQ769x2_CBSTATUS1 0x0085
#define TIBQ769x2_CBSTATUS2 0x0086
#define TIBQ769x2_CBSTATUS3 0x0087
#define TIBQ769x2_FET_CONTROL 0x0097
#define TIBQ769x2_REG12_CONTROL 0x0098
#define TIBQ769x2_OTP_WR_CHECK 0x00A0
#define TIBQ769x2_OTP_WRITE 0x00A1
#define TIBQ769x2_READ_CAL1 0xF081
#define TIBQ769x2_CAL_CUV 0xF090
#define TIBQ769x2_CAL_COV 0xF091

// Command Only Subcommands 
#define TIBQ769x2_EXIT_DEEPSLEEP 0x000E
#define TIBQ769x2_DEEPSLEEP 0x000F
#define TIBQ769x2_SHUTDOWN 0x0010
#define TIBQ769x2_BQ769x2_RESET 0x0012 //"RESET" in documentation
#define TIBQ769x2_PDSGTEST 0x001C
#define TIBQ769x2_FUSE_TOGGLE 0x001D
#define TIBQ769x2_PCHGTEST 0x001E
#define TIBQ769x2_CHGTEST 0x001F
#define TIBQ769x2_DSGTEST 0x0020
#define TIBQ769x2_FET_ENABLE 0x0022
#define TIBQ769x2_PF_ENABLE 0x0024
#define TIBQ769x2_PF_RESET 0x0029
#define TIBQ769x2_SEAL 0x0030
#define TIBQ769x2_RESET_PASSQ 0x0082
#define TIBQ769x2_PTO_RECOVER 0x008A
#define TIBQ769x2_SET_CFGUPDATE 0x0090
#define TIBQ769x2_EXIT_CFGUPDATE 0x0092
#define TIBQ769x2_DSG_PDSG_OFF 0x0093
#define TIBQ769x2_CHG_PCHG_OFF 0x0094
#define TIBQ769x2_ALL_FETS_OFF 0x0095
#define TIBQ769x2_ALL_FETS_ON 0x0096
#define TIBQ769x2_SLEEP_ENABLE 0x0099
#define TIBQ769x2_SLEEP_DISABLE 0x009A
#define TIBQ769x2_OCDL_RECOVER 0x009B
#define TIBQ769x2_SCDL_RECOVER 0x009C
#define TIBQ769x2_LOAD_DETECT_RESTART 0x009D
#define TIBQ769x2_LOAD_DETECT_ON 0x009E
#define TIBQ769x2_LOAD_DETECT_OFF 0x009F
#define TIBQ769x2_CFETOFF_LO 0x2800
#define TIBQ769x2_DFETOFF_LO 0x2801
#define TIBQ769x2_ALERT_LO 0x2802
#define TIBQ769x2_HDQ_LO 0x2806
#define TIBQ769x2_DCHG_LO 0x2807
#define TIBQ769x2_DDSG_LO 0x2808
#define TIBQ769x2_CFETOFF_HI 0x2810
#define TIBQ769x2_DFETOFF_HI 0x2811
#define TIBQ769x2_ALERT_HI 0x2812
#define TIBQ769x2_HDQ_HI 0x2816
#define TIBQ769x2_DCHG_HI 0x2817
#define TIBQ769x2_DDSG_HI 0x2818
#define TIBQ769x2_PF_FORCE_A 0x2857
#define TIBQ769x2_PF_FORCE_B 0x29A3
#define TIBQ769x2_SWAP_COMM_MODE 0x29BC
#define TIBQ769x2_SWAP_TO_I2C 0x29E7
#define TIBQ769x2_SWAP_TO_SPI 0x7C35
#define TIBQ769x2_SWAP_TO_HDQ 0x7C40

class AP_BattMonitor_TIBQ76952 : public AP_BattMonitor_Backend
{
public:
    // Subcommand operation types
    enum SubcommandType {
        READ = 0,    // Read operation (R)
        WRITE = 1,   // Write operation with 1 byte data (W)
        WRITE2 = 2   // Write operation with 2 bytes data (W2)
    };

    /// Constructor
    AP_BattMonitor_TIBQ76952(AP_BattMonitor &mon,
                          AP_BattMonitor::BattMonitor_State &mon_state,
                          AP_BattMonitor_Params &params);

    bool has_cell_voltages() const override { return true; }  // TODO: BQ76952 can read individual cells
    bool has_temperature() const override { return false; }   // TODO: BQ76952 has temperature sensors
    bool has_current() const override { return false; }       // For now, only voltage implemented
    bool get_cycle_count(uint16_t &cycles) const override { return false; }

    void init(void) override;
    void read() override;

    static const struct AP_Param::GroupInfo var_info[];

protected:
    // AP_HAL::Device *dev;
    AP_HAL::OwnPtr<AP_HAL::Device> dev;

    uint8_t calc_crc8(const uint8_t *data, uint8_t len) const;
    bool read_register(uint8_t reg_addr, uint8_t *reg_data, uint8_t count) const;
    bool write_register(uint8_t reg_addr, const uint8_t *reg_data, uint8_t count) const;
    void set_register(uint16_t reg_addr, uint32_t reg_data, uint8_t len) const;
    bool sub_command(uint16_t command) const;
    bool sub_command(uint16_t command, uint16_t data, SubcommandType type, uint8_t *rx_data = nullptr, uint8_t rx_len = 32) const;
    bool direct_command(uint16_t command, uint16_t data, SubcommandType type, uint8_t *rx_data = nullptr, uint8_t rx_len = 2) const;
    uint8_t checksum(const uint8_t *data, uint8_t len) const;
    uint32_t read_device_number(void) const;
    uint32_t read_fw_version(void) const;
    uint32_t read_hv_version(void) const;
    uint16_t read_voltage(uint8_t command) const;
    uint16_t read_alarm_status(void) const;
    void alarm_status_update(void);
    uint16_t read_permanent_fail_status_A(void) const;
    uint16_t read_permanent_fail_status_B(void) const;
    uint16_t read_permanent_fail_status_C(void) const;
    uint16_t read_permanent_fail_status_D(void) const;
    void permanent_fail_status_update(void) const;
    uint16_t read_battery_status(void) const;
    void battery_status_update(void) const;

    // periodic timer callback
    void timer();

    // configure device
    bool configure() const;

    bool configured;
    bool callback_registered;
    uint32_t failed_reads;
    uint32_t last_configure_ms;

    struct {
        uint16_t count;     // number of readings, values below should be divided by this number
        float voltage;      // battery voltage in volts
        float current;      // battery current in amps
        float temp;         // battery temperature in degrees Celsius
        float health_pct;   // battery health percentage
        HAL_Semaphore sem;
    } accumulate;

    AP_Float max_voltage;  // Maximum pack voltage parameter
};

#endif // AP_BATTERY_TIBQ76952_ENABLED
