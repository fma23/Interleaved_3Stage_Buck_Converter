#include "DSP28x_Project.h"     // DSP28x Headerfile
#include "F2806x_Cla_defines.h"
#include "InterLvbuckHardware.h"



void Comparator_Config(void)
{

    /* Configure Comparator 1: link cmp1 to pwm3*/
    EALLOW;
    SysCtrlRegs.PCLKCR3.bit.COMP1ENCLK = 1;
    Comp1Regs.COMPCTL.bit.COMPDACEN = 1;             // Power up Comparator locally
    Comp1Regs.COMPCTL.bit.COMPSOURCE = 0;            // Connect the inverting input to internal DAC
    Comp1Regs.DACCTL.bit.DACSOURCE = 1;              // 0 - DACVAL; 1 - Internal ramp for slope compensation
    Comp1Regs.COMPCTL.bit.QUALSEL = 5;               // Comparator output must be active for 4 consecutive clocks before resetting the RAMP
    Comp1Regs.DACCTL.bit.RAMPSOURCE = 2;             // 0 - PMW1; 1 - PWM2, 2 -PWM3  4-PWM5 ...so on
    Comp1Regs.RAMPDECVAL_SHDW = slopeval;
    EPwm1Regs.HRPCTL.bit.PWMSYNCSEL = 1;             // PWM SYNC generated at CTR = ZRO for synchronizing internal ramp
    Comp1Regs.COMPCTL.bit.CMPINV = 0;                // Comparator Output passed
    EDIS;

    /* Configure Comparator 2: link cmp2 to pwm4*/
    EALLOW;
    SysCtrlRegs.PCLKCR3.bit.COMP2ENCLK = 1;
    Comp2Regs.COMPCTL.bit.COMPDACEN = 1;             // Power up Comparator locally
    Comp2Regs.COMPCTL.bit.COMPSOURCE = 0;            // Connect the inverting input to internal DAC
    Comp2Regs.DACCTL.bit.DACSOURCE = 1;              // 0 - DACVAL; 1 - Internal ramp for slope compensation
    Comp2Regs.COMPCTL.bit.QUALSEL = 5;               // Comparator output must be active for 4 consecutive clocks before resetting the RAMP
    Comp2Regs.DACCTL.bit.RAMPSOURCE = 3;             // 0 - PMW1; 1 - PWM2, 3- PWM4 ...so on link cmp2 to pwm4
    Comp2Regs.RAMPDECVAL_SHDW = slopeval;
    EPwm2Regs.HRPCTL.bit.PWMSYNCSEL = 1;             // PWM SYNC generated at CTR = ZRO for synchronizing internal ramp
    Comp2Regs.COMPCTL.bit.CMPINV = 0;                // Comparator Output passed
    EDIS;

    /* Configure Comparator 3: link cmp1 to pwm5 */
    EALLOW;
    SysCtrlRegs.PCLKCR3.bit.COMP3ENCLK = 1;
    Comp3Regs.COMPCTL.bit.COMPDACEN = 1;             // Power up Comparator locally
    Comp3Regs.COMPCTL.bit.COMPSOURCE = 0;            // Connect the inverting input to internal DAC
    Comp3Regs.DACCTL.bit.DACSOURCE = 1;              // 0 - DACVAL; 1 - Internal ramp for slope compensation
    Comp3Regs.COMPCTL.bit.QUALSEL = 5;               // Comparator output must be active for 4 consecutive clocks before resetting the RAMP
    Comp3Regs.DACCTL.bit.RAMPSOURCE = 4;             // 0 - PMW1; 1 - PWM2, 5- PWM6 ...so on
    Comp3Regs.RAMPDECVAL_SHDW = slopeval;
    EPwm3Regs.HRPCTL.bit.PWMSYNCSEL = 1;             // PWM SYNC generated at CTR = ZRO for synchronizing internal ramp
    Comp3Regs.COMPCTL.bit.CMPINV = 0;                // Comparator Output passed
    EDIS;
}


void ADCs_Config(void)
{
    InitAdc();
    InitAdcAio();


    EALLOW;
    AdcRegs.INTSEL1N2.bit.INT1E = 1;                         /* Enabled ADCINT1 */
    AdcRegs.INTSEL1N2.bit.INT1CONT = 0;                      /* Disable ADCINT1 Continuous mode */
    AdcRegs.INTSEL1N2.bit.INT1SEL = 0;                       /* setup EOC0 to trigger ADCINT1 to fire */
    AdcRegs.ADCSOC0CTL.bit.CHSEL = 0x3;                      /* Set SOC0 channel select to ADCINB0 */
    AdcRegs.ADCSOC0CTL.bit.TRIGSEL = ADCTRIG_EPWM1_SOCB;     /* Set SOC0 start trigger on EPWM1B, due to round-robin SOC0 converts first then SOC1 */
    AdcRegs.ADCSOC0CTL.bit.ACQPS = 6;                        /* Set SOC0 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1) */
    AdcRegs.ADCCTL1.bit.INTPULSEPOS = 0;                     /* Configure early interrupts */
    EDIS;

    EALLOW;
    AdcRegs.INTSEL1N2.bit.INT2E = 1;                           /* Enabled ADCINT2 */
    AdcRegs.INTSEL1N2.bit.INT2CONT = 0;                        /* Disable ADCINT2 Continuous mode */
    AdcRegs.INTSEL1N2.bit.INT2SEL = 1;                         /* setup EOC1 to trigger ADCINT2 to fire */
    AdcRegs.ADCSOC1CTL.bit.CHSEL = 0x3;                        /* Set SOC1 channel select to ADCINB0 */
    AdcRegs.ADCSOC1CTL.bit.TRIGSEL = ADCTRIG_EPWM2_SOCB;       /* Set SOC1 start trigger on EPWM2A, due to round-robin SOC1 converts first then SOC1 */
    AdcRegs.ADCSOC1CTL.bit.ACQPS = 6;                          /* Set SOC0 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1) */
    AdcRegs.ADCCTL1.bit.INTPULSEPOS = 0;                       /* Configure early interrupts */
    EDIS;


    EALLOW;
    AdcRegs.INTSEL3N4.bit.INT3E = 1;                            /* Enabled ADCINT3 */
    AdcRegs.INTSEL3N4.bit.INT3CONT = 0;                         /* Disable ADCINT3 Continuous mode */
    AdcRegs.INTSEL3N4.bit.INT3SEL = 2;                          /* setup EOC2 to trigger ADCINT3 to fire */
    AdcRegs.ADCSOC2CTL.bit.CHSEL = 0x3;                         /* Set SOC2 channel select to ADCINB0*/
    AdcRegs.ADCSOC2CTL.bit.TRIGSEL= ADCTRIG_EPWM6_SOCB;         /* Set SOC2 start trigger on EPWM6B, due to round-robin SOC2 converts first then SOC2 */
    AdcRegs.ADCSOC2CTL.bit.ACQPS = 6;                           /* Set SOC2 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1) */
    AdcRegs.ADCCTL1.bit.INTPULSEPOS = 0;                        /* Configure early interrupts */
    EDIS;

}


void PWMs_Config(void)
{
  /* Configure PWMs */

   EALLOW;
   /* Assumes ePWM1 clock is already enabled in InitSysCtrl(); */
   EPwm1Regs.ETSEL.bit.SOCBEN = 1;        /* Enable SOC on B group */
   EPwm1Regs.ETSEL.bit.SOCBSEL = 6;       /* Select SOC from from CPMB on upcount */
   EPwm1Regs.ETPS.bit.SOCBPRD = 1;        /* Generate pulse on 1st event */

   /* Set period / duty / count mode */
   EPwm1Regs.TBPHS.half.TBPHS = 0;             /* Set Phase register to zero */
   EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;     /* Phase loading disabled */
   EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;    /* Clock ratio to SYSCLKOUT */
   EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;
   EPwm1Regs.TBCTL.bit.PRDLD = TB_SHADOW;

   EPwm1Regs.AQCTLA.bit.ZRO = AQ_SET;          /* Set PWM1A on Zero */
   EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;        /* Clear PWM1A on match on count A up */
   EPwm1Regs.AQCTLB.bit.ZRO = AQ_SET;          /* Set PWM1B on Zero */
   EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR;        /* Clear PWM1B on match on count B up */

   EPwm1Regs.CMPB = DutyCycle_PWMB;                       /* Set compare B value */
   EPwm1Regs.CMPA.half.CMPA = DutyCycle_PWMA;             /* Set compare A value */
   EPwm1Regs.TBPRD = PERIOD;                   /* Set period for ePWM1 */
   EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;  /* Count up and start */
   EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_PRD;
   EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO; // sync "down-stream"
   EDIS;

   /*****************************************************************************/

   /* Assumes ePWM2 clock is already enabled in InitSysCtrl(); */
   EALLOW;
   EPwm2Regs.ETSEL.bit.SOCBEN = 1;        /* Enable SOC on B group */
   EPwm2Regs.ETSEL.bit.SOCBSEL = 6;       /* Select SOC from from CPMB on upcount */
   EPwm2Regs.ETPS.bit.SOCBPRD = 1;        /* Generate pulse on 1st event */

   /* Set period / duty / count mode */
   EPwm2Regs.TBPHS.half.TBPHS = PHASE1;             /* Set Phase register to zero */
   EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE;           /* Phase loading disabled */
   EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;         /* Clock ratio to SYSCLKOUT */
   EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;
   EPwm2Regs.TBCTL.bit.PRDLD = TB_SHADOW;

   EPwm2Regs.AQCTLA.bit.ZRO = AQ_SET;          /* Set PWM2A on Zero */
   EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;        /* Clear PWM2A on match on count A up */
   EPwm2Regs.AQCTLB.bit.ZRO = AQ_SET;          /* Set PWM1B on Zero */
   EPwm2Regs.AQCTLB.bit.CBU = AQ_CLEAR;        /* Clear PWM1B on match on count B up */

   EPwm2Regs.CMPB = DutyCycle_PWMB;                       /* Set compare B value */
   EPwm2Regs.CMPA.half.CMPA = DutyCycle_PWMA;             /* Set compare A value */
   EPwm2Regs.TBPRD = PERIOD;                   /* Set period for ePWM1 */
   EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;  /* Count up and start */

   EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_PRD;
   EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN; // sync "down-stream"
   EDIS;

   /*****************************************************************************/

   /* Assumes ePWM6 clock is already enabled in InitSysCtrl(); */
   EALLOW;
   EPwm6Regs.ETSEL.bit.SOCBEN = 1;        /* Enable SOC on B group */
   EPwm6Regs.ETSEL.bit.SOCBSEL = 6;       /* Select SOC from from CPMB on upcount */
   EPwm6Regs.ETPS.bit.SOCBPRD = 1;        /* Generate pulse on 1st event */

   /*PWM6: Set period / duty / count mode */
   EPwm6Regs.TBPHS.half.TBPHS = PHASE2;             /* Set Phase register to zero */
   EPwm6Regs.TBCTL.bit.PHSEN = TB_ENABLE;           /* Phase loading disabled */
   EPwm6Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;         /* Clock ratio to SYSCLKOUT */
   EPwm6Regs.TBCTL.bit.CLKDIV = TB_DIV1;
   EPwm6Regs.TBCTL.bit.PRDLD = TB_SHADOW;

   EPwm6Regs.AQCTLA.bit.ZRO = AQ_SET;          /* Set PWM1A on Zero */
   EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR;        /* Clear PWM6A on match on count A up */
   EPwm6Regs.AQCTLB.bit.ZRO = AQ_SET;          /* Set PWM6B on Zero */
   EPwm6Regs.AQCTLB.bit.CBU = AQ_CLEAR;        /* Clear PWM6B on match on count B up */

   EPwm6Regs.CMPB = DutyCycle_PWMB;                         /* Set compare B value */
   EPwm6Regs.CMPA.half.CMPA = DutyCycle_PWMA;               /* Set compare A value */
   EPwm6Regs.TBPRD = PERIOD;                                /* Set period for ePWM1 */
   EPwm6Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;               /* Count up and start */
   EPwm6Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm6Regs.CMPCTL.bit.LOADAMODE = CC_CTR_PRD;
   EPwm6Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;              // sync "down-stream"
   EDIS;


   /*****************************************************************************/

   /* Assumes ePWM3 clock is already enabled in InitSysCtrl(); */
   EALLOW;
   /*PWM3: Set period / duty / count mode */
   EPwm3Regs.TBPHS.half.TBPHS = 0;             /* Set Phase register to zero */
   EPwm3Regs.TBCTL.bit.PHSEN = TB_ENABLE;      /* Phase loading disabled */
   EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;    /* Clock ratio to SYSCLKOUT */
   EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;
   EPwm3Regs.TBCTL.bit.PRDLD = TB_SHADOW;

   EPwm3Regs.AQCTLA.bit.ZRO = AQ_SET;          /* Set PWM3A on Zero */
   EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;        /* Clear PWM3A on match on count A up */
   EPwm3Regs.AQCTLB.bit.ZRO = AQ_SET;          /* Set PWM3B on Zero */
   EPwm3Regs.AQCTLB.bit.CBU = AQ_CLEAR;        /* Clear PWM3B on match on count B up */


  // EPwm3Regs.CMPB = DutyCycle_PWMB;                       /* Set compare B value */
   EPwm3Regs.CMPA.half.CMPA =DutyCycle_PWMA;             /* Set compare A value */
   EPwm3Regs.TBPRD = PERIOD;                             /* Set period for ePWM3 */
   EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; //TB_COUNT_UP;            /* Count up and start */
   EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm3Regs.CMPCTL.bit.LOADAMODE =  CC_CTR_PRD;
   EPwm3Regs.CMPCTL.bit.LOADBMODE =  CC_CTR_PRD;

   EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;           // sync "down-stream" slave
   EDIS;



   /* Configure trip zone based on Comparator 3 output and configure blanking */
   EALLOW;
   EPwm3Regs.TZSEL.bit.DCAEVT2 = 1;                      // Disable DCAEVT2 as a CBC trip source for this ePWM module /* Digital compare, output A, cycle by cycle */

   EPwm3Regs.TZCTL.bit.TZA = TZ_FORCE_LO;                /* EPWM3A will go low */
   EPwm3Regs.TZCTL.bit.TZB = TZ_FORCE_HI;                /* EPWM3B goes HI */

   EPwm3Regs.DCTRIPSEL.bit.DCAHCOMPSEL = DC_COMP1OUT;    // DCAH = Comparator 1 output
   EPwm3Regs.TZDCSEL.bit.DCAEVT2 = TZ_DCAH_HI;           /* DCAEVT2 =  DCAL High (will become active as Comparator output goes High) */
                                                         /* DCAEVT2 = DCAH high (will become EPwm1Regs.DCTRIPSEL.bit.DCAHCOMPSEL = DC_COMP2OUT; DCAH = Comparator 2 output*/
   EPwm3Regs.DCACTL.bit.EVT2SRCSEL = DC_EVT2;            /* DCAEVT2 = DCAEVT2 (not filtered) */
   EPwm3Regs.DCACTL.bit.EVT2FRCSYNCSEL = DC_EVT_ASYNC;   /* Take async path */
   EPwm3Regs.DCFCTL.bit.PULSESEL = 0;                    /* Time-base counter equal to period (TBCTR =TBPRD) */
   EPwm3Regs.DCFCTL.bit.BLANKINV = 0;                    /* Blanking window inverted */
   EPwm3Regs.DCFCTL.bit.BLANKE = 1;                      /* Blanking Window Enable */
   EPwm3Regs.DCFCTL.bit.SRCSEL = 0;                      /* Source Is DCAEVT1 Signal */
   EPwm3Regs.DCFWINDOW = 0; //34;                        /* length blanking window: this corresponds to: 34*11.1ns=377.4ns*/
   EPwm3Regs.DCFOFFSET = 5;                              /* 5*11.1=55.5ns blanking window offset & then blanking window begins */


   // Active high complementary PWMs - Set up the deadband
   EPwm3Regs.DBCTL.bit.IN_MODE = DBA_ALL;
   EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
   EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
   EPwm3Regs.DBRED = 15;
   EPwm3Regs.DBFED = 15;
   EDIS;

   /* Assumes ePWM4 clock is already enabled in InitSysCtrl(); */
   EALLOW;
   /*PWM4: Set period / duty / count mode */
   EPwm4Regs.TBPHS.half.TBPHS = PHASE1;             /* Set Phase register to zero */
   EPwm4Regs.TBCTL.bit.PHSEN = TB_ENABLE;           /* Phase loading disabled */
   EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;         /* Clock ratio to SYSCLKOUT */
   EPwm4Regs.TBCTL.bit.CLKDIV = TB_DIV1;
   EPwm4Regs.TBCTL.bit.PRDLD = TB_SHADOW;

   EPwm4Regs.AQCTLA.bit.ZRO = AQ_SET;          /* Set PWM4A on Zero */
   EPwm4Regs.AQCTLA.bit.CAU = AQ_CLEAR;        /* Clear PWM4A on match on count A up */
   EPwm4Regs.AQCTLB.bit.ZRO = AQ_SET;          /* Set PWM4B on Zero */
   EPwm4Regs.AQCTLB.bit.CBU = AQ_CLEAR;        /* Clear PWM4B on match on count B up */

   EPwm4Regs.CMPA.half.CMPA =DutyCycle_PWMA;             /* Set compare A value */
   EPwm4Regs.TBPRD = PERIOD;                             /* Set period for ePWM1 */
   EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;            /* Count up and start */
   EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_CTR_PRD;
   EPwm4Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;          // sync "down-stream" slave
   EDIS;

   /* Configure trip zone based on Comparator 2 output and configure blanking */
   EALLOW;
   EPwm4Regs.TZSEL.bit.DCAEVT2 = 1;                      /* Digital compare, output A, cycle by cycle */
   EPwm4Regs.TZCTL.bit.TZA = TZ_FORCE_LO;                /* EPWM4A will go low */
   EPwm4Regs.TZCTL.bit.TZB = TZ_FORCE_HI;                /* EPWM4B goes HI */

   EPwm4Regs.DCTRIPSEL.bit.DCAHCOMPSEL = DC_COMP2OUT;    // DCAH = Comparator 2 output
   EPwm4Regs.TZDCSEL.bit.DCAEVT2 = TZ_DCAH_HI;           /** DCAEVT2 =  DCAL High (will become active as Comparator output goes High) */
                                                          /* DCAEVT2 = DCAH high (will become EPwm1Regs.DCTRIPSEL.bit.DCAHCOMPSEL = DC_COMP2OUT; DCAH = Comparator 2 output*/
   EPwm4Regs.DCACTL.bit.EVT2SRCSEL = DC_EVT2;            /* DCAEVT2 = DCAEVT2 (not filtered) */
   EPwm4Regs.DCACTL.bit.EVT2FRCSYNCSEL = DC_EVT_ASYNC;   /* Take async path */
   EPwm4Regs.DCFCTL.bit.PULSESEL = 0;                    /* Time-base counter equal to period (TBCTR =TBPRD) */
   EPwm4Regs.DCFCTL.bit.BLANKINV = 0;                    /* Blanking window inverted */
   EPwm4Regs.DCFCTL.bit.BLANKE = 1;                      /* Blanking Window Enable */
   EPwm4Regs.DCFCTL.bit.SRCSEL = 0;                      /* Source Is DCAEVT1 Signal */
   EPwm4Regs.DCFWINDOW = 0; //34;                        /*length blanking window: this corresponds to: 34*11.1ns=377.4ns*/
   EPwm4Regs.DCFOFFSET = 5;                              /* 5*11.1=55.5ns blanking window offset & then blanking window begins */
   EDIS;

   // Active high complementary PWMs - Set up the deadband
   EPwm4Regs.DBCTL.bit.IN_MODE = DBA_ALL;
   EPwm4Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
   EPwm4Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
   EPwm4Regs.DBRED = 15;
   EPwm4Regs.DBFED = 15;

   /*****************************************************************************/

   /* Assumes ePWM6 clock is already enabled in InitSysCtrl(); */
   EALLOW;
   /*PWM5: Set period / duty / count mode */
   EPwm5Regs.TBPHS.half.TBPHS = PHASE2;             /* Set Phase register to zero */
   EPwm5Regs.TBCTL.bit.PHSEN = TB_ENABLE;      /* Phase loading disabled */
   EPwm5Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;    /* Clock ratio to SYSCLKOUT */
   EPwm5Regs.TBCTL.bit.CLKDIV = TB_DIV1;
   EPwm5Regs.TBCTL.bit.PRDLD = TB_SHADOW;

   EPwm5Regs.AQCTLA.bit.ZRO = AQ_SET;          /* Set PWM5A on Zero */
   EPwm5Regs.AQCTLA.bit.CAU = AQ_CLEAR;        /* Clear PWM5A on match on count A up */
   EPwm5Regs.AQCTLB.bit.ZRO = AQ_SET;          /* Set PWM5B on Zero */
   EPwm5Regs.AQCTLB.bit.CBU = AQ_CLEAR;        /* Clear PWM5B on match on count B up */

   EPwm5Regs.CMPA.half.CMPA =DutyCycle_PWMA;             /* Set compare A value */
   EPwm5Regs.TBPRD = PERIOD;                             /* Set period for ePWM1 */
   EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;            /* Count up and start */
   EPwm5Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm5Regs.CMPCTL.bit.LOADAMODE = CC_CTR_PRD;
   EPwm5Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;           // sync "down-stream"
   EDIS;


   /* Configure trip zone based on Comparator 1 output and configure blanking */
   EALLOW;
   EPwm5Regs.TZSEL.bit.DCAEVT2 = 1;                      /* Digital compare, output A, cycle by cycle */

   EPwm5Regs.TZCTL.bit.TZA = TZ_FORCE_LO;                /* EPWM5A will go low */
   EPwm5Regs.TZCTL.bit.TZB = TZ_FORCE_HI;                /* EPWM5B goes HI */

   EPwm5Regs.DCTRIPSEL.bit.DCAHCOMPSEL = DC_COMP3OUT;    // DCAH = Comparator 1 output
   EPwm5Regs.TZDCSEL.bit.DCAEVT2 = TZ_DCAH_HI;           /** DCAEVT2 =  DCAL High (will become active as Comparator output goes High) */
                                                           /* DCAEVT2 = DCAH high (will become EPwm1Regs.DCTRIPSEL.bit.DCAHCOMPSEL = DC_COMP2OUT; DCAH = Comparator 2 output*/
   EPwm5Regs.DCACTL.bit.EVT2SRCSEL = DC_EVT2;            /* DCAEVT2 = DCAEVT2 (not filtered) */
   EPwm5Regs.DCACTL.bit.EVT2FRCSYNCSEL = DC_EVT_ASYNC;   /* Take async path */
   EPwm5Regs.DCFCTL.bit.PULSESEL = 0;                    /* Time-base counter equal to period (TBCTR =TBPRD) */
   EPwm5Regs.DCFCTL.bit.BLANKINV = 0;                    /* Blanking window inverted */
   EPwm5Regs.DCFCTL.bit.BLANKE = 1;                      /* Blanking Window Enable */
   EPwm5Regs.DCFCTL.bit.SRCSEL = 0;                      /* Source Is DCAEVT1 Signal */
   EPwm5Regs.DCFWINDOW = 0; //34;                        /*length blanking window: this corresponds to: 34*11.1ns=377.4ns*/
   EPwm5Regs.DCFOFFSET = 5;                              /* 5*11.1=55.5ns blanking window offset & then blanking window begins */
   EDIS;

   // Active high complementary PWMs - Set up the deadband
   EPwm5Regs.DBCTL.bit.IN_MODE = DBA_ALL;
   EPwm5Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
   EPwm5Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
   EPwm5Regs.DBRED =15;
   EPwm5Regs.DBFED =15;

}


// Gpio_select -
void Gpio_select(void)
{
    EALLOW;

    /************************************ PWMs  *****************************************/
    //  PWM1A
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;     // 0=GPIO,  1=EPWM1A,  2=Resv,  3=Resv
    //  GpioCtrlRegs.GPADIR.bit.GPIO0 = 1;      // 1=OUTput,  0=INput
    GpioDataRegs.GPACLEAR.bit.GPIO0 = 1;    // uncomment if --> Set Low initially
    //  GpioDataRegs.GPASET.bit.GPIO0 = 1;      // uncomment if --> Set High initially
    //------------------------------------------------------------------------------------
    //  EPWM1B
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;     // 0=GPIO,  1=EPWM1B,  2=Resv,  3=COMP1OUT
    //  GpioCtrlRegs.GPADIR.bit.GPIO1 = 1;      // 1=OUTput,  0=INput
    GpioDataRegs.GPACLEAR.bit.GPIO1 = 1;    // uncomment if --> Set Low initially
    //  GpioDataRegs.GPASET.bit.GPIO1 = 1;      // uncomment if --> Set High initially
    //------------------------------------------------------------------------------------
    //PWM2A
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;     // 0=GPIO,  1=EPWM2A,  2=Resv,  3=Resv
    //  GpioCtrlRegs.GPADIR.bit.GPIO2 = 0;      // 1=OUTput,  0=INput
    GpioDataRegs.GPACLEAR.bit.GPIO2 = 1;    // uncomment if --> Set Low initially
    //  GpioDataRegs.GPASET.bit.GPIO2 = 1;      // uncomment if --> Set High initially
    //--------------------------------------------------------------------------------------
    // PWM2B
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;     // 0=GPIO,  1=EPWM2B,  2=SPISOMI-A,  3=COMP2OUT
    //  GpioCtrlRegs.GPADIR.bit.GPIO3 = 1;      // 1=OUTput,  0=INput
    GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;    // uncomment if --> Set Low initially
    //  GpioDataRegs.GPASET.bit.GPIO3 = 1;      // uncomment if --> Set High initially
    //--------------------------------------------------------------------------------------
    //PWM3A
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;     // 0=GPIO,  1=EPWM3A,  2=Resv,  3=Resv
    //  GpioCtrlRegs.GPADIR.bit.GPIO2 = 0;      // 1=OUTput,  0=INput
    GpioDataRegs.GPACLEAR.bit.GPIO4 = 1;    // uncomment if --> Set Low initially
    //  GpioDataRegs.GPASET.bit.GPIO2 = 1;      // uncomment if --> Set High initially
    //--------------------------------------------------------------------------------------
    // PWM3B
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;     // 0=GPIO,  1=EPWM3B,  2=SPISOMI-A,  3=COMP2OUT
    //  GpioCtrlRegs.GPADIR.bit.GPIO3 = 1;      // 1=OUTput,  0=INput
    GpioDataRegs.GPACLEAR.bit.GPIO5 = 1;    // uncomment if --> Set Low initially
    //  GpioDataRegs.GPASET.bit.GPIO3 = 1;      // uncomment if --> Set High initially

    //PWM4A
    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;     // 0=GPIO,  1=EPWM4A,  2=Resv,  3=Resv
    //  GpioCtrlRegs.GPADIR.bit.GPIO2 = 0;      // 1=OUTput,  0=INput
    GpioDataRegs.GPACLEAR.bit.GPIO6 = 1;    // uncomment if --> Set Low initially
    //  GpioDataRegs.GPASET.bit.GPIO2 = 1;      // uncomment if --> Set High initially
    //--------------------------------------------------------------------------------------
    // PWM4B
     GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1;     // 0=GPIO,  1=EPWM4B,  2=SPISOMI-A,  3=COMP2OUT
     //  GpioCtrlRegs.GPADIR.bit.GPIO3 = 1;      // 1=OUTput,  0=INput
     GpioDataRegs.GPACLEAR.bit.GPIO7 = 1;    // uncomment if --> Set Low initially
     //  GpioDataRegs.GPASET.bit.GPIO3 = 1;      // uncomment if --> Set High initially

     //PWM5A
     GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 1;     // 0=GPIO,  1=EPWM5A,  2=Resv,  3=Resv
     //  GpioCtrlRegs.GPADIR.bit.GPIO2 = 0;      // 1=OUTput,  0=INput
     GpioDataRegs.GPACLEAR.bit.GPIO8 = 1;    // uncomment if --> Set Low initially
     //  GpioDataRegs.GPASET.bit.GPIO2 = 1;      // uncomment if --> Set High initially
     //--------------------------------------------------------------------------------------
     // PWM5B
     GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 1;         // 0=GPIO,  1=EPWM5B,  2=SPISOMI-A,  3=COMP2OUT
     //  GpioCtrlRegs.GPADIR.bit.GPIO9 = 1;      // 1=OUTput,  0=INput
     GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;        // uncomment if --> Set Low initially
     //  GpioDataRegs.GPASET.bit.GPIO9 = 1;      // uncomment if --> Set High initially

     //PWM6A
     GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 1;        // 0=GPIO,  1=EPWM6A,  2=Resv,  3=Resv
     //  GpioCtrlRegs.GPADIR.bit.GPIO10 = 0;      // 1=OUTput,  0=INput
     GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;       // uncomment if --> Set Low initially
     //  GpioDataRegs.GPASET.bit.GPIO10 = 1;      // uncomment if --> Set High initially
     //--------------------------------------------------------------------------------------
     // PWM6B
     GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 1;        // 0=GPIO,  1=EPWM6B,  2=SPISOMI-A,  3=COMP2OUT
     //  GpioCtrlRegs.GPADIR.bit.GPIO11 = 1;      // 1=OUTput,  0=INput
     GpioDataRegs.GPACLEAR.bit.GPIO11 = 1;       // uncomment if --> Set Low initially
     //  GpioDataRegs.GPASET.bit.GPIO11 = 1;      // uncomment if --> Set High initially


    /**************************************************************************************/
    //GPIO FUNCION
    //Use these for BUCK Dev board
    GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 0;    // 0=GPIO,  1=Resv,  2=Resv,  3=Resv
    GpioCtrlRegs.GPADIR.bit.GPIO12 = 1;     // 1=OUTput,  0=INput
    //GpioDataRegs.GPBCLEAR.bit.GPIO12 = 1;   // uncomment if --> Set Low initially
    GpioDataRegs.GPASET.bit.GPIO12 = 1;     // uncomment if --> Set High initially

    GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 0;    // 0=GPIO,  1=Resv,  2=Resv,  3=Resv
    GpioCtrlRegs.GPADIR.bit.GPIO22 = 1;     // 1=OUTput,  0=INput
    //  GpioDataRegs.GPBCLEAR.bit.GPIO32 = 1;   // uncomment if --> Set Low initially
    GpioDataRegs.GPASET.bit.GPIO22 = 1;     // uncomment if --> Set High initially

    GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 0;    // 0=GPIO,  1=Resv,  2=Resv,  3=Resv
    GpioCtrlRegs.GPBDIR.bit.GPIO32 = 1;     // 1=OUTput,  0=INput
    //  GpioDataRegs.GPBCLEAR.bit.GPIO32 = 1;   // uncomment if --> Set Low initially
    GpioDataRegs.GPBSET.bit.GPIO32 = 1;     // uncomment if --> Set High initially

    GpioCtrlRegs.GPBMUX1.bit.GPIO39 = 0;    // 0=GPIO,  1=Resv,  2=Resv,  3=Resv
    GpioCtrlRegs.GPBDIR.bit.GPIO39 = 1;     // 1=OUTput,  0=INput
    //  GpioDataRegs.GPBCLEAR.bit.GPIO32 = 1;   // uncomment if --> Set Low initially
    GpioDataRegs.GPBSET.bit.GPIO39 = 1;     // uncomment if --> Set High initially

    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 0;    // 0=GPIO,  1=EPWM6A,  2=Resv,
    GpioCtrlRegs.GPADIR.bit.GPIO10 = 1;     // 1=OUTput,  0=INput
    //GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;   // uncomment if --> Set Low initially
    GpioDataRegs.GPASET.bit.GPIO10 = 1;     // uncomment if --> Set High initially

    /**************************************************************************************/
    // COMPARATOR IOs

    GpioCtrlRegs.GPBMUX1.bit.GPIO42 = 3;     // 0=GPIO,  1=EPWM1B,  2=Resv,  3=COMP1OUT
    //GpioCtrlRegs.GPBDIR.bit.GPIO42 = 1;      // 1=OUTput,  0=INput
    GpioDataRegs.GPBCLEAR.bit.GPIO42 = 1;    // uncomment if --> Set Low initially
    // GpioDataRegs.GPASET.bit.GPIO1 = 1;      // uncomment if --> Set High initially

    GpioCtrlRegs.GPBMUX1.bit.GPIO43 =3;     // 0=GPIO,  1=EPWM2B,  2=SPISOMI-A,  3=COMP2OUT
    //GpioCtrlRegs.GPBDIR.bit.GPIO43 = 1;      // 1=OUTput,  0=INput
    GpioDataRegs.GPBCLEAR.bit.GPIO43 = 1;    // uncomment if --> Set Low initially
    //GpioDataRegs.GPASET.bit.GPIO3 = 1;      // uncomment if --> Set High initially

    GpioCtrlRegs.GPBMUX1.bit.GPIO34 =3;     // 0=GPIO,  1=EPWM2B,  2=SPISOMI-A,  3=COMP3OUT
    // GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;      // 1=OUTput,  0=INput
    GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;    // uncomment if --> Set Low initially
    // GpioDataRegs.GPASET.bit.GPIO3 = 1;      // uncomment if --> Set High initially

   EDIS;
}


