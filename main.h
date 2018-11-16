#ifndef main_H
#define main_H



void init_cla(void);

__interrupt void cla1_task1_isr(void);
__interrupt void cla1_task2_isr(void);
__interrupt void cla1_task3_isr(void);
__interrupt void cla1_task4_isr(void);


__interrupt void cpu_timer1_isr(void);



void Gpio_select(void);
void MemCopy(Uint16 *SourceAddr, Uint16* SourceEndAddr, Uint16* DestAddr);
void DeviceInit(void);

void Comparator_Config(void);
void PWMs_Config(void);
void ADCs_Config(void);


#endif


