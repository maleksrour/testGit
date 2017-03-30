// MALEK srour
// os.c
// Runs on LM4F120/TM4C123
// A very simple real time operating system with minimal features.
// Daniel Valvano
// January 29, 2015

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2015

   Programs 4.4 through 4.12, section 4.2

 Copyright 2015 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

#include <stdint.h>
#include "os.h"
#include "PLL.h"
#include "../inc/tm4c123gh6pm.h"

#define NVIC_ST_CTRL_R          (*((volatile uint32_t *)0xE000E010))
#define NVIC_ST_CTRL_CLK_SRC    0x00000004  // Clock Source
#define NVIC_ST_CTRL_INTEN      0x00000002  // Interrupt enable
#define NVIC_ST_CTRL_ENABLE     0x00000001  // Counter mode
#define NVIC_ST_RELOAD_R        (*((volatile uint32_t *)0xE000E014))
#define NVIC_ST_CURRENT_R       (*((volatile uint32_t *)0xE000E018))
#define NVIC_INT_CTRL_R         (*((volatile uint32_t *)0xE000ED04))
#define NVIC_INT_CTRL_PENDSTSET 0x04000000  // Set pending SysTick interrupt
#define NVIC_SYS_PRI3_R         (*((volatile uint32_t *)0xE000ED20))  // Sys. Handlers 12 to 15 Priority

volatile int OS_Time_ms = 0; //System time

// function definitions in osasm.s
void OS_DisableInterrupts(void); // Disable interrupts
void OS_EnableInterrupts(void);  // Enable interrupts
int32_t StartCritical(void);
void EndCritical(int32_t primask);
void StartOS(void);
void (*TaskSW1)(void);   // user function
void (*TaskSW2)(void);   // user function
void (*PeriodicTask1)(void); //first periodic task
void (*PeriodicTask2)(void); //second periodic task
uint32_t priority1;
uint32_t priority2;

#define NUMTHREADS  10        // maximum number of threads
#define STACKSIZE   256      // number of 32-bit words in stack

static int NUM_ACTV_THREADS = 0;

tcbType tcbs[NUMTHREADS];
tcbType *RunPt;
tcbType *NRunPt;
int32_t Stacks[NUMTHREADS][STACKSIZE];

// ******** OS_Init ************
// initialize operating system, disable interrupts until OS_Launch
// initialize OS controlled I/O: systick, 80 MHz PLL
// input:  none
// output: none
void OS_Init(void){
  OS_DisableInterrupts();
  PLL_Init(Bus80MHz);         // set processor clock to 80 MHz
  NVIC_ST_CTRL_R = 0;         // disable SysTick during setup
  NVIC_ST_CURRENT_R = 0;      // any write to current clears it
  NVIC_SYS_PRI3_R =(NVIC_SYS_PRI3_R&0x00FFFFFF)|0xE0000000; // priority 7
}

void SetInitialStack(int i){
  tcbs[i].sp = &Stacks[i][STACKSIZE-16]; // thread stack pointer
  Stacks[i][STACKSIZE-1] = 0x01000000;   // thumb bit
  Stacks[i][STACKSIZE-3] = (uint32_t)(&OS_Kill);   // R14 -- LR (to avoid crash when task doesn't have an infinite loop)
  Stacks[i][STACKSIZE-4] = 0x12121212;   // R12
  Stacks[i][STACKSIZE-5] = 0x03030303;   // R3
  Stacks[i][STACKSIZE-6] = 0x02020202;   // R2
  Stacks[i][STACKSIZE-7] = 0x01010101;   // R1
  Stacks[i][STACKSIZE-8] = 0x00000000;   // R0
  Stacks[i][STACKSIZE-9] = 0x11111111;   // R11
  Stacks[i][STACKSIZE-10] = 0x10101010;  // R10
  Stacks[i][STACKSIZE-11] = 0x09090909;  // R9
  Stacks[i][STACKSIZE-12] = 0x08080808;  // R8
  Stacks[i][STACKSIZE-13] = 0x07070707;  // R7
  Stacks[i][STACKSIZE-14] = 0x06060606;  // R6
  Stacks[i][STACKSIZE-15] = 0x05050505;  // R5
  Stacks[i][STACKSIZE-16] = 0x04040404;  // R4
}

///******** OS_Launch ***************
// start the scheduler, enable interrupts
// Inputs: number of 20ns clock cycles for each time slice
//         (maximum of 24 bits)
// Outputs: none (does not return)
void OS_Launch(uint32_t theTimeSlice){
  NVIC_ST_RELOAD_R = theTimeSlice - 1; // reload value
  NVIC_ST_CTRL_R = 0x00000007; // enable, core clock and interrupt arm
	NRunPt = RunPt;
  StartOS();                   // start on the first task
}

//----------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------//
//---------------------------------------------Malek & Sarnab-----------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------//


//**************OS_AddThreads***************
int OS_AddThread(void(*task)(void),int stack_size, int priority)
{ 
	int32_t status;
	int i;
  status = StartCritical();
	if(NUM_ACTV_THREADS == 0)     //1st thread to be added (link it to itself)
	{
		//link and initialize stack n tcb
		tcbs[0].next = &tcbs[0];
		tcbs[0].prev = &tcbs[0];
		SetInitialStack(0); Stacks[0][STACKSIZE-2] = (int32_t)(task); // PC
		tcbs[0].id = 1;
		tcbs[0].priority = priority;
		NUM_ACTV_THREADS++;
		RunPt = &tcbs[0];       // thread 0 will run first
		EndCritical(status);
		return 1;
	}
	
	for(i=0;i<NUMTHREADS;i++)      // checking for available tcbs
	{
		if(tcbs[i].id == 0)
			break;
	}
	
	if (i == 10) {
		EndCritical(status);
		return 0;
	}
	
	else {
		tcbs[i].next = RunPt->next;
		tcbs[i].prev = RunPt;
		RunPt->next->prev = &tcbs[i];
		RunPt->next = &tcbs[i];
		SetInitialStack(i); Stacks[i][STACKSIZE-2] = (int32_t)(task); // PC
		tcbs[i].id = i+1;
		tcbs[i].priority = priority;
		NUM_ACTV_THREADS++;
		EndCritical(status);
		return 1; 
	}              // successful
}

//********OS_Suspend*********
void OS_Suspend(void)
{
	NVIC_ST_CURRENT_R = 0;          //counter reset
	NVIC_INT_CTRL_R = 0x04000000;   //trigger systick
}

//********OS_Sleep*********
void OS_Sleep(uint32_t time)
{
	RunPt->sleep = time;
	NVIC_ST_CURRENT_R = 0;          //counter reset
	NVIC_INT_CTRL_R = 0x04000000;   //trigger systick
}

//********OS_Kill*********
void OS_Kill(void)
{
	//NumCreated--;   // used in lab2.c for display
	NUM_ACTV_THREADS--; // used for our OS.c
	RunPt->id = 0;
	RunPt->prev->next = RunPt->next;
	RunPt->next->prev = RunPt->prev;
	NRunPt = RunPt->next;
	NVIC_ST_CURRENT_R = 0;          //counter reset
	NVIC_INT_CTRL_R = 0x04000000;   //trigger systick
}

void OS_AddPeriodicThread(void(*task)(void),uint32_t period, uint32_t priority) {
	long sr;
  sr = StartCritical(); 
	if ((SYSCTL_RCGCTIMER_R & 0x08) == 0) {
		SYSCTL_RCGCTIMER_R |= 0x08;   // 0) activate TIMER3
		PeriodicTask1 = task;          // user function
		TIMER3_CTL_R = 0x00000000;    // 1) disable TIMER3A during setup
		TIMER3_CFG_R = 0x00000000;    // 2) configure for 32-bit mode
		TIMER3_TAMR_R = 0x00000002;   // 3) configure for periodic mode, default down-count settings
		TIMER3_TAILR_R = period-1;    // 4) reload value
		TIMER3_TAPR_R = 0;            // 5) bus clock resolution
		TIMER3_ICR_R = 0x00000001;    // 6) clear TIMER3A timeout flag
		TIMER3_IMR_R = 0x00000001;    // 7) arm timeout interrupt
		NVIC_PRI8_R = (NVIC_PRI8_R&0x00FFFFFF)| priority<<29; // 8) priority defined by user
	// interrupts enabled in the main program after all devices initialized
	// vector number 51, interrupt number 35
		NVIC_EN1_R = 1<<3;           // 9) enable IRQ 35 in NVIC
		TIMER3_CTL_R = 0x00000001;    // 10) enable TIMER3A
		EndCritical(sr);
	}
	else {
		SYSCTL_RCGCTIMER_R |= 0x10;   // 0) activate TIMER4
		PeriodicTask2 = task;          // user function
		TIMER4_CTL_R = 0x00000000;    // 1) disable TIMER4A during setup
		TIMER4_CFG_R = 0x00000000;    // 2) configure for 32-bit mode
		TIMER4_TAMR_R = 0x00000002;   // 3) configure for periodic mode, default down-count settings
		TIMER4_TAILR_R = period-1;    // 4) reload value
		TIMER4_TAPR_R = 0;            // 5) bus clock resolution
		TIMER4_ICR_R = 0x00000001;    // 6) clear TIMER1A timeout flag
		TIMER4_IMR_R = 0x00000001;    // 7) arm timeout interrupt
		NVIC_PRI17_R = (NVIC_PRI17_R&0xFF00FFFF)| priority<<21; // 8) priority defined by user
	// interrupts enabled in the main program after all devices initialized
	// vector number 86, interrupt number 70
		NVIC_EN2_R = 1<<6;           // 9) enable IRQ 70 in NVIC
		TIMER4_CTL_R = 0x00000001;    // 10) enable TIMER4A
		EndCritical(sr);
	}
}

//---------------- Used for 1 ms timer -------------------- //
void OS_AddPeriodicTimer(uint32_t period_ms, uint32_t priority)
{
	long sr;
  sr = StartCritical(); 
  SYSCTL_RCGCTIMER_R |= 0x01;   // 0) activate TIMER0
//  PeriodicTask = task;          // user function
  TIMER0_CTL_R = 0x00000000;    // 1) disable TIMER0A during setup
  TIMER0_CFG_R = 0x00000000;    // 2) configure for 32-bit mode
  TIMER0_TAMR_R = 0x00000002;   // 3) configure for periodic mode, default down-count settings
  TIMER0_TAILR_R = period_ms*80000-1;    // 4) reload value (60000 = 1000000/12.5)
  TIMER0_TAPR_R = 0;            // 5) bus clock resolution
  TIMER0_ICR_R = 0x00000001;    // 6) clear TIMER0A timeout flag
  TIMER0_IMR_R = 0x00000001;    // 7) arm timeout interrupt
  NVIC_PRI4_R = (NVIC_PRI4_R&0x00FFFFFF)| priority<<29; // 8) priority defined by user
// interrupts enabled in the main program after all devices initialized
// vector number 35, interrupt number 19
  NVIC_EN0_R = 1<<19;           // 9) enable IRQ 19 in NVIC
  TIMER0_CTL_R = 0x00000001;    // 10) enable TIMER0A
  EndCritical(sr);
}

void OS_TimeInit(void)        // 12.5ns timer
{
	long sr;
  sr = StartCritical(); 
  SYSCTL_RCGCWTIMER_R |= 0x02;   // 0) activate wide TIMER1
//  PeriodicTask = task;          // user function
  WTIMER1_CTL_R = 0x00000000;    // 1) disable wide TIMER1A during setup
  int delay = WTIMER1_CTL_R;
  WTIMER1_CFG_R = 0x00000000;    // 2) configure for 64-bit mode
  WTIMER1_TAMR_R = 0x00000012;   // 3) configure for periodic mode, up-count settings
  WTIMER1_TBILR_R = 0xF0000000;    // 4) reload value 63:32
	WTIMER1_TAILR_R = 0x00000000;    // 4) reload value 31:0
  WTIMER1_TAPR_R = 0;            // 5) bus clock resolution
  WTIMER1_ICR_R = 0x00000001;    // 6) clear TIMER1A timeout flag
//  WTIMER1_IMR_R = 0x00000001;    // 7) arm timeout interrupt
//  NVIC_PRI4_R = (NVIC_PRI4_R&0x00FFFFFF)| priority<<29; // 8) priority defined by user
// interrupts enabled in the main program after all devices initialized
// vector number 35, interrupt number 19
//  NVIC_EN0_R = 1<<19;           // 9) enable IRQ 19 in NVIC
  WTIMER1_CTL_R = 0x00000101;    // 10) enable TIMER1A
  EndCritical(sr);
}

uint64_t OS_Time(void)         // 12.5ns resolution
{
	uint64_t i = WTIMER1_TBV_R;
	return ((i << 32) | WTIMER1_TAV_R);
}

unsigned long OS_TimeDifference(unsigned long start, unsigned long stop) {
	return (stop - start);
}
void OS_ClearMsTime(void)
{
	OS_Time_ms = 0;
}

uint32_t OS_MsTime(void)
{
	return (OS_Time_ms);
}

//******** OS_Id *************** 
// returns the thread ID for the currently running thread
unsigned long OS_Id(void)
{
	return (RunPt->id);
}

//******** OS_AddSW1Task *************** 
// add a background task to run whenever the SW1 (PF4) button is pushed
void OS_AddSW1Task(void(*task)(void), unsigned long priority)   // function to be executed on SW1(PF4) interrupt
{
	SYSCTL_RCGCGPIO_R |= 0x00000020; // (a) activate clock for port F
	//FallingEdges = 0;             // (b) initialize counter
	TaskSW1 = task;
	//priority1 = priority;
  GPIO_PORTF_DIR_R &= ~0x10;    // (c) make PF4 in (built-in button)
  GPIO_PORTF_AFSEL_R &= ~0x10;  //     disable alt funct on PF4
  GPIO_PORTF_DEN_R |= 0x10;     //     enable digital I/O on PF4
  GPIO_PORTF_PCTL_R &= ~0x000F0000; // configure PF4 as GPIO
  GPIO_PORTF_AMSEL_R = 0;       //     disable analog functionality on PF
  GPIO_PORTF_PUR_R |= 0x10;     //     enable weak pull-up on PF4
  GPIO_PORTF_IS_R &= ~0x10;     // (d) PF4 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x10;    //     PF4 is not both edges
  GPIO_PORTF_IEV_R &= ~0x10;    //     PF4 falling edge event
  GPIO_PORTF_ICR_R = 0x10;      // (e) clear flag4
  GPIO_PORTF_IM_R |= 0x10;      // (f) arm interrupt on PF4 *** No IME bit as mentioned in Book ***
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|(priority<<21); //
  NVIC_EN0_R = 0x40000000;      // (h) enable interrupt 30 in NVIC
}
//******** OS_AddSW2Task *************** 
// add a background task to run whenever the SW2 (PF0) button is pushed
void OS_AddSW2Task(void(*task)(void), unsigned long priority)   // function to be executed on SW2(PF0) interrupt
{
	SYSCTL_RCGCGPIO_R |= 0x00000020; // (a) activate clock for port F
	//FallingEdges = 0;             // (b) initialize counter
	TaskSW2 = task;
	//priority2 = priority;
	GPIO_PORTF_LOCK_R = 0x4C4F434B;       //  unlock REQUIRED for PF0
	GPIO_PORTF_CR_R |= 0x01;       // commit PF0
  GPIO_PORTF_DIR_R &= ~0x01;    // (c) make PF0 in (built-in button)
  GPIO_PORTF_AFSEL_R &= ~0x01;  //     disable alt funct on PF0
  GPIO_PORTF_DEN_R |= 0x01;     //     enable digital I/O on PF0
  GPIO_PORTF_PCTL_R &= ~0x0000000F; // configure PF0 as GPIO
  GPIO_PORTF_AMSEL_R = 0;       //     disable analog functionality on PF
  GPIO_PORTF_PUR_R |= 0x01;     //     enable weak pull-up on PF0
  GPIO_PORTF_IS_R &= ~0x01;     // (d) PF0 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x01;    //     PF0 is not both edges
  GPIO_PORTF_IEV_R &= ~0x01;    //     PF0 falling edge event
  GPIO_PORTF_ICR_R = 0x01;      // (e) clear flag0
  GPIO_PORTF_IM_R |= 0x01;      // (f) arm interrupt on PF0 *** No IME bit as mentioned in Book ***
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|(priority<<21); // 
  NVIC_EN0_R = 0x40000000;      // (h) enable interrupt 30 in NVIC
}


void Timer0A_Handler(void)          //1ms timer
{
  TIMER0_ICR_R = TIMER_ICR_TATOCINT;// acknowledge timer0A timeout
  int i;
	for(i = 0; i<NUMTHREADS;i++)
		if(tcbs[i].id!=0 && tcbs[i].sleep>0)     //decrement the sleep value
			tcbs[i].sleep-=1;                      // of sleeping threads
	OS_Time_ms+= 1;														 // increment CPU time
}

void Timer3A_Handler(void) { // add periodic thread timer
	TIMER3_ICR_R = TIMER_ICR_TATOCINT;// acknowledge timer3A timeout
	(*PeriodicTask1)();
}

void Timer4A_Handler(void) { // add periodic thread timer
	TIMER4_ICR_R = TIMER_ICR_TATOCINT;// acknowledge timer4A timeout
	(*PeriodicTask2)();
}

void GPIOPortF_Handler(void) {
	int s = GPIO_PORTF_RIS_R & 0x11;
	if(s == 16){
		(*TaskSW1)();
		GPIO_PORTF_ICR_R = 0x10;      // acknowledge flag4
	}
	else if(s == 1){
		(*TaskSW2)();
		GPIO_PORTF_ICR_R = 0x01;      // acknowledge flag0
	}
}

// ----------------------- SEMAPHORES ----------------------------------- //
void OS_InitSemaphore(Sema4Type *semaPt, long value) {
	semaPt->Value = value;
	semaPt->BlockedPt = 0;
}
int i;
void OS_Wait(Sema4Type *semaPt) {
	OS_DisableInterrupts();
	(semaPt->Value) = (semaPt->Value) - 1;
	if (semaPt->Value < -3) {
		i = 1;
	}
	if( (semaPt->Value) < 0 ) { // busy
		OS_Block(semaPt); // blocked semaphore
		OS_Suspend(); //trigger thread switch
		//OS_EnableInterrupts();
	}
	OS_EnableInterrupts();
}

void OS_Signal(Sema4Type *semaPt) {
	OS_DisableInterrupts();
	(semaPt->Value) = (semaPt->Value) + 1;
	if( (semaPt->Value) <= 0 ) {
		OS_Wake(semaPt); // wake first thread blocked on this semaphore
		OS_Suspend(); //trigger thread switch
		//OS_EnableInterrupts();
	}
	OS_EnableInterrupts();
}

void OS_bWait(Sema4Type *semaPt) {
	OS_DisableInterrupts();
	while( (semaPt->Value) == 0 ) { // busy
		OS_EnableInterrupts();
		OS_Suspend(); //trigger thread switch
		OS_DisableInterrupts();
	}
	(semaPt->Value) = 0;
	OS_EnableInterrupts();	
}

void OS_bSignal(Sema4Type *semaPt) {
	OS_DisableInterrupts();
	(semaPt->Value) = 1;
	OS_Suspend(); //trigger thread switch
	OS_EnableInterrupts();	
}

// ------------------------------ OS FIFO ----------------------------------- //
// Two-index implementation of the OS_ FIFO
// can hold 0 to OS_FIFO_SIZE elements
uint32_t OS_FIFO_SIZE;  // must be a power of 2
uint32_t volatile OS_PutI;// put next
uint32_t volatile OS_GetI;// get next
OS_FIFODataType static OS_Fifo[256]; //long type for now, to change refer to OS.h file

//FIFO semaphores variables
Sema4Type data_available;
Sema4Type space_available;

// initialize index FIFO
void OS_Fifo_Init(uint32_t size){ 
	long sr;
  sr = StartCritical(); // make atomic
	// initialize semaphores
	OS_FIFO_SIZE = size;
	OS_InitSemaphore(&data_available, 0); 
	OS_InitSemaphore(&space_available, size);
  OS_PutI = OS_GetI = 0;  // Empty
  EndCritical(sr);
}
// add element to end of index FIFO
// return OS_FIFO_SUCCESS if successful
int OS_Fifo_Put(OS_FIFODataType data){
	//OS_Wait(&space_available); (called by an event (background thread))
	if  ( space_available.Value == 0 ) { return(OS_FIFO_FAIL); }
	else {
		space_available.Value = space_available.Value - 1;
		OS_Fifo[OS_PutI&(OS_FIFO_SIZE-1)] = data; // put
		OS_PutI++;  // Success, update
		OS_Signal(&data_available);
		return(OS_FIFO_SUCCESS);
	}
}
// remove element from front of index FIFO
// return OS_FIFO_SUCCESS if successful
unsigned long OS_Fifo_Get(void){
	OS_Wait(&data_available);
  unsigned long data = OS_Fifo[OS_GetI&(OS_FIFO_SIZE-1)];
  OS_GetI++;  // Success, update
	OS_Signal(&space_available);
  return data;
}
// number of elements in index FIFO
// 0 to OS_FIFOS_IZE-1
uint32_t OS_Fifo_Size(void){
 return ((uint32_t)(OS_PutI-OS_GetI));
}

// ------------------------------ MAILBOX ----------------------------------- //
//Mailbox semaphores variables
Sema4Type Send;
Sema4Type Ack;
unsigned long Mail;

void OS_MailBox_Init(void)
{
	OS_InitSemaphore(&Send, 0); 
	OS_InitSemaphore(&Ack, 0);
	Mail = 0;
}

void OS_MailBox_Send(unsigned long data)
{
	Mail = data;
	OS_Signal(&Send);
	OS_Wait(&Ack);
}

unsigned long OS_MailBox_Recv(void)
{
	OS_Wait(&Send);
	unsigned long theData = Mail;
	OS_Signal(&Ack);
	return theData;
}
	
//---------------------------- Priority Scheduler in C-------------------------- //
void Scheduler(void) {
	//long sr = StartCritical();
	uint32_t max = 255; //max
	tcbType *bestPt;
	tcbType *pt; 
	// !!! NOTE !!! PREVENTING STARVATION in presence of CPU bound task //
	// Linking after waking should be done opposite to scheduling after blocking (prev to Runpt vs. next to Runpt) //
	//if ((RunPt->id == 0) || (RunPt->blocked == 1)) RunPt = RunPt->next; 
	pt = NRunPt;
	do{
		pt = pt->next; //skips the one being switched
		if( ((pt->priority) < max) && ((pt->sleep) == 0) ){
			max = pt->priority;
			bestPt = pt;
		}
	} while(pt != NRunPt);
	RunPt = bestPt;
	NRunPt = RunPt;
	//EndCritical(sr);
}
int block=0,wake=0;
//----------------------------------- Thread Blocking -------------------------- //
// Delinks the thread from TCB and links it to the semaphore blocked linked list
void OS_Block(Sema4Type *semaPt) {
	block++;
	//delink the current thread from tcb linked list and set the block 
	RunPt->next->prev = RunPt->prev;
	RunPt->prev->next = RunPt->next;
	RunPt->blocked = 1;
	//link the blocked thread to the semaphore list in priority order
	tcbType *thisTCB = RunPt;
	tcbType *pt = semaPt->BlockedPt;
	int flag = 0; //flag to check the corner case of comparing with last blocked thread
	//corner case: no blocked threads on this semaphore
	if (pt == 0){
		semaPt->BlockedPt = thisTCB;
		thisTCB->semaPrev = 0;
		thisTCB->semaNext = 0;
	}
	else if (thisTCB->priority < pt->priority) { //corner case: inserting in the beginning of blocked list
		thisTCB->semaNext = semaPt->BlockedPt;
		semaPt->BlockedPt = thisTCB;
		pt->semaPrev = thisTCB;
		thisTCB->semaPrev = 0;
	}
	else { //insert this TCB in correct place according to priority first and then bounded waiting
		while(pt->semaNext){
			if (thisTCB->priority < pt->priority){ flag = 1; break; }
			pt = pt->semaNext;
		}
		if (flag){
			thisTCB->semaNext = pt;
			thisTCB->semaPrev = pt->semaPrev;
			pt->semaPrev->semaNext = thisTCB;
			pt->semaPrev = thisTCB;
		}
		else {
			if (thisTCB->priority < pt->priority) {
				thisTCB->semaNext = pt;
				thisTCB->semaPrev = pt->semaPrev;
				pt->semaPrev->semaNext = thisTCB;
				pt->semaPrev = thisTCB;
			}
			else {
				pt->semaNext = thisTCB;
				thisTCB->semaNext = 0;
				thisTCB->semaPrev = pt;
			}
		}
	}
	NRunPt = RunPt->next;
}

//----------------------------------- Thread Waking -------------------------- //
// Delinks the thread from blocked linked list and links it to the TCB linked list
void OS_Wake(Sema4Type *semaPt) {
	wake++;
	tcbType *thisTCB = semaPt->BlockedPt;
	thisTCB->blocked = 0; //it is not blocked anymore
	// delink the first thread from the blocked linked list
	semaPt->BlockedPt = thisTCB->semaNext;
	if (semaPt->BlockedPt) { semaPt->BlockedPt->semaPrev = 0; } // check if we have remaining blocked threads 
	// link the first thread in blocked list to tcb list (before the current runnin thread)
	// !!! NOTE !!! PREVENTING STARVATION in presence of CPU bound task //
	// Linking after waking should be done opposite to scheduling after blocking (prev to Runpt vs. next to Runpt) //
	thisTCB->next = NRunPt;
	thisTCB->prev = NRunPt->prev;
	thisTCB->semaNext = 0;
	thisTCB->semaPrev = 0;
	NRunPt->prev->next = thisTCB;
	NRunPt->prev = thisTCB;
}



int OS_AddProcess(void(*entry)(void), void *text, void *data, unsigned long stackSize, unsigned long priority)
{
	return 0;
}


