
#include <stdbool.h>
#include <sam.h>

void initializeSystemFor48MHz( void )
{
	// Change the timing of the NVM access
	NVMCTRL->CTRLB.bit.RWS = NVMCTRL_CTRLB_RWS_HALF_Val; // 1 wait state for operating at 2.7-3.3V at 48MHz.

	// Enable the bus clock for the clock system.
	PM->APBAMASK.bit.GCLK_ = true;

	// Initialise the DFLL to run in closed-loop mode at 48MHz
	// 1. Make a software reset of the clock system.
	GCLK->CTRL.bit.SWRST = true;
	while (GCLK->CTRL.bit.SWRST && GCLK->STATUS.bit.SYNCBUSY) {};
	// 2. Make sure the OCM8M keeps running.
	SYSCTRL->OSC8M.bit.ONDEMAND = 0;
	// 3. Set the division factor to 64, which reduces the 1MHz source to 15.625kHz
	GCLK->GENDIV.reg =
	GCLK_GENDIV_ID(3) | // Select generator 3
	GCLK_GENDIV_DIV(64); // Set the division factor to 64
	// 4. Create generic clock generator 3 for the 15KHz signal of the DFLL
	GCLK->GENCTRL.reg =
	GCLK_GENCTRL_ID(3) | // Select generator 3
	GCLK_GENCTRL_SRC_OSC8M | // Select source OSC8M
	GCLK_GENCTRL_GENEN; // Enable this generic clock generator
	while (GCLK->STATUS.bit.SYNCBUSY) {}; // Wait for synchronization
	// 5. Configure DFLL with the
	GCLK->CLKCTRL.reg =
	GCLK_CLKCTRL_ID_DFLL48M | // Target is DFLL48M
	GCLK_CLKCTRL_GEN(3) | // Select generator 3 as source.
	GCLK_CLKCTRL_CLKEN; // Enable the DFLL48M
	while (GCLK->STATUS.bit.SYNCBUSY) {}; // Wait for synchronization
	// 6. Workaround to be able to configure the DFLL.
	SYSCTRL->DFLLCTRL.bit.ONDEMAND = false;
	while (!SYSCTRL->PCLKSR.bit.DFLLRDY) {}; // Wait for synchronization.
	// 7. Change the multiplication factor.
	SYSCTRL->DFLLMUL.bit.MUL = 3072; // 48MHz / (1MHz / 64)
	SYSCTRL->DFLLMUL.bit.CSTEP = 1; // Coarse step = 1
	SYSCTRL->DFLLMUL.bit.FSTEP = 1; // Fine step = 1
	while (!SYSCTRL->PCLKSR.bit.DFLLRDY) {}; // Wait for synchronization.
	// 8. Start closed-loop mode
	SYSCTRL->DFLLCTRL.reg |=
	SYSCTRL_DFLLCTRL_MODE | // 1 = Closed loop mode.
	SYSCTRL_DFLLCTRL_QLDIS; // 1 = Disable quick lock.
	while (!SYSCTRL->PCLKSR.bit.DFLLRDY) {}; // Wait for synchronization.
	// 9. Clear the lock flags.
	SYSCTRL->INTFLAG.bit.DFLLLCKC = 1;
	SYSCTRL->INTFLAG.bit.DFLLLCKF = 1;
	SYSCTRL->INTFLAG.bit.DFLLRDY = 1;
	// 10. Enable the DFLL
	SYSCTRL->DFLLCTRL.bit.ENABLE = true;
	while (!SYSCTRL->PCLKSR.bit.DFLLRDY) {}; // Wait for synchronization.
	// 11. Wait for the fine and coarse locks.
	while (!SYSCTRL->INTFLAG.bit.DFLLLCKC && !SYSCTRL->INTFLAG.bit.DFLLLCKF) {};
	// 12. Wait until the DFLL is ready.
	while (!SYSCTRL->INTFLAG.bit.DFLLRDY) {};

	// Switch the main clock speed.
	// 1. Set the divisor of generic clock 0 to 0
	GCLK->GENDIV.reg =
	GCLK_GENDIV_ID(0) | // Select generator 0
	GCLK_GENDIV_DIV(0);
	while (GCLK->STATUS.bit.SYNCBUSY) {}; // Wait for synchronization
	// 2. Switch generic clock 0 to the DFLL
	GCLK->GENCTRL.reg =
	GCLK_GENCTRL_ID(0) | // Select generator 0
	GCLK_GENCTRL_SRC_DFLL48M | // Select source DFLL
	GCLK_GENCTRL_IDC | // Set improved duty cycle 50/50
	GCLK_GENCTRL_GENEN; // Enable this generic clock generator
	while (GCLK->STATUS.bit.SYNCBUSY) {}; // Wait for synchronization
}