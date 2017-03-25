#include "stm32f4xx.h"

#define wielk_tabl 50							// tablica na dane z portu USART

volatile uint32_t timer_ms = 0;		// licznik delay_ms (pozwala na odczekiwanie zadanego czasu)

void SysTick_Handler() {					// obsluga przerwania od SysTick potrzebna do odmierzania czasu
	if(timer_ms > 0)
		timer_ms--;
}

void delay_ms(int delay) {				// funkcja oczekiwania okreslony czas
	timer_ms = delay;
	while(timer_ms);
}

void wyslij_znak(char s) {				// funkcja wysylajaca pojedynczy znak przez port USART1
	while(!(USART1->SR & USART_SR_TXE));
	USART1->DR = s;
}

void wyslij_dane(char *s) {				// funkcja wysylajaca ciagu znaków przez port USART1
	while(*s)
		wyslij_znak(*s++);
}

void USART1_IRQHandler() {				// obsluga [rzerwania od USART1
	int czas = 0;
	char dane;
	if(USART1->SR & USART_SR_RXNE) {	// Czy sa dane do odczytania -> jesli tak odczytaj dane i je odeslij
		while(czas < 600) {
			while(USART1->SR & USART_SR_RXNE) {
				dane = (uint16_t)(USART1->DR & (uint16_t)0x01FF);
				wyslij_znak(dane);
				czas = 0;
			}
			czas++;
		}
		czas = 0;
	}
}

int main(void) {			// glówna funkcja programu
	int dane_analog = 0;
	
/* KONFIGURACJA ZEGARÓW */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; 
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN | RCC_APB2ENR_USART1EN| RCC_APB2ENR_ADC1EN;
	
/* KONFIGURACJA PORTÓW GPIO */
	GPIOD->MODER = GPIO_MODER_MODER13_0 | GPIO_MODER_MODER12_0 |
								 GPIO_MODER_MODER14_0 | GPIO_MODER_MODER15_0;	// piny dla diod na plytce jako wyjscia (PD12, PD13, PD14, PD15)
/* PORTY TIM4 */
	GPIOD->MODER |= GPIO_MODER_MODER15_1;												// Tryb funkcji alternatywnej dla diody pod portem PD15
	GPIOD->MODER &= ~GPIO_MODER_MODER15_0;											// Tryb funkcji alternatywnej dla diody pod portem PD15
	GPIOD->AFR[1] |= GPIO_AFRH_AFRH7_1;													// Funkcja alternatywna PD15 -> chanel TIM4
/* PORTY USART1 */
	GPIOA->MODER |= GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1;	// Tryb funkcji alternatywnej USART1 (PA9-TX i PA10-RX)
	GPIOA->AFR[1] |= GPIO_AFRH_AFRH2_0 | GPIO_AFRH_AFRH2_1 |		
									 GPIO_AFRH_AFRH2_2 |  GPIO_AFRH_AFRH3_0 | 
									 GPIO_AFRH_AFRH3_1 | GPIO_AFRH_AFRH3_2;			// Funkcja alternatywna USART1 dla portów PA9-TX i PA10-RX
	
/* ZMIANA ZEGARA PROCESORA Z 16 NA 168 MHZ */
	//WLACZENIE ZEWNETRZNEGO GENERATORA HSE
	RCC->CR = (uint32_t)0x00000001;
	RCC->CR |= RCC_CR_HSEON;
	while(!(RCC->CR & RCC_CR_HSERDY)) {}	// Czekanie na gotowosc HSE
	//KONFIGURACJA PETLI PLL
	RCC->PLLCFGR = (uint32_t)0x00;
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC | RCC_PLLCFGR_PLLP_0 |
									RCC_PLLCFGR_PLLM_1 | RCC_PLLCFGR_PLLN_7 | 
									RCC_PLLCFGR_PLLN_5 | RCC_PLLCFGR_PLLN_3;
	RCC->CR |= RCC_CR_PLLON;
	while(!(RCC->CR & RCC_CR_PLLRDY)) {}	// Czekanie na gotowosc PLL
	//USTAWIENIE ZWLOKI PAMIECI
  FLASH->ACR |= FLASH_ACR_LATENCY_5WS;
	//PRZELACZENIE ZRODLA SYGNALU ZEGAROWEGO NA HSE
 	RCC->CFGR &= ~RCC_CFGR_SW_0;
	RCC->CFGR |= RCC_CFGR_SW_1;
	//WYLACZENIE HSI
	RCC->CR &= ~((uint32_t)0x00000001);
		
/* KONFIGURACJA OPÓZNIENIA SysTick */		
	SysTick->LOAD = (SystemCoreClock / 1000);
	SysTick->VAL = 0;
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk |
                   SysTick_CTRL_ENABLE_Msk;

/* KONFIGURACJA TIM4 */
	TIM4->ARR = 1000;
	TIM4->CCR4 = 200;
	TIM4->CCMR2 = TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;
	TIM4->CCER = TIM_CCER_CC4E;
	TIM4->CR1  = TIM_CR1_CEN;

/* KONFIGURACJA ADC */
	//PA6 -> ADC12_IN6
	GPIOA->MODER |= GPIO_MODER_MODER6_0 | GPIO_MODER_MODER6_1;
	ADC1->CR2 |= ADC_CR2_ADON;
	ADC1->SQR3 = 0x06;
	ADC1->SMPR2 = ADC_SMPR2_SMP6_2;

/* KONFIGURACJA USART1 */
	USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;		// USART2
	USART1->CR1 |= USART_CR1_RXNEIE;
	USART1->BRR = (SystemCoreClock / 115200);
/* KONFIGURACJA (WLACZENIE OBSLUGI) przerwania od USART1 RX */
	NVIC_EnableIRQ(USART1_IRQn);
	
	while(1) {
		if(~(ADC1->CR2 & ADC_CR2_SWSTART)){		// jesli ADC1 nie pracjuje -> wykonaj kolejny pomiar
			ADC1->CR2 |= ADC_CR2_SWSTART;
			dane_analog = ADC1->DR;
			if(dane_analog < 1000)
				TIM4->CCR4 = (dane_analog);
		}
	}
}