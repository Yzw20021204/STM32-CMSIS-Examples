
#include <stdint.h>

#include "stm32f4xx.h"

//-------------------- Clock Ayarı (168MHz) --------------------
void clock_init(void) {
    RCC->CR |= RCC_CR_HSEON;                               // HSE osilatörünü aç
    while (!(RCC->CR & RCC_CR_HSERDY));                    // HSE hazır olana kadar bekle

    // PLL yapılandırması: HSE = 8 MHz, PLLN = 336, PLLP = 2 → 168 MHz
    RCC->PLLCFGR = (8 << RCC_PLLCFGR_PLLM_Pos) |            // PLLM = 8
                   (336 << RCC_PLLCFGR_PLLN_Pos) |          // PLLN = 336
                   (0 << RCC_PLLCFGR_PLLP_Pos) |            // PLLP = 2 (00)
                   RCC_PLLCFGR_PLLSRC_HSE;                  // Kaynak HSE

    RCC->CR |= RCC_CR_PLLON;                               // PLL’i başlat
    while (!(RCC->CR & RCC_CR_PLLRDY));                    // PLL hazır olana kadar bekle

    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN |          // I/D cache aç
                 FLASH_ACR_PRFTEN |                         // Prefetch enable
                 FLASH_ACR_LATENCY_5WS;                     // 5 wait state

    RCC->CFGR |= RCC_CFGR_SW_PLL;                          // PLL’i sistem clock olarak seç
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL); // PLL seçilene kadar bekle
}

//-------------------- GPIO Ayarı (PA4 DAC çıkışı) --------------------
void gpio_pa4_analog_init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;                    // GPIOA clock enable
    GPIOA->MODER |= (3 << (4 * 2));                         // PA4 → Analog mode (11)
}

//-------------------- DAC Başlatma --------------------
void dac_init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;                      // DAC clock enable
    DAC->CR |= DAC_CR_EN1;                                  // DAC Channel 1 enable
}

//-------------------- DAC Değer Yazma --------------------
// value: 0 - 4095 (12-bit)
void dac_write(uint16_t value) {
    if (value > 4095) value = 4095;                         // Limit kontrolü
    DAC->DHR12R1 = value;                                   // 12-bit right-aligned data
}

//-------------------- Runtime Değer Değiştirme --------------------
void dac_set_voltage(float voltage) {
    // 3.3V referans varsayıldı → 0-3.3V arası
    uint16_t dac_val = (uint16_t)((voltage / 3.3f) * 4095);
    dac_write(dac_val);
}

//-------------------- main --------------------
int main(void) {
    clock_init();                                           // Sistem saatini ayarla
    gpio_pa4_analog_init();                                 // PA4 analog mod
    dac_init();                                             // DAC başlat

    while (1) {
        dac_set_voltage(1.65f);                             // ~1.65V çıkış
        for (volatile int i = 0; i < 1000000; i++);          // Basit delay

        dac_set_voltage(3.0f);                              // ~3.0V çıkış
        for (volatile int i = 0; i < 1000000; i++);
    }
}

