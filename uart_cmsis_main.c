#include "stm32f4xx.h"                                        // STM32F4 donanım tanımları

#define uint32_t SystemCoreClock = 168000000;                 // Sistem saat frekansını tanımla (168MHz)

void clock_init(void) {
    // 1. HSE'yi aktif et
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));  // HSE hazır olana kadar bekle

    // 2. PLL ayarları (örnek: HSE = 8 MHz → SYSCLK = 168 MHz)
    RCC->PLLCFGR = (8 << RCC_PLLCFGR_PLLM_Pos)   |  // PLLM = 8
                   (336 << RCC_PLLCFGR_PLLN_Pos) |  // PLLN = 336
                   (0 << RCC_PLLCFGR_PLLP_Pos)   |  // PLLP = 2
                   (RCC_PLLCFGR_PLLSRC_HSE)      |  // PLL kaynağı HSE
                   (7 << RCC_PLLCFGR_PLLQ_Pos);     // PLLQ = 7 (USB için)

    // 3. PLL'i aktif et
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));  // PLL hazır olana kadar bekle

    // 4. Flash ayarları (wait state)
    FLASH->ACR |= FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN;
    FLASH->ACR |= FLASH_ACR_LATENCY_5WS;  // 168 MHz için 5 WS

    // 5. AHB, APB1, APB2 prescaler
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;     // AHB = SYSCLK / 1
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;    // APB1 = HCLK / 4 (max 42 MHz)
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;    // APB2 = HCLK / 2 (max 84 MHz)

    // 6. PLL'i sistem saat kaynağı olarak seç
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);  // PLL aktif oldu mu?
}

/*

Amaç STM32F407 mikrodenetleyicisini harici osilatör (HSE) ile çalıştırıp, PLL üzerinden 168 MHz sistem saat frekansı elde etmektir.

RCC->CR: 		Clock Control Register
RCC_CR_HSEON:   HSE osilatörü (harici kristal) aktif edilir. Discovery kartta bu 8 MHz kristaldir.
RCC_CR_HSERDY:  HSE osilatörünün hazır olup olmadığını bildirir (hazır = 1)

Bu ayarlarla HSE = 8 MHz iken PLL çıkışı = 168 MHz olur
	PLLM = 8: HSE frekansı 8 MHz olduğundan, 1 MHz'e ölçekleme yapılır.
	PLLN = 336: 1 MHz × 336 = 336 MHz VCO çıkışı
	PLLP = 2: Sisteme 336 / 2 = 168 MHz verilir.
	PLLSRC = HSE: PLL kaynak olarak harici kristali kullanır.
	PLLQ = 7: USB, SDIO gibi çevresel saatlerin oluşturulmasında kullanılır. (48 MHz: 336 / 7 ≈ 48 MHz)


RCC_CR_PLLON: PLL'i etkinleştirir
RCC_CR_PLLRDY: PLL kilitlenip çıkış sinyali üretmeye başladığında 1 olur. Hazır beklenmelidir.

Yüksek saat hızlarında flash belleğe erişim süresi kritik hale gelir
	LATENCY_5WS: 168 MHz’de en az 5 wait state gereklidir (bkz: STM32F4 datasheet).
	ICEN: Instruction Cache enable
	DCEN: Data Cache enable
	PRFTEN: Prefetch buffer enable
	Bu ayarlar, flash erişimini hızlandırır ve stabilite sağlar.

Prescaler ayarları sistem saatini AHB, APB1 ve APB2 bus’larına böler
	HB: Ana veri yolu. Genellikle en yüksek frekansta çalışır. (168 MHz)
	APB1: Düşük hızlı çevresel birimler (TIM2-5, USART2, I2C1 vs.) → max 42 MHz
	APB2: Yüksek hızlı çevresel birimler (TIM1, USART1, SPI1 vs.) → max 84 MHz
	Bu bölme işlemleri, çevresel donanımların güvenli çalışmasını sağlar.

Sistem saat kaynağı seçim biti
	00 = HSI (default)
	01 = HSE
	10 = PLL
	SW: Sistem saat kaynağı seçim biti
	SWS: Seçilen sistem saat kaynağının hangi olduğunu bildirir (feedback)
	PLL seçilir ve sistem çekirdeği, tüm işlemciler 168 MHz ile çalışmaya başlar.

clock_init() fonksiyonu çalıştıktan sonra SystemCoreClock değişkeni güncellenmediği için
bazı CMSIS tabanlı delay fonksiyonları yanlış çalışabilir. Gerekirse elle güncelleyin: SystemCoreClock = 168000000;

*/

void gpio_init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;                      // GPIOA saatini aktif et

    // PA2 ve PA3: Alternate Function (AF7 = USART2)
    GPIOA->MODER &= ~((3 << (2 * 2)) | (3 << (3 * 2)));       // PA2 ve PA3 mode bitlerini temizle
    GPIOA->MODER |= ((2 << (2 * 2)) | (2 << (3 * 2)));        // PA2 ve PA3'ü Alternate Function moduna al

    GPIOA->AFR[0] &= ~((0xF << (4 * 2)) | (0xF << (4 * 3)));  // PA2 ve PA3 AF bitlerini temizle
    GPIOA->AFR[0] |= ((7 << (4 * 2)) | (7 << (4 * 3)));       // PA2 ve PA3 için AF7 (USART2) seç
}

/*

1. GPIOA Saatinin Aktifleştirilmesi:
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
		RCC (Reset and Clock Control) register'ındaki AHB1ENR (AHB1 peripheral clock enable register) kullanılıyor
		GPIOA'nın saat sinyali aktif ediliyor
		Bu yapılmazsa GPIOA registerlarına erişilemez

2. Pin Modu Ayarları (PA2 ve PA3):
	GPIOA->MODER &= ~((3 << (2 * 2)) | (3 << (3 * 2)));
		MODER register'ında her pin için 2 bit kullanılır
		PA2 için: bit 4-5, PA3 için: bit 6-7
		3 << (2 * 2) = 3 << 4 = 11 binary (bit 4-5'i temizle)
		3 << (3 * 2) = 3 << 6 = 11 binary (bit 6-7'yi temizle)
		Bu satır önce ilgili bitleri sıfırlıyor

	GPIOA->MODER |= ((2 << (2 * 2)) | (2 << (3 * 2)));
		2 << 4 = 10 binary → PA2'yi Alternate Function moduna alıyor
		2 << 6 = 10 binary → PA3'ü Alternate Function moduna alıyor
		Mod seçenekleri: 00=Input, 01=Output, 10=AF, 11=Analog

3. Alternate Function Seçimi:
	GPIOA->AFR[0] &= ~((0xF << (4 * 2)) | (0xF << (4 * 3)));
		AFR[0] register'ı pin 0-7 için alternate function seçimi
		Her pin için 4 bit kullanılır
		PA2 için: bit 8-11, PA3 için: bit 12-15
		0xF = 1111 binary, önce ilgili bitleri temizliyor

	GPIOA->AFR[0] |= ((7 << (4 * 2)) | (7 << (4 * 3)));
		PA2 için: 7 << 8 → AF7 seçiliyor
		PA3 için: 7 << 12 → AF7 seçiliyor
		AF7 = USART1/2/3 fonksiyonu

*/

void usart2_init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;                     // USART2 saatini aktif et

    // USART2 clock = APB1 = 42 MHz → 9600 baud
    USART2->BRR = 42000000 / 9600;                            // Baud rate hesapla ve ayarla (42MHz / 9600)

    USART2->CR1 |= USART_CR1_TE | USART_CR1_RE;               // Transmit ve Receive aktif et
    USART2->CR1 |= USART_CR1_UE;                              // USART2'yi aktif et
}

/*

1. USART2 Saatinin Aktifleştirilmesi:
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
		USART2, APB1 bus'ında bağlı
		APB1ENR register'ında USART2EN biti aktif ediliyor
		Bu olmadan USART2 registerlarına erişilemez

2. Baud Rate Hesaplama ve Ayarlama:
	USART2->BRR = 42000000 / 9600;
		BRR (Baud Rate Register) baud rate'i belirler
		APB1 clock = 42MHz (clock_init'te ayarlandı)
		Formül: BRR = fCK / Baud Rate
		42,000,000 / 9600 = 4375 (ondalık)
		Bu değer BRR register'ına yazılır

3. Transmit ve Receive Aktifleştirme:
	USART2->CR1 |= USART_CR1_TE | USART_CR1_RE;
		CR1 (Control Register 1) USART kontrol ayarları
		TE (Transmitter Enable) biti: Veri gönderme aktif
		RE (Receiver Enable) biti: Veri alma aktif
		Bu bitler olmadan TX/RX çalışmaz

4. USART2'yi Aktifleştirme:
	USART2->CR1 |= USART_CR1_UE;
		UE (USART Enable) biti USART2'yi tamamen aktif eder
		Bu bit son olarak açılmalı
		Tüm konfigürasyon tamamlandıktan sonra yapılır

Varsayılan Ayarlar:
	8 bit veri (default)
	1 stop bit (default)
	Parity yok (default)
	Hardware flow control yok (default)

*/

void usart2_send_char(char c) {
    while (!(USART2->SR & USART_SR_TXE));                     // TX buffer boş olana kadar bekle
    USART2->DR = c;                                           // Karakteri data register'a yaz
}

char usart2_read_char(void) {
    while (!(USART2->SR & USART_SR_RXNE));                    // RX buffer dolu olana kadar bekle
    return USART2->DR;                                        // Data register'dan karakteri oku
}

void usart2_send_string(const char *str) {
    while (*str) {                                            // String sonu kontrolü
        usart2_send_char(*str++);                             // Her karakteri sırayla gönder
    }
}

int main(void) {
    clock_init();                                             // Sistem saatini başlat
    gpio_init();                                              // GPIO pinlerini yapılandır
    usart2_init();                                            // USART2'yi başlat

    usart2_send_string("USART2 baglantisi basarili!\r\n");    // Başlangıç mesajı gönder

    while (1) {                                               // Sonsuz döngü
        char c = usart2_read_char();                          // Karakter al
        usart2_send_char(c);                                  // Echo - aldığı karakteri geri gönder
    }
}
