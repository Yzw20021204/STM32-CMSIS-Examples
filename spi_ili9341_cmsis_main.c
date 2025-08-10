#include "stm32f4xx.h"
#include "_rs_fonts.h"

/*

Bağlantılar:

	PA5 → SCK
	PA7 → MOSI
	PA4 → CS
	PA2 → DC
	PA1 → RESET
	VCC → 3.3V
	GND → GND
	LED → 3,3V

*/

#define CS_LOW()  (GPIOA->BSRR = (1 << (4 + 16)))
#define CS_HIGH() (GPIOA->BSRR = (1 << 4))
#define DC_LOW()  (GPIOA->BSRR = (1 << (2 + 16)))
#define DC_HIGH() (GPIOA->BSRR = (1 << 2))
#define RST_LOW() (GPIOA->BSRR = (1 << (1 + 16)))
#define RST_HIGH()(GPIOA->BSRR = (1 << 1))

/*

BSRR (Bit Set Reset Register): GPIO pinlerini hızlıca set/reset etmek için kullanılır
Alt 16 bit: Pin set etmek için (1 yazmak)
Üst 16 bit: Pin reset etmek için (0 yazmak)
Örnek: (1 << (4 + 16)) = Bit 20'yi set et = PA4'ü LOW yap
Avantajı: Atomik işlem, interrupt'a karşı güvenli

Pin Fonksiyonları:
	CS (Chip Select): SPI slave seçimi, LOW aktif
	DC (Data/Command): LOW=komut, HIGH=data
	RST (Reset): LCD reset pini, LOW aktif

*/

void delay_ms(uint32_t ms)  // Delay fonksiyonu
{
    for(volatile uint32_t i = 0; i < (ms * 1000); i++);
}

/*

volatile: Derleyicinin optimizasyonunu önler
Basit busy-wait: CPU'yu meşgul tutar
Gerçek zamanlama: Tam değil, yaklaşık (clock frekansına bağlı)
168 MHz'de: Her döngü ~6ns, 1000 döngü ≈ 6µs (kabaca 1ms'ye yakın)

Alternatif Çözümler:
	SysTick timer kullanımı (daha hassas)
	Hardware timer (non-blocking)

*/

void clock_init(void) {
    // 1. HSE'yi aktif et
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));

    // 2. PLL ayarları
    RCC->PLLCFGR = (8 << RCC_PLLCFGR_PLLM_Pos)   |
                   (336 << RCC_PLLCFGR_PLLN_Pos) |
                   (0 << RCC_PLLCFGR_PLLP_Pos)   |
                   (RCC_PLLCFGR_PLLSRC_HSE)      |
                   (7 << RCC_PLLCFGR_PLLQ_Pos);

    // 3. PLL'i aktif et
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    // 4. Flash ayarları
    FLASH->ACR |= FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN;
    FLASH->ACR |= FLASH_ACR_LATENCY_5WS;

    // 5. Prescaler ayarları
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;

    // 6. PLL'i sistem saat kaynağı yap
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

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

Flash Latency:
	Frekans             Wait State
	0-30 MHz 			0 WS
	30-60 MHz 			1 WS
	60-90 MHz 			2 WS
	90-120 MHz 			3 WS
	120-150 MHz 		4 WS
	150-168 MHz 		5 WS

*/

void gpio_spi_init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  // GPIOA clock enable

    // PA5 = SCK, PA7 = MOSI için Alternate Function
    GPIOA->MODER |= (2 << (5 * 2)) | (2 << (7 * 2));  // PA5, PA7
    GPIOA->AFR[0] |= (5 << (5 * 4)) | (5 << (7 * 4)); // AF5

    // PA4 = CS, PA1 = RESET, PA2 = DC için Output
    GPIOA->MODER |= (1 << (4 * 2)) | (1 << (1 * 2)) | (1 << (2 * 2));
    GPIOA->OSPEEDR |= (3 << (4 * 2)) | (3 << (1 * 2)) | (3 << (2 * 2)) |
                      (3 << (5 * 2)) | (3 << (7 * 2)); // High speed

    // İlk durumlar
    CS_HIGH();
    RST_HIGH();
    DC_HIGH();
}

/*

PA5, PA7 → SPI1 (SCK, MOSI), AF5 olarak ayarlanır.
PA4 → CS (chip select)
PA2 → DC (data/command)
PA1 → RST (reset)

Hepsi output veya AF mode olarak ayarlanmıştır.
GPIOx->OSPEEDR → Tüm pinlerde yüksek hız seçilmiştir (50 MHz+ SPI için gerekli).
BSRR kullanılarak başlangıçta pinler yüksek seviyeye çekilmiştir.

Bit hesaplaması:
	PA5 için: bit 10-11 → (5 × 2) = 10
	(2 << 10) = GPIO alternate function mode

Alternate Function Register:
	AFR[0]: Pin 0-7 için (AFRLow)
	AFR[1]: Pin 8-15 için (AFRHigh)
	AF5: SPI1/SPI4 için alternate function

Output Speed Register:
	Değer     Hız
	00Low     (2 MHz)
	01Medium  (25 MHz)
	10Fast 	  (50 MHz)
	11High    (100 MHz)

SPI için neden yüksek hız?
	Fast switching için
	Signal integrity
	EMI azaltma

*/

void spi1_init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    // SPI1 konfigürasyon
    SPI1->CR1 = SPI_CR1_MSTR   // Master mode
              | SPI_CR1_BR_1   // Baudrate = fPCLK/8 (yaklaşık 10.5 MHz)
              | SPI_CR1_SSM    // Software NSS
              | SPI_CR1_SSI;   // Internal NSS high

    SPI1->CR1 |= SPI_CR1_SPE;  // SPI enable
}

/*

MSTR → Master seçilir
BR_1 → Baud rate fPCLK/8 (APB2 = 84 MHz → SPI ≈ 10.5 MHz)
SSM/SSI → Yazılımsal NSS yönetimi yapılır.
SPE → SPI etkinleştirilir.

SPI1 Özellikler:
	APB2 bus'ında (84 MHz)
	Full-duplex
	Hardware CRC
	DMA support

MSTR (Master Mode):
	1 = Master
	0 = Slave

BR (Baud Rate):
	BR[2:0]   Prescaler    Frekans (84MHz'de)
	000       /2           42 MHz
	001       /4           21 MHz
	010       /8           10.5 MHz
	011       /16          5.25 MHz

	BR_1 = 010: /8 prescaler → 10.5 MHz

SSM/SSI (Software Slave Management):
	SSM=1: Software NSS control
	SSI=1: Internal NSS high (master mode için)

Neden ayrı enable?
	Configuration tamamlandıktan sonra aktif edilir.

*/

void spi1_send(uint8_t data) {
    while (!(SPI1->SR & SPI_SR_TXE)); // TX buffer boş mu?
    *(volatile uint8_t *)&SPI1->DR = data;
    while (!(SPI1->SR & SPI_SR_RXNE)); // RX tamamlandı mı?
    (void)SPI1->DR; // RX buffer'ı temizle
    while (SPI1->SR & SPI_SR_BSY); // Transfer bitti mi?
}

/*

TXE flag’i set olana kadar bekler (gönderme tamponu boş).
Veriyi DR register’ına yazar.
RXNE kontrolü ile alım da bitmiş mi kontrol edilir (SPI çift yönlüdür).
BSY → Transfer tamamlanmadan çıkmaz.

TXE (Transmit Buffer Empty):
	0 = TX buffer full
	1 = TX buffer empty, yeni data gönderilebilir

RXNE (Receive Buffer Not Empty):
	0 = RX buffer empty
	1 = RX buffer'da data var

BSY (Busy Flag):
	0 = SPI idle
	1 = SPI communication ongoing

Transfer Sequence:
	TXE bekle: TX buffer boş olana kadar
	Data yaz: DR register'a 8-bit data
	RXNE bekle: Transfer tamamlanana kadar
	Dummy read: RX buffer'ı temizle
	BSY bekle: Tüm bit'ler gönderilene kadar

Neden dummy read?
	SPI full-duplex, her gönderimde bir data alır. ILI9341 MISO kullanmadığı için dummy data gelir.

*/

void ili9341_send_cmd(uint8_t cmd) {
    DC_LOW();  // Command mode
    CS_LOW();
    spi1_send(cmd);
    CS_HIGH();
}

/*

DC_LOW() → Komut gönderileceğini belirtir.
DC_HIGH() → Veri gönderileceğini belirtir.
CS_LOW()/CS_HIGH() → TFT modülüne erişim.
ili9341_send_cmd(uint8_t cmd), ili9341_send_data(uint8_t data) Bu iki fonksiyon, tüm diğer çizim/ayar fonksiyonlarının temel yapı taşını oluşturur.

ILI9341 Protocol:
	4-wire SPI: SCK, MOSI, CS, DC
	DC pin: Data/Command seçimi
		LOW = Command
		HIGH = Data
	CS pin: Chip select (active LOW)

*/

void ili9341_send_data(uint8_t data) {
    DC_HIGH(); // Data mode
    CS_LOW();
    spi1_send(data);
    CS_HIGH();
}

/*
DC_LOW() → Komut gönderileceğini belirtir.
DC_HIGH() → Veri gönderileceğini belirtir.
CS_LOW()/CS_HIGH() → TFT modülüne erişim.
ili9341_send_cmd(uint8_t cmd), ili9341_send_data(uint8_t data) Bu iki fonksiyon, tüm diğer çizim/ayar fonksiyonlarının temel yapı taşını oluşturur.

ILI9341 Protocol:
	4-wire SPI: SCK, MOSI, CS, DC
	DC pin: Data/Command seçimi
		LOW = Command
		HIGH = Data
	CS pin: Chip select (active LOW)

*/

void ili9341_reset(void) {
    RST_LOW();
    delay_ms(10);
    RST_HIGH();
    delay_ms(120);
}

/*

RST_LOW() → Donanım reset pinini kısa süre sıfıra çeker.
delay_ms(...) → Uygun bekleme süreleri datasheette belirtilen şekilde uygulanır.

Reset Timing (Datasheet):
	Reset LOW: Minimum 10µs
	Reset HIGH: 120ms wait (internal initialization)

Neden reset gerekli?
	Power-on durumunda LCD undefined state'de
	Reset ile bilinen duruma getiriyor

*/

void ili9341_init(void) {
    ili9341_reset();

    // Software Reset
    ili9341_send_cmd(0x01);
    delay_ms(120);

    // Power Control A
    ili9341_send_cmd(0xCB);
    ili9341_send_data(0x39);
    ili9341_send_data(0x2C);
    ili9341_send_data(0x00);
    ili9341_send_data(0x34);
    ili9341_send_data(0x02);

    // Power Control B
    ili9341_send_cmd(0xCF);
    ili9341_send_data(0x00);
    ili9341_send_data(0xC1);
    ili9341_send_data(0x30);

    // Driver Timing Control A
    ili9341_send_cmd(0xE8);
    ili9341_send_data(0x85);
    ili9341_send_data(0x00);
    ili9341_send_data(0x78);

    // Driver Timing Control B
    ili9341_send_cmd(0xEA);
    ili9341_send_data(0x00);
    ili9341_send_data(0x00);

    // Power on Sequence Control
    ili9341_send_cmd(0xED);
    ili9341_send_data(0x64);
    ili9341_send_data(0x03);
    ili9341_send_data(0x12);
    ili9341_send_data(0x81);

    // Pump ratio control
    ili9341_send_cmd(0xF7);
    ili9341_send_data(0x20);

    // Power Control 1
    ili9341_send_cmd(0xC0);
    ili9341_send_data(0x23);

    // Power Control 2
    ili9341_send_cmd(0xC1);
    ili9341_send_data(0x10);

    // VCOM Control 1
    ili9341_send_cmd(0xC5);
    ili9341_send_data(0x3E);
    ili9341_send_data(0x28);

    // VCOM Control 2
    ili9341_send_cmd(0xC7);
    ili9341_send_data(0x86);

    // Memory Access Control
    ili9341_send_cmd(0x36);
    ili9341_send_data(0x48); // MX, BGR

    // Pixel Format
    ili9341_send_cmd(0x3A);
    ili9341_send_data(0x55); // 16 bit

    // Frame Rate Control
    ili9341_send_cmd(0xB1);
    ili9341_send_data(0x00);
    ili9341_send_data(0x18);

    // Display Function Control
    ili9341_send_cmd(0xB6);
    ili9341_send_data(0x08);
    ili9341_send_data(0x82);
    ili9341_send_data(0x27);

    // 3Gamma Function Disable
    ili9341_send_cmd(0xF2);
    ili9341_send_data(0x00);

    // Gamma curve selected
    ili9341_send_cmd(0x26);
    ili9341_send_data(0x01);

    // Set Gamma
    ili9341_send_cmd(0xE0);
    ili9341_send_data(0x0F);
    ili9341_send_data(0x31);
    ili9341_send_data(0x2B);
    ili9341_send_data(0x0C);
    ili9341_send_data(0x0E);
    ili9341_send_data(0x08);
    ili9341_send_data(0x4E);
    ili9341_send_data(0xF1);
    ili9341_send_data(0x37);
    ili9341_send_data(0x07);
    ili9341_send_data(0x10);
    ili9341_send_data(0x03);
    ili9341_send_data(0x0E);
    ili9341_send_data(0x09);
    ili9341_send_data(0x00);

    // Set Gamma
    ili9341_send_cmd(0xE1);
    ili9341_send_data(0x00);
    ili9341_send_data(0x0E);
    ili9341_send_data(0x14);
    ili9341_send_data(0x03);
    ili9341_send_data(0x11);
    ili9341_send_data(0x07);
    ili9341_send_data(0x31);
    ili9341_send_data(0xC1);
    ili9341_send_data(0x48);
    ili9341_send_data(0x08);
    ili9341_send_data(0x0F);
    ili9341_send_data(0x0C);
    ili9341_send_data(0x31);
    ili9341_send_data(0x36);
    ili9341_send_data(0x0F);

    // Exit Sleep
    ili9341_send_cmd(0x11);
    delay_ms(120);

    // Display ON
    ili9341_send_cmd(0x29);
    delay_ms(120);
}

/*

ILI9341 kontrolcüsünü başlatır, güç yönetimi, gamma ayarları, piksel formatı gibi tüm temel parametreleri konfigüre eder.
Bu kısım doğrudan ILI9341 datasheet’teki önerilen init sekansıdır.

Örn: 0x36 (Memory Access Control) → Görüntü yönü ve renk sıralaması.
	 0x3A → RGB565 seçimi için 0x55 (16-bit renk)
	 0x11 → Sleep mode çıkışı
	 0x29 → Ekranı aktif et

*/

void ili9341_set_address(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
    ili9341_send_cmd(0x2A); // Column address
    ili9341_send_data(x1 >> 8);
    ili9341_send_data(x1 & 0xFF);
    ili9341_send_data(x2 >> 8);
    ili9341_send_data(x2 & 0xFF);

    ili9341_send_cmd(0x2B); // Page address
    ili9341_send_data(y1 >> 8);
    ili9341_send_data(y1 & 0xFF);
    ili9341_send_data(y2 >> 8);
    ili9341_send_data(y2 & 0xFF);

    ili9341_send_cmd(0x2C); // Memory write
}

/*

Görüntü belleğinde hangi koordinatlara yazılacağını belirler.
0x2A: Sütun (X) aralığı
0x2B: Satır (Y) aralığı
0x2C: Belleğe yazma başlat (memory write)
Bu komutlar olmadan TFT’ye doğru konumda veri yazmak mümkün değildir.


Address System:
	X koordinat: 0-239 (240 pixel)
	Y koordinat: 0-319 (320 pixel)
	16-bit adres: Big-endian format
	Window: (x1,y1) to (x2,y2) rectangle

Command Sequence:
	0x2A: Column address set
	4 byte: X start (2 byte) + X end (2 byte)
	0x2B: Page address set
	4 byte: Y start (2 byte) + Y end (2 byte)
	0x2C: Memory write command

Sonraki data'lar: Pixel data olarak yazılır

*/

void ili9341_draw_pixel(uint16_t x, uint16_t y, uint16_t color) {
    ili9341_set_address(x, y, x, y);
    DC_HIGH(); // Data mode
    CS_LOW();
    spi1_send(color >> 8);    // High byte first
    spi1_send(color & 0xFF);  // Low byte second
    CS_HIGH();
}

/*

Belirli bir koordinata tek bir renkli piksel çizer.
set_address(...) ile konum seçilir.
İki byte veri gönderilir: 16-bit RGB565 formatında renk.

Color = 0xF800 (Red)
Binary: 1111100000000000
        RRRRR GGGGGG BBBBB
        31    0      0

Byte Sırası: Big-endian
	İlk byte: color >> 8 (üst 8 bit)
	İkinci byte: color & 0xFF (alt 8 bit)

*/

void ili9341_fill_screen(uint16_t color) {
    ili9341_set_address(0, 0, 239, 319);
    DC_HIGH();
    CS_LOW();
    for(uint32_t i = 0; i < (240 * 320); i++) {
        spi1_send(color >> 8);
        spi1_send(color & 0xFF);
    }
    CS_HIGH();
}

/*

Tüm ekranı verilen bir renkle doldurur.
set_address(0, 0, 239, 319) → Tam ekran alanı seçilir.
Ardından 240x320 adet 2 byte veri gönderilir.
Bu fonksiyon ekranda temizleme ve arka plan hazırlamada temel fonksiyondur.

Optimizasyon:
	Tek address setting: Tüm ekran için
	Sequential write: Address otomatik artıyor
	CS LOW tutma: Transfer hızlandırma
	Total pixels: 240 × 320 = 76,800

Transfer süresi:
	76,800 pixel × 2 byte × 8 bit = 1,228,800 bit
	10.5 MHz SPI'de ≈ 117ms

*/

void ili9341_draw_char(uint16_t x, uint16_t y, char c, uint16_t color, uint16_t bg) {
    if (c < 0x20 || c > 0x7F) return;

    const uint8_t *chr = font5x7[c - 0x20];

    for (int i = 0; i < 5; i++) {
        uint8_t line = chr[i];
        for (int j = 0; j < 8; j++) {
            uint16_t pixel_color = (line & (1 << j)) ? color : bg;
            ili9341_draw_pixel(x + i, y + j, pixel_color);
        }
    }
}

/*

5x7 font kullanarak ekrana bir karakter çizer.
font5x7[] tablosundan alınan karakter bitmap'i dikey olarak satır satır yazdırılır.
pixel_color → Font pikseli varsa yazı rengi, yoksa arka plan.
Her karakter 5 sütun ve 8 satır (daha çok 7 aktif satır) olarak çizilir.

Font System:
	Font boyutu: 5×7 pixel + 1 boşluk = 6×8
	ASCII aralık: 0x20 (space) to 0x7F (~)
	Font array: font5x7[95][5]
	Index hesabı: c - 0x20

Bit Pattern (column by column):
	uint8_t font5x7[1][5] = {0x3E, 0x51, 0x49, 0x45, 0x3E}; // '0' karakter
	Column 0: 0x3E = 00111110
	Column 1: 0x51 = 01010001
	Column 2: 0x49 = 01001001
	Column 3: 0x45 = 01000101
	Column 4: 0x3E = 00111110

*/

void ili9341_draw_string(uint16_t x, uint16_t y, const char *str, uint16_t color, uint16_t bg) {
    while (*str) {
        ili9341_draw_char(x, y, *str++, color, bg);
        x += 6; // 5 piksel + 1 boşluk
    }
}

/*

Bir string’i karakter karakter ekrana yazdırır.
x += 6 → 5 piksel karakter + 1 piksel boşluk
Karakter sınırı ASCII 0x20 - 0x7F ile sınırlıdır.

*/


int main(void) {

    clock_init();
    gpio_spi_init();
    spi1_init();

    // Ekranı temizle
    ili9341_init();
    ili9341_fill_screen(0x0000); // Siyah arka plan

    // Test yazısı
    ili9341_draw_string(50, 60, "Merhaba Talha Bey???", 0xF800, 0x0000); // Kırmızı yazı, siyah arka plan

    while (1) {
        // Ana döngü
    }
}
