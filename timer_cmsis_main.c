
#include <stdint.h>
#include "stm32f4xx.h"

uint32_t SystemCoreClock = 168000000;

/* ---------- Prototipler ---------- */
void clock_init(void);
void gpio_pd12_init(void);
void tim2_1hz_init(void);
void NMI_Handler(void);
int main(void);


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


void gpio_pd12_init(void)
{
	/* Enable clock for GPIOD */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;        // GPIOD portu için saat sinyalini etkinleştir
	/* Small delay to ensure clock enabled (read back) */
	(void)RCC->AHB1ENR;                         // Saat sinyalinin etkinleştiğinden emin olmak için geri okuma yap

	/* PD12 -> General purpose output (MODER = 01) */
	/* Clear bits then set */
	GPIOD->MODER &= ~(0x3 << (12 * 2));         // PD12 pini için mod bitlerini temizle (24-25. bitler)
	GPIOD->MODER |= (0x1 << (12 * 2));          // PD12 pinini genel amaçlı çıkış (01) olarak ayarla

	/* Output type: push-pull (OTYPER = 0) -> default is 0, ensure it */
	GPIOD->OTYPER &= ~(1 << 12);                // PD12 pinini push-pull çıkış tipi olarak ayarla (0)

	/* Speed: medium/high (choose as needed). Set OSPEEDR = 10 for medium-high */
	GPIOD->OSPEEDR &= ~(0x3 << (12 * 2));       // PD12 pini için hız bitlerini temizle
	GPIOD->OSPEEDR |= (0x2 << (12 * 2));        // PD12 pini için orta-yüksek hız (10) ayarla

	/* No pull-up/pull-down */
	GPIOD->PUPDR &= ~(0x3 << (12 * 2));         // PD12 pini için pull-up/pull-down dirençlerini devre dışı bırak

	/* Start with LED off */
	GPIOD->BSRR = (1 << (12 + 16));             // PD12 pinini sıfırla (LED'i kapat)
}

/*

RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	Amaç: GPIOD çevresinin clock’unu açar.
	Neden gerekli: RCC (Reset and Clock Control) altında AHB1 bus üzerindeki her GPIO portu için ayrı clock enable biti vardır. Clock kapalıysa o portun register’larına erişmek çalışmaz veya default reset modunda kalır.
	Bit konumu:
		RCC_AHB1ENR_GPIODEN = (1 << 3) (Port A=0, B=1, C=2, D=3).
		Bu bit 1 yapılınca GPIOD aktif olur.

(void)RCC->AHB1ENR;
	Amaç: “Dummy read” işlemi.
	Neden: Bazı işlemcilerde clock enable işleminin etkili olması için bus senkronizasyonu gerekir. Okuma işlemi yazılan değerin gerçekten set edildiğini garanti eder.
	Bu satır olmadan da genellikle çalışır ama güvenlik için kullanılır.

GPIOD->MODER &= ~(0x3 << (12 * 2));
GPIOD->MODER |=  (0x1 << (12 * 2));
	Amaç: PD12 pininin mode ayarını yapmak.
	MODER register’ı: Her pin için 2 bitlik alan vardır:
		00 = Giriş
		01 = Genel amaçlı çıkış (output)
		10 = Alternatif fonksiyon (AF)
		11 = Analog
	İşlem:
		Önce &= ~ ile 12. pinin 2 bitlik alanını sıfırlıyoruz ((12*2) offset).
		Sonra |= ile 01 yazarak çıkış moduna alıyoruz.

GPIOD->OTYPER &= ~(1 << 12);
	Amaç: Çıkış tipini push-pull yapmak.
	OTYPER register’ı:
		0 = Push-pull (default, iki yönlü sürücü)
		1 = Open-drain
	PD12 için: Bit 12’yi 0 yaparak push-pull seçtik.

GPIOD->OSPEEDR &= ~(0x3 << (12 * 2));
GPIOD->OSPEEDR |=  (0x2 << (12 * 2));
	Amaç: Çıkış hızını belirlemek.
	OSPEEDR register’ı: Her pin için 2 bit:
		00 = Düşük hız
		01 = Orta hız
		10 = Yüksek hız
		11 = Çok yüksek hız
	Seçim: 10 (yüksek hız) yaptık. LED gibi yavaş yüklerde fark etmez ama hızlı değişen sinyallerde önemli olur.

GPIOD->PUPDR &= ~(0x3 << (12 * 2));
	Amaç: Pull-up / Pull-down devrelerini kapatmak.
	PUPDR register’ı: Her pin için 2 bit:
		00 = Pull-up yok / Pull-down yok
		01 = Pull-up aktif
		10 = Pull-down aktif
	Seçim: 00 yaparak pasif bıraktık. Çünkü çıkış modunda bu dirençler gereksiz.

GPIOD->BSRR = (1 << (12 + 16));
	Amaç: LED’i başlangıçta kapatmak.
	BSRR register’ı:
		Alt 16 bit (0–15) → pin set (1 yapılırsa pin HIGH)
		Üst 16 bit (16–31) → pin reset (1 yapılırsa pin LOW)
	İşlem: (12+16) ile 28. biti 1 yapıyoruz → PD12 LOW olur (LED söner).
	Avantaj: BSRR’ye yazmak atomik bir işlemdir. ODR register’ı gibi read-modify-write gerektirmez, yarış durumu (race condition) riski düşüktür.

*/

void tim2_1hz_init(void)
{

	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;         // TIM2 timer'ı için saat sinyalini etkinleştir (APB1 veri yolu)
	(void)RCC->APB1ENR;                         // Saat sinyalinin etkinleştiğinden emin olmak için geri okuma yap

	/* Assumptions:
	SystemCoreClock = 168000000
	APB1 prescaler = 4 -> APB1 clock = 42 MHz
	Timer clock for TIM2 = APB1 * 2 = 84 MHz
	Target: 1 Hz update (periodic)
	*/
	uint32_t timer_clk = SystemCoreClock / 2;   // Timer saat frekansını hesapla (168MHz/2 = 84MHz)
	uint32_t desired_hz = 1U;                   // Hedef frekansı 1 Hz olarak ayarla

	/* Choose prescaler to bring counter clock down to something manageable:
	e.g. prescaler = 8400-1 -> prescaled clock = 10000 Hz; then ARR = 10000-1 => 1s
	*/
	uint32_t prescaler = 8400 - 1;              // Ön bölücü değerini hesapla (8400-1 = 8399)
	uint32_t arr = (timer_clk / (prescaler + 1)) / desired_hz - 1; // Otomatik yeniden yükleme değerini hesapla

	TIM2->PSC = prescaler;                      // Ön bölücü kaydedicisini ayarla (PSC)
	TIM2->ARR = arr;                           // Otomatik yeniden yükleme kaydedicisini ayarla (ARR)

	TIM2->DIER |= TIM_DIER_UIE;                // Güncelleme kesme işlemini etkinleştir

	TIM2->CR1 |= TIM_CR1_CEN;                  // Sayıcıyı etkinleştir (yukarı sayma modu)

	NVIC_SetPriority(TIM2_IRQn, 2U);          // TIM2 kesmesi için öncelik seviyesini 2 olarak ayarla
	NVIC_EnableIRQ(TIM2_IRQn);                // TIM2 kesmesini NVIC'de etkinleştir
}

/*

TIM2 saatini açmak, prescaler/ARR ile 1 Hz olacak şekilde yapılandırmak, güncelleme kesmesini etkinleştirmek ve NVIC’de kesmeyi açmak.

1) RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	Ne yapar: TIM2 çevresinin (peripheral) clock’unu açar. TIM2 registlerine yazmadan önce clock aktif olmalı, aksi halde yazılan değerler etkisiz kalır.
	Neden okuma (readback) yapılır: (void)RCC->APB1ENR; gibi bir okuma, bus’ta clock enable yazmasının fiziksel olarak tamamlanmasını garanti etmeye yardımcı olur (birkaç örnek ve uygulama tavsiyesi olarak kullanılır).

2) Timer clock hesaplaması — neden 84 MHz kullandık?
	Varsayım: SystemCoreClock = 168_000_000 Hz ve APB1 prescaler = 4 (tipik 168 MHz konfigürasyonunda).
	Adım adım:
		PCLK1 = SystemCoreClock / APB1_prescaler
			168_000_000 / 4 = 42_000_000 (çünkü 168 / 4 = 42, sonra üç çift 000 eklenir).
		TIM2 gibi APB1 üzerindeki timer’lar için kural: eğer APB1 prescaler ≠ 1 ise timer clock = PCLK1 * 2.
			42_000_000 * 2 = 84_000_000 Hz.
	Sonuç: timer_clk = 84_000_000 Hz (bu değeri kodda timer_clk = SystemCoreClock / 2; ile kestirmeci olarak aldık).

3) PSC (prescaler) ve ARR seçimi (1 Hz için)
	PSC register’ı yazılan değer +1 ile bölme yapar (yani PSC_reg = N-1 ise gerçek bölme faktörü N).
	Örnek seçim:
		Prescale faktörü = 8400 → yazılacak PSC = 8400 - 1 = 8399.
		Hesap: prescaled_clk = timer_clk / 8400
			84_000_000 / 8_400 = 10_000 Hz (84 / 8.4 = 10 → yine sıfırlarla uyumlu).
		İstediğimiz frekans = desired_hz = 1 Hz, bu yüzden ARR = prescaled_clk / desired_hz - 1
			ARR = 10_000 / 1 - 1 = 9_999.
	Önemli notlar:
		PSC register’ı 16-bit’dir (max 65_535). TIM2 bir 32-bit sayaçtır; ARR değeri TIM2 için 32-bit genişliğe sahiptir.
		PSC ve ARR kayıtları 0-bazlıdır (yani istenen bölme/period için -1 kuralı uygulanır).

4) Register yazma sırası ve anında güncelleme
	Güvenli sıra önerisi:
		TIM2 kapalıyken PSC/ARR yaz.
		ARPE (auto-reload preload) gerekiyorsa açıkla (TIM2->CR1 |= TIM_CR1_ARPE;) — böylece ARR shadow register’a yazılır.
		Eğer hemen PSC/ARR değeri aktif olsun istersen TIM2->EGR = TIM_EGR_UG; (UG = update generation). Bu satır PSC/ARR’ın gölge kayıtlardan ana sayaça kopyalanmasını zorlar (immediate reload).
		TIM2->PSC = prescaler; TIM2->ARR = arr; yazıldıktan sonra TIM2->DIER |= TIM_DIER_UIE; ve TIM2->CR1 |= TIM_CR1_CEN; ile başlatılıyor. Eğer ARPE set edilmemişse veya hemen etki isteniyorsa EGR = UG kullanılması uygundur.

5) TIM2->DIER |= TIM_DIER_UIE;
	Ne yapar: Update interrupt enable bit’ini açar. Bu bit kapalıysa timer overflow olsa bile NVIC’ye IRQ gelmez.
	DIER içinde başka maskeler (ör. CCx interrupt) de bulunur; sadece güncelleme (overflow/update) için UIE açıldı.

6) Sayacı başlatma: TIM2->CR1 |= TIM_CR1_CEN;
	Sayaç çalışmaya başlar (upcounting). Eğer CR1 içinde CEN bit’i set edilmeden önce PSC/ARR shadow mekanizmaları doğru şekilde yüklenmişse istenen periyot doğru olur.

7) NVIC ayarı
	NVIC_SetPriority(TIM2_IRQn, 2U); — öncelik atanır. Cortex-M4’te daha küçük sayı = daha yüksek öncelik.
		(STM32F4’te tipik implementasyon 4 bit priority kullanır → 0..15 aralığı olabilir; proje NVIC grup/alt-bölüm ayarlarına bağlıdır.)
	NVIC_EnableIRQ(TIM2_IRQn); — NVIC’de IRQ hattı etkinleştirilir.
	Sıralama tavsiyesi:
		Kesme konfigürasyonu yapılırken (DIER/CR1) NVIC’in en son etkinleştirilmesi güvenlidir; aksi halde kısa süreli yanlış tetiklemeler (spurious) görülebilir.

*/

void TIM2_IRQHandler(void)
{

	if (TIM2->SR & TIM_SR_UIF) {              // Güncelleme kesme bayrağının set olup olmadığını kontrol et

		TIM2->SR &= ~TIM_SR_UIF;               // Güncelleme kesme bayrağını temizle
		GPIOD->ODR ^= (1 << 12);               // PD12 pininin durumunu tersine çevir (LED'i aç/kapat)

	}
}

/*

ISR içinde hangi kontrol/temizleme/işlemler neden ve hangi sırayla yapılmalı.

1) Bayrak kontrolü: if (TIM2->SR & TIM_SR_UIF) { ... }
	Ne kontrol ediliyor: SR register’ındaki UIF (Update Interrupt Flag). Timer overflow/update olduğunda fiziksel olarak bu bit set olur.
	ISR çağrıldıysa muhtemelen bu bit set’dir; yine de doğrulama iyi pratiktir (IRQ hattı aynı MPU’nun başka kaynaklardan da çağrılabiliyor olabilir).

2) Bayrağı temizleme: TIM2->SR &= ~TIM_SR_UIF;
	Neden temizlenir: UIF temizlenmezse NVIC tekrar aynı IRQ için sürekli tetikleyebilir.
	Ne zaman temizlenmeli: Genelde erken temizlemek tavsiye edilir — yani uzun işlem yapmadan önce çağrı üstünde bayrağı clear etmelisiniz. Böylece işlem uzun sürse bile yeni bir update gelirse ISR tekrar kuyruklanabilir veya kaybolmaz (uygun öncelik/konfigüre göre).
	Alternatif/öneri: ARPE kullanılıp EGR = UG ile kontrol edilen yerlerde farklı davranış olabilir; fakat normalde SR’ı temizleme (&= ~TIM_SR_UIF) yaygın pratiktir.
	Not: bazı kaynaklar TIMx->SR = 0; tarzı kullanmaktan kaçınır çünkü SR register’ında başka anlamlı flag’ler olabilir — sadece ilgili biti temizlemek en güvenlidir.

3) İş (LED toggle): GPIOD->ODR ^= (1 << 12);
	Ne yapar: PD12 ODR bitini XOR ile tersler (LED toggle).
	Dikkat edilmesi gerekenler:
		ODR’ye read-modify-write ile yazma yapmak ana kod ile yarışma (race condition) durumuna yol açabilir; eğer ana kod aynı pin üzerinde farklı değişiklikler yapıyorsa kilitlenme/uyumsuzluk olabilir.
		Daha atomik şekilde pin set/reset için BSRR kullanılabilir (set: BSRR = (1<<n), reset: BSRR = (1<<(n+16))), fakat bir “toggle” için tek bir BSRR yazımı yoktur — toggle yapmak için ya ODR ^= mask ya da özel bir hardware toggle register (varsa) gerekir.
		ISR içinde kısa tutulmalıdır — LED toggle idealdir (çok kısa).

4) ISR’in kısa tutulması ve yan işler
	Kural: ISR içinde ağır işi yapmayın. ISR sadece bayrağı temizlesin ve gerekiyorsa bir volatile flag ayarlasın; ağır iş (ör.UART, disk, uzun hesaplama) ana döngüde işlenmelidir.
	Örnek: volatile uint8_t led_tick = 0; ve ISR’de led_tick = 1; okuyup ana döngüde işleme.

5) Re-entrancy / tekrar tetikleme
	Eğer clear geç yapılırsa veya çok kısa sürede yeni update gelirse ISR tekrar hemen çağrılabilir. Bu isteniyorsa sorun yok; değilse interrupt frekansını düşürün veya handler’da durum kontrolü yapın.

*/

int main(void)
{

	clock_init();
    gpio_pd12_init(); /* Initialize GPIO PD12 */
    tim2_1hz_init();  /* Initialize TIM2 for 1 Hz update interrupts */

    while (1)
    {
        __WFI();  /* Wait For Interrupt - puts core to sleep until an interrupt occurs */
    }

    return 0;
}

/*

İyileştirme önerileri (pratik ipuçları)
	EGR = UG: PSC/ARR’ı yazdıktan sonra TIM2->EGR = TIM_EGR_UG; kullanarak yeni değerlerin anında yansıtılmasını sağlayın.
	ARPE: TIM2->CR1 |= TIM_CR1_ARPE; ile auto-reload preload etkinleştirilebilir; böylece ARR yazımı shadow register’a gider ve güncelleme UG ile yapılır.
	Atomic LED toggle: Eğer ana kod ODR’yi değiştiriyorsa birlikte olası yarış durumunu kaldırmak için LED toggle yerine ISR ana koda flag koysun.
	NVIC priority planlama: Eğer birden fazla IRQ varsa, öncelik stratejisini proje gereksinimine göre (kritik zamanlı işler yüksek öncelik) tasarlayın.
	Sağlama okuması: Clock enable sonrası volatile read-back yapın: (void)RCC->APB1ENR; — bazı platformlarda tavsiye edilir.

*/
