
#include <stdint.h>

#include "stm32f4xx.h"  // CMSIS başlık dosyası

void clock_init(void) {
    // HSE başlat
    RCC->CR |= RCC_CR_HSEON;                          // HSE osilatörünü aktif et
    while (!(RCC->CR & RCC_CR_HSERDY));               // HSE hazır olana kadar bekle

    // PLL konfigürasyonu (HSE = 8 MHz, SYSCLK = 168 MHz)
    RCC->PLLCFGR = (8 << RCC_PLLCFGR_PLLM_Pos) |       // PLLM = 8
                   (336 << RCC_PLLCFGR_PLLN_Pos) |     // PLLN = 336
                   (0 << RCC_PLLCFGR_PLLP_Pos) |       // PLLP = 2 (00)
                   (RCC_PLLCFGR_PLLSRC_HSE) |          // PLL kaynağı = HSE
                   (7 << RCC_PLLCFGR_PLLQ_Pos);        // PLLQ = 7 (USB için 48 MHz)

    RCC->CR |= RCC_CR_PLLON;                           // PLL'i aktif et
    while (!(RCC->CR & RCC_CR_PLLRDY));                // PLL hazır olana kadar bekle

    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN |     // Flash önbellekleri aktif et
                 FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_5WS; // 5 wait state

    RCC->CFGR |= RCC_CFGR_SW_PLL;                      // PLL'i sistem clock olarak seç
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL); // PLL'e geçişi bekle
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
    // GPIOD ve GPIOA clocklarını aç
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;               // D portu clock'u
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;               // A portu clock'u

    // PD12 LED çıkış konfigürasyonu
    GPIOD->MODER |= (1 << (12 * 2));                   // PD12 output mode
    GPIOD->OTYPER &= ~(1 << 12);                       // Push-pull
    GPIOD->OSPEEDR |= (3 << (12 * 2));                 // High speed
    GPIOD->PUPDR &= ~(3 << (12 * 2));                  // Pull-up/down yok

    // PA0 giriş konfigürasyonu (Buton)
    GPIOA->MODER &= ~(3 << (0 * 2));                   // PA0 input mode
    GPIOA->PUPDR &= ~(3 << (0 * 2));                   // Pull-up/down yok
}

/*

RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	Ne yapar: GPIOD için saat (clock) enable bitini set eder. GPIO register’larına erişebilmek için ilgili portun clock’unun açık olması gerekir.
	Neden önemli: Clock kapalıysa yazılan register değerleri etkisiz kalır (ignored).
	Not: |= ile set etme güvenlidir (diğer AHB1ENR bitlerini bozmaz). Bazı projelerde volatile read-back ((void)RCC->AHB1ENR;) ile senkronizasyon yapılır; bunu öneririm.

RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	Ne yapar: GPIOA için clock’u açar. (PA0 buton için gerekli.)

GPIOD->MODER |= (1 << (12 * 2));
	Ne yapar: PD12 için MODER bitlerinin 01 olacak şekilde (output) en azından düşük biti 1 yapar.
	Dikkat: Bu yazım önce temizleme yapmadan |= ile doğrudan 1 yazıyor; eğer önceki konfigürasyonda ilgili üst bit set ise MODER[25:24] = 11 gibi istenmeyen durum oluşabilir. En güvenli yöntem: önce GPIOD->MODER &= ~(3 << (12*2)); ile temizleyip sonra GPIOD->MODER |= (1 << (12*2)); ile 01 yazmaktır.
	Özet: Bu satırın amacı PD12'yi genel amaçlı çıkış yapmak.

GPIOD->OTYPER &= ~(1 << 12);
	Ne yapar: PD12 çıkış tipini push-pull (0) yapar. 1 olursa open-drain olur. Push-pull LED gibi sürmeler için uygundur.

GPIOD->OSPEEDR |= (3 << (12 * 2));
	Ne yapar: PD12 için çıkış hızını çok yüksek (11) olarak ayarlar. LED için bu genelde gereksiz ama sinyal kenarlarının hızlı olmasını istiyorsanız uygundur.
	Not: Daha düşük hız (01 veya 10) ile EMI ve güç tüketimi azaltılabilir.

GPIOD->PUPDR &= ~(3 << (12 * 2));
	Ne yapar: PD12 için pull-up/pull-down kapatılır (00). Çıkış pini için genelde pull'lara gerek yoktur.

GPIOA->MODER &= ~(3 << (0 * 2));
	Ne yapar: PA0 için MODER’ı 00 (input) yapar. (Buton için input modu.)

GPIOA->PUPDR &= ~(3 << (0 * 2));
	Ne yapar: PA0 için pull-up/pull-down yok (floating).
	Önemli uyarı:
		Eğer kart üzerindeki buton devresi harici bir pull-down/pull-up içermiyorsa PA0 floating (kararsız) kalır.
		Bu durumda dahili pull-up veya pull-down kullanılmalı (GPIOA->PUPDR |= (1<< (0*2)); pull-up veya |= (2 << (0*2)); pull-down).
		Discovery kartında fiziksel devrenin durumuna göre internal pull konfigrasyonu değiştirilmeli.

Ek pratik notlar / öneriler:
	Clock enable sonrası kısa read-back ((void)RCC->AHB1ENR;) eklemek bus synchronisation için faydalıdır.
	MODER ayarı yaparken önce ilgili 2 biti temizlemek (&= ~mask) ve sonra set etmek (|= value) en güvenli yöntemdir. Bu, istenmeyen bit kalıntılarını engeller.
	LED toggling için ISR’de GPIOD->ODR ^= (1<<12) kullandınız; bu RMW (read-modify-write) işlemi olabilir.
	Eğer ana kod da aynı pini değiştirecekse yarış durumları olur.
	Atomik set/reset için BSRR kullanmak (set veya reset ayrı yazma) daha güvenlidir; toggle için donanım destek yoksa ISR tek değiştiren olmalı veya kritik bölüm kullanılmalı.

*/

void exti0_init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;              // SYSCFG clock'unu aç

    SYSCFG->EXTICR[0] &= ~(0xF << 0);                  // EXTI0'ı PA0'a bağla
    SYSCFG->EXTICR[0] |= (0x0 << 0);                   // PA0 seçildi

    EXTI->IMR |= (1 << 0);                             // EXTI0 mask enable
    EXTI->RTSR |= (1 << 0);                            // Rising edge tetikle
    EXTI->FTSR &= ~(1 << 0);                           // Falling edge disable

    NVIC_SetPriority(EXTI0_IRQn, 2);                   // Öncelik
    NVIC_EnableIRQ(EXTI0_IRQn);                        // EXTI0 kesmesini aktif et
}

/*

RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	Ne yapar: SYSCFG (System Configuration Controller) çevresinin clock’unu açar. SYSCFG, EXTI hattı ile hangi GPIO portunun ilişkilendirileceğini belirleyen EXTICR register’larına sahiptir. SYSCFG clock kapalıysa EXTICR yazılamaz.

SYSCFG->EXTICR[0] &= ~(0xF << 0);
	Ne yapar: EXTI0 için 4-bit alanı temizler (EXTI0, EXTICR[0] içindeki bits [3:0]). EXTICR[0] dört adet 4-bit alanı tutar: EXTI0..EXTI3.
	Detay: Her EXTIx için 4 bitlik değer vardır; 0 = PA[x], 1 = PB[x], 2 = PC[x], 3 = PD[x], ... (port kodları). Bu mapping ile bir EXTI hattını hangi GPIO portunun pininin tetikleyeceğini seçersiniz.

SYSCFG->EXTICR[0] |= (0x0 << 0);
	Ne yapar: EXTI0 için port A seçilir (0x0 = PA). Bu satır aslında örnekte sembolik olarak yazılmıştır; 0x0 eklemek etkisizdir ama niyeti açıklar. Eğer EXTI0 için PB0 seçilecekse 0x1 yazılmalı.

EXTI->IMR |= (1 << 0);
	Ne yapar: EXTI interrupt mask register’ında (IMR) bit0’ı set eder; bu, EXTI0 hattından gelen interruptların NVIC’e iletilmesine izin verir.
	Alternatif: EXTI->EMR event mask register’ı event (IRQ olmadan event) için kullanılır; biz interrupt istiyoruz, bu yüzden IMR kullanıldı.

EXTI->RTSR |= (1 << 0);
	Ne yapar: Rising Trigger Selection Register’da bit0 set edilir — yani PA0’da 0→1 geçişi EXTI0 interrupt’ını tetikleyecek.
	Not: Eğer hem yükselen hem düşen kenarda tetikleme isterseniz EXTI->FTSR |= (1<<0); ile düşen kenarı da açabilirsiniz.

EXTI->FTSR &= ~(1 << 0);
	Ne yapar: Falling Trigger Selection Register’daki bit0’ı temizler — düşen kenar ile tetikleme kapatılmış olur. (Kodda seçici olarak düşen kenarı devre dışı bırakıyor.)

NVIC_SetPriority(EXTI0_IRQn, 2);
	Ne yapar: CMSIS wrapper fonksiyonunu çağırarak EXTI0 IRQ hattının önceliğini 2 olarak ayarlar.
	Önemli: Cortex-M NVIC’de daha küçük sayı = daha yüksek öncelik. Sisteminizde __NVIC_PRIO_BITS değerine göre izin verilen öncelik aralığı değişir (STM32F4 tipik olarak 4 bit -> 0..15). Uygulamanın preemption + subpriority stratejisine göre değer seçin.

NVIC_EnableIRQ(EXTI0_IRQn);
	Ne yapar: NVIC içinde EXTI0 IRQ hattını etkinleştirir — EXTI interruptları artık NVIC tarafından çekilebilir hale gelir.

Ek pratik notlar / güvenlik:
	SYSCFG->EXTICR mapping hata yapmaya müsait alandır: yanlış EXTICR index veya yanlış bit offset koymak kolaydır. (EXTICR[ n/4 ], bit offset = (n%4)*4).
	Eğer aynı EXTI line’ı (ör. EXTI0) için birden fazla port kaynağı yazıldıysa (çok nadir), sadece EXTICR seçimi geçerlidir — donanım tek kaynak kullanır.
	EXTI->IMR |= şeklinde set etmek diğer IMR bitlerini bozmaz; ancak bazı yazma-davranışlı registerlarda read-modify-write tehlikeleri olabilir; EXTI IMR için bu normaldir.
	Debounce: mekanik butonlarda bouncing var — eğer direkt bu ISR içinde kritik işlem yapıyorsanız yanlış tetiklemeleri engellemek için:
		(a) Yazılım debouncing: ISR içinde EXTI->IMR &= ~(1<<0) ile kesmeyi disable et, bir timer başlat, timer süresi dolunca tekrar IMR set et.
		(b) Donanımsal çözüm: RC veya Schmitt trigger.
		(c) Kısa yazılım gecikmesi + durum doğrulaması (iyi bir yöntem değil; ISR içinde busy-wait önerilmez).

*/

void EXTI0_IRQHandler(void) {
    if (EXTI->PR & (1 << 0)) {                         // Kesme bayrağı set edilmiş mi?
        GPIOD->ODR ^= (1 << 12);                       // PD12 LED toggle
        EXTI->PR |= (1 << 0);                          // Bayrağı temizle
    }
}


/*

if (EXTI->PR & (1 << 0)) {
	Ne yapar: EXTI pending register (PR) içindeki bit0’ın set olup olmadığını kontrol eder.
	ISR çağrılmış olsa bile iyi pratiktir — özellikle EXTI9_5 gibi paylaşılan IRQ handler’larda hangi EXTI hattının tetiklediğini kontrol etmek zorunludur.
	(EXTI0_IRQHandler yalnızca line0 için ayrılmış olsa da kontrol koymak güvenlidir.)

GPIOD->ODR ^= (1 << 12);
	Ne yapar: PD12’nin ODR register’ını XOR ile tersler (toggle) — LED yanıyorsa söner, sönükse yanar.
	Dikkat/Riskler: ODR ^= mask bir read-modify-write işlemidir; başka bir işlem veya ISR aynı anda ODR üzerinde değişiklik yapıyorsa yarış (race) olabilir.
	Ancak eğer sadece bu ISR PD12 ile ilgileniyorsa pratikte sorun olmaz. Daha atomik ve güvenli LED set/reset için BSRR kullanılabilir (ancak BSRR toggle sağlamaz; toggle için ayrı logic gerekir).
	Performans: ISR içinde kısa ve hızlı kalın — toggle hızlı bir işlem olduğu için uygundur.

EXTI->PR |= (1 << 0);
	Ne yapar: EXTI pending bit’ini temizlemeye çalışıyor. EXTI PR register’ı write-1-to-clear mantığı ile çalışır: pending biti temizlemek için o bite 1 yazmalısınız.
	En güvenli yazım: EXTI->PR = (1 << 0); — bu yazma doğrudan ilgili biti 1 yapar ve temizler; okuma-modify-yazma yerine doğrudan yazma daha temizdir. EXTI->PR |= (1<<0) da pratikte çalışır çünkü OR sonucu 1 yazılır; ancak bazı kod stili rehberleri PR = mask biçimini önerir.
	Sıralama uyarısı: Genelde erken temizlemek (işlemden hemen önce) veya işlem tamamlandıktan sonra temizlemek uygulamaya göre değişir.
	Erken temizleme, ISR uzun sürse bile yeni gelen kenarların yeniden tetiklenmesine olanak verir.
	Eğer debounce veya disable-reenable yaklaşımı kullanıyorsanız önce IMR disable, sonra PR temizleme daha güvenlidir.

Ek konular / tavsiyeler:
	Paylaşılan IRQ’ler: EXTI5..9 ve EXTI10..15 gibi IRQ hattı paylaşımında handler içinde hangi EXTI bitlerinin set olduğunu tek tek kontrol edip temizlemelisiniz (if (EXTI->PR & (1<<5)) { /.../ EXTI->PR = (1<<5); }).
	Debounce: ISR içinde kısa delay koymak (örn. for loop ile) tavsiye edilmez; bunun yerine timer tabanlı yeniden etkinleştirme iyi bir yaklaşımdır: ISR’da EXTI->IMR &= ~(1<<0); ile maskle, timer_start(10..50 ms), timer IRQ’da EXTI->PR = (1<<0); EXTI->IMR |= (1<<0); gibi.
	Preemption ve NVIC önceliği: Eğer EXTI handler’ınız kritik bir sürede çalışıyorsa, daha yüksek öncelikli başka bir interrupt tarafından kesilebilir. ISR içindeki kritik register değişikliklerini düşünerek NVIC önceliklerinizi planlayın.
	Atomic toggle alternatifi: Eğer ana kod da LED’i değiştirebiliyorsa, ISR’de atomik davranmak için:
		a) ISR sadece bir volatile flag set etsin; ana döngü flag’i görüp LED toggle yapsın.
		b) Veya ana kod LED’i değiştirmesin, sadece ISR değiştirsin (en basit).
		c) Donanımsal toggle register yok—bazı MCU’larda ODR toggle register olabilir; STM32F4'te yok.

*/

int main(void) {
    clock_init();                                      // Clock'u 168 MHz yap
    gpio_init();                                       // GPIO ayarları
    exti0_init();                                      // EXTI0 (PA0) ayarları

    while (1) {
        // Ana döngüde iş yok, kesme tetiklenecek
    }
}
