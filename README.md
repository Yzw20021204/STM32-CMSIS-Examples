Bu repository, STM32 mikrodenetleyiciler ile HAL kütüphanesi kullanmadan, doğrudan CMSIS (Cortex Microcontroller Software Interface Standard) tabanlı programlama örneklerini içerir.
Amacım, STM32 programlamaya yeni başlayanların donanımın çalışma mantığını en temel seviyede öğrenmesini sağlamaktır.

📌 İçerik
  Her örnek proje, tek bir temel konuyu ele alacak şekilde hazırlanmıştır.
  Kodlar açıklama satırları ile desteklenmiş ve kolayca anlaşılacak şekilde yazılmıştır.

Mevcut konular:

  GPIO (Giriş/Çıkış kontrolü)
  
  EXTI (Harici kesme işlemleri)
  
  ADC (Analog-Dijital Dönüşüm)
  
  DAC (Dijital-Analog Dönüşüm)
  
  PWM (Darbe genişlik modülasyonu)
  
  SPI, I2C, UART gibi haberleşme protokolleri (eklenecek)
  

🎯 Hedef
  STM32’nin donanım kayıt seviyesinde nasıl çalıştığını göstermek
  
  HAL gibi yüksek seviyeli kütüphanelere bağımlı olmadan programlama yapabilmeyi öğretmek
  
  Elektronik ve gömülü sistemlerde temel donanım kontrol mantığını anlamayı kolaylaştırmak

🛠️ Donanım ve Araçlar
  Geliştirme Kartı: STM32F4 Discovery
  
  IDE: STM32CubeIDE veya Keil uVision
  
  Dil: C (CMSIS Core & Device Headers)
