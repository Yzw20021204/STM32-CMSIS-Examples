# STM32 CMSIS Examples

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![STM32F4](https://img.shields.io/badge/STM32-F4%20Discovery-orange.svg)](https://www.st.com/en/evaluation-tools/stm32f4discovery.html)
[![CMSIS](https://img.shields.io/badge/CMSIS-Core%20%26%20Device-green.svg)](https://arm-software.github.io/CMSIS_5/Core/html/index.html)

Bu repository, STM32 mikrodenetleyiciler ile **HAL kütüphanesi kullanmadan**, doğrudan **CMSIS (Cortex Microcontroller Software Interface Standard)** tabanlı programlama örneklerini içerir. 

## 🎯 Amaç

STM32 programlamaya yeni başlayanların donanımın çalışma mantığını **en temel seviyede** öğrenmesini sağlamak:

- ✅ **Donanım kayıt seviyesinde** nasıl çalıştığını anlamak
- ✅ HAL gibi **yüksek seviyeli kütüphanelere bağımlılığı** azaltmak
- ✅ **Gerçek donanım kontrolü** mantığını kavramak
- ✅ **Register manipülasyonu** ile programlama yapmayı öğrenmek

## 📚 Mevcut Örnekler

| Konu | Açıklama | Durum |
|------|----------|-------|
| **GPIO** | Dijital giriş/çıkış kontrolü, LED kontrolü, buton okuma | ✅ Hazır |
| **EXTI** | Harici kesme işlemleri, interrupt handling | ✅ Hazır |
| **ADC** | Analog-Dijital dönüşüm, sensör okuma | ✅ Hazır |
| **DAC** | Dijital-Analog dönüşüm, analog sinyal üretimi | ✅ Hazır |
| **PWM** | Darbe genişlik modülasyonu, motor kontrolü | ✅ Hazır |
| **UART** | Seri haberleşme protokolü | ✅ Hazır |
| **SPI** | Seri Peripheral Interface | ✅ Hazır |
| **I2C** | Inter-Integrated Circuit | ✅ Hazır |
| **Timer** | Zamanlayıcı işlemleri | ✅ Hazır |

## 🛠️ Gereksinimler

### Donanım
- **STM32F4 Discovery Kit** (STM32F407VGT6)
- USB Kablo (programlama için)
- Breadboard ve jumper kablolar (opsiyonel)

### Yazılım
- **STM32CubeIDE** (önerilen) veya **Keil µVision**
- **ST-Link Utility** (debug için)
- **Git** (repository klonlama için)

## 🚀 Kurulum ve Çalıştırma

### 1. Repository'yi Klonlayın
```
git clone https://github.com/TalhaYaman98/STM32-CMSIS-Examples.git
cd STM32-CMSIS-Examples
```

### 2. STM32CubeIDE'de Açın
1. STM32CubeIDE'yi açın
2. `File -> Import -> Existing Projects into Workspace`
3. Klonladığınız klasörü seçin
4. İstediğiniz örnek projeyi seçin

### 3. Derleyin ve Yükleyin
1. Projeyi seçin ve `Ctrl+B` ile derleyin
2. STM32F4 Discovery kartınızı USB ile bağlayın
3. `Run -> Debug` ile programı yükleyin ve çalıştırın

## 🤝 Katkıda Bulunma

Projeye katkıda bulunmak isteyenler:

1. Repository'yi fork edin
2. Yeni bir branch oluşturun (`git checkout -b feature/yeni-ozellik`)
3. Değişikliklerinizi commit edin (`git commit -am 'Yeni özellik eklendi'`)
4. Branch'inizi push edin (`git push origin feature/yeni-ozellik`)
5. Pull Request oluşturun

### Katkı Kuralları
- Kod yorumlarını Türkçe yazın
- Her örnek için ayrı klasör oluşturun
- README dosyasını güncel tutun
- Register seviyesinde kod yazın (HAL kullanmayın)

## 📚 Faydalı Kaynaklar

- [STM32F4 Reference Manual](https://www.st.com/resource/en/reference_manual/rm0090-stm32f405415-stm32f407417-stm32f427437-and-stm32f429439-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- [STM32F407 Datasheet](https://www.st.com/resource/en/datasheet/stm32f407vg.pdf)
- [CMSIS Documentation](https://arm-software.github.io/CMSIS_5/Core/html/index.html)
- [STM32 Programming Manual](https://www.st.com/resource/en/programming_manual/pm0214-stm32-cortexm4-mcus-and-mpus-programming-manual-stmicroelectronics.pdf)

## ⚠️ Önemli Notlar

- Bu örnekler **eğitim amaçlıdır** ve ticari projelerde kullanılmadan önce test edilmelidir
- **Register seviyesinde** programlama yaptığımız için dikkatli olmak gerekir
- Her örnek **standalone** çalışacak şekilde tasarlanmıştır
- **STM32F4 Discovery** kartı için optimize edilmiştir


⭐ Bu projeyi beğendiyseniz **star** vermeyi unutmayın!

🐛 Hata bulursanız veya öneriniz varsa **issue** açmaktan çekinmeyin.

💡 Yeni örnek önerilerinizi **discussions** bölümünde paylaşabilirsiniz.
