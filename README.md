# Exercise 5: Demonstrating Resource Contention in Multitasking Systems

## Tujuan
Exercise ini bertujuan untuk:
1. Menunjukkan bagaimana kontensi akses terhadap sumber daya bersama dapat terjadi dalam sistem multitasking.
2. Mengidentifikasi dampak negatif dari tidak adanya mekanisme perlindungan sumber daya.
3. Memberikan pemahaman tentang pentingnya penggunaan mekanisme pengelolaan sumber daya, seperti semaphore atau mutex.

## Peralatan
1. STM32
2. STM32 Cube IDE
3. Free RTOS
4. Debugger
   
## Langkah-langkah
1. Buat proyek baru di STM32CubeIDE atau gunakan template yang disediakan.
2. Aktifkan fitur FreeRTOS melalui STM32CubeMX dengan konfigurasi yang sesuai.
3. Tambahkan dua tugas yang akan bersaing untuk menggunakan sumber daya bersama, seperti UART atau GPIO.
4. Tulis dua tugas (Task1 dan Task2) dalam file main.c.
5. Pastikan kedua tugas mencoba mengakses sumber daya bersama tanpa mekanisme pengelolaan seperti semaphore.
6. Gunakan fungsi osDelay() untuk membuat jeda eksekusi dan mempermudah observasi kontensi.
7. Flash kode ke STM32 Discovery Board.
8. Jalankan program dan perhatikan perilaku tugas melalui LED, UART output, atau alat debugging.

## Hasil yang diharapkan
**Tanpa Proteksi Sumber Daya**
1. Tugas-tugas mungkin saling tumpang tindih dalam mengakses sumber daya bersama.
2. Muncul konflik akses, seperti data yang tumpang tindih pada UART atau LED yang menyala tidak sesuai.

**Dengan Proteksi Sumber Daya (Semaphore)**
1. Setelah menambahkan semaphore sebagai mekanisme pengelolaan, tugas-tugas dapat mengakses sumber daya secara bergantian.
2. Konflik dapat dihindari, dan sistem berfungsi dengan lebih stabil dan terprediksi.

## Hasil Percobaan


https://github.com/user-attachments/assets/9e9000de-e68e-497f-ae2f-ab8044b2e2f1

