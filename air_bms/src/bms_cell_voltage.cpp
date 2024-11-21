#include <iostream>
#include <cstring>
#include <cstdint>

// UART protokolü için bir buffer uzunluğu
#define XFER_BUFFER_LENGTH 13  // Çerçeve boyutu
#define START_BYTE 0xAA        // Başlangıç baytı
#define STOP_BYTE  0x55        // Bitiş baytı

// UART arayüzü için varsayımsal fonksiyonlar
bool sendUARTCommand(uint8_t* txBuffer, uint8_t length);
bool receiveUARTResponse(uint8_t* rxBuffer, uint8_t length);

// Hücre voltajlarını almak için basit bir fonksiyon
bool getCellVoltages(uint16_t* cellVoltages, uint8_t numberOfCells) {
    uint8_t txBuffer[XFER_BUFFER_LENGTH] = {0};
    uint8_t rxBuffer[XFER_BUFFER_LENGTH] = {0};
    uint8_t checksum = 0;

    // Komut gönderme: CELL_VOLTAGES
    txBuffer[0] = START_BYTE;       // Başlangıç baytı
    txBuffer[1] = 0x95;             // CELL_VOLTAGES komut kodu
    txBuffer[12] = STOP_BYTE;       // Durdurma baytı

    // Checksum hesaplama
    for (uint8_t i = 0; i < XFER_BUFFER_LENGTH - 1; i++) {
        checksum += txBuffer[i];
    }
    txBuffer[11] = checksum;        // Checksum'u çerçevenin sonuna ekle

    // UART üzerinden komutu gönder
    if (!sendUARTCommand(txBuffer, XFER_BUFFER_LENGTH)) {
        std::cerr << "Error: Failed to send command!" << std::endl;
        return false;
    }

    // Gelen çerçeveleri işleme
    uint8_t cellNo = 0;
    uint8_t frameNo = 0;

    while (cellNo < numberOfCells) {
        // UART üzerinden cevabı al
        if (!receiveUARTResponse(rxBuffer, XFER_BUFFER_LENGTH)) {
            std::cerr << "Error: Failed to receive response!" << std::endl;
            return false;
        }

        // Başlangıç ve durdurma baytlarını kontrol et
        if (rxBuffer[0] != START_BYTE || rxBuffer[12] != STOP_BYTE) {
            std::cerr << "Error: Invalid frame structure!" << std::endl;
            return false;
        }

        // Çerçeve numarası doğrulama
        if (rxBuffer[4] != frameNo) {
            std::cerr << "Error: Frame order mismatch!" << std::endl;
            return false;
        }

        // Çerçevedeki her hücre voltajını oku
        for (uint8_t i = 0; i < 3; i++) {
            if (cellNo >= numberOfCells) break;

            // 2 byte hücre voltajını birleştir
            cellVoltages[cellNo] = (rxBuffer[5 + i * 2] << 8) | rxBuffer[6 + i * 2];
            std::cout << "Cell " << (int)cellNo + 1 << ": " << cellVoltages[cellNo] << " mV" << std::endl;

            cellNo++;
        }

        // Bir sonraki çerçeve numarasına geç
        frameNo++;
    }

    return true;
}

// UART üzerinden komut gönderme (örnek uygulama)
bool sendUARTCommand(uint8_t* txBuffer, uint8_t length) {
    // UART portundan veri gönder
    // Örneğin: write(serial_fd, txBuffer, length);
    std::cout << "Sent command: ";
    for (uint8_t i = 0; i < length; i++) {
        std::cout << std::hex << (int)txBuffer[i] << " ";
    }
    std::cout << std::endl;
    return true;
}

// UART üzerinden cevap alma (örnek uygulama)
bool receiveUARTResponse(uint8_t* rxBuffer, uint8_t length) {
    // UART portundan veri al
    // Örneğin: read(serial_fd, rxBuffer, length);
    // Simüle edilen bir cevap:
    uint8_t simulatedResponse[XFER_BUFFER_LENGTH] = {
        START_BYTE, 0x95, 0x00, 0x00, 0x01, // Frame 1, Cell voltages start
        0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0x00, STOP_BYTE
    };
    memcpy(rxBuffer, simulatedResponse, length);

    std::cout << "Received response: ";
    for (uint8_t i = 0; i < length; i++) {
        std::cout << std::hex << (int)rxBuffer[i] << " ";
    }
    std::cout << std::endl;

    return true;
}

// Main test fonksiyonu
int main() {
    const uint8_t numberOfCells = 6; // Örneğin, 6 hücre
    uint16_t cellVoltages[numberOfCells] = {0};

    if (getCellVoltages(cellVoltages, numberOfCells)) {
        std::cout << "Cell voltages received successfully!" << std::endl;
    } else {
        std::cerr << "Failed to get cell voltages!" << std::endl;
    }

    return 0;
}
