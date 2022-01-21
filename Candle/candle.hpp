#pragma once

#include "usbDevice.hpp"
#include "mab_types.hpp"
#include "md80.hpp"

#include <string>
#include <thread>
#include <vector>
namespace mab
{
    enum CANdleMode_E
    {
        CONFIG,
        UPDATE
    };

    enum CANdleBaudrate_E : uint8_t
    {
        CAN_BAUD_1M = 1,
        CAN_BAUD_5M = 5,
        CAN_BAUD_8M = 8
    };
    #pragma pack(push, 1)   //Ensures there in no padding (dummy) bytes in the structures below



    struct stdUsbFrame_t
    {
        uint8_t id;
        std::vector<StdMd80CommandFrame_t> md80Frames;
    };
    
    #pragma pack(pop)

    class Candle
    {
    private:
        UsbDevice*usb;
        std::thread receiverThread;
        std::thread transmitterThread;
        CANdleMode_E mode = CANdleMode_E::CONFIG;
        stdUsbFrame_t stdFrame;

        bool shouldStopReceiver;
        bool shouldStopTransmitter;

        void transmitNewStdFrame();

        void receive();
        void transmit();

        bool inUpdateMode();
        bool inConfigMode();
        Md80* getMd80FromList(uint16_t id);
    public:
        Candle(CANdleBaudrate_E canBaudrate);
        ~Candle();
        std::vector<Md80> md80s;

        bool addMd80(uint16_t canId);

        bool configCandleBaudrate(CANdleBaudrate_E canBaudrate);
        bool ping();
        bool configMd80Can(uint16_t canId, uint16_t newId, CANdleBaudrate_E newBaudrateMbps, unsigned int newTimeout);
        bool configMd80Save(uint16_t canId);
        bool configMd80SetZero(uint16_t canId);
        bool configMd80SetCurrentLimit(uint16_t canId, float currentLimit);
        bool controlMd80Enable(uint16_t canId, bool enable);
        bool controlMd80Mode(uint16_t canId, Md80Mode_E mode);
        
        bool begin();
        bool end();
        bool isOk();
    };
}
