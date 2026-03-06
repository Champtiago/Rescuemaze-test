#ifndef TCS_h
#define TCS_h
//#include <FastLED.h>
#include "Adafruit_TCS34725.h"
#include "MUX.h"
#include <Adafruit_ADS1X15.h>
#include <Wire.h>
#include "RGBConverter.h"

#define TCS_ADDR 0x30 
constexpr uint8_t blackThreshold=40;

class TCS {
    private:
        Adafruit_TCS34725 tcs_ = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
        MUX mux_;

        static constexpr int8_t millisToWait_ = 60;

        //BLUE TILE
        static constexpr float kRedValueInBlue_ = 40;
        static constexpr float kGreenValueInBlue_ = 50;
        static constexpr float kBlueValueInBlue_ = 66;

        //BLACK TILE
        static constexpr float kRedValueInBlack_ = 30.00;
        static constexpr float kGreenValueInBlack_ = 20.00;
        static constexpr float kBlueValueInBlack_ = 20.00;

        //CHECKPOINT TILE
        static constexpr float kRedValueInCheckpoint_ = 308.00;
        static constexpr float kGreenValueInCheckpoint_ = 330.00;
        static constexpr float kBlueValueInCheckpoint_ = 325.00;
        static constexpr float kClearValueInCheckpoint_ = 520.00;

        static constexpr uint8_t rgbThreshold=25;
        static constexpr uint8_t CheckpointThreshold=50;
        static constexpr float kCheckpointClearThreshold=100;
        

        static constexpr float kMinRedValueInBlue_ = kRedValueInBlue_-rgbThreshold;
        static constexpr float kMaxRedValueInBlue_ = kRedValueInBlue_+rgbThreshold;

        static constexpr float kMinGreenValueInBlue_ = kGreenValueInBlue_-rgbThreshold;
        static constexpr float kMaxGreenValueInBlue_ = kGreenValueInBlue_+rgbThreshold;

        static constexpr float kMinBlueValueInBlue_ = kBlueValueInBlue_-rgbThreshold;
        static constexpr float kMaxBlueValueInBlue_ = kBlueValueInBlue_+rgbThreshold;

        static constexpr float kMaxRedValueInRed_ = 0;
        static constexpr float kMinRedValueInRed_ = 0;

        static constexpr float kMaxGreenValueInRed_ = 0;
        static constexpr float kMinGreenValueInRed_ = 0;

        static constexpr float kMaxBlueValueInRed_ = 0;
        static constexpr float kMinBlueValueInRed_ = 0;

        static constexpr float kMinRedValueInBlack_ = kRedValueInBlack_-rgbThreshold;
        static constexpr float kMaxRedValueInBlack_ = kRedValueInBlack_+rgbThreshold;

        static constexpr float kMinGreenValueInBlack_ = kGreenValueInBlack_-rgbThreshold;
        static constexpr float kMaxGreenValueInBlack_ = kGreenValueInBlack_+rgbThreshold;

        static constexpr float kMinBlueValueInBlack_ = kBlueValueInBlack_-rgbThreshold;
        static constexpr float kMaxBlueValueInBlack_ = kBlueValueInBlack_+rgbThreshold;

        static constexpr float kMinRedValueInCheckpoint_ = kRedValueInCheckpoint_-CheckpointThreshold;
        static constexpr float kMaxRedValueInCheckpoint_ = kRedValueInCheckpoint_+CheckpointThreshold;

        static constexpr float kMinGreenValueInCheckpoint_ = kGreenValueInCheckpoint_-CheckpointThreshold;
        static constexpr float kMaxGreenValueInCheckpoint_ = kGreenValueInCheckpoint_+CheckpointThreshold;
        static constexpr float kMinBlueValueInCheckpoint_ = kBlueValueInCheckpoint_-CheckpointThreshold;
        static constexpr float kMaxBlueValueInCheckpoint_ = kBlueValueInCheckpoint_+CheckpointThreshold;

        static constexpr float kMinCheckpointClear = kClearValueInCheckpoint_ - kCheckpointClearThreshold;
        static constexpr float kMaxCheckpointClear = kClearValueInCheckpoint_ + kCheckpointClearThreshold;

        static constexpr char kRedColor_ = 'r';
        static constexpr char kBlueColor_ = 'B';
        static constexpr char kBlackColor_ = 'N';
        static constexpr char kCheckpointColor_ = 'C';
        static constexpr char kUndefinedColor_ = 'U';

        void setDefaultValues();

    public:
        float red_;
        float green_;
        float blue_;
        float clear_;
        TCS();

        TCS(const uint8_t posMux);
        
        void init();

        void setMux(const uint8_t posMux);

        void printRGB();

        void printRGBC();

        void printColor();

        void updateRGB();

        void updateRGBC();

        char getColor();

};

#endif