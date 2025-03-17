#ifndef Warbler_h

#if !defined(ESP32)
#error Only supports ESP32
#endif

#include "ReelTwo.h"
#include "core/SetupEvent.h"
#include "core/AnimatedEvent.h"
#include "AudioFrequencyBitmap.h"
#include "Audio.h"
#include "Antenna.h"

#undef MIX_AUDIO_FLOAT

#define USE_WARBLER_DEBUG
#ifdef USE_WARBLER_DEBUG
#define WARBLER_DEBUG_PRINT(s) DEBUG_PRINT(s)
#define WARBLER_DEBUG_PRINTLN(s) DEBUG_PRINTLN(s)
#define WARBLER_DEBUG_PRINT_HEX(s) DEBUG_PRINT_HEX(s)
#define WARBLER_DEBUG_PRINTLN_HEX(s) DEBUG_PRINTLN_HEX(s)
#else
#define WARBLER_DEBUG_PRINT(s)
#define WARBLER_DEBUG_PRINTLN(s)
#define WARBLER_DEBUG_PRINT_HEX(s)
#define WARBLER_DEBUG_PRINTLN_HEX(s)
#endif

#ifndef WARBLER_NUM_TRACKS
#define WARBLER_NUM_TRACKS 10
#endif

#ifndef WARBLER_QUEUE_DEPTH
#define WARBLER_QUEUE_DEPTH 10
#endif

#ifndef WARBLER_DECODE_BUFFER_SIZE
#define WARBLER_DECODE_BUFFER_SIZE 2048
#endif

#ifndef I2S_DOUT_PIN
#define I2S_DOUT_PIN 25  // I2S audio output
#endif

#ifndef I2S_LRC_PIN
#define I2S_LRC_PIN  26  // I2S audio output
#endif

#ifndef I2S_BCLK_PIN
#define I2S_BCLK_PIN 27  // I2S audio output
#endif

class WarblerAudio: public AnimatedEvent, public SetupEvent, protected Audio
{
public:
    WarblerAudio(fs::FS &fs, AudioFrequency* audioBitmap = nullptr,
                uint8_t bclk = I2S_BCLK_PIN, uint8_t lrc = I2S_LRC_PIN, uint8_t dout = I2S_DOUT_PIN) :
        fAudioBitmap(audioBitmap),
        fFS(fs)
    {
        *fNextSong = '\0';
        setPinout(bclk, lrc, dout);
    }

    AudioFrequency* getAudioBitmap() {
        return fAudioBitmap;
    }

    void setAudioBitmap(AudioFrequency* audioBitmap) {
        fAudioBitmap = audioBitmap;
    }

    void setBalance(int8_t bal = 0)
    {
        Audio::setBalance(bal);
    }

    void setVolume(uint8_t vol)
    {
        Audio::setVolume(vol);
    }

    inline bool isComplete() const
    {
        return fActiveRemaining == 0;
    }

    void stop()
    {
        fStopStream = true;
    }

    void startMotor() {
        fMotorStart = true;
    }

    void stopMotor() {
        fMotorStop = true;
    }

    virtual void setup() override
    {
        xTaskCreatePinnedToCore(
              audioLoopTask,
              "WarblerAudio",
              10000,
              nullptr,
              2,
              &fAudioTask,
              0);
        fPOD = esp_partition_find_first((esp_partition_type_t)0xBA, (esp_partition_subtype_t)0xBE, NULL);
        printf("fPOD: %p\n", fPOD);
        if (initTracks())
            setSampleFilter(audioFilter);
    }

    virtual void animate() override
    {
        if (fActiveRemaining != 0)
            return;
        if (fMotorStart)
        {
            fMotorStart = false;
            fMotorStop = false;
            fMotorSoundOffset = 0;
            fMotorSoundSize = sizeof(AntennaUlaw);
            fMotorSound = AntennaUlaw;
            clearDecodeBuffer();
            fActiveRemaining = fMotorSoundSize;
            if (!isPlaying())
                connecttonull();
            return;
        }
        if (fNextTrack == -1 && fNextWarble == -1 && fNumQueue > 0)
        {
            uint32_t now = millis();
            if (fQueue->fStartMS == 0)
                fQueue->fStartMS = now;
            if (fQueue->fStartMS + fQueue->fDelayMS <= now)
            {
                fNextTrack = fQueue->fTrack;
                fNextWarble = fQueue->fWarble;
                // printf("Play %d:%d\n", fNextTrack, fNextWarble);
                if (--fNumQueue > 0)
                {
                    memmove(&fQueue[0], &fQueue[1], fNumQueue * sizeof(fQueue[0]));
                    // printf("Remaining: %d\n", fNumQueue);
                }
            }
        }
        if (fNextTrack != -1 && fNextWarble != -1)
        {
            DEBUG_PRINT("NEXT WARBLE ["); DEBUG_PRINT(fNextTrack);
            DEBUG_PRINT(":"); DEBUG_PRINT(fNextWarble); DEBUG_PRINTLN("]");
            if (fPOD != nullptr /*&& isPlaying()*/)
            {
                static uint32_t sPODOffset[204] = {
                    // subdir 0
                    113910, 154626, 397176, 627042, 700547, 964414,
                    // subdir 1
                    1018338, 1058499, 1115832, 1285888, 1357069, 1408269, 1450384, 1496920, 1568561, 1608695, 1652974, 1706009, 1762360, 1816099, 1864716, 1914983, 1973829, 2014977, 2063097, 2110740, 2422786, 2472411,
                    // subdir 2
                    2514526, 2566804, 2615973, 2682076, 2721432, 2819329, 2874227, 2931145, 2975063, 3014492, 3059396, 3097721, 3190063, 3241173, 3285552, 3325445, 3379384, 3418234, 3459262, 3516834, 3588433, 3644753,
                    // subdir 3
                    3686006, 3756201, 3796212, 3849838, 3888882, 3973913, 4018805, 4059206, 4101530, 4142982, 4194911, 4236994, 4299173, 4349719, 4400163, 4450718, 4501505, 4549144, 4593826, 4640521, 4690556, 4724580,
                    // subdir 4
                    4766450, 4812048, 4873872, 4919579, 4962431, 5008667, 5066773, 5113155, 5166697, 5205667, 5245663, 5288769, 5337150, 5406927, 5456948, 5503358, 5546981, 5609819, 5663939, 5701375, 5757899, 5796740,
                    // subdir 5
                    5840760, 5884443, 5935445, 5974965, 6013724, 6072595, 6123828, 6161229, 6202129, 6254128, 6296922, 6341056, 6378386, 6428019, 6466382, 6522767, 6563104, 6611800, 6653417, 6713201, 6763756, 6822268,
                    // subdir 6
                    6864127, 6926338, 6978078, 7046760, 7108930, 7160485, 7217911, 7276282, 7319127, 7361790, 7428414, 7481721, 7527625, 7588725, 7646260, 7690742, 7752566, 7811861, 7874699, 7921572, 7986944, 8037419,
                    // subdir 7
                    8074114, 8111184, 8147277, 8183547, 8235682, 8271932, 8307682, 8344390, 8380417, 8417654, 8454947, 8492251, 8529555, 8566788, 8604031, 8652867, 8689678, 8726866, 8762297, 8797725, 8834421, 8871188,
                    // subdir 8
                    8906039, 8940822, 8975375, 9010302, 9045116, 9080085, 9114409, 9148893, 9184246, 9219531, 9254760, 9289914, 9323834, 9358149, 9393221, 9427714, 9462412, 9497280, 9532509, 9566813, 9600816, 9636101,
                    // subdir 9
                    9669386, 9708850, 9745545, 9784453, 9817477, 9850998, 9909963, 9943150, 9977070, 10010814, 10043803, 10076820, 10110647, 10143664, 10177584, 10211261, 10248085, 10281830, 10315246, 10349079, 10382418, 10415742,
                 };
                uint32_t warbleOffset = 0;
                uint32_t warbleLength = 0;
                uint32_t activeRemaining = 0;
                const int numWarbles = SizeOfArray(sPODOffset) / SizeOfArray(fTrackCount);
                printf("numWarbles: %d\n", numWarbles);
                if (fNextWarble > numWarbles)
                    fNextWarble = int(float(fNextWarble) / fTrackCount[fNextTrack] * numWarbles);
                uint32_t warbleTrack = fNextWarble + fNextTrack * numWarbles;
                warbleOffset = (warbleTrack > 0) ? sPODOffset[warbleTrack-1] : 0;
                warbleLength = sPODOffset[warbleTrack];
                activeRemaining = warbleLength - warbleOffset;
                fPODActive = true;
                fPODOffset = warbleOffset;
                if (activeRemaining > 0)
                    clearDecodeBuffer();
                fActiveRemaining = activeRemaining;
                if (activeRemaining > 0 && !isPlaying())
                    connecttonull();
            }
            else
            {
                fPODActive = false;
                if (fNextTrack != fActiveTrack)
                {
                    if (fActiveTrack != -1)
                        closeTrack();
                    openTrack(fNextTrack);
                }
                DEBUG_PRINTLN(fActiveTrack);
                if (fActiveTrack != -1)
                {
                    uint32_t offsetTable;
                    uint32_t trackCount = 0;
                    uint32_t warbleOffset = 0;
                    uint32_t warbleLength = 0;
                    uint32_t activeRemaining = 0;
                    printf("fActiveTrack: %d\n", fActiveTrack);
                    printf("TABLE: %u\n", (uint32_t)fTrackOffset[fActiveTrack]);
                    if (fTrackOffset[fActiveTrack] != nullptr)
                    {
                        warbleOffset = (fNextWarble > 0) ? fTrackOffset[fActiveTrack][fNextWarble-1] : 0;
                        warbleLength = fTrackOffset[fActiveTrack][fNextWarble];
                        activeRemaining = warbleLength - warbleOffset;
                        DEBUG_PRINTLN(warbleOffset);
                        if (!fFile.seek(warbleOffset + sizeof(offsetTable), SeekSet))
                        {
                            DEBUG_PRINTLN("Failed to seek");
                            activeRemaining = 0;
                        }
                        DEBUG_PRINTLN(activeRemaining);
                    }
                    else if (fFile.seek(0, SeekSet) &&
                             fFile.read((uint8_t*)&offsetTable, sizeof(offsetTable)) == sizeof(offsetTable) &&
                             fFile.seek(offsetTable, SeekSet) &&
                             fFile.read((uint8_t*)&trackCount, sizeof(trackCount)) == sizeof(trackCount))
                    {
                        if (fNextWarble > 0)
                        {
                            if (fFile.seek((fNextWarble-1)*sizeof(warbleLength), SeekCur) &&
                                fFile.read((uint8_t*)&warbleOffset, sizeof(warbleOffset)) == sizeof(warbleOffset) &&
                                fFile.read((uint8_t*)&warbleLength, sizeof(warbleLength)) == sizeof(warbleLength))
                            {
                                activeRemaining = warbleLength - warbleOffset;
                            }
                        }
                        else if (fFile.read((uint8_t*)&warbleLength, sizeof(warbleLength)) == sizeof(warbleLength))
                        {
                            activeRemaining = warbleLength;
                        }
                        if (!fFile.seek(warbleOffset + sizeof(offsetTable), SeekSet))
                        {
                            activeRemaining = 0;
                        }
                    }
                    else
                    {
                        DEBUG_PRINTLN("Failed to locate");
                    }
                    if (activeRemaining > 0)
                        clearDecodeBuffer();
                    fActiveRemaining = activeRemaining;
                    if (activeRemaining > 0 && !isPlaying()) {
                        connecttonull();
                    }
                }
            }
            fNextTrack = fNextWarble = -1;
        }
    }

    bool isPlaying()
    {
        return isRunning() && (isLocalFile() || isWebStream());
    }

    void play(const char* file)
    {
        strncpy(fNextSong, file, sizeof(fNextSong)-1);
    }

    void playNext(int track = -1, int32_t warble = -1)
    {
        if (track == -1)
            track = random(SizeOfArray(fTrackCount));
        track = max(min(track, int(SizeOfArray(fTrackCount))), 0);
        if (warble == -1)
            warble = random(fTrackCount[track]);
        warble = max(min(warble, fTrackCount[track]), 0);
        fNextTrack = track;
        fNextWarble = warble;
    }

    void queue(uint32_t delayMS = 0, int track = -1, int32_t warble = -1)
    {
        if (track == -1)
            track = random(SizeOfArray(fTrackCount));
        track = max(min(track, int(SizeOfArray(fTrackCount))), 0);
        if (warble == -1)
            warble = random(fTrackCount[track]);
        warble = max(min(warble, fTrackCount[track]), 0);
        if (fNumQueue < SizeOfArray(fQueue))
        {
            fQueue[fNumQueue].fTrack = track;
            fQueue[fNumQueue].fWarble = warble;
            fQueue[fNumQueue].fStartMS = 0;
            fQueue[fNumQueue].fDelayMS = delayMS;
            fNumQueue++;
        }
    }

#ifdef MIX_AUDIO_FLOAT
    float read()
#else
    int16_t read()
#endif
    {
        static short sDecode[256] =
        {
            -32124, -31100, -30076, -29052, -28028, -27004, -25980, -24956,
            -23932, -22908, -21884, -20860, -19836, -18812, -17788, -16764,
            -15996, -15484, -14972, -14460, -13948, -13436, -12924, -12412,
            -11900, -11388, -10876, -10364, -9852,  -9340,  -8828,  -8316,
            -7932,  -7676,  -7420,  -7164,  -6908,  -6652,  -6396,  -6140,
            -5884,  -5628,  -5372,  -5116,  -4860,  -4604,  -4348,  -4092,
            -3900,  -3772,  -3644,  -3516,  -3388,  -3260,  -3132,  -3004,
            -2876,  -2748,  -2620,  -2492,  -2364,  -2236,  -2108,  -1980,
            -1884,  -1820,  -1756,  -1692,  -1628,  -1564,  -1500,  -1436,
            -1372,  -1308,  -1244,  -1180,  -1116,  -1052,  -988,   -924,
            -876,   -844,   -812,   -780,   -748,   -716,   -684,   -652,
            -620,   -588,   -556,   -524,   -492,   -460,   -428,   -396,
            -372,   -356,   -340,   -324,   -308,   -292,   -276,   -260,
            -244,   -228,   -212,   -196,   -180,   -164,   -148,   -132,
            -120,   -112,   -104,   -96,    -88,    -80,    -72,    -64,
            -56,    -48,    -40,    -32,    -24,    -16,    -8,     0,

            32124,  31100,  30076,  29052,  28028,  27004,  25980,  24956,
            23932,  22908,  21884,  20860,  19836,  18812,  17788,  16764,
            15996,  15484,  14972,  14460,  13948,  13436,  12924,  12412,
            11900,  11388,  10876,  10364,  9852,   9340,   8828,   8316,
            7932,   7676,   7420,   7164,   6908,   6652,   6396,   6140,
            5884,   5628,   5372,   5116,   4860,   4604,   4348,   4092,
            3900,   3772,   3644,   3516,   3388,   3260,   3132,   3004,
            2876,   2748,   2620,   2492,   2364,   2236,   2108,   1980,
            1884,   1820,   1756,   1692,   1628,   1564,   1500,   1436,
            1372,   1308,   1244,   1180,   1116,   1052,   988,    924,
            876,    844,    812,    780,    748,    716,    684,    652,
            620,    588,    556,    524,    492,    460,    428,    396,
            372,    356,    340,    324,    308,    292,    276,    260,
            244,    228,    212,    196,    180,    164,    148,    132,
            120,    112,    104,    96,     88,     80,     72,     64,
            56,     48,     40,     32,     24,     16,     8,      0
        };
    #ifdef MIX_AUDIO_FLOAT
        float sample = 0;
    #else
        int16_t sample = 0;
    #endif
        if (fActiveRemaining > 0)
        {
            if (fDecodePtr == &fDecodeBuffer[sizeof(fDecodeBuffer)])
                fillDecodeBuffer();
        #ifdef MIX_AUDIO_FLOAT
            sample = float(sDecode[*fDecodePtr++]) / SHRT_MAX;
        #else
            sample = sDecode[*fDecodePtr++];
        #endif
            fActiveRemaining--;
        }
        return sample;
    }

    virtual void audio_eof_mp3(const char *info) override
    {
        printf("END OF FILE\n");
        connecttonull();
    }

private:
    File fFile;
    TaskHandle_t fAudioTask = nullptr;
    int fNextTrack = -1;
    int fNextWarble = -1;
    unsigned fNumQueue = 0;
    bool fOTAInProgress = false;
    struct {
        int fTrack;
        int fWarble;
        uint32_t fStartMS; 
        uint32_t fDelayMS;
    } fQueue[WARBLER_QUEUE_DEPTH];
    int8_t fActiveTrack = -1;
    volatile uint32_t fActiveRemaining = 0;
    uint8_t fDecodeBuffer[WARBLER_DECODE_BUFFER_SIZE];
    uint8_t* fDecodePtr = &fDecodeBuffer[sizeof(fDecodeBuffer)];
    int32_t fTrackCount[WARBLER_NUM_TRACKS] = {};
    uint32_t* fTrackOffset[WARBLER_NUM_TRACKS] = {};
    AudioFrequency* fAudioBitmap = nullptr;
    const esp_partition_t* fPOD = nullptr;
    bool fPODActive = false;
    bool fMotorStart = false;
    bool fMotorStop = false;
    volatile uint8_t* fMotorSound = nullptr;
    volatile uint32_t fMotorSoundOffset = 0;
    volatile uint32_t fMotorSoundSize = 0;
    volatile uint32_t fPODOffset = 0;
    fs::FS &fFS;
    bool fStopStream = false;
    char fNextSong[128];

    void clearDecodeBuffer()
    {
        fDecodePtr = &fDecodeBuffer[sizeof(fDecodeBuffer)];
    }

    void fillDecodeBuffer()
    {
        ssize_t bytesRead = 0;
        fDecodePtr = fDecodeBuffer;
        if (fMotorSound)
        {
            if (!fMotorStop) {
                bytesRead = min(fMotorSoundSize - fMotorSoundOffset, sizeof(fDecodeBuffer));
                memcpy(fDecodeBuffer, (char*)&fMotorSound[fMotorSoundOffset], bytesRead);
                fMotorSoundOffset += bytesRead;
            }
        }
        else if (fPODActive)
        {
            bytesRead = min((unsigned int)fActiveRemaining, sizeof(fDecodeBuffer));
            if (esp_partition_read(fPOD, fPODOffset, fDecodeBuffer, bytesRead) != ESP_OK)
            {
                bytesRead = 0;
            }
            fPODOffset += bytesRead;
        }
        else
        {
            // dont care about errors output will be silent
            bytesRead = fFile.readBytes((char*)fDecodeBuffer, sizeof(fDecodeBuffer));
        }
        // should never but if we have a read failure or underrun clear to zero
        if (bytesRead < 0)
            bytesRead = 0;
        if (bytesRead != sizeof(fDecodeBuffer))
        {
            memset(&fDecodeBuffer[bytesRead], '\0', sizeof(fDecodeBuffer) - bytesRead);
        }
    }

    static WarblerAudio*& activeWarbler()
    {
        static WarblerAudio* sWarbler;
        return sWarbler;
    }

    bool openTrack(unsigned i)
    {
        char buf[16];
        snprintf(buf, sizeof(buf), "/sys%d.dat", i);
        fFile = fFS.open(buf);
        fActiveTrack = i;
        return (fFile == true);
    }

    void closeTrack()
    {
        fFile.close();
        fActiveTrack = -1;
    }

    bool initTracks()
    {
        if (!psramInit()) {
            // return false;
        }
        bool success = (activeWarbler() == nullptr);
        for (unsigned i = 0; i < SizeOfArray(fTrackCount); i++)
        {
            if (openTrack(i))
            {
                uint32_t offsetTable;
                int32_t trackCount;
                if (fFile.seek(0, SeekSet) &&
                    fFile.read((uint8_t*)&offsetTable, sizeof(offsetTable)) == sizeof(offsetTable) &&
                    fFile.seek(offsetTable, SeekSet) &&
                    fFile.read((uint8_t*)&trackCount, sizeof(trackCount)) == sizeof(trackCount))
                {
                    fTrackCount[i] = trackCount;
                    fTrackOffset[i] = (uint32_t*)ps_calloc(trackCount, sizeof(uint32_t));
                    WARBLER_DEBUG_PRINT("Track #"); WARBLER_DEBUG_PRINT(i);
                    WARBLER_DEBUG_PRINT(": "); WARBLER_DEBUG_PRINT(trackCount);
                    WARBLER_DEBUG_PRINT(" tableSize: "); WARBLER_DEBUG_PRINTLN(trackCount * sizeof(uint32_t));
                    if (fTrackOffset[i] != nullptr)
                    {
                        size_t trackOffsetSize = trackCount*sizeof(uint32_t);
                        if (fFile.read((uint8_t*)fTrackOffset[i], trackOffsetSize) != trackOffsetSize)
                        {
                            // Failed to read track offsets disable track
                            WARBLER_DEBUG_PRINTLN("Failed to read offset table");
                            free(fTrackOffset[i]);
                            fTrackOffset[i] = nullptr;
                            fTrackCount[i] = 0;
                        }
                        else
                        {
                            DEBUG_PRINTLN_HEX((uint32_t)fTrackOffset[i]);
                        }
                    }
                    else
                    {
                        WARBLER_DEBUG_PRINTLN("Failed to allocate offset table");
                    }
                }
                closeTrack();
            }
            else
            {
                // success = false;
            }
        }
        printf("success=%d\n", success);
        if (success)
            activeWarbler() = this;
        return success;
    }

    static inline float mixSamples(float s1, float s2)
    {
        s1 = (s1 + s2) - (s1 * s2);
        // // clip if necessary
        if (s1 < -1.0f)
            s1 = -1.0f;
        if (s1 > 1.0f)
            s1 = 1.0f;
        return s1;
    }

    static void audioFilter(unsigned numBits, unsigned numChannels, const int16_t* samples, unsigned sampleCount)
    {
        // return;
        WarblerAudio* active = activeWarbler();
        if (active == nullptr)
            return;
        if (active->fActiveRemaining == 0)
        {
            AudioFrequency* bitmap = active->fAudioBitmap;
            if (!active->isNullStream() && bitmap != nullptr)
                bitmap->processSamples(numBits, numChannels, samples, sampleCount);
            return;
        }
    #ifdef MIX_AUDIO_FLOAT
        static float sPlayVolume = 1.0f;
        static float sTrackVolume = 1.0f;
        if (numBits == 8)
        {
            uint8_t* outp = (uint8_t*)samples;
            if (numChannels == 1)
            {
                for (size_t i = 0; i < sampleCount*2; i++)
                {
                    float s2 = active->read() * sPlayVolume;
                    float s1 = (float(*outp) / UCHAR_MAX) * sTrackVolume;
                    *outp++ = uint8_t(mixSamples(s1, s2) * UCHAR_MAX);
                }
            }
            else
            {
                for (size_t i = 0; i < sampleCount*2; i++)
                {
                    float s2 = active->read() * sPlayVolume;
                    for (size_t ci = 0; ci < numChannels; ci++)
                    {
                        float s1 = (float(*outp) / UCHAR_MAX) * sTrackVolume;
                        *outp++ = uint8_t(mixSamples(s1, s2) * UCHAR_MAX);
                    }
                }
            }
        }
        else if (numBits == 16)
        {
            int16_t* outp = (int16_t*)samples;
            if (numChannels == 1)
            {
                for (size_t i = 0; i < sampleCount; i++)
                {
                    float s2 = active->read();
                    float s1 = (float(*outp) / SHRT_MAX) * sTrackVolume;
                    *outp++ = int16_t(mixSamples(s1, s2) * SHRT_MAX);
                }
            }
            else
            {
                for (size_t i = 0; i < sampleCount; i++)
                {
                    float s2 = active->read() * sPlayVolume;
                    for (size_t ci = 0; ci < numChannels; ci++)
                    {
                        float s1 = (float(*outp) / SHRT_MAX) * sTrackVolume;
                        *outp++ = int16_t(mixSamples(s1, s2) * SHRT_MAX);
                    }
                }
            }
        }
    #else
        if (numBits == 8)
        {
            printf("numBits?\n");
            uint8_t* outp = (uint8_t*)samples;
            if (numChannels == 1)
            {
                for (size_t i = 0; i < sampleCount*2; i++)
                {
                    int32_t s2 = active->read();
                    int32_t s1 = (*outp - 0x80) << 8;
                    *outp++ = uint8_t((float((s1 + s2) - (s1 * s2)) / SHRT_MAX) * UCHAR_MAX);
                }
            }
            else
            {
                for (size_t i = 0; i < sampleCount*2; i++)
                {
                    int32_t s2 = active->read();
                    for (size_t ci = 0; ci < numChannels; ci++)
                    {
                        int32_t s1 = (*outp - 0x80) << 8;
                        *outp++ = uint8_t((float((s1 + s2) - (s1 * s2)) / SHRT_MAX) * UCHAR_MAX);
                    }
                }
            }
        }
        else if (numBits == 16)
        {
            int16_t* outp = (int16_t*)samples;
            if (numChannels == 1)
            {
                for (size_t i = 0; i < sampleCount; i++)
                {
                    int32_t s2 = active->read();
                    int32_t s1 = *outp;
                    int32_t mixed = s1 + s2;
                    if (mixed > SHRT_MAX) mixed = SHRT_MAX;
                    if (mixed < SHRT_MIN) mixed = SHRT_MIN;
                    *outp++ = static_cast<int16_t>(mixed);
                }
            }
            else
            {
                for (size_t i = 0; i < sampleCount; i++)
                {
                    int32_t s2 = active->read();
                    for (size_t ci = 0; ci < numChannels; ci++)
                    {
                        int32_t s1 = *outp;
                        int32_t mixed = s1 + s2;
                        if (mixed > SHRT_MAX) mixed = SHRT_MAX;
                        if (mixed < SHRT_MIN) mixed = SHRT_MIN;
                        *outp++ = static_cast<int16_t>(mixed);
                    }
                }
            }
        }
    #endif
    }

    void audioLoop()
    {
        if (fStopStream || *fNextSong != 0)
        {
            fStopStream = false;
            Audio::stopSong();
            if (*fNextSong == '\0')
                connecttonull();
        }
        if (*fNextSong)
        {
            if (!connecttoFS(fFS, fNextSong)) {
                connecttonull();
            }
            *fNextSong = '\0';
        }

        Audio::loop();
        if (fOTAInProgress)
            AnimatedEvent::process();
    }

    static void audioLoopTask(void*)
    {
        for (;;)
        {
            WarblerAudio* active = activeWarbler();
            if (active != nullptr)
                active->audioLoop();
            vTaskDelay(1);
        }
    }
};
#endif
