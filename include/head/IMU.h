#pragma once

////////////////////////////////

#include "core/AnimatedEvent.h"
#include "core/SetupEvent.h"
#include "Adafruit_BNO08x.h"

////////////////////////////////

class IMU: protected Adafruit_BNO08x, public AnimatedEvent, public SetupEvent {
public:
    struct Orientation {
        float x;
        float y;
        float z;
        int8_t accuracy;
    };

    IMU() : Adafruit_BNO08x(-1) {}

    virtual void setup() override {
        if (!begin_I2C()) {
            DEBUG_PRINTLN("Failed to find BNO08x chip");
            fReportIntervalUs = 0;
        }
        DEBUG_PRINTLN("BNO08x Found!");
    }

    virtual void animate() override {
        fHasOrientation = false;
        if (fReportIntervalUs) {
            if (wasReset()) {
                setReports(fReportType, fReportIntervalUs);
            }
            sh2_SensorValue_t sensorValue;
            if (getSensorEvent(&sensorValue)) {
                switch (sensorValue.sensorId) {
                    case SH2_ARVR_STABILIZED_RV:
                        quaternionToEulerRV(sensorValue.un.arvrStabilizedRV, fEuler, false);
                    case SH2_GYRO_INTEGRATED_RV:
                        // faster (more noise?)
                        quaternionToEulerGI(sensorValue.un.gyroIntegratedRV, fEuler, false);
                        break;
                }
                fOrientation.x = fEuler.pitch;
                fOrientation.y = -fEuler.roll;
                fOrientation.z = fEuler.yaw;
                fOrientation.accuracy = sensorValue.status;
                fHasOrientation = true;
            }
        }
    }

    inline bool isValid() const {
        return (fReportIntervalUs != 0);
    }

    inline bool hasOrientation() const {
        return fHasOrientation;
    }

    inline float x() const {
        return fOrientation.x;
    }

    inline float y() const {
        return fOrientation.y;
    }

    inline float z() const {
        return fOrientation.z;
    }

    inline int accuracy() const {
        return fOrientation.accuracy;
    }

    Orientation orientation() const {
        static Orientation sZero;
        return (fHasOrientation) ? fOrientation : sZero;
    }

protected:
    struct Euler {
        float yaw;
        float pitch;
        float roll;
    };

    Euler fEuler;
    Orientation fOrientation;
    bool fHasOrientation = false;
    sh2_SensorValue_t fSensorValue;
    sh2_SensorId_t fReportType = SH2_ARVR_STABILIZED_RV;
    long fReportIntervalUs = 5000;

    void setReports(sh2_SensorId_t reportType, long report_interval) {
        DEBUG_PRINTLN("Setting desired reports");
        if (!enableReport(reportType, report_interval)) {
            DEBUG_PRINTLN("Could not enable stabilized remote vector");
        }
    }

    void quaternionToEuler(float qr, float qi, float qj, float qk, Euler &euler, bool degrees = false) {
        float sqr = sq(qr);
        float sqi = sq(qi);
        float sqj = sq(qj);
        float sqk = sq(qk);

        euler.yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
        euler.pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
        euler.roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

        if (degrees) {
            euler.yaw *= RAD_TO_DEG;
            euler.pitch *= RAD_TO_DEG;
            euler.roll *= RAD_TO_DEG;
        }
    }

    void quaternionToEulerRV(const sh2_RotationVectorWAcc_t &rotational_vector, Euler &euler, bool degrees = false) {
        quaternionToEuler(
            rotational_vector.real,
            rotational_vector.i,
            rotational_vector.j,
            rotational_vector.k,
            euler,
            degrees);
    }

    void quaternionToEulerGI(const sh2_GyroIntegratedRV_t &rotational_vector, Euler &euler, bool degrees = false) {
        quaternionToEuler(
            rotational_vector.real,
            rotational_vector.i,
            rotational_vector.j,
            rotational_vector.k,
            euler,
            degrees);
    }
};
