/**
 * @file CutterHal_ClearCore.cpp
 * @brief ClearCore hardware implementation of HAL
 *
 * This file is only compiled when CUTTER_PLATFORM_CLEARCORE is defined.
 */

#ifdef CUTTER_PLATFORM_CLEARCORE

#include "CutterHal.h"
#include "ClearCore.h"

namespace CutterHal {

// Pin index to Connector mapping
static ClearCore::Connector* GetConnector(uint8_t pin) {
    static ClearCore::Connector* connectors[] = {
        &ConnectorIO0, &ConnectorIO1, &ConnectorIO2, &ConnectorIO3,
        &ConnectorIO4, &ConnectorIO5, &ConnectorDI6, &ConnectorDI7,
        &ConnectorDI8, &ConnectorA9, &ConnectorA10, &ConnectorA11,
        &ConnectorA12
    };
    if (pin < 13) {
        return connectors[pin];
    }
    return nullptr;
}

// Motor index to MotorDriver mapping
static ClearCore::MotorDriver* GetMotor(uint8_t motor) {
    static ClearCore::MotorDriver* motors[] = {
        &ConnectorM0, &ConnectorM1, &ConnectorM2, &ConnectorM3
    };
    if (motor < 4) {
        return motors[motor];
    }
    return nullptr;
}

// Convert HAL pin index to ClearCorePins enum
static ClearCorePins ToClearCorePin(uint8_t pin) {
    if (pin == PIN_INVALID) {
        return CLEARCORE_PIN_INVALID;
    }
    return static_cast<ClearCorePins>(pin);
}

bool ReadDigitalPin(uint8_t pin) {
    auto* conn = GetConnector(pin);
    if (conn) {
        return conn->State();
    }
    return false;
}

bool InputRisen(uint8_t pin) {
    if (pin >= 6 && pin <= 12) {
        // Pins 6-12 are DigitalIn or DigitalInAnalogIn
        auto* conn = static_cast<ClearCore::DigitalIn*>(GetConnector(pin));
        if (conn) {
            return conn->InputRisen();
        }
    }
    return false;
}

bool InputFallen(uint8_t pin) {
    if (pin >= 6 && pin <= 12) {
        auto* conn = static_cast<ClearCore::DigitalIn*>(GetConnector(pin));
        if (conn) {
            return conn->InputFallen();
        }
    }
    return false;
}

void WriteDigitalPin(uint8_t pin, bool value) {
    auto* conn = GetConnector(pin);
    if (conn) {
        conn->State(value);
    }
}

int16_t ReadAnalogPin(uint8_t pin) {
    // Pins 9-12 are analog capable (DigitalInAnalogIn type)
    if (pin >= 9 && pin <= 12) {
        auto* conn = static_cast<ClearCore::DigitalInAnalogIn*>(GetConnector(pin));
        if (conn) {
            // AnalogVoltage returns millivolts, convert to 0-4095 range
            // ClearCore analog range is 0-10V, so 10000mV max
            int32_t mv = conn->AnalogVoltage();
            return static_cast<int16_t>((mv * 4095) / 10000);
        }
    }
    return 0;
}

void SetPwmDuty(uint8_t pin, uint16_t duty) {
    auto* conn = GetConnector(pin);
    if (conn && pin <= 5) {
        // PWM expects 0-255 range
        uint8_t duty8 = static_cast<uint8_t>(duty >> 8);
        conn->State(duty8);
    }
}

void SetHBridgeValue(uint8_t pin, int16_t value) {
    if (pin == 4 || pin == 5) {
        auto* conn = static_cast<ClearCore::DigitalInOutHBridge*>(GetConnector(pin));
        if (conn) {
            conn->State(value);
        }
    }
}

void StartTone(uint8_t pin, uint16_t freq, int16_t amplitude) {
    if (pin == 4 || pin == 5) {
        auto* conn = static_cast<ClearCore::DigitalInOutHBridge*>(GetConnector(pin));
        if (conn) {
            conn->ToneAmplitude(amplitude);
            conn->ToneContinuous(freq);
        }
    }
}

void StopTone(uint8_t pin) {
    if (pin == 4 || pin == 5) {
        auto* conn = static_cast<ClearCore::DigitalInOutHBridge*>(GetConnector(pin));
        if (conn) {
            conn->ToneStop();
        }
    }
}

void ConfigurePinMode(uint8_t pin, uint8_t mode) {
    auto* conn = GetConnector(pin);
    if (conn) {
        conn->Mode(static_cast<ClearCore::Connector::ConnectorModes>(mode));
    }
}

void SetMotorClockRate(uint8_t rate) {
    ClearCore::MotorManager::MotorClockRates clockRate;
    switch (rate) {
        case CLOCK_RATE_LOW:
            clockRate = ClearCore::MotorManager::CLOCK_RATE_LOW;
            break;
        case CLOCK_RATE_HIGH:
            clockRate = ClearCore::MotorManager::CLOCK_RATE_HIGH;
            break;
        case CLOCK_RATE_NORMAL:
        default:
            clockRate = ClearCore::MotorManager::CLOCK_RATE_NORMAL;
            break;
    }
    ClearCore::MotorMgr.MotorInputClocking(clockRate);
}

void EnableMotor(uint8_t motor, bool enable) {
    auto* m = GetMotor(motor);
    if (m) {
        m->EnableRequest(enable);
    }
}

bool MoveRelative(uint8_t motor, int32_t steps) {
    auto* m = GetMotor(motor);
    if (m) {
        return m->Move(steps);
    }
    return false;
}

bool MoveAbsolute(uint8_t motor, int32_t position) {
    auto* m = GetMotor(motor);
    if (m) {
        return m->Move(position, ClearCore::MotorDriver::MOVE_TARGET_ABSOLUTE);
    }
    return false;
}

bool MoveVelocity(uint8_t motor, int32_t velocity) {
    auto* m = GetMotor(motor);
    if (m) {
        return m->MoveVelocity(velocity);
    }
    return false;
}

void StopMotor(uint8_t motor, bool immediate) {
    auto* m = GetMotor(motor);
    if (m) {
        if (immediate) {
            m->MoveStopAbrupt();
        } else {
            m->MoveStopDecel();
        }
    }
}

void SetMotorParams(uint8_t motor, int32_t velMax, int32_t accelMax) {
    auto* m = GetMotor(motor);
    if (m) {
        m->VelMax(velMax);
        m->AccelMax(accelMax);
    }
}

int32_t GetMotorPosition(uint8_t motor) {
    auto* m = GetMotor(motor);
    if (m) {
        return m->PositionRefCommanded();
    }
    return 0;
}

void SetMotorPosition(uint8_t motor, int32_t position) {
    auto* m = GetMotor(motor);
    if (m) {
        m->PositionRefSet(position);
    }
}

bool StepsComplete(uint8_t motor) {
    auto* m = GetMotor(motor);
    if (m) {
        return m->StepsComplete();
    }
    return true;
}

void SetHlfbMode(uint8_t motor, uint8_t mode) {
    auto* m = GetMotor(motor);
    if (m) {
        ClearCore::MotorDriver::HlfbModes ccMode;
        switch (mode) {
            case HLFB_MODE_HAS_BIPOLAR_PWM:
                ccMode = ClearCore::MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM;
                break;
            case HLFB_MODE_STATIC:
            default:
                ccMode = ClearCore::MotorDriver::HLFB_MODE_STATIC;
                break;
        }
        m->HlfbMode(ccMode);
    }
}

uint8_t GetHlfbState(uint8_t motor) {
    auto* m = GetMotor(motor);
    if (m) {
        return static_cast<uint8_t>(m->HlfbState());
    }
    return 0;
}

bool IsMotorReady(uint8_t motor) {
    auto* m = GetMotor(motor);
    if (m) {
        return m->StatusReg().bit.ReadyState;
    }
    return false;
}

bool IsMotorInFault(uint8_t motor) {
    auto* m = GetMotor(motor);
    if (m) {
        return m->IsInHwFault();
    }
    return false;
}

bool HasMotorAlerts(uint8_t motor) {
    auto* m = GetMotor(motor);
    if (m) {
        return m->StatusReg().bit.AlertsPresent;
    }
    return false;
}

bool IsPinInFault(uint8_t pin) {
    // Only IO-0 to IO-5 have overcurrent detection
    if (pin <= 5) {
        auto* conn = static_cast<ClearCore::DigitalInOut*>(GetConnector(pin));
        if (conn) {
            return conn->IsInHwFault();
        }
    }
    return false;
}

// === Limit Switches ===

bool SetLimitSwitchNeg(uint8_t motor, uint8_t pin) {
    auto* m = GetMotor(motor);
    if (m) {
        return m->LimitSwitchNeg(ToClearCorePin(pin));
    }
    return false;
}

bool SetLimitSwitchPos(uint8_t motor, uint8_t pin) {
    auto* m = GetMotor(motor);
    if (m) {
        return m->LimitSwitchPos(ToClearCorePin(pin));
    }
    return false;
}

bool InPosLimit(uint8_t motor) {
    auto* m = GetMotor(motor);
    if (m) {
        return m->StatusReg().bit.InPositiveLimit;
    }
    return false;
}

bool InNegLimit(uint8_t motor) {
    auto* m = GetMotor(motor);
    if (m) {
        return m->StatusReg().bit.InNegativeLimit;
    }
    return false;
}

bool HasMotionCanceledNegLimit(uint8_t motor) {
    auto* m = GetMotor(motor);
    if (m) {
        return m->AlertReg().bit.MotionCanceledNegativeLimit;
    }
    return false;
}

bool HasMotionCanceledPosLimit(uint8_t motor) {
    auto* m = GetMotor(motor);
    if (m) {
        return m->AlertReg().bit.MotionCanceledPositiveLimit;
    }
    return false;
}

void ClearMotorAlerts(uint8_t motor) {
    auto* m = GetMotor(motor);
    if (m) {
        m->ClearAlerts();
    }
}

// === E-Stop ===

bool SetMotorEStop(uint8_t motor, uint8_t pin) {
    auto* m = GetMotor(motor);
    if (m) {
        return m->EStopConnector(ToClearCorePin(pin));
    }
    return false;
}

void SetMotorEStopDecel(uint8_t motor, uint32_t decel) {
    auto* m = GetMotor(motor);
    if (m) {
        m->EStopDecelMax(decel);
    }
}

bool HasMotionCanceledEStop(uint8_t motor) {
    auto* m = GetMotor(motor);
    if (m) {
        return m->AlertReg().bit.MotionCanceledSensorEStop;
    }
    return false;
}

// === Timing ===

uint32_t Milliseconds() {
    return ClearCore::SysTiming::Instance().Milliseconds();
}

}  // namespace CutterHal

#endif  // CUTTER_PLATFORM_CLEARCORE
