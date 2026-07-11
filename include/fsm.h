#pragma once

#include <Arduino.h>
#include <deque>
#include "config.h"

// ── FSM State ───────────────────────────────────────────────────────────────

enum FSMState : uint8_t {
    FSM_IDLE = 0,
    FSM_PROBING = 1,
    FSM_ACTIVE = 2
};

// ── Audio Frame (IPC from Core 0 → Core 1) ─────────────────────────────────

struct AudioFrame {
    float main_db;
    float sec_db;
    float amb_db;
};

// ── FSM Event Output ────────────────────────────────────────────────────────

struct FSMEvent {
    enum Type : uint8_t { NONE = 0, START = 1, END = 2 };
    Type type;
    float timestamp_sec;           // Unix timestamp of this event
    float probing_started_sec;     // Unix timestamp when PROBING was entered
    float active_at_sec;           // Unix timestamp when ACTIVE was entered
    float duration_sec;            // Valid only for END events
};

// ── Runtime Config (mutable via MQTT) ───────────────────────────────────────

struct RuntimeConfig {
    float main_snr_threshold = MAIN_SNR_DB;
    float sec_snr_threshold = SEC_SNR_DB;
    float confirm_sec = CONFIRM_SEC;
    float probing_timeout_sec = PROBING_TIMEOUT_SEC;
    float active_timeout_sec = ACTIVE_TIMEOUT_SEC;
    bool debug_enabled = false;
};

// ── FSM Engine ──────────────────────────────────────────────────────────────

class FSMEngine {
public:
    FSMEngine();

    /// Process one frame of audio, return an event if state transition occurs.
    FSMEvent processFrame(const AudioFrame& frame, float unix_timestamp_sec);

    /// Reset to IDLE with all history cleared.
    void reset();

    /// Apply a new runtime configuration (from MQTT config).
    void applyConfig(const RuntimeConfig& cfg);

    /// Return current runtime configuration.
    const RuntimeConfig& getConfig() const { return _config; }

    /// Return current FSM state.
    int state() const { return _state; }

    /// Return number of START events detected since boot.
    int eventsToday() const { return _eventsToday; }

private:
    // State
    FSMState _state;
    float _stateEnteredSec;
    float _lastSignalSec;
    float _eventStartedSec;
    float _probingActiveSec;
    bool  _signalPrev;
    int   _eventsToday;
    RuntimeConfig _config;

    // Probe timestamps for verbose event payloads
    float _probingStartedSec;
    float _activeAtSec;

    // Ring buffer for pulse validation (12 seconds @ 0.1s frames)
    static constexpr size_t HISTORY_SIZE = 120;
    std::deque<uint8_t> _signalHistory;

    // Gap timeout debounce
    static constexpr int SIGNAL_STREAK_MIN = 3;   // consecutive frames (~60ms) to reset gap timer
    unsigned long _lastSignalMs;                   // millis() of last sustained signal
    int _signalStreak;                             // consecutive signal-present frames

    // Internal methods
    bool _signalPulseIsValid(bool currentSignalPresent);
    void _processStateMachine(float frameEndSec, float frameSec,
                              bool signalPresent, bool pulseTrainOk,
                              float gapSec);
};