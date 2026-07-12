#pragma once

#include <Arduino.h>
#include <cstdint>
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
    int64_t timestamp_sec;           // Unix epoch seconds of this event
    int64_t probing_started_sec;     // Unix epoch seconds when PROBING was entered
    int64_t active_at_sec;           // Unix epoch seconds when ACTIVE was entered
    float duration_sec;              // Duration in seconds (valid only for END events)
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
    FSMEvent processFrame(const AudioFrame& frame, int64_t unix_timestamp_sec);

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
    int64_t _stateEnteredSec;
    int64_t _lastSignalSec;
    int64_t _eventStartedSec;
    float _probingActiveSec;         // accumulates 0.1s frame windows
    bool  _signalPrev;
    int   _eventsToday;
    RuntimeConfig _config;

    // Probe timestamps for verbose event payloads
    int64_t _probingStartedSec;
    int64_t _activeAtSec;

    // Pulse validation parameters and state
    static constexpr unsigned long CYCLE_TARGET_MS = 1200;
    static constexpr unsigned long CYCLE_TOLERANCE_MS = 300;
    static constexpr int REQUIRED_CYCLES = 2;
    unsigned long _lastRisingEdgeMs;
    int _validCycleCount;

    // Gap timeout debounce
    static constexpr int SIGNAL_STREAK_MIN = 2;   // consecutive frames (~40ms) to reset gap timer
    unsigned long _lastSignalMs;                   // millis() of last sustained signal
    int _signalStreak;                             // consecutive signal-present frames

    // Internal methods
    void _processStateMachine(int64_t frameEndSec, float frameSec,
                              bool signalPresent, bool signalRisingEdge,
                              float gapSec);
};