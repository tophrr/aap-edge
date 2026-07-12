#pragma once

#include <Arduino.h>
#include <cstdint>
#include <deque>
#include "config.h"

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
    PulseValidationState _pulseValState;

    // Internal methods
    void _processStateMachine(int64_t frameEndSec, float frameSec,
                              bool signalPresent, bool signalRisingEdge,
                              float gapSec);
};