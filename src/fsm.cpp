#include "fsm.h"
#include "config.h"
#include <cmath>

// ── Constructor ─────────────────────────────────────────────────────────────

FSMEngine::FSMEngine()
    : _state(FSM_IDLE)
    , _stateEnteredSec(0)
    , _lastSignalSec(0)
    , _eventStartedSec(0)
    , _probingActiveSec(0.0f)
    , _signalPrev(false)
    , _eventsToday(0)
    , _probingStartedSec(0)
    , _activeAtSec(0)
    , _lastRisingEdgeMs(0)
    , _validCycleCount(0)
    , _lastSignalMs(0)
    , _signalStreak(0)
{
}

// ── Public API ──────────────────────────────────────────────────────────────

void FSMEngine::reset() {
    _state = FSM_IDLE;
    _stateEnteredSec = 0;
    _lastSignalSec = 0;
    _eventStartedSec = 0;
    _probingActiveSec = 0.0f;
    _signalPrev = false;
    _eventsToday = 0;
    _probingStartedSec = 0;
    _activeAtSec = 0;
    _lastRisingEdgeMs = 0;
    _validCycleCount = 0;
    _lastSignalMs = 0;
    _signalStreak = 0;
}

void FSMEngine::applyConfig(const RuntimeConfig& cfg) {
    _config = cfg;
}

// ── Frame Processing ────────────────────────────────────────────────────────

FSMEvent FSMEngine::processFrame(const AudioFrame& frame, int64_t unix_timestamp_sec) {
    float main_snr = frame.main_db - frame.amb_db;
    float sec_snr = frame.sec_db - frame.amb_db;
    bool signal_present = (
        main_snr >= _config.main_snr_threshold &&
        sec_snr >= _config.sec_snr_threshold
    );

    // Compute gap using millis() with debounce:
    // Require SIGNAL_STREAK_MIN consecutive signal-present frames
    // before resetting the gap timer, preventing brief noise spikes
    // from restarting the timeout.
    unsigned long nowMs = millis();
    if (signal_present) {
        _signalStreak = min(_signalStreak + 1, SIGNAL_STREAK_MIN);
    } else {
        _signalStreak = 0;
    }
    if (_signalStreak >= SIGNAL_STREAK_MIN) {
        _lastSignalMs = nowMs;
    }
    float gapSec = (nowMs - _lastSignalMs) / 1000.0f;

    bool debouncedSignal = (_signalStreak >= SIGNAL_STREAK_MIN);
    bool signalRisingEdge = debouncedSignal && !_signalPrev;
    _signalPrev = debouncedSignal;

    if (signalRisingEdge) {
        if (_lastRisingEdgeMs == 0) {
            _validCycleCount = 0;
            DEBUG_PRINTF("[FSM] First rising edge @ %lu ms\n", nowMs);
        } else {
            unsigned long deltaMs = nowMs - _lastRisingEdgeMs;
            if (deltaMs >= (CYCLE_TARGET_MS - CYCLE_TOLERANCE_MS) &&
                deltaMs <= (CYCLE_TARGET_MS + CYCLE_TOLERANCE_MS)) {
                _validCycleCount++;
                DEBUG_PRINTF("[FSM] Valid cycle: deltaMs=%lu ms (count=%d)\n", deltaMs, _validCycleCount);
            } else {
                _validCycleCount = 0;
                DEBUG_PRINTF("[FSM] Invalid cycle: deltaMs=%lu ms (resetting count)\n", deltaMs);
            }
        }
        _lastRisingEdgeMs = nowMs;
    }

    // ── Verbose debug: limit serial output to avoid starving the task loop ─
    if (g_debugEnabled) {
        static int fsmFrame = 0;
        static unsigned long lastFsmPrintMs = 0;
        fsmFrame++;
        {
            static const char* stateNames[] = {"IDLE", "PROBING", "ACTIVE"};
            const char* stName = _state <= 2 ? stateNames[_state] : "???";
            bool shouldPrint = fsmFrame <= 20 ||
                (nowMs - lastFsmPrintMs) >= DEBUG_PRINT_INTERVAL_MS ||
                signal_present;
            if (shouldPrint) {
                lastFsmPrintMs = nowMs;
                Serial.printf("[FSM] #%d | main_snr=%.1f sec_snr=%.1f | signal=%s cycles=%d | state=%s\n",
                    fsmFrame, (double)main_snr, (double)sec_snr,
                    signal_present ? "YES" : "no",
                    _validCycleCount,
                    stName);
            }
        }
    }

    // Save previous state to detect transitions
    FSMState prevState = _state;

    _processStateMachine(unix_timestamp_sec, WINDOW_SEC, signal_present, signalRisingEdge, gapSec);

    // Build event based on transition
    FSMEvent event;
    event.type = FSMEvent::NONE;
    event.timestamp_sec = unix_timestamp_sec;
    event.probing_started_sec = _probingStartedSec;
    event.active_at_sec = _activeAtSec;
    event.duration_sec = 0.0f;

    if (prevState == FSM_PROBING && _state == FSM_ACTIVE) {
        event.type = FSMEvent::START;
    } else if (prevState == FSM_ACTIVE && _state == FSM_IDLE) {
        event.type = FSMEvent::END;
        event.duration_sec = _lastSignalSec - _eventStartedSec;
    }

    return event;
}

// ── State Machine ───────────────────────────────────────────────────────────

void FSMEngine::_processStateMachine(
    int64_t frameEndSec, float frameSec,
    bool signalPresent, bool signalRisingEdge,
    float gapSec
) {
    if (signalPresent) {
        _lastSignalSec = frameEndSec;
        if (_state == FSM_PROBING) {
            _probingActiveSec += frameSec;
        }
    }

    switch (_state) {
        case FSM_IDLE:
            if (signalRisingEdge) {
                _state = FSM_PROBING;
                _stateEnteredSec = frameEndSec;
                _probingStartedSec = frameEndSec;
                _probingActiveSec = 0.0f;
                _lastSignalMs = millis();
                DEBUG_PRINTF("[FSM] IDLE \u2192 PROBING (rising edge @ %lld)\n", (long long)frameEndSec);
            }
            break;

        case FSM_PROBING: {
            if (!signalPresent && gapSec > _config.probing_timeout_sec) {
                _state = FSM_IDLE;
                _validCycleCount = 0;
                _lastRisingEdgeMs = 0;
                DEBUG_PRINTF("[FSM] PROBING \u2192 IDLE (gap %.1fs > %.1fs)\n",
                    (double)gapSec, (double)_config.probing_timeout_sec);
            } else if (_probingActiveSec > _config.probing_timeout_sec) {
                _state = FSM_IDLE;
                _validCycleCount = 0;
                _lastRisingEdgeMs = 0;
                DEBUG_PRINTF("[FSM] PROBING \u2192 IDLE (no pulse after %.1fs)\n",
                    (double)_probingActiveSec);
            } else if (_validCycleCount >= REQUIRED_CYCLES) {
                _state = FSM_ACTIVE;
                _stateEnteredSec = frameEndSec;
                _activeAtSec = frameEndSec;
                _eventStartedSec = frameEndSec;
                _lastSignalMs = millis();
                ++_eventsToday;
                DEBUG_PRINTF("[FSM] PROBING \u2192 ACTIVE (pulse confirmed @ %.1fs)\n",
                    (double)_probingActiveSec);
            }
            break;
        }

        case FSM_ACTIVE: {
            if (gapSec > _config.active_timeout_sec) {
                _state = FSM_IDLE;
                _validCycleCount = 0;
                _lastRisingEdgeMs = 0;
                DEBUG_PRINTF("[FSM] ACTIVE \u2192 IDLE (gap %.1fs > %.1fs)\n",
                    (double)gapSec, (double)_config.active_timeout_sec);
            }
            break;
        }
    }
}