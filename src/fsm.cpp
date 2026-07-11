#include "fsm.h"
#include "config.h"
#include <cmath>

// ── Constructor ─────────────────────────────────────────────────────────────

FSMEngine::FSMEngine()
    : _state(FSM_IDLE)
    , _stateEnteredSec(0.0f)
    , _lastSignalSec(0.0f)
    , _eventStartedSec(0.0f)
    , _probingActiveSec(0.0f)
    , _signalPrev(false)
    , _eventsToday(0)
    , _probingStartedSec(0.0f)
    , _activeAtSec(0.0f)
    , _lastSignalMs(0)
    , _signalStreak(0)
{
}

// ── Public API ──────────────────────────────────────────────────────────────

void FSMEngine::reset() {
    _state = FSM_IDLE;
    _stateEnteredSec = 0.0f;
    _lastSignalSec = 0.0f;
    _eventStartedSec = 0.0f;
    _probingActiveSec = 0.0f;
    _signalPrev = false;
    _eventsToday = 0;
    _probingStartedSec = 0.0f;
    _activeAtSec = 0.0f;
    _lastSignalMs = 0;
    _signalStreak = 0;
    _signalHistory.clear();
}

void FSMEngine::applyConfig(const RuntimeConfig& cfg) {
    _config = cfg;
}

// ── Frame Processing ────────────────────────────────────────────────────────

FSMEvent FSMEngine::processFrame(const AudioFrame& frame, float unix_timestamp_sec) {
    float main_snr = frame.main_db - frame.amb_db;
    float sec_snr = frame.sec_db - frame.amb_db;
    bool signal_present = (
        main_snr >= _config.main_snr_threshold &&
        sec_snr >= _config.sec_snr_threshold
    );

    // Record signal before pulse validation
    _signalHistory.push_back(signal_present ? 1 : 0);
    while (_signalHistory.size() > HISTORY_SIZE) {
        _signalHistory.pop_front();
    }

    bool pulse_train_ok = _signalPulseIsValid(signal_present);

    // ── Verbose debug: limit serial output to avoid starving the task loop ─
    if (g_debugEnabled) {
        static int fsmFrame = 0;
        static unsigned long lastFsmPrintMs = 0;
        fsmFrame++;
        {
            static const char* stateNames[] = {"IDLE", "PROBING", "ACTIVE"};
            const char* stName = _state <= 2 ? stateNames[_state] : "???";
            unsigned long nowMs = millis();
            bool shouldPrint = fsmFrame <= 20 ||
                (nowMs - lastFsmPrintMs) >= DEBUG_PRINT_INTERVAL_MS ||
                signal_present;
            if (shouldPrint) {
                lastFsmPrintMs = nowMs;
                Serial.printf("[FSM] #%d | main_snr=%.1f sec_snr=%.1f | signal=%s pulse=%s | state=%s\n",
                    fsmFrame, (double)main_snr, (double)sec_snr,
                    signal_present ? "YES" : "no",
                    pulse_train_ok ? "OK" : "NO",
                    stName);
            }
        }
    }

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

    // Save previous state to detect transitions
    FSMState prevState = _state;

    _processStateMachine(unix_timestamp_sec, WINDOW_SEC, signal_present, pulse_train_ok, gapSec);

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

// ── Pulse Validation ────────────────────────────────────────────────────────

bool FSMEngine::_signalPulseIsValid(bool currentSignalPresent) {
    if (!currentSignalPresent) return false;

    // Build a copy with current signal appended
    auto samples = _signalHistory;
    samples.push_back(1);

    if (samples.size() < 3) return false;

    // Count transitions
    int transitions = 0;
    for (size_t i = 1; i < samples.size(); ++i) {
        if (samples[i] != samples[i - 1]) ++transitions;
    }
    return transitions >= 3;
}

// ── State Machine ───────────────────────────────────────────────────────────

void FSMEngine::_processStateMachine(
    float frameEndSec, float frameSec,
    bool signalPresent, bool pulseTrainOk,
    float gapSec
) {
    bool signalRisingEdge = signalPresent && !_signalPrev;
    _signalPrev = signalPresent;

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
                DEBUG_PRINTF("[FSM] IDLE \u2192 PROBING (rising edge @ %.1f)\n", (double)frameEndSec);
            }
            break;

        case FSM_PROBING: {
            if (!signalPresent && gapSec > _config.probing_timeout_sec) {
                _state = FSM_IDLE;
                DEBUG_PRINTF("[FSM] PROBING \u2192 IDLE (gap %.1fs > %.1fs)\n",
                    (double)gapSec, (double)_config.probing_timeout_sec);
            } else if (signalPresent && !pulseTrainOk &&
                       _probingActiveSec > _config.probing_timeout_sec) {
                _state = FSM_IDLE;
                DEBUG_PRINTF("[FSM] PROBING \u2192 IDLE (no pulse after %.1fs)\n",
                    (double)_probingActiveSec);
            } else if (pulseTrainOk && signalPresent &&
                       _probingActiveSec > _config.confirm_sec) {
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
                DEBUG_PRINTF("[FSM] ACTIVE \u2192 IDLE (gap %.1fs > %.1fs)\n",
                    (double)gapSec, (double)_config.active_timeout_sec);
            }
            break;
        }
    }
}