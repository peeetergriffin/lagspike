#include <asp/time/SystemTime.hpp>
#include "PhysicsMonitor.hpp"

using namespace geode::prelude;

PhysicsMonitor* g_physicsMonitorInstance = nullptr;

PhysicsMonitor* PhysicsMonitor::getInstance() {
    if (!g_physicsMonitorInstance) {
        g_physicsMonitorInstance = new PhysicsMonitor();
    }
    return g_physicsMonitorInstance;
}

PhysicsMonitor::PhysicsMonitor() 
    : m_isMonitoring(false),
      m_lastPhysicsTick_us(0),
      m_totalFrames(0),
      m_missedFrames(0),
      m_audioPhysicsDrift_ms(0) {}

PhysicsMonitor::TimerNode* PhysicsMonitor::TimerNode::create(PhysicsMonitor* monitor) {
    auto node = new TimerNode();
    node->m_monitor = monitor;
    if (node->init()) {
        node->autorelease();
        return node;
    }
    delete node;
    return nullptr;
}

bool PhysicsMonitor::TimerNode::init() {
    if (!CCNode::init()) return false;
    this->schedule(schedule_selector(TimerNode::update), 0.0f);
    return true;
}

void PhysicsMonitor::TimerNode::update(float dt) {
    if (m_monitor) {
        float perTickDelta_ms = dt * 250.0f;
        uint64_t totalFrame_us = static_cast<uint64_t>(dt * 1000000.0f);
        uint64_t subTickStep_us = totalFrame_us / 4;

        auto frameTime = asp::time::SystemTime::now();
        uint64_t base_now_us = frameTime.timeSinceEpoch().micros();
        
        uint64_t frame_start_us = base_now_us - totalFrame_us;
        
        for (int i = 0; i < 4; i++) {
            uint64_t current_tick_us = frame_start_us + (subTickStep_us * (i + 1));
            
            m_monitor->beginPhysicsTick(perTickDelta_ms, current_tick_us);
            m_monitor->processInputIntoPhysics(current_tick_us);
        }
    }
}

void PhysicsMonitor::beginPhysicsTick(float syntheticDelta_ms, uint64_t now_us) {
    if (now_us == 0) {
        now_us = asp::time::SystemTime::now().timeSinceEpoch().micros();
    }

    float delta_ms = syntheticDelta_ms;
    
    if (delta_ms < 0) {
        if (m_lastPhysicsTick_us > 0) {
            uint64_t delta_us = now_us - m_lastPhysicsTick_us;
            delta_ms = delta_us / 1000.0f;
        } else {
            delta_ms = IDEAL_FRAME_MS;
        }
    }

    if (m_lastPhysicsTick_us > 0 || syntheticDelta_ms >= 0) {
        float deviation = std::abs(delta_ms - IDEAL_FRAME_MS);
        bool is_stable = deviation < JITTER_THRESHOLD_MS;

        if (!is_stable) {
            m_missedFrames++;
        }

        PhysicsFrame frame = {
            now_us,
            delta_ms,
            is_stable,
            false,
            0,
            false
        };

        m_physicsHistory.push_back(frame);
        if (m_physicsHistory.size() > MAX_HISTORY) {
            m_physicsHistory.pop_front();
        }
        
        if (m_totalFrames < 5 || m_totalFrames % 60 == 0) {
            float jitter = calculateJitter();
            log::debug("PhysicsTick #{}: delta={:.3f}ms, deviation={:.3f}ms, stable={}, jitter={:.3f}ms",
                m_totalFrames, delta_ms, deviation, is_stable ? "Y" : "N", jitter);
        }
    }

    m_lastPhysicsTick_us = now_us;
    m_totalFrames++;
    m_isMonitoring = true;
}

void PhysicsMonitor::recordButtonInput(bool isStep) {
    auto now = asp::time::SystemTime::now();
    uint64_t hardware_time_us = now.timeSinceEpoch().micros();

    InputEvent event = {
        hardware_time_us,
        0,
        isStep,
        0
    };

    m_pendingInput = event;
}

void PhysicsMonitor::processInputIntoPhysics(uint64_t now_us) {
    if (m_pendingInput.hardware_timestamp_us > 0) {
        if (now_us == 0) {
            now_us = asp::time::SystemTime::now().timeSinceEpoch().micros();
        }

        m_pendingInput.processed_timestamp_us = now_us;
        m_pendingInput.input_latency_ms = 
            (m_pendingInput.processed_timestamp_us - m_pendingInput.hardware_timestamp_us) / 1000.0f;

        if (!m_physicsHistory.empty()) {
            auto& lastFrame = m_physicsHistory.back();
            lastFrame.was_input = true;
            lastFrame.input_latency_us = m_pendingInput.processed_timestamp_us - m_pendingInput.hardware_timestamp_us;
            log::info("Input recorded: {:.3f}ms latency", m_pendingInput.input_latency_ms);
        }

        m_inputHistory.push_back(m_pendingInput);
        if (m_inputHistory.size() > MAX_HISTORY) {
            m_inputHistory.pop_front();
        }

        m_pendingInput = InputEvent();
    }
}

float PhysicsMonitor::calculateJitter() const {
    if (m_physicsHistory.size() < 2) return 0.0f;

    float mean = 0.0f;
    for (const auto& frame : m_physicsHistory) mean += frame.delta_ms;
    mean /= m_physicsHistory.size();

    float variance = 0.0f;
    for (const auto& frame : m_physicsHistory) {
        float diff = frame.delta_ms - mean;
        variance += diff * diff;
    }
    variance /= m_physicsHistory.size();

    return std::sqrt(variance);
}

float PhysicsMonitor::getAverageFrameDelta() const {
    if (m_physicsHistory.empty()) return IDEAL_FRAME_MS;
    float sum = 0.0f;
    for (const auto& frame : m_physicsHistory) sum += frame.delta_ms;
    return sum / m_physicsHistory.size();
}

float PhysicsMonitor::calculateAudioPhysicsDrift() const {
    if (m_physicsHistory.size() < 2) return 0.0f;
    
    float totalDelta = 0.0f;
    int validFrames = 0;
    for (const auto& frame : m_physicsHistory) {
        if (frame.delta_ms <= 50.0f) {
            totalDelta += frame.delta_ms;
            validFrames++;
        }
    }
    
    if (validFrames == 0) return 0.0f;
    
    return totalDelta - (validFrames * IDEAL_FRAME_MS);
}

bool PhysicsMonitor::isEngineStable() const {
    if (m_physicsHistory.size() < 10) return false;
    return (calculateJitter() < JITTER_THRESHOLD_MS) && (getStabilityPercentage() >= 95.0f);
}

const std::deque<PhysicsMonitor::PhysicsFrame>& PhysicsMonitor::getPhysicsHistory() const {
    return m_physicsHistory;
}

const std::deque<PhysicsMonitor::InputEvent>& PhysicsMonitor::getInputHistory() const {
    return m_inputHistory;
}

uint32_t PhysicsMonitor::getMissedFrames() const {
    return m_missedFrames;
}

uint32_t PhysicsMonitor::getTotalFrames() const {
    return m_totalFrames;
}

float PhysicsMonitor::getStabilityPercentage() const {
    if (m_totalFrames == 0) return 100.0f;
    return 100.0f * (m_totalFrames - m_missedFrames) / m_totalFrames;
}

void PhysicsMonitor::reset() {
    m_physicsHistory.clear();
    m_inputHistory.clear();
    m_lastPhysicsTick_us = 0;
    m_totalFrames = 0;
    m_missedFrames = 0;
    m_audioPhysicsDrift_ms = 0;
    m_isMonitoring = false;
}