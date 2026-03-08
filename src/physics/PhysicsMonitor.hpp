#pragma once

#include <Geode/Geode.hpp>

using namespace geode::prelude;

class PhysicsMonitor {
public:
	static PhysicsMonitor* getInstance();

	class TimerNode : public CCNode {
	public:
		static TimerNode* create(PhysicsMonitor* monitor);
		bool init() override;
		void update(float dt) override;

	private:
		friend class PhysicsMonitor;
		PhysicsMonitor* m_monitor = nullptr;
	};

	struct PhysicsFrame {
		uint64_t timestamp_us;
		float delta_ms;
		bool is_stable;
		bool was_input;
		uint32_t input_latency_us;
		bool is_artificial_stutter;
	};

	struct InputEvent {
		uint64_t hardware_timestamp_us;
		uint64_t processed_timestamp_us;
		bool is_step_input;
		float input_latency_ms;
	};

	static constexpr float PHYSICS_HZ = 240.0f;
	static constexpr float IDEAL_FRAME_MS = 1000.0f / PHYSICS_HZ;
	static constexpr float IDEAL_FRAME_US = IDEAL_FRAME_MS * 1000.0f;
	static constexpr float JITTER_THRESHOLD_MS = 2.5f;
	static constexpr size_t MAX_HISTORY = 60;

	PhysicsMonitor();
	
	void beginPhysicsTick(float syntheticDelta_ms = -1.0f, uint64_t now_us = 0);
	void recordButtonInput(bool isStep);
	void processInputIntoPhysics(uint64_t now_us);
	
	float calculateJitter() const;
	float getAverageFrameDelta() const;
	float calculateAudioPhysicsDrift() const;
	bool isEngineStable() const;
	
	const std::deque<PhysicsFrame>& getPhysicsHistory() const;
	const std::deque<InputEvent>& getInputHistory() const;
	
	uint32_t getMissedFrames() const;
	uint32_t getTotalFrames() const;
	float getStabilityPercentage() const;
	
	void reset();

private:
	bool m_isMonitoring;
	uint64_t m_lastPhysicsTick_us;
	uint32_t m_totalFrames;
	uint32_t m_missedFrames;
	float m_audioPhysicsDrift_ms;

	std::deque<PhysicsFrame> m_physicsHistory;
	std::deque<InputEvent> m_inputHistory;
	InputEvent m_pendingInput;
};
