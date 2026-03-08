#include "PhysicsOverlay.hpp"
#include "physics/PhysicsMonitor.hpp"

using namespace geode::prelude;

PhysicsOverlay* g_physicsOverlay = nullptr;

PhysicsOverlay* PhysicsOverlay::create() {
	auto pRet = new PhysicsOverlay();
	if (pRet && pRet->init()) {
		pRet->autorelease();
		return pRet;
	}
	delete pRet;
	return nullptr;
}

bool PhysicsOverlay::init() {
	if (!CCNode::init()) return false;

	m_showGraph = true;
	m_graphNode = CCDrawNode::create();
	m_graphNode->setPosition(10, 90);
	this->addChild(m_graphNode);

	m_statusLabel = CCLabelBMFont::create("", "bigFont.fnt");
	m_statusLabel->setPosition(ccp(100, 50));
	m_statusLabel->setScale(0.5f);
	this->addChild(m_statusLabel);

	m_jitterLabel = CCLabelBMFont::create("", "bigFont.fnt");
	m_jitterLabel->setPosition(ccp(100, 30));
	m_jitterLabel->setScale(0.5f);
	this->addChild(m_jitterLabel);

	m_driftLabel = CCLabelBMFont::create("", "bigFont.fnt");
	m_driftLabel->setPosition(ccp(100, 10));
	m_driftLabel->setScale(0.5f);
	this->addChild(m_driftLabel);

	m_stabilityLabel = CCLabelBMFont::create("", "bigFont.fnt");
	m_stabilityLabel->setPosition(ccp(100, 70));
	m_stabilityLabel->setScale(0.5f);
	this->addChild(m_stabilityLabel);

	this->schedule(schedule_selector(PhysicsOverlay::updateDisplay), 0.016f);

	return true;
}

void PhysicsOverlay::updateDisplay(float dt) {
	auto monitor = PhysicsMonitor::getInstance();

	float jitter = monitor->calculateJitter();
	char jitterStr[64];
	snprintf(jitterStr, sizeof(jitterStr), "Jitter: %.3f ms", jitter);
	m_jitterLabel->setCString(jitterStr);

	float stability = monitor->getStabilityPercentage();
	char stabilityStr[64];
	snprintf(stabilityStr, sizeof(stabilityStr), "Stability: %.1f%%", stability);
	m_stabilityLabel->setCString(stabilityStr);

	float drift = monitor->calculateAudioPhysicsDrift();
	char driftStr[64];
	snprintf(driftStr, sizeof(driftStr), "A/P Drift: %.2f ms", drift);
	m_driftLabel->setCString(driftStr);

	bool isStable = monitor->isEngineStable();
	uint32_t totalFrames = monitor->getTotalFrames();
	char statusStr[64];
	snprintf(statusStr, sizeof(statusStr), isStable ? "[STABLE]" : "[UNSTABLE-J]");
	m_statusLabel->setCString(statusStr);
	m_statusLabel->setColor(isStable ? ccGREEN : ccRED);

	static uint32_t lastLogFrame = 0;
	if (totalFrames - lastLogFrame >= 60) {
		log::debug("DisplayUpdate: stability={:.1f}%, jitter={:.3f}ms, stable={}, frames={}",
			stability, jitter, isStable ? "Y" : "N", totalFrames);
		lastLogFrame = totalFrames;
	}

	if (m_showGraph) {
		redrawGraph();
	}
}

void PhysicsOverlay::redrawGraph() {
	m_graphNode->clear();

	auto monitor = PhysicsMonitor::getInstance();
	const auto& history = monitor->getPhysicsHistory();

	if (history.empty()) return;

	float graphWidth = 150;
	float graphHeight = 60;
	float pixelPerFrame = graphWidth / PhysicsMonitor::MAX_HISTORY;

	m_graphNode->drawSegment(ccp(0, 0), ccp(graphWidth, 0), 1, ccc4f(0, 1, 0, 1));
	m_graphNode->drawSegment(ccp(0, 0), ccp(0, graphHeight), 1, ccc4f(0, 1, 0, 1));

	float targetY = (PhysicsMonitor::IDEAL_FRAME_MS / 10.0f) * graphHeight;
	m_graphNode->drawSegment(ccp(0, targetY), ccp(graphWidth, targetY), 1, ccc4f(1, 1, 0, 1));

	float x = pixelPerFrame / 2.0f;
	for (size_t i = 0; i < history.size(); ++i) {
		float delta_ms = history[i].delta_ms;
		float barHeight = (delta_ms / 10.0f) * graphHeight;

		ccColor4F color = history[i].is_stable ? ccc4f(0, 1, 0, 1) : ccc4f(1, 0, 0, 1);

		if (history[i].was_input) {
			color = ccc4f(0, 0, 1, 1);
		}

		m_graphNode->drawSegment(ccp(x, 0), ccp(x, barHeight), pixelPerFrame * 0.8f, color);
		x += pixelPerFrame;
	}
}
