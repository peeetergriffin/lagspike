#include <Geode/Geode.hpp>
#include <Geode/modify/PlayLayer.hpp>

#include "physics/PhysicsMonitor.hpp"
#include "ui/PhysicsOverlay.hpp"

using namespace geode::prelude;

class $modify(PhysicsMonitorPLHook, PlayLayer) {
	void setupHasCompleted() {
		PlayLayer::setupHasCompleted();
		
		PhysicsMonitor::getInstance()->reset();
		
		auto timerNode = PhysicsMonitor::TimerNode::create(PhysicsMonitor::getInstance());
		if (timerNode) {
			m_uiLayer->addChild(timerNode);
		}
		
		g_physicsOverlay = PhysicsOverlay::create();
		
		if (g_physicsOverlay) {
			m_uiLayer->addChild(g_physicsOverlay);
			g_physicsOverlay->setPosition(10, 10);
		} else {
			log::error("Failed to create physics overlay!");
		}
	}
};
