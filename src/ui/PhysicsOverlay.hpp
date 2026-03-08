#pragma once

#include <Geode/Geode.hpp>
#include "physics/PhysicsMonitor.hpp"

using namespace geode::prelude;

class PhysicsOverlay : public CCNode {
public:
	static PhysicsOverlay* create();
	bool init() override;
	
	void updateDisplay(float dt);
	void redrawGraph();	
	void setShow(bool visible);
	bool isShow() const;
private:
	CCDrawNode* m_graphNode = nullptr;
	CCLabelBMFont* m_statusLabel = nullptr;
	CCLabelBMFont* m_jitterLabel = nullptr;
	CCLabelBMFont* m_driftLabel = nullptr;
	CCLabelBMFont* m_stabilityLabel = nullptr;
	bool m_showGraph = true;
};

extern PhysicsOverlay* g_physicsOverlay;
