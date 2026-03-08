#include <Geode/Geode.hpp>
#include "physics/PhysicsMonitor.hpp"

using namespace geode::prelude;

$on_mod(Loaded) {
	PhysicsMonitor::getInstance();
}