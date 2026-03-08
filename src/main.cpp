#include <Geode/Geode.hpp>
#include <Geode/loader/GameEvent.hpp>
#include <Geode/loader/SettingV3.hpp>
#include "physics/PhysicsMonitor.hpp"
#include "ui/PhysicsOverlay.hpp"

using namespace geode::prelude;

$on_mod(Loaded) {
	listenForKeybindSettingPresses("toggle-overlay", [](Keybind const& keybind, bool down, bool repeat, double timestamp) {
		if (down && !repeat && g_physicsOverlay) {
			g_physicsOverlay->setShow(!g_physicsOverlay->isShow());
		}
	});
}