//! Noticing drones lost from the mission, for the ground station to launch replacements (when
//! the operator has that switched on).
//!
//! The ground station only knows a drone by the statuses it sends, so "lost" means silent: no
//! status for `LOST_AFTER`, from anywhere in the swarm (statuses are relayed). A drone sent home
//! (return to launch) isn't lost: it lands at the base and leaves the mission on purpose. One
//! that's heard from again (it was only out of contact) is back.

use std::collections::BTreeSet;
use std::time::Duration;

/// A drone silent this long is lost: well past the swarm's own link timeout, so a drone just
/// briefly out of contact isn't replaced for it.
pub const LOST_AFTER: Duration = Duration::from_secs(10);

/// `SWARM_DRONE_STATUS.order` for a drone sent home (return to launch).
const ORDER_RETURN_HOME: u8 = 3;

#[derive(Debug, PartialEq, Eq)]
pub enum FleetEvent {
    /// Silent for `LOST_AFTER`: crashed, or out of contact for good.
    Lost(u8),
    /// A drone taken for lost was heard from again.
    Back(u8),
}

#[derive(Default)]
pub struct LossWatch {
    lost: BTreeSet<u8>,
}

impl LossWatch {
    /// Each drone the ground station knows of: (system ID, time since its last status, the
    /// order it last reported). Returns what changed since the last call.
    pub fn update(&mut self, drones: impl IntoIterator<Item = (u8, Duration, u8)>) -> Vec<FleetEvent> {
        let mut events = Vec::new();
        for (id, silent, order) in drones {
            let lost = silent >= LOST_AFTER && order != ORDER_RETURN_HOME;
            if lost && self.lost.insert(id) {
                events.push(FleetEvent::Lost(id));
            } else if silent < LOST_AFTER && self.lost.remove(&id) {
                events.push(FleetEvent::Back(id));
            }
        }
        events
    }

    /// A new mission: nobody is lost from it yet.
    pub fn clear(&mut self) {
        self.lost.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const S: Duration = Duration::from_secs(1);

    #[test]
    fn a_silent_drone_is_lost_once_and_back_when_heard_again() {
        let mut watch = LossWatch::default();
        assert_eq!(watch.update([(1, 3 * S, 0), (2, S, 0)]), vec![]);
        assert_eq!(watch.update([(1, 11 * S, 0), (2, S, 0)]), vec![FleetEvent::Lost(1)]);
        // Still silent: not lost a second time.
        assert_eq!(watch.update([(1, 20 * S, 0), (2, S, 0)]), vec![]);
        assert_eq!(watch.update([(1, S, 0), (2, S, 0)]), vec![FleetEvent::Back(1)]);
    }

    #[test]
    fn a_drone_sent_home_is_not_lost() {
        let mut watch = LossWatch::default();
        // It landed and left the mission: it goes silent on purpose.
        assert_eq!(watch.update([(4, 30 * S, ORDER_RETURN_HOME)]), vec![]);
    }

    #[test]
    fn a_new_mission_starts_with_nobody_lost() {
        let mut watch = LossWatch::default();
        watch.update([(1, 11 * S, 0)]);
        watch.clear();
        // Drone 1 of the new mission, flying: nothing to report coming back.
        assert_eq!(watch.update([(1, S, 0)]), vec![]);
    }
}
