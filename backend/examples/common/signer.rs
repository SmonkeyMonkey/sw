//! The ground station's signing, for the smoke tests that play it: the operator key comes from
//! the base, in answer to a heartbeat on the wired base link, and orders and sticks go on the
//! air signed with it (see `swarm_brain::auth`, and gcs/src/link.rs for the real thing).

use std::time::{Duration, Instant};

use mavlink_swarm::mavlink_core::MavHeader;
use mavlink_swarm::MavMessage;
use swarm_brain::auth::Operator;
use swarm_brain::radio;

pub const BASE: &str = "127.0.0.1:14650";

#[derive(Default)]
pub struct Signer {
    /// The operator key, and when the mission clock read 0 here.
    key: Option<(Operator, Instant)>,
    /// Every radio frame is signed with the link key (see `swarm_brain::radio`).
    link: Option<radio::Signer>,
}

impl Signer {
    /// Takes the base's SWARM_OPERATOR_KEY, if that's what `msg` is.
    pub fn heard(&mut self, msg: &MavMessage) {
        let MavMessage::SWARM_OPERATOR_KEY(k) = msg else { return };
        let zero = Instant::now() - Duration::from_millis(k.mission_ms as u64);
        match &mut self.key {
            Some((operator, at)) if operator.mission() == k.mission => *at = zero,
            _ => {
                self.key = Some((Operator::new(k.mission, k.secret_key), zero));
                self.link = Some(radio::Signer::new(k.link_key, 255));
            }
        }
    }

    /// `msg` as it goes on the air: orders and sticks signed with the operator key, and the
    /// frame with the link key. Unsigned before the base has handed them over: the drones
    /// won't hear it.
    pub fn on_air(&mut self, header: MavHeader, msg: &MavMessage) -> Vec<u8> {
        let msg = self.sign(msg);
        match (&mut self.link, &self.key) {
            (Some(link), Some((_, zero))) => link.frame(header, &msg, zero.elapsed().as_millis() as u32).raw_bytes().to_vec(),
            _ => radio::unsigned(header, &msg).raw_bytes().to_vec(),
        }
    }

    pub fn ready(&self) -> bool {
        self.key.is_some()
    }

    /// Orders and sticks signed, anything else as it is.
    fn sign(&mut self, msg: &MavMessage) -> MavMessage {
        let Some((operator, zero)) = &mut self.key else { return msg.clone() };
        let now_ms = zero.elapsed().as_millis() as u32;
        match msg {
            MavMessage::COMMAND_LONG(c) => MavMessage::SWARM_SIGNED_COMMAND(operator.sign_command(c, now_ms)),
            MavMessage::MANUAL_CONTROL(m) => MavMessage::SWARM_SIGNED_STICKS(operator.sign_sticks(m, now_ms)),
            _ => msg.clone(),
        }
    }
}
