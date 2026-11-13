//! MAVLink 2 message signing on the radio (<https://mavlink.io/en/guide/message_signing.html>):
//! every frame on the air carries a 48-bit SHA-256 signature made with the mission's link key, so
//! only the swarm's own radios (its drones, the base, the ground station) get a hearing. Someone
//! else on the frequency can't put frames on it (a beacon, a sighting, a status, anything) or
//! play back old ones; what they send is dropped before it's even read.
//!
//! It's one key for everyone, though: a captured drone knows it. What keeps a captured drone
//! from giving orders or making up sightings in others' names are the Ed25519 signatures on
//! those (see `auth`); this keeps out everyone else, for a hash per frame (~0.1 ms on the board,
//! where an Ed25519 check is 6).
//!
//! Two departures from the spec, both because drones have no calendar but do share the mission
//! clock (`SensorReading::time_ms`):
//!  - the timestamp counts the mission clock (in the spec's 10 µs units), not time since 2015;
//!  - a frame is taken only within `WINDOW_MS` of our clock, but not only if newer than the last
//!    from its link: the radio delivers frames out of order (each with its own delay), and that
//!    rule would drop many. A frame played back within the window is one more copy of a frame
//!    just sent, which the swarm copes with anyway: the radio and its relays duplicate frames.
//!
//! Every radio signs with its own system ID as the link ID; a relay signs what it passes on
//! anew, under the original header.

use mavlink_swarm::mavlink_core::{MAVLinkV2MessageRaw, MavHeader, Message, MavlinkVersion};
use mavlink_swarm::MavMessage;
use sha2::{Digest, Sha256};

/// How far a frame's timestamp may be from our clock, either way: the radio's delay (up to a
/// tenth of a second), our clock only moving once a tick (50 ms), and the ground station's
/// copy of the mission clock (a few ms off).
pub const WINDOW_MS: u32 = 1_000;

/// MAVLink 2 framing: magic byte, header, checksum and signature sizes.
const STX: u8 = 0xFD;
const HEADER: usize = 9;
const CHECKSUM: usize = 2;
/// Link ID, timestamp, signature.
const SIGNATURE: usize = 1 + 6 + 6;
const IFLAG_SIGNED: u8 = 0x01;

/// Signs the frames one radio sends.
pub struct Signer {
    key: [u8; 32],
    link_id: u8,
    /// The last timestamp used: each frame's is later than the last, as the spec asks.
    last: u64,
}

impl Signer {
    /// `link_id`: the radio's own system ID.
    pub fn new(key: [u8; 32], link_id: u8) -> Self {
        Self { key, link_id, last: 0 }
    }

    /// `msg` under `header`, signed, as sent at `now_ms` on the mission clock. Its bytes are
    /// `raw_bytes()`.
    pub fn frame(&mut self, header: MavHeader, msg: &MavMessage, now_ms: u32) -> MAVLinkV2MessageRaw {
        let mut raw = MAVLinkV2MessageRaw::new();
        raw.serialize_message_for_signing(header, msg);
        self.last = (now_ms as u64 * 100).max(self.last + 1);
        let at = 1 + HEADER + raw.payload_length() as usize + CHECKSUM;
        let bytes = raw.as_mut_slice();
        bytes[at] = self.link_id;
        bytes[at + 1..at + 7].copy_from_slice(&self.last.to_le_bytes()[..6]);
        let signature = signature(&self.key, &bytes[..at + 7]);
        bytes[at + 7..at + SIGNATURE].copy_from_slice(&signature);
        raw
    }
}

/// The spec's sha256_48: the first 6 bytes of SHA-256 over the key and the frame up to its
/// signature (link ID and timestamp included).
fn signature(key: &[u8; 32], signed: &[u8]) -> [u8; 6] {
    let mut hash = Sha256::new();
    hash.update(key);
    hash.update(signed);
    let mut out = [0; 6];
    out.copy_from_slice(&hash.finalize()[..6]);
    out
}

/// The frame in `raw` (one whole, CRC-checked MAVLink 2 frame), if it's signed with the link
/// key within `WINDOW_MS` of `now_ms`. `None` for anything else, unsigned frames included.
pub fn open(raw: &MAVLinkV2MessageRaw, key: &[u8; 32], now_ms: u32) -> Option<(MavHeader, MavMessage)> {
    if !signed(raw, key, now_ms) {
        return None;
    }
    parse(raw)
}

/// Whether `raw` is signed with the link key within `WINDOW_MS` of `now_ms`.
pub fn signed(raw: &MAVLinkV2MessageRaw, key: &[u8; 32], now_ms: u32) -> bool {
    let bytes = raw.raw_bytes();
    if bytes.first() != Some(&STX) || raw.incompatibility_flags() & IFLAG_SIGNED == 0 {
        return false;
    }
    let at = 1 + HEADER + raw.payload_length() as usize + CHECKSUM;
    let (Some(signed), Some(theirs)) = (bytes.get(..at + 7), bytes.get(at + 7..at + SIGNATURE)) else { return false };
    let mut stamp = [0; 8];
    stamp[..6].copy_from_slice(&signed[at + 1..at + 7]);
    let sent_ms = u64::from_le_bytes(stamp) / 100;
    // Not constant-time: what it would leak is when a guess at a 6-byte signature is right
    // up to some byte, about a frame the sender doesn't have the key for anyway, and the
    // window closes on it within a second.
    sent_ms.abs_diff(now_ms as u64) <= WINDOW_MS as u64 && theirs == signature(key, signed)
}

/// `msg` under `header` without a signature, for links that are wires (the board's to its body,
/// the base's to the ground station).
pub fn unsigned(header: MavHeader, msg: &MavMessage) -> MAVLinkV2MessageRaw {
    let mut raw = MAVLinkV2MessageRaw::new();
    raw.serialize_message(header, msg);
    raw
}

/// The message in a frame, signed or not: for what comes in over a wire.
pub fn parse(raw: &MAVLinkV2MessageRaw) -> Option<(MavHeader, MavMessage)> {
    let header = MavHeader { system_id: raw.system_id(), component_id: raw.component_id(), sequence: raw.sequence() };
    let msg = MavMessage::parse(MavlinkVersion::V2, raw.message_id(), raw.payload()).ok()?;
    Some((header, msg))
}

#[cfg(test)]
mod tests {
    use super::*;
    use mavlink_swarm::HEARTBEAT_DATA;

    const KEY: [u8; 32] = [7; 32];

    fn heartbeat() -> MavMessage {
        MavMessage::HEARTBEAT(HEARTBEAT_DATA::default())
    }

    fn header() -> MavHeader {
        MavHeader { system_id: 3, component_id: 1, sequence: 9 }
    }

    /// Back from bytes, as a receiver reads it off the air (CRC checked).
    fn received(bytes: &[u8]) -> Option<MAVLinkV2MessageRaw> {
        let mut reader = mavlink_swarm::mavlink_core::MavlinkReader::new(bytes);
        match reader.read_raw_message::<MavMessage>(MavlinkVersion::V2) {
            Ok(mavlink_swarm::mavlink_core::MAVLinkMessageRaw::V2(raw)) => Some(raw),
            _ => None,
        }
    }

    #[test]
    fn a_signed_frame_opens_with_the_key_only() {
        let raw = Signer::new(KEY, 3).frame(header(), &heartbeat(), 5_000);
        let raw = received(raw.raw_bytes()).expect("a good frame, CRC and all");
        assert_eq!(open(&raw, &KEY, 5_000), Some((header(), heartbeat())));
        assert_eq!(open(&raw, &[8; 32], 5_000), None);
    }

    #[test]
    fn an_unsigned_or_altered_frame_doesnt() {
        assert_eq!(open(&unsigned(header(), &heartbeat()), &KEY, 5_000), None);
        let signed = Signer::new(KEY, 3).frame(header(), &heartbeat(), 5_000);
        // Its signature on a frame from someone else (a valid frame, CRC and all).
        let mut other = MAVLinkV2MessageRaw::new();
        other.serialize_message_for_signing(MavHeader { system_id: 4, ..header() }, &heartbeat());
        let at = 1 + HEADER + signed.payload_length() as usize + CHECKSUM;
        other.as_mut_slice()[at..at + SIGNATURE].copy_from_slice(&signed.raw_bytes()[at..at + SIGNATURE]);
        assert!(open(&signed, &KEY, 5_000).is_some());
        assert_eq!(open(&other, &KEY, 5_000), None);
    }

    #[test]
    fn an_old_frame_played_back_doesnt() {
        let raw = Signer::new(KEY, 3).frame(header(), &heartbeat(), 5_000);
        assert!(open(&raw, &KEY, 5_000 + WINDOW_MS).is_some());
        assert_eq!(open(&raw, &KEY, 5_000 + WINDOW_MS + 1), None);
    }
}
