//! Who said what. Every drone signs its own sightings with its own Ed25519 key, so a captured
//! drone can lie in its own name but can't put words in anyone else's mouth: no more votes for
//! a made-up target than it has drones.
//!
//! The launch base is the fleet's authority. For each mission it gives every drone a fresh key
//! and a certificate: the drone's public key and system ID, signed by the authority. Drones know
//! only the authority's public key; a drone's certificate travels with its sightings
//! (SWARM_KEY_CERT), so anyone who relays a sighting can pass on what it takes to check it, and
//! a drone that joins mid-mission needs nothing but its own keys.
//!
//! What's signed is what the observer saw: who, when on its own clock, where, how fast. What
//! relays change on the way (hops, age, responders, the target's name as the swarm settles it,
//! trace context) isn't, and so isn't vouched for.
//!
//! Orders are signed too, with the mission's operator key: the ground station holds the secret
//! half (the base hands it over their wire), the drones the public half (in their `Keys`). No
//! drone can give an order, then, captured or not, and nobody else on the air either. Each
//! order carries when it was given on the mission clock, so a recorded one can't be played back
//! (see `Brain::fresh`). And the base can revoke a captured drone's key for the rest of the
//! mission (`Authority::revoke`).

use ed25519_dalek::{Signature, Signer, SigningKey, VerifyingKey};
use mavlink_swarm::{
    COMMAND_LONG_DATA, MANUAL_CONTROL_DATA, SWARM_KEY_CERT_DATA, SWARM_REVOKE_DATA, SWARM_SIGNED_COMMAND_DATA,
    SWARM_SIGNED_STICKS_DATA, SWARM_TARGET_SIGHTING_DATA,
};
use sha2::Digest;

/// One drone's keys for one mission, as the authority hands them out.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Keys {
    /// Signed into every certificate and sighting, so nothing carries over from another mission.
    pub mission: u32,
    pub sysid: u8,
    pub secret: [u8; 32],
    /// The authority's signature on this drone's public key (see `certificate_bytes`).
    pub certificate: [u8; 64],
    /// The authority's public key: what every certificate is checked against.
    pub authority: [u8; 32],
    /// The mission's operator public key: what every order is checked against.
    pub operator: [u8; 32],
    /// The mission's radio link key (see `radio`): shared by every radio of the swarm.
    pub link: [u8; 32],
}

impl Keys {
    pub fn public(&self) -> [u8; 32] {
        SigningKey::from_bytes(&self.secret).verifying_key().to_bytes()
    }

    /// This drone's certificate, as sent over the radio.
    pub fn certificate_msg(&self) -> SWARM_KEY_CERT_DATA {
        SWARM_KEY_CERT_DATA { mission: self.mission, sysid: self.sysid, public_key: self.public(), signature: self.certificate }
    }

    /// Signs `s` as this drone's own observation, made at `observed_ms` on its clock. Derives
    /// the signing key each time (a scalar multiplication): the brain keeps its own instead.
    pub fn sign_sighting(&self, s: &mut SWARM_TARGET_SIGHTING_DATA, observed_ms: u32) {
        sign(&SigningKey::from_bytes(&self.secret), self.mission, self.sysid, s, observed_ms);
    }
}

pub(crate) fn sign(key: &SigningKey, mission: u32, sysid: u8, s: &mut SWARM_TARGET_SIGHTING_DATA, observed_ms: u32) {
    s.observer_sysid = sysid;
    s.observed_ms = observed_ms;
    s.signature = key.sign(&sighting_bytes(mission, s)).to_bytes();
}

/// The fleet's certificate authority (the launch base). Never on a drone.
pub struct Authority(SigningKey);

impl Authority {
    pub fn from_secret(secret: [u8; 32]) -> Self {
        Self(SigningKey::from_bytes(&secret))
    }

    /// Keys for drone `sysid` this mission: `secret` should be fresh randomness.
    pub fn issue(&self, mission: u32, sysid: u8, secret: [u8; 32]) -> Keys {
        let public = SigningKey::from_bytes(&secret).verifying_key().to_bytes();
        Keys {
            mission,
            sysid,
            secret,
            certificate: self.0.sign(&certificate_bytes(mission, sysid, &public)).to_bytes(),
            authority: self.0.verifying_key().to_bytes(),
            operator: SigningKey::from_bytes(&self.operator_secret(mission)).verifying_key().to_bytes(),
            link: self.link_key(mission),
        }
    }

    /// The mission's operator key, for the ground station.
    pub fn operator_secret(&self, mission: u32) -> [u8; 32] {
        self.derive(OPERATOR_DOMAIN, mission)
    }

    /// The mission's radio link key (see `radio`), for every radio of the swarm.
    pub fn link_key(&self, mission: u32) -> [u8; 32] {
        self.derive(LINK_DOMAIN, mission)
    }

    /// A key of the mission's, derived from the authority's own: the base keeps nothing per
    /// mission, and a new mission is a new key.
    fn derive(&self, domain: &[u8], mission: u32) -> [u8; 32] {
        let mut hash = sha2::Sha512::new();
        hash.update(domain);
        hash.update(self.0.to_bytes());
        hash.update(mission.to_le_bytes());
        let mut secret = [0; 32];
        secret.copy_from_slice(&hash.finalize()[..32]);
        secret
    }

    /// Revokes drone `sysid`'s key for the rest of the mission.
    pub fn revoke(&self, mission: u32, sysid: u8) -> SWARM_REVOKE_DATA {
        SWARM_REVOKE_DATA { mission, sysid, signature: self.0.sign(&revoke_bytes(mission, sysid)).to_bytes() }
    }
}

/// The ground station's half of the operator key: signs its orders.
pub struct Operator {
    key: SigningKey,
    mission: u32,
    /// The last order's `issued_ms`: each one is later than the last, so two given in the same
    /// millisecond still come in order.
    last_ms: u32,
}

impl Operator {
    pub fn new(mission: u32, secret: [u8; 32]) -> Self {
        Self { key: SigningKey::from_bytes(&secret), mission, last_ms: 0 }
    }

    pub fn mission(&self) -> u32 {
        self.mission
    }

    pub fn public(&self) -> [u8; 32] {
        self.key.verifying_key().to_bytes()
    }

    fn issue_at(&mut self, now_ms: u32) -> u32 {
        self.last_ms = now_ms.max(self.last_ms + 1);
        self.last_ms
    }

    /// `cmd`, given at `now_ms` on the mission clock.
    pub fn sign_command(&mut self, cmd: &COMMAND_LONG_DATA, now_ms: u32) -> SWARM_SIGNED_COMMAND_DATA {
        let mut c = SWARM_SIGNED_COMMAND_DATA {
            mission: self.mission,
            issued_ms: self.issue_at(now_ms),
            param1: cmd.param1,
            param2: cmd.param2,
            param3: cmd.param3,
            param4: cmd.param4,
            param5: cmd.param5,
            param6: cmd.param6,
            param7: cmd.param7,
            command: cmd.command,
            target_system: cmd.target_system,
            signature: [0; 64],
        };
        c.signature = self.key.sign(&command_bytes(&c)).to_bytes();
        c
    }

    pub fn sign_sticks(&mut self, m: &MANUAL_CONTROL_DATA, now_ms: u32) -> SWARM_SIGNED_STICKS_DATA {
        let mut s = SWARM_SIGNED_STICKS_DATA {
            mission: self.mission,
            issued_ms: self.issue_at(now_ms),
            x: m.x,
            y: m.y,
            z: m.z,
            r: m.r,
            target: m.target,
            signature: [0; 64],
        };
        s.signature = self.key.sign(&sticks_bytes(&s)).to_bytes();
        s
    }
}

/// The order as a plain COMMAND_LONG, for carrying out.
pub fn command_of(c: &SWARM_SIGNED_COMMAND_DATA) -> COMMAND_LONG_DATA {
    COMMAND_LONG_DATA {
        param1: c.param1,
        param2: c.param2,
        param3: c.param3,
        param4: c.param4,
        param5: c.param5,
        param6: c.param6,
        param7: c.param7,
        command: c.command,
        target_system: c.target_system,
        target_component: 0,
        confirmation: 0,
    }
}

/// Domain-separated, so a signature on one kind of thing can never pass for the other.
const CERT_DOMAIN: &[u8; 14] = b"swarm-cert-v1\0";
const SIGHTING_DOMAIN: &[u8; 18] = b"swarm-sighting-v1\0";
const COMMAND_DOMAIN: &[u8; 17] = b"swarm-command-v1\0";
const STICKS_DOMAIN: &[u8; 16] = b"swarm-sticks-v1\0";
const REVOKE_DOMAIN: &[u8; 16] = b"swarm-revoke-v1\0";
const OPERATOR_DOMAIN: &[u8; 18] = b"swarm-operator-v1\0";
const LINK_DOMAIN: &[u8; 14] = b"swarm-link-v1\0";

fn certificate_bytes(mission: u32, sysid: u8, public: &[u8; 32]) -> [u8; 14 + 4 + 1 + 32] {
    let mut b = [0; 51];
    b[..14].copy_from_slice(CERT_DOMAIN);
    b[14..18].copy_from_slice(&mission.to_le_bytes());
    b[18] = sysid;
    b[19..].copy_from_slice(public);
    b
}

/// The observation itself, as the observer made it.
pub(crate) fn sighting_bytes(mission: u32, s: &SWARM_TARGET_SIGHTING_DATA) -> [u8; 18 + 4 + 1 + 4 + 16] {
    let mut b = [0; 43];
    b[..18].copy_from_slice(SIGHTING_DOMAIN);
    b[18..22].copy_from_slice(&mission.to_le_bytes());
    b[22] = s.observer_sysid;
    b[23..27].copy_from_slice(&s.observed_ms.to_le_bytes());
    for (i, v) in [s.x, s.y, s.vx, s.vy].into_iter().enumerate() {
        b[27 + 4 * i..31 + 4 * i].copy_from_slice(&v.to_le_bytes());
    }
    b
}

pub(crate) fn command_bytes(c: &SWARM_SIGNED_COMMAND_DATA) -> [u8; 17 + 4 + 4 + 1 + 2 + 28] {
    let mut b = [0; 56];
    b[..17].copy_from_slice(COMMAND_DOMAIN);
    b[17..21].copy_from_slice(&c.mission.to_le_bytes());
    b[21..25].copy_from_slice(&c.issued_ms.to_le_bytes());
    b[25] = c.target_system;
    b[26..28].copy_from_slice(&(c.command as u16).to_le_bytes());
    for (i, v) in [c.param1, c.param2, c.param3, c.param4, c.param5, c.param6, c.param7].into_iter().enumerate() {
        b[28 + 4 * i..32 + 4 * i].copy_from_slice(&v.to_le_bytes());
    }
    b
}

pub(crate) fn sticks_bytes(s: &SWARM_SIGNED_STICKS_DATA) -> [u8; 16 + 4 + 4 + 1 + 8] {
    let mut b = [0; 33];
    b[..16].copy_from_slice(STICKS_DOMAIN);
    b[16..20].copy_from_slice(&s.mission.to_le_bytes());
    b[20..24].copy_from_slice(&s.issued_ms.to_le_bytes());
    b[24] = s.target;
    for (i, v) in [s.x, s.y, s.z, s.r].into_iter().enumerate() {
        b[25 + 2 * i..27 + 2 * i].copy_from_slice(&v.to_le_bytes());
    }
    b
}

fn revoke_bytes(mission: u32, sysid: u8) -> [u8; 16 + 4 + 1] {
    let mut b = [0; 21];
    b[..16].copy_from_slice(REVOKE_DOMAIN);
    b[16..20].copy_from_slice(&mission.to_le_bytes());
    b[20] = sysid;
    b
}

/// Whether `r` is the authority's, for this mission.
pub(crate) fn revocation_valid(r: &SWARM_REVOKE_DATA, mission: u32, authority: &[u8; 32]) -> bool {
    r.mission == mission && verify(authority, &revoke_bytes(mission, r.sysid), &r.signature)
}

/// Strict Ed25519 verification (no malleable or small-order keys and signatures).
pub(crate) fn verify(public: &[u8; 32], msg: &[u8], signature: &[u8; 64]) -> bool {
    VerifyingKey::from_bytes(public).is_ok_and(|key| key.verify_strict(msg, &Signature::from_bytes(signature)).is_ok())
}

/// Whether `cert` is the authority's, for this mission.
pub(crate) fn certificate_valid(cert: &SWARM_KEY_CERT_DATA, mission: u32, authority: &[u8; 32]) -> bool {
    cert.mission == mission && verify(authority, &certificate_bytes(mission, cert.sysid, &cert.public_key), &cert.signature)
}
