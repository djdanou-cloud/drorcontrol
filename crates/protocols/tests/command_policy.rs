//! CommandPolicy (T0-16): plan 08 §4 classes and lockouts, ADR-0009 delivery, audit chain.

use std::time::{Duration, Instant};

use mavlink_codec::Message;
use mavlink_codec::enums::{MavCmd, MavResult};
use mavlink_codec::messages::CommandAck;
use protocols::audit::{self, AuditLog};
use protocols::command::*;

const T: Target = Target {
    sysid: 1,
    compid: 1,
};

fn connected() -> LinkInfo {
    LinkInfo {
        state: LinkState::Connected,
        heartbeat_age: Some(Duration::from_millis(200)),
    }
}
fn lost() -> LinkInfo {
    LinkInfo {
        state: LinkState::Lost,
        heartbeat_age: Some(Duration::from_millis(7200)),
    }
}
fn disarmed() -> VehicleFacts {
    VehicleFacts {
        armed: Some(false),
        custom_mode: Some(5),
    }
}
fn armed() -> VehicleFacts {
    VehicleFacts {
        armed: Some(true),
        custom_mode: Some(5),
    }
}
fn ack(cmd: u16, result: MavResult) -> CommandAck {
    CommandAck {
        command: cmd,
        result: result.0,
        ..Default::default()
    }
}
fn rtl() -> Command {
    Command::Rtl {
        expect_mode: Some(6),
    }
}

/// Request + submit, confirming through the card when the class needs it (C only here).
fn send_full(
    p: &mut CommandPolicy,
    cmd: Command,
    facts: &VehicleFacts,
    link: LinkInfo,
    now: Instant,
) -> (CommandId, Vec<Outgoing>, Vec<CommandStatus>) {
    let approval = match p.request(T, cmd.clone(), facts, link, now).unwrap() {
        Request::Ready { .. } => Approval::None,
        Request::Confirm(c) => Approval::Confirmed(c.token),
    };
    p.submit(T, cmd, approval, facts, link, now).unwrap()
}

fn send_c(
    p: &mut CommandPolicy,
    cmd: Command,
    facts: &VehicleFacts,
    link: LinkInfo,
    now: Instant,
) -> (CommandId, Vec<Outgoing>) {
    let (id, out, _) = send_full(p, cmd, facts, link, now);
    (id, out)
}

fn long(m: &Message) -> &mavlink_codec::messages::CommandLong {
    match m {
        Message::CommandLong(c) => c,
        other => panic!("expected COMMAND_LONG, got {other:?}"),
    }
}

// ---- classes and confirmation ------------------------------------------------------------

#[test]
fn classes_follow_the_operational_profile() {
    // ADR-0011: flight controls are single action; two-stage only for terminal actions.
    for c in [
        Command::Arm,
        rtl(),
        Command::Land { expect_mode: None },
        Command::SetMode { custom_mode: 5 },
        Command::Takeoff { alt_m: 10.0 },
        Command::Reposition {
            lat_deg: 1.0,
            lon_deg: 2.0,
            alt_m: 30.0,
        },
        Command::MotorTest {
            motor: 1,
            throttle_pct: 5.0,
            duration_s: 2.0,
            props_removed: true,
        },
    ] {
        assert_eq!(c.class(), Class::B, "{c:?}");
    }
    for c in [
        Command::Disarm,
        Command::ForceDisarm,
        Command::Reboot,
        Command::ParamReset,
    ] {
        assert_eq!(c.class(), Class::C, "{c:?}");
    }
    assert_eq!(Command::ForceArm.class(), Class::D);
}

#[test]
fn class_c_is_refused_without_a_valid_token() {
    let now = Instant::now();
    let mut p = CommandPolicy::new(PolicyConfig::default());
    let f = disarmed();
    assert_eq!(
        p.submit(T, Command::Reboot, Approval::None, &f, connected(), now)
            .unwrap_err(),
        Refusal::ConfirmationRequired
    );
    assert!(matches!(
        p.submit(
            T,
            Command::Reboot,
            Approval::Confirmed(Token(999)),
            &f,
            connected(),
            now
        ),
        Err(Refusal::BadToken(_))
    ));

    // Valid once.
    let Request::Confirm(c) = p.request(T, Command::Reboot, &f, connected(), now).unwrap() else {
        panic!()
    };
    assert!(
        p.submit(
            T,
            Command::Reboot,
            Approval::Confirmed(c.token),
            &f,
            connected(),
            now
        )
        .is_ok()
    );
    assert!(
        matches!(
            p.submit(
                T,
                Command::Reboot,
                Approval::Confirmed(c.token),
                &f,
                connected(),
                now
            ),
            Err(Refusal::BadToken(_))
        ),
        "single use"
    );

    // Expired after the 5 s card timeout.
    let Request::Confirm(c) = p.request(T, Command::Reboot, &f, connected(), now).unwrap() else {
        panic!()
    };
    let late = now + Duration::from_secs(5);
    assert!(matches!(
        p.submit(
            T,
            Command::Reboot,
            Approval::Confirmed(c.token),
            &f,
            connected(),
            late
        ),
        Err(Refusal::BadToken(_))
    ));

    // Bound to the vehicle and to the exact command.
    let Request::Confirm(c) = p.request(T, Command::Disarm, &f, connected(), now).unwrap() else {
        panic!()
    };
    let other = Target {
        sysid: 2,
        compid: 1,
    };
    assert!(matches!(
        p.submit(
            other,
            Command::Disarm,
            Approval::Confirmed(c.token),
            &f,
            connected(),
            now
        ),
        Err(Refusal::BadToken(_))
    ));
    let Request::Confirm(c) = p.request(T, Command::Disarm, &f, connected(), now).unwrap() else {
        panic!()
    };
    assert!(matches!(
        p.submit(
            T,
            Command::ForceDisarm,
            Approval::Confirmed(c.token),
            &f,
            connected(),
            now
        ),
        Err(Refusal::BadToken(_))
    ));
}

#[test]
fn class_d_needs_the_typed_phrase_and_a_reason() {
    let now = Instant::now();
    let mut p = CommandPolicy::new(PolicyConfig::default());
    p.set_vehicle_name(1, "WHOOP-3");
    let f = disarmed();
    let kill = |p: &mut CommandPolicy, text: &str, reason: &str| {
        let Request::Confirm(c) = p
            .request(T, Command::ForceArm, &f, connected(), now)
            .unwrap()
        else {
            panic!()
        };
        assert_eq!(c.class, Class::D);
        assert_eq!(c.challenge.as_deref(), Some("WHOOP-3"));
        p.submit(
            T,
            Command::ForceArm,
            Approval::Typed {
                token: c.token,
                text: text.into(),
                reason: reason.into(),
            },
            &f,
            connected(),
            now,
        )
    };
    assert_eq!(
        kill(&mut p, "whoop", "fire").unwrap_err(),
        Refusal::TypedMismatch
    );
    assert_eq!(
        kill(&mut p, "WHOOP-3", "  ").unwrap_err(),
        Refusal::ReasonRequired
    );
    assert!(kill(&mut p, "WHOOP-3", "battery fire on landing").is_ok());

    // A plain class C confirmation is not enough for class D.
    let Request::Confirm(c) = p
        .request(T, Command::ForceArm, &f, connected(), now)
        .unwrap()
    else {
        panic!()
    };
    assert_eq!(
        p.submit(
            T,
            Command::ForceArm,
            Approval::Confirmed(c.token),
            &f,
            connected(),
            now
        )
        .unwrap_err(),
        Refusal::TypedMismatch
    );
}

#[test]
fn class_b_needs_no_token() {
    let now = Instant::now();
    let mut p = CommandPolicy::new(PolicyConfig::default());
    let go = Command::Reposition {
        lat_deg: 31.25,
        lon_deg: 34.79,
        alt_m: 40.0,
    };
    assert_eq!(
        p.request(T, go.clone(), &armed(), connected(), now)
            .unwrap(),
        Request::Ready { class: Class::B }
    );
    let (_, out, _) = p
        .submit(T, go, Approval::None, &armed(), connected(), now)
        .unwrap();
    match &out[0].msg {
        Message::CommandInt(c) => {
            assert_eq!(c.command, MavCmd::DO_REPOSITION.0);
            assert_eq!(
                (c.x, c.y, c.z, c.frame),
                (312_500_000, 347_900_000, 40.0, 3)
            );
            assert_eq!(c.param2, 1.0, "MAV_DO_REPOSITION_FLAGS_CHANGE_MODE");
        }
        other => panic!("DO_REPOSITION must be COMMAND_INT, got {other:?}"),
    }
}

// ---- arguments and lockouts ----------------------------------------------------------------

#[test]
fn arguments_are_validated() {
    let now = Instant::now();
    let mut p = CommandPolicy::new(PolicyConfig::default());
    let bad = |p: &mut CommandPolicy, c: Command| {
        matches!(
            p.request(T, c, &disarmed(), connected(), now),
            Err(Refusal::InvalidArgument(_))
        )
    };
    assert!(bad(&mut p, Command::Takeoff { alt_m: 0.0 }));
    assert!(bad(&mut p, Command::Takeoff { alt_m: f32::NAN }));
    assert!(bad(&mut p, Command::Takeoff { alt_m: 5000.0 }));
    assert!(bad(
        &mut p,
        Command::Reposition {
            lat_deg: 91.0,
            lon_deg: 0.0,
            alt_m: 10.0
        }
    ));
    assert!(bad(
        &mut p,
        Command::Reposition {
            lat_deg: 0.0,
            lon_deg: f64::INFINITY,
            alt_m: 10.0
        }
    ));
    let mt = |motor, throttle_pct, duration_s| Command::MotorTest {
        motor,
        throttle_pct,
        duration_s,
        props_removed: true,
    };
    assert!(bad(&mut p, mt(0, 10.0, 2.0)));
    assert!(bad(&mut p, mt(1, 120.0, 2.0)));
    assert!(bad(&mut p, mt(1, 10.0, 0.0)));
    assert!(bad(&mut p, mt(1, 10.0, 60.0)));
    assert!(
        p.request(T, mt(1, 10.0, 2.0), &disarmed(), connected(), now)
            .is_ok()
    );
}

#[test]
fn vehicle_state_lockouts() {
    let now = Instant::now();
    let mut p = CommandPolicy::new(PolicyConfig::default());
    let locked = |p: &mut CommandPolicy, c: Command, f: &VehicleFacts| {
        matches!(
            p.request(T, c, f, connected(), now),
            Err(Refusal::Precondition(_))
        )
    };
    let mt = |props_removed| Command::MotorTest {
        motor: 1,
        throttle_pct: 5.0,
        duration_s: 2.0,
        props_removed,
    };
    assert!(locked(&mut p, mt(true), &armed()));
    assert!(
        locked(&mut p, mt(true), &VehicleFacts::default()),
        "armed state unknown"
    );
    assert!(
        locked(&mut p, mt(false), &disarmed()),
        "props-off attestation"
    );
    assert!(locked(&mut p, Command::ParamReset, &armed()));
    assert!(locked(&mut p, Command::Reboot, &armed()));
    assert!(!locked(&mut p, Command::ParamReset, &disarmed()));

    // Single-action commands are checked again at submit.
    assert!(matches!(
        p.submit(T, mt(true), Approval::None, &armed(), connected(), now),
        Err(Refusal::Precondition(_))
    ));
    // State can change between card and confirm: checked again at submit.
    let Request::Confirm(c) = p
        .request(T, Command::Reboot, &disarmed(), connected(), now)
        .unwrap()
    else {
        panic!()
    };
    assert!(matches!(
        p.submit(
            T,
            Command::Reboot,
            Approval::Confirmed(c.token),
            &armed(),
            connected(),
            now
        ),
        Err(Refusal::Precondition(_))
    ));
}

// ---- ADR-0009: link state never refuses -----------------------------------------------------

#[test]
fn link_state_never_refuses_a_command() {
    let now = Instant::now();
    for state in [
        LinkState::Connected,
        LinkState::Degraded,
        LinkState::Stale,
        LinkState::Lost,
    ] {
        let link = LinkInfo {
            state,
            heartbeat_age: Some(Duration::from_secs(9)),
        };
        let mut p = CommandPolicy::new(PolicyConfig::default());
        for cmd in [
            rtl(),
            Command::Land { expect_mode: None },
            Command::Arm,
            Command::Disarm,
            Command::SetMode { custom_mode: 4 },
        ] {
            let approval = match p.request(T, cmd.clone(), &armed(), link, now).unwrap() {
                Request::Ready { .. } => Approval::None,
                Request::Confirm(c) => {
                    if state != LinkState::Connected {
                        assert!(c.summary.contains("may not arrive"), "{}", c.summary);
                    }
                    Approval::Confirmed(c.token)
                }
            };
            let (_, out, _) = p
                .submit(T, cmd.clone(), approval, &armed(), link, now)
                .unwrap();
            assert_eq!(out.len(), 1, "{cmd:?} sent at once on a {state} link");
            assert_eq!(
                out[0].flood,
                state != LinkState::Connected,
                "flooded on every link unless CONNECTED"
            );
        }
    }
}

#[test]
fn retransmits_with_backoff_and_counts_confirmation() {
    let t0 = Instant::now();
    let mut p = CommandPolicy::new(PolicyConfig::default());
    let (_, first) = send_c(&mut p, rtl(), &armed(), lost(), t0);
    assert_eq!(long(&first[0].msg).confirmation, 0);
    let mut sends = vec![Duration::ZERO];
    let mut confirmations = vec![0u8];
    let mut t = t0;
    while t < t0 + Duration::from_secs(12) {
        t += Duration::from_millis(10);
        let (out, _) = p.poll(t, |_| lost());
        for o in out {
            assert!(o.flood);
            sends.push(t - t0);
            confirmations.push(long(&o.msg).confirmation);
        }
    }
    // Gaps: 0.5 s, then ×1.5 per attempt, capped at 2 s (10 ms polling granularity).
    let want = [500.0, 750.0, 1125.0, 1687.5, 2000.0, 2000.0];
    let gaps: Vec<f64> = sends
        .windows(2)
        .map(|w| (w[1] - w[0]).as_secs_f64() * 1000.0)
        .collect();
    assert!(gaps.len() >= want.len(), "{gaps:?}");
    for (g, w) in gaps.iter().zip(want) {
        assert!(*g >= w && *g < w + 20.0, "gaps {gaps:?}");
    }
    assert_eq!(
        confirmations,
        (0..confirmations.len() as u8).collect::<Vec<_>>()
    );
}

#[test]
fn retries_stop_on_ack_and_command_is_verified_by_state() {
    let t0 = Instant::now();
    let mut p = CommandPolicy::new(PolicyConfig::default());
    let (id, _) = send_c(&mut p, Command::Arm, &disarmed(), connected(), t0);
    let t1 = t0 + Duration::from_millis(300);
    let s = p.on_ack(
        T,
        &ack(MavCmd::COMPONENT_ARM_DISARM.0, MavResult::ACCEPTED),
        &disarmed(),
        t1,
    );
    assert_eq!(
        s[0].state,
        CommandState::Acknowledged,
        "ACK alone is not success"
    );
    let (out, _) = p.poll(t0 + Duration::from_secs(3), |_| connected());
    assert!(out.is_empty(), "no retransmission after ACK");
    let s = p.on_state(1, &armed(), t0 + Duration::from_secs(3));
    assert_eq!(s[0].state, CommandState::Verified);
    assert_eq!(p.status(id, t0).unwrap().summary(), "ARM: VERIFIED");
}

#[test]
fn state_verifies_even_if_the_ack_was_lost() {
    let t0 = Instant::now();
    let mut p = CommandPolicy::new(PolicyConfig::default());
    let (id, _) = send_c(
        &mut p,
        Command::SetMode { custom_mode: 4 },
        &armed(),
        lost(),
        t0,
    );
    let mut f = armed();
    f.custom_mode = Some(4);
    assert_eq!(
        p.on_state(1, &f, t0 + Duration::from_secs(2))[0].state,
        CommandState::Verified
    );
    let (out, _) = p.poll(t0 + Duration::from_secs(4), |_| lost());
    assert!(out.is_empty());
    assert_eq!(p.status(id, t0).unwrap().state, CommandState::Verified);
}

#[test]
fn in_progress_and_rejection() {
    let t0 = Instant::now();
    let mut p = CommandPolicy::new(PolicyConfig::default());
    let (_, _) = send_c(
        &mut p,
        Command::Takeoff { alt_m: 20.0 },
        &armed(),
        connected(),
        t0,
    );
    let s = p.on_ack(
        T,
        &ack(MavCmd::NAV_TAKEOFF.0, MavResult::IN_PROGRESS),
        &armed(),
        t0,
    );
    assert_eq!(s[0].state, CommandState::InProgress);
    assert!(
        p.poll(t0 + Duration::from_secs(5), |_| connected())
            .0
            .is_empty(),
        "no retransmission while in progress"
    );
    let s = p.on_ack(
        T,
        &ack(MavCmd::NAV_TAKEOFF.0, MavResult::DENIED),
        &armed(),
        t0 + Duration::from_secs(6),
    );
    assert_eq!(s[0].state, CommandState::Rejected(MavResult::DENIED.0));
    assert_eq!(s[0].summary(), "TAKEOFF 20 m: REJECTED (denied)");
}

#[test]
fn no_observable_state_means_accepted_on_ack() {
    let t0 = Instant::now();
    let mut p = CommandPolicy::new(PolicyConfig::default());
    let (_, _) = send_c(
        &mut p,
        Command::Rtl { expect_mode: None },
        &armed(),
        connected(),
        t0,
    );
    let s = p.on_ack(
        T,
        &ack(MavCmd::NAV_RETURN_TO_LAUNCH.0, MavResult::ACCEPTED),
        &armed(),
        t0,
    );
    assert_eq!(s[0].state, CommandState::Accepted);
}

#[test]
fn newer_command_supersedes_and_cancel_stops() {
    let t0 = Instant::now();
    let mut p = CommandPolicy::new(PolicyConfig::default());
    let (rtl_id, _) = send_c(&mut p, rtl(), &armed(), lost(), t0);
    let (arm_id, _) = send_c(&mut p, Command::Disarm, &armed(), lost(), t0);
    let (land_id, _, changed) = send_full(
        &mut p,
        Command::Land {
            expect_mode: Some(9),
        },
        &armed(),
        lost(),
        t0,
    );
    assert_eq!(changed[0].id, rtl_id);
    assert_eq!(changed[0].state, CommandState::Superseded(land_id));
    assert!(
        !p.status(arm_id, t0).unwrap().state.is_final(),
        "other groups are untouched"
    );

    assert_eq!(
        p.cancel(land_id, t0).unwrap().state,
        CommandState::Cancelled
    );
    let (out, _) = p.poll(t0 + Duration::from_secs(10), |_| lost());
    assert!(
        out.iter().all(|o| o.id == arm_id),
        "only the disarm is still being retried"
    );
    assert!(p.cancel(land_id, t0).is_none(), "cancel is idempotent");
}

#[test]
fn nothing_is_sent_after_the_lifetime() {
    let cfg = PolicyConfig::default();
    for (cmd, life) in [(rtl(), cfg.lifetime_nav), (Command::Arm, cfg.lifetime)] {
        let t0 = Instant::now();
        let mut p = CommandPolicy::new(cfg.clone());
        let (id, _) = send_c(&mut p, cmd.clone(), &disarmed(), lost(), t0);
        let deadline = t0 + life;
        let mut expired_at = None;
        let mut t = t0;
        while t < deadline + Duration::from_secs(20) {
            t += Duration::from_millis(7);
            let (out, changed) = p.poll(t, |_| lost());
            for o in &out {
                assert!(
                    t < deadline,
                    "{cmd:?} sent at {:?} after its {life:?} lifetime",
                    t - t0
                );
                assert_eq!(o.id, id);
            }
            for s in changed {
                if s.state == (CommandState::Expired { acked: false }) {
                    expired_at.get_or_insert(t);
                }
            }
        }
        let e = expired_at.expect("expired");
        assert!(
            e >= deadline && e - deadline < Duration::from_millis(10),
            "expires at its deadline"
        );
        let s = p.status(id, t).unwrap();
        assert!(
            s.summary()
                .starts_with(&format!("{}: NOT CONFIRMED — EXPIRED", cmd.label())),
            "{}",
            s.summary()
        );
    }
}

#[test]
fn status_line_shows_link_and_time_left() {
    let t0 = Instant::now();
    let mut p = CommandPolicy::new(PolicyConfig::default());
    let (id, _) = send_c(&mut p, rtl(), &armed(), lost(), t0);
    let line = p.status(id, t0 + Duration::from_secs(8)).unwrap().summary();
    assert_eq!(
        line,
        "RTL: SENT — NOT CONFIRMED (link LOST 7.2 s), attempt 1, 22 s left"
    );
}

// ---- encodings (verified against the pinned common.xml) -----------------------------------

#[test]
fn messages_encode_like_the_spec() {
    let m = |c: Command| c.to_message(T, 0);
    let arm = m(Command::Arm);
    assert_eq!(
        (long(&arm).command, long(&arm).param1, long(&arm).param2),
        (400, 1.0, 0.0)
    );
    let kill = m(Command::ForceDisarm);
    assert_eq!((long(&kill).param1, long(&kill).param2), (0.0, 21196.0));
    let mode = m(Command::SetMode { custom_mode: 6 });
    assert_eq!(
        (long(&mode).command, long(&mode).param1, long(&mode).param2),
        (176, 1.0, 6.0)
    );
    assert_eq!(long(&m(rtl())).command, 20);
    assert_eq!(long(&m(Command::Land { expect_mode: None })).command, 21);
    let to = m(Command::Takeoff { alt_m: 15.0 });
    assert_eq!((long(&to).command, long(&to).param7), (22, 15.0));
    let mt = m(Command::MotorTest {
        motor: 3,
        throttle_pct: 7.0,
        duration_s: 2.0,
        props_removed: true,
    });
    assert_eq!(
        (
            long(&mt).command,
            long(&mt).param1,
            long(&mt).param2,
            long(&mt).param3,
            long(&mt).param4
        ),
        (209, 3.0, 0.0, 7.0, 2.0)
    );
    assert_eq!(
        (
            long(&m(Command::Reboot)).command,
            long(&m(Command::Reboot)).param1
        ),
        (246, 1.0)
    );
    assert_eq!(
        (
            long(&m(Command::ParamReset)).command,
            long(&m(Command::ParamReset)).param1
        ),
        (245, 2.0)
    );
    for c in [Command::Arm, rtl(), Command::Reboot] {
        let msg = c.to_message(T, 0);
        assert_eq!(
            (long(&msg).target_system, long(&msg).target_component),
            (1, 1)
        );
    }
}

// ---- audit -----------------------------------------------------------------------------------

#[test]
fn audit_chain_records_everything_and_detects_tampering() {
    let t0 = Instant::now();
    let mut p = CommandPolicy::new(PolicyConfig::default());
    assert!(
        p.submit(
            T,
            Command::Disarm,
            Approval::None,
            &armed(),
            connected(),
            t0
        )
        .is_err()
    );
    let (id, _) = send_c(&mut p, Command::Arm, &disarmed(), lost(), t0);
    p.on_ack(T, &ack(400, MavResult::ACCEPTED), &disarmed(), t0);
    p.on_state(1, &armed(), t0);
    let events = p.drain_audit();
    let texts: Vec<&str> = events.iter().map(|e| e.text.as_str()).collect();
    assert!(
        texts
            .iter()
            .any(|t| t.starts_with("refused DISARM: needs confirmation")),
        "{texts:?}"
    );
    assert!(texts.iter().any(|t| t.starts_with("accepted ARM")));
    assert!(
        texts
            .iter()
            .any(|t| t.contains("attempt 1 (link LOST, all links)"))
    );
    assert!(texts.contains(&"ack ARM: accepted"));
    assert!(texts.contains(&"ARM: Verified"));
    assert!(events.iter().any(|e| e.id == Some(id)));

    let mut log = AuditLog::new(Vec::new());
    for (i, e) in events.iter().enumerate() {
        log.append(1_700_000_000_000_000 + i as u64, e).unwrap();
    }
    let bytes = log.into_inner();
    let (n, last) = audit::verify(bytes.as_slice()).unwrap();
    assert_eq!(n, events.len());

    // Resume the chain in a second writer.
    let mut more = AuditLog::resume(Vec::new(), last);
    more.append(1_800_000_000_000_000, &events[0]).unwrap();
    let mut joined = bytes.clone();
    joined.extend(more.into_inner());
    assert_eq!(
        audit::verify(joined.as_slice()).unwrap().0,
        events.len() + 1
    );

    // Any edit breaks the chain at that line.
    let text = String::from_utf8(bytes).unwrap();
    let tampered = text.replacen("accepted ARM", "accepted DISARM", 1);
    let line = text
        .lines()
        .position(|l| l.contains("accepted ARM"))
        .unwrap()
        + 1;
    assert_eq!(audit::verify(tampered.as_bytes()).unwrap_err().line, line);
    let dropped: String = text
        .lines()
        .enumerate()
        .filter(|(i, _)| *i != 1)
        .map(|(_, l)| format!("{l}\n"))
        .collect();
    assert_eq!(audit::verify(dropped.as_bytes()).unwrap_err().line, 2);
}

/// COMMAND_ACK carries only the command number, and ARM and DISARM share it. A late ACK for a
/// superseded ARM must not be taken as the answer to the DISARM that replaced it.
#[test]
fn late_ack_of_a_superseded_command_is_not_misattributed() {
    let t0 = Instant::now();
    let mut p = CommandPolicy::new(PolicyConfig::default());
    let (_arm, _) = send_c(&mut p, Command::Arm, &disarmed(), connected(), t0);
    let t1 = t0 + Duration::from_millis(100);
    let (disarm, _) = send_c(&mut p, Command::Disarm, &disarmed(), connected(), t1);
    // The vehicle's DENIED for the ARM arrives after the DISARM was sent.
    let late = p.on_ack(
        T,
        &ack(400, MavResult::DENIED),
        &disarmed(),
        t1 + Duration::from_millis(50),
    );
    assert!(late.is_empty(), "ambiguous ACK ignored: {late:?}");
    assert_eq!(p.status(disarm, t1).unwrap().state, CommandState::Pending);
    let (out, _) = p.poll(t1 + Duration::from_millis(1100), |_| connected());
    assert_eq!(out.len(), 1, "DISARM still retried");
    // After the ambiguity window an ACK is attributed normally.
    let s = p.on_ack(
        T,
        &ack(400, MavResult::ACCEPTED),
        &armed(),
        t1 + Duration::from_millis(2500),
    );
    assert_eq!(s[0].id, disarm);
    assert_eq!(s[0].state, CommandState::Acknowledged);
    assert!(
        p.drain_audit()
            .iter()
            .any(|e| e.text.contains("ignored: may answer a superseded"))
    );
}

/// ADR-0011: a mode change is one action (no card, no token), goes out at once, and is never
/// sent after its 3 s lifetime: unanswered, it expires and the operator is alerted.
#[test]
fn mode_change_is_single_action_and_expires_after_three_seconds() {
    let t0 = Instant::now();
    let mut p = CommandPolicy::new(PolicyConfig::default());
    let loiter = Command::SetMode { custom_mode: 5 };
    assert_eq!(
        p.request(T, loiter.clone(), &armed(), lost(), t0).unwrap(),
        Request::Ready { class: Class::B }
    );
    let (id, out, _) = p
        .submit(T, loiter, Approval::None, &armed(), lost(), t0)
        .unwrap();
    assert_eq!(out.len(), 1, "sent immediately");
    let mut last_send = t0;
    let mut expired = None;
    let mut t = t0;
    while t < t0 + Duration::from_secs(6) {
        t += Duration::from_millis(5);
        let (out, changed) = p.poll(t, |_| lost());
        if !out.is_empty() {
            last_send = t;
        }
        for c in changed {
            if c.state == (CommandState::Expired { acked: false }) {
                expired.get_or_insert(t);
            }
        }
    }
    assert!(
        last_send < t0 + Duration::from_secs(3),
        "no mode change after 3 s"
    );
    let e = expired.expect("expired");
    assert!(e >= t0 + Duration::from_secs(3) && e < t0 + Duration::from_millis(3010));
    assert!(
        p.status(id, t)
            .unwrap()
            .summary()
            .contains("NOT CONFIRMED — EXPIRED")
    );
}

/// ADR-0011: emergency motor termination is two-stage (card/slide) without typed text, so it
/// stays fast; it still cannot be sent without the card.
#[test]
fn emergency_termination_is_two_stage_without_typed_text() {
    let now = Instant::now();
    let mut p = CommandPolicy::new(PolicyConfig::default());
    assert_eq!(
        p.submit(
            T,
            Command::ForceDisarm,
            Approval::None,
            &armed(),
            connected(),
            now
        )
        .unwrap_err(),
        Refusal::ConfirmationRequired
    );
    let Request::Confirm(c) = p
        .request(T, Command::ForceDisarm, &armed(), connected(), now)
        .unwrap()
    else {
        panic!("termination is two-stage")
    };
    assert_eq!((c.class, c.challenge.clone()), (Class::C, None));
    assert!(
        p.submit(
            T,
            Command::ForceDisarm,
            Approval::Confirmed(c.token),
            &armed(),
            connected(),
            now
        )
        .is_ok()
    );
}
