import math
import unittest
from unittest.mock import MagicMock, patch

from beep_bridge import beep_bridge as bridge


WALL_CONTACT = {
    "front": 0.127,
    "front_left": 0.128,
    "front_right": 0.157,
    "left": 0.192,
    "right": 1.125,
    "rear": 1.038,
}
FRONT_BLOCKED_REAR_CLEAR = dict(WALL_CONTACT, left=0.55, right=0.70, rear=1.038)


class CollisionSafetyTests(unittest.TestCase):
    def setUp(self):
        bridge._reset_motion_state_for_tests()

    def tearDown(self):
        bridge._reset_motion_state_for_tests()

    def test_room_scale_defaults_and_backpack_pose_are_not_boxed_in(self):
        self.assertEqual((bridge.HARD_CLEARANCE_M, bridge.FOOTPRINT_SIDE_M,
                          bridge.TURN_SWEEP_M, bridge.TURN_START_CLEARANCE_M,
                          bridge.FOOTPRINT_REAR_M, bridge.FOOTPRINT_DIAGONAL_M,
                          bridge.FOOTPRINT_FRONT_M, bridge.FORWARD_CORRIDOR_M,
                          bridge.SIDE_STEER_M, bridge.ROBOT_FOOTPRINT_RADIUS_M),
                         (0.20, 0.24, 0.28, 0.28, 0.28, 0.30, 0.38, 0.45, 0.36, 0.28))
        sectors = {"front": 0.634, "front_left": 0.532, "front_right": 2.169,
                   "left": 0.376, "right": 2.723, "rear": 1.644}
        self.assertEqual(bridge.choose_explore_action(sectors)[0], "forward")
        self.assertEqual(bridge.choose_fluent_steering(sectors), "curveright")
        status = {"scan_seen": True, "scan_age_s": 0.01, "sectors": sectors}
        lease = bridge.begin_motion("test", max_duration=5)
        try:
            with patch.object(bridge, "snapshot", return_value=status):
                bridge.app_packet_guard("forward", lease, lease.deadline)
        finally:
            bridge.end_motion(lease)

    def test_reverse_front_proximity_floor_and_two_distinct_loss_scans(self):
        baseline = {"front": 0.95, "front_left": 0.75, "front_right": 0.92,
                    "left": 0.6, "right": 0.6, "rear": 0.8}
        def state(count, front, front_left):
            return {"scan_seen": True, "scan_age_s": 0.01,
                    "scan_count": count, "slam": {"usable": True},
                    "pose": {"yaw": 0.0},
                    "sectors": dict(baseline, front=front, front_left=front_left)}
        with patch.object(bridge, "snapshot", return_value=state(1, 0.85, 0.379)):
            result = bridge.supervise_lidar_motion("back", 0.3, baseline_sectors=baseline,
                                                   baseline_pose={"yaw": 0.0})
        self.assertEqual(result["reason"], "front_clearance_worsening_during_backoff")
        samples = [state(1, 0.64, 0.62), state(1, 0.64, 0.62), state(2, 0.64, 0.62)]
        with patch.object(bridge, "snapshot", side_effect=samples):
            result = bridge.supervise_lidar_motion("back", 0.4, baseline_sectors=baseline,
                                                   baseline_pose={"yaw": 0.0})
        self.assertEqual(result["reason"], "front_clearance_worsening_during_backoff")
        stable = state(3, 0.88, 0.70)
        with patch.object(bridge, "snapshot", return_value=stable):
            result = bridge.supervise_lidar_motion("back", 0.01, baseline_sectors=baseline,
                                                   baseline_pose={"yaw": 0.0})
        self.assertEqual(result["reason"], "window_complete")

    def test_recorded_wall_contact_stops_instead_of_strafing_or_scraping(self):
        action, duration, reason = bridge.choose_explore_action(WALL_CONTACT)
        self.assertEqual((action, duration), ("stop", 0.0))
        self.assertEqual(reason, "footprint_boxed_in_stop")

    def test_front_obstacle_backs_away_only_with_full_side_and_rear_clearance(self):
        action, duration, reason = bridge.choose_explore_action(FRONT_BLOCKED_REAR_CLEAR)
        self.assertEqual(action, "back")
        self.assertEqual(duration, bridge.REVERSE_ESCAPE_MAX_S)
        self.assertEqual(reason, "front_footprint_breach_backoff")

    def test_every_required_sector_fails_closed_when_missing_or_invalid(self):
        clear = {
            "front": 1.2, "front_left": 0.8, "front_right": 0.8,
            "left": 0.7, "right": 0.7, "rear": 0.7,
        }
        for name in clear:
            with self.subTest(sector=name):
                incomplete = dict(clear)
                incomplete.pop(name)
                self.assertEqual(
                    bridge.choose_explore_action(incomplete),
                    ("stop", 0.0, "lidar_sector_missing_or_invalid"),
                )
        for invalid in (None, 0.0, -1.0, float("nan"), "not-a-range"):
            with self.subTest(invalid=invalid):
                self.assertEqual(bridge.choose_explore_action(dict(clear, front=invalid))[0], "stop")

    def test_boxed_in_state_stops_instead_of_guessing(self):
        sectors = dict(WALL_CONTACT, rear=0.30, right=0.31)
        action, duration, reason = bridge.choose_explore_action(sectors)
        self.assertEqual((action, duration), ("stop", 0.0))
        self.assertEqual(reason, "footprint_boxed_in_stop")

    def test_side_footprint_breach_stops_even_when_lidar_centerline_is_open(self):
        sectors = {
            "front": 1.2, "front_left": 0.8, "front_right": 0.8,
            "left": 0.23, "right": 1.0, "rear": 1.0,
        }
        action, duration, reason = bridge.choose_explore_action(sectors)
        self.assertEqual((action, duration), ("stop", 0.0))
        self.assertEqual(reason, "side_footprint_breach_stop")

    def test_forward_requires_conservative_full_body_corridor(self):
        clear = {
            "front": 1.2, "front_left": 0.8, "front_right": 0.8,
            "left": 0.7, "right": 0.7, "rear": 0.7,
        }
        self.assertEqual(bridge.choose_explore_action(clear)[0], "forward")
        self.assertEqual(bridge.choose_explore_action(dict(clear, front_left=0.27))[0], "back")

    def test_backoff_progress_requires_real_front_clearance_gain(self):
        stalled = dict(FRONT_BLOCKED_REAR_CLEAR, front=0.13, front_left=0.13)
        improved = dict(FRONT_BLOCKED_REAR_CLEAR, front=0.24, front_left=0.24, front_right=0.24)
        self.assertFalse(bridge.escape_made_progress("back", FRONT_BLOCKED_REAR_CLEAR, stalled))
        self.assertTrue(bridge.escape_made_progress("back", FRONT_BLOCKED_REAR_CLEAR, improved))
        self.assertGreaterEqual(bridge.ESCAPE_CLEARANCE_GAIN_M, 0.08)

    def test_turn_progress_requires_heading_change_and_safe_sweep(self):
        sectors = {
            "front": 0.56, "front_left": 0.48, "front_right": 0.52,
            "left": 0.48, "right": 0.72, "rear": 0.60,
        }
        before_pose = {"yaw": 0.0}
        self.assertFalse(bridge.escape_made_progress(
            "turnright", sectors, sectors, before_pose, {"yaw": -0.05}
        ))
        self.assertTrue(bridge.escape_made_progress(
            "turnright", sectors, sectors, before_pose, {"yaw": -math.radians(18)}
        ))
        self.assertFalse(bridge.escape_made_progress(
            "turnright", sectors, dict(sectors, front_left=0.20),
            before_pose, {"yaw": -math.radians(18)}
        ))

    def test_sweep_clear_in_former_turn_deadband_can_start_turn(self):
        marginal = {
            "front": 0.43, "front_left": 0.65, "front_right": 0.46,
            "left": 0.58, "right": 0.43, "rear": 1.0,
        }
        self.assertEqual(bridge.TURN_START_CLEARANCE_M, bridge.TURN_SWEEP_M)
        action, _, reason = bridge.choose_explore_action(marginal)
        self.assertEqual((action, reason), ("turnleft", "footprint_sweep_left"))

    def test_backoff_only_when_turn_sweep_is_actually_blocked(self):
        blocked = {"front": 0.43, "front_left": 0.65, "front_right": 0.46,
                   "left": 0.58, "right": 0.27, "rear": 1.0}
        self.assertEqual(bridge.choose_explore_action(blocked),
                         ("back", bridge.REVERSE_ESCAPE_MAX_S, "turn_sweep_blocked_backoff"))

    def test_front_breach_in_former_start_margin_band_turns(self):
        marginal = {"front": 0.44, "front_left": 0.48, "front_right": 0.49,
                    "left": 0.58, "right": 0.47, "rear": 1.0}
        self.assertEqual(bridge.choose_explore_action(marginal)[0], "turnleft")

    def test_exact_sweep_floor_can_start_turn_but_below_floor_backs_off(self):
        marginal = {"front": 0.43, "front_left": 0.65, "front_right": 0.50,
                    "left": 0.70, "right": bridge.TURN_SWEEP_M, "rear": 1.0}
        self.assertEqual(bridge.choose_explore_action(marginal)[0], "turnleft")
        blocked = dict(marginal, right=bridge.TURN_SWEEP_M - 0.001)
        self.assertEqual(bridge.choose_explore_action(blocked)[0], "back")

    def test_front_breach_prefers_guarded_turn_when_full_start_margin_exists(self):
        recoverable = {
            "front": 0.37, "front_left": 0.54, "front_right": 0.51,
            "left": 0.62, "right": 0.55, "rear": 1.0,
        }
        action, _, reason = bridge.choose_explore_action(recoverable)
        self.assertEqual(action, "turnleft")
        self.assertEqual(reason, "front_breach_sweep_left")


    def test_front_breach_uses_bounded_right_shift_to_open_turn_margin(self):
        constrained_left = {
            "front": 0.38, "front_left": 0.29, "front_right": 0.49,
            "left": 0.27, "right": 0.57, "rear": 1.0,
        }
        action, duration, reason = bridge.choose_explore_action(constrained_left)
        self.assertEqual((action, duration, reason),
                         ("right", 0.35, "bounded_lateral_turn_setup"))
        self.assertEqual(bridge.explore_step_for(action), 5)

    def test_bounded_right_shift_requires_measured_left_clearance_gain(self):
        before = {
            "front": 0.38, "front_left": 0.29, "front_right": 0.49,
            "left": 0.27, "right": 0.57, "rear": 1.0,
        }
        improved = dict(before, left=0.422, right=0.55)
        stalled = dict(before, left=0.274, right=0.55)
        self.assertTrue(bridge.escape_made_progress("right", before, improved))
        self.assertFalse(bridge.escape_made_progress("right", before, stalled))

    def test_guarded_turn_progress_window_fails_stalled_motion(self):
        self.assertFalse(bridge.turn_window_stalled(0.50, math.radians(0.0)))
        self.assertTrue(bridge.turn_window_stalled(0.75, math.radians(4.0)))
        self.assertFalse(bridge.turn_window_stalled(0.75, math.radians(12.0)))

    def test_local_exploration_alternates_ineffective_measured_yaw_turn(self):
        state = {
            "scan_seen": True,
            "scan_age_s": 0.01,
            "sectors": {
                "front": 0.43, "front_left": 0.65, "front_right": 0.46,
                "left": 0.58, "right": 0.50, "rear": 1.0,
            },
            "pose": {"yaw": 0.0},
            "slam": {"usable": True, "pose_valid": True},
        }
        guarded_result = {"ok": True, "reason": "target_reached:20.0deg", "elapsed_s": 0.8}
        with patch.object(bridge, "snapshot", return_value=state), \
                patch.object(bridge, "guarded_slam_turn", return_value=guarded_result) as turn, \
                patch.object(bridge, "motor_send") as motor, \
                patch.object(bridge, "stop_burst"):
            result = bridge.lidar_walk(max_duration=1.2, save=False)
        self.assertEqual(result["reason"], "escape_attempt_limit")
        self.assertEqual(turn.call_count, 4)
        turns = [call.kwargs["turn"] for call in turn.call_args_list]
        self.assertEqual(turns, ["left", "right", "left", "right"])
        for call in turn.call_args_list:
            kwargs = call.kwargs
            self.assertEqual((kwargs["degrees"], kwargs["step"]), (20.0, 30))
            self.assertGreaterEqual(kwargs["max_duration"], 1.0)
            self.assertLessEqual(kwargs["max_duration"], 1.2)
        motor.assert_not_called()

    def test_curve_continuation_does_not_override_guarded_turn(self):
        clear = {"front": 0.8, "front_left": 0.7, "front_right": 0.7,
                 "left": 0.7, "right": 0.8, "rear": 0.9}
        self.assertTrue(bridge.can_continue_in_gait_curve("forward", "curveright", clear))
        near = dict(clear, front=0.374, front_right=0.5)
        action, _, _ = bridge.choose_explore_action(near)
        self.assertEqual(action, "turnright")
        self.assertFalse(bridge.can_continue_in_gait_curve(action, "curveright", near))
        self.assertFalse(bridge.can_continue_in_gait_curve("forward", "curveright", near))
        self.assertFalse(bridge.can_continue_in_gait_curve("back", "curveright", clear))

    def test_packet_veto_replans_into_reverse_without_cancelling_walk(self):
        clear = {"front": 0.95, "front_left": 0.8, "front_right": 0.8,
                 "left": 0.7, "right": 0.7, "rear": 0.9}
        blocked = dict(clear, front=0.27)
        opened = dict(clear, front=0.68)
        def status(sectors, scan_seen=True):
            return {"scan_seen": scan_seen, "scan_age_s": 0.01,
                    "scan_count": 4, "sectors": sectors, "pose": {"yaw": 0.0},
                    "slam": {"usable": True, "pose_valid": True,
                             "pose_age_s": 0.01, "map_age_s": 0.01}}
        veto = bridge.AppPacketCollisionVeto("forward", blocked, 4)
        with patch.object(bridge, "snapshot", side_effect=[
                status(clear), status(clear), status(blocked), status(blocked),
                status(opened), status(opened, scan_seen=False), status(opened)]), \
                patch.object(bridge, "apply_navigation_course", side_effect=veto), \
                patch.object(bridge, "continuous_reverse_escape", return_value={
                    "ok": True, "reason": "reverse_clearance_gain_reached", "elapsed_s": 0.5}) as reverse, \
                patch.object(bridge, "pose_copy", return_value={"yaw": 0.0}), \
                patch.object(bridge, "stop_burst") as stop, \
                patch.object(bridge.time, "sleep"):
            result = bridge.lidar_walk(max_duration=5, save=False)
        self.assertEqual(result["reason"], "scan_stale_or_missing")
        self.assertEqual(result["trace"][0]["why"], "app_packet_collision_veto")
        self.assertEqual(result["trace"][1]["action"], "back")
        reverse.assert_called_once()
        self.assertGreaterEqual(stop.call_count, 1)

    def test_packet_guard_preserves_rear_side_and_turn_envelope(self):
        clear = {"front": 0.3, "front_left": 0.2, "front_right": 0.2,
                 "left": 0.7, "right": 0.7, "rear": 0.9}
        status = {"scan_seen": True, "scan_age_s": 0.01,
                  "scan_count": 6, "sectors": clear, "slam": {"usable": True}}
        lease = bridge.begin_motion("test", max_duration=5)
        try:
            with patch.object(bridge, "snapshot", return_value=status):
                bridge.app_packet_guard("back", lease, lease.deadline)
                with self.assertRaises(bridge.AppPacketCollisionVeto):
                    bridge.app_packet_guard("turnleft", lease, lease.deadline)
            # A close front is not a reverse veto, but a rear breach is.
            with patch.object(bridge, "snapshot", return_value=dict(status,
                    sectors=dict(clear, rear=0.27))):
                with self.assertRaises(bridge.AppPacketCollisionVeto):
                    bridge.app_packet_guard("back", lease, lease.deadline)
        finally:
            bridge.end_motion(lease)

    def test_prewrite_packet_veto_preserves_lease_but_partial_write_cancels(self):
        clear = {"front": 0.62, "front_left": 0.8, "front_right": 0.8,
                 "left": 0.7, "right": 0.7, "rear": 0.9}
        packet = bridge.pkt(0x11, [0, 100])
        lease = bridge.begin_motion("test", max_duration=5)
        try:
            with patch.object(bridge, "app_packet_guard", side_effect=
                    bridge.AppPacketCollisionVeto("forward", clear, 7)):
                with self.assertRaises(bridge.AppPacketCollisionVeto):
                    bridge.app_transaction("forward", [(packet, 0)])
            self.assertFalse(lease.cancel_event.is_set())
            socket = MagicMock()
            socket.__enter__.return_value = socket
            socket.recv.return_value = b""
            socket.send.return_value = len(packet)
            veto = bridge.AppPacketCollisionVeto("forward", clear, 8)
            with patch.object(bridge.socket, "create_connection", return_value=socket), \
                    patch.object(bridge, "app_packet_guard", side_effect=[None, None, None, None, veto]):
                with self.assertRaises(bridge.AppPacketCollisionVeto) as caught:
                    bridge.app_transaction("forward", [(packet, 0)])
            self.assertEqual(caught.exception.sent_bytes, len(packet))
            self.assertTrue(lease.cancel_event.is_set())
            socket.send.assert_called_once_with(packet)
        finally:
            bridge.end_motion(lease)

    def test_legacy_forward_endpoint_delegates_to_six_sector_guard(self):
        guarded = {"ok": False, "reason": "footprint_clearance_rejected_before_start"}
        with patch.object(bridge, "forward_continuous_until", return_value=guarded) as continuous:
            result = bridge.forward_until(target_front=0.10, max_duration=2.0)

        continuous.assert_called_once_with(target_front=0.38, max_duration=2.0, min_target=0.38)
        self.assertEqual(result["controller"], "forward_continuous_until")
        self.assertEqual(result["mode"], "forward_until")
        self.assertFalse(result["ok"])
        self.assertFalse(hasattr(bridge, "_forward_until_legacy_disabled"))

    def test_forward_supervisor_enforces_corridor_threshold_while_moving(self):
        state = {
            "scan_seen": True,
            "scan_age_s": 0.01,
            "sectors": {
                "front": 0.44, "front_left": 0.80, "front_right": 0.80,
                "left": 0.70, "right": 0.70, "rear": 0.70,
            },
        }
        with patch.object(bridge, "snapshot", return_value=state):
            result = bridge.supervise_lidar_motion("forward", 1.0)
        self.assertFalse(result["ok"])
        self.assertEqual(result["reason"], "forward_footprint_breach")

    def test_continuous_approach_rejects_front_below_autonomous_floor(self):
        state = {
            "scan_seen": True,
            "scan_age_s": 0.01,
            "slam": {"active": True, "pose_valid": True, "usable": True,
                     "pose_age_s": 0.01, "map_age_s": 0.01},
            "pose": {"yaw": 0.0},
            "sectors": {
                "front": 0.37, "front_left": 0.80, "front_right": 0.80,
                "left": 0.70, "right": 0.70, "rear": 0.70,
            },
        }
        with patch.object(bridge, "snapshot", return_value=state), \
                patch.object(bridge, "motor_send") as motor:
            result = bridge.forward_continuous_until(target_front=0.25, max_duration=1.0)
        self.assertFalse(result["ok"])
        self.assertEqual(result["reason"], "footprint_clearance_rejected_before_start")
        motor.assert_not_called()

    def test_continuous_approach_aborts_if_front_floor_is_crossed(self):
        safe = {
            "scan_seen": True,
            "scan_age_s": 0.01,
            "slam": {"active": True, "pose_valid": True, "usable": True,
                     "pose_age_s": 0.01, "map_age_s": 0.01},
            "pose": {"yaw": 0.0},
            "sectors": {
                "front": 0.80, "front_left": 0.80, "front_right": 0.80,
                "left": 0.70, "right": 0.70, "rear": 0.70,
            },
        }
        breached = dict(safe, sectors=dict(safe["sectors"], front=0.37))
        with patch.object(bridge, "snapshot", side_effect=[safe, breached]), \
                patch.object(bridge, "motor_send") as motor, \
                patch.object(bridge, "stop_burst"):
            result = bridge.forward_continuous_until(target_front=0.38, max_duration=1.0)
        self.assertFalse(result["ok"])
        self.assertTrue(result["reason"].startswith("footprint_clearance_breach:front=0.370"))
        motor.assert_called_once_with("forward", step=None)

    def test_backward_supervisor_stops_when_rear_clearance_closes(self):
        state = {
            "scan_seen": True,
            "scan_age_s": 0.01,
            "slam": {"usable": True},
            "sectors": {
                "front": 0.20, "front_left": 0.22, "front_right": 0.24,
                "left": 0.5, "right": 0.5, "rear": 0.27,
            },
        }
        with patch.object(bridge, "snapshot", return_value=state):
            result = bridge.supervise_lidar_motion("back", 1.0)
        self.assertFalse(result["ok"])
        self.assertEqual(result["reason"], "rear_clearance_breach")

    def test_backward_supervisor_stops_before_side_envelope_is_lost(self):
        state = {
            "scan_seen": True,
            "scan_age_s": 0.01,
            "slam": {"usable": True},
            "sectors": dict(FRONT_BLOCKED_REAR_CLEAR, left=0.23),
        }
        with patch.object(bridge, "snapshot", return_value=state):
            result = bridge.supervise_lidar_motion("back", 1.0)
        self.assertFalse(result["ok"])
        self.assertEqual(result["reason"], "side_clearance_breach_during_backoff")

    def test_backward_supervisor_stops_when_front_clearance_worsens(self):
        baseline = dict(FRONT_BLOCKED_REAR_CLEAR, front=0.87, front_left=0.412, front_right=0.719)
        worsened = dict(baseline, front=0.54, front_left=0.37, front_right=0.69)
        state = {
            "scan_seen": True,
            "scan_age_s": 0.01,
            "slam": {"usable": True},
            "sectors": worsened,
            "pose": {"yaw": 0.01},
            "scan_count": 101,
        }
        with patch.object(bridge, "snapshot", side_effect=[state, dict(state, scan_count=101),
                                                           dict(state, scan_count=102)]):
            result = bridge.supervise_lidar_motion(
                "back", 0.30, baseline_sectors=baseline, baseline_pose={"yaw": 0.0})
        self.assertFalse(result["ok"])
        self.assertEqual(result["reason"], "front_clearance_worsening_during_backoff")

    def test_backward_supervisor_ignores_one_worsening_scan_then_recovers(self):
        baseline = dict(FRONT_BLOCKED_REAR_CLEAR, front=0.87, front_left=0.62, front_right=0.72)
        bad = dict(baseline, front=0.57, front_left=0.56, front_right=0.69)
        def sample(count, sectors):
            return {"scan_seen": True, "scan_age_s": 0.01, "scan_count": count,
                    "slam": {"usable": True}, "sectors": sectors, "pose": {"yaw": 0.0}}
        states = [sample(1, bad), sample(1, bad), sample(2, baseline)]
        with patch.object(bridge, "snapshot", side_effect=states):
            result = bridge.supervise_lidar_motion(
                "back", 0.21, baseline_sectors=baseline, baseline_pose={"yaw": 0.0})
        self.assertTrue(result["ok"])
        self.assertEqual(result["reason"], "window_complete")

    def test_post_backoff_turn_requires_safe_settled_envelope(self):
        reason = {"ok": False, "reason": "front_clearance_worsening_during_backoff"}
        status = {"moving": False, "scan_seen": True, "scan_age_s": 0.01,
                  "slam": {"usable": True},
                  "sectors": dict(FRONT_BLOCKED_REAR_CLEAR, front=0.55,
                                  front_left=0.51, front_right=0.50, left=0.8,
                                  right=0.55, rear=0.9)}
        self.assertEqual(bridge.turn_after_stopped_backoff(reason, status), "turnleft")
        self.assertEqual(bridge.turn_after_stopped_backoff(
            reason, dict(status, sectors=dict(status["sectors"], front_right=0.43))), "turnleft")
        self.assertIsNone(bridge.turn_after_stopped_backoff(
            reason, dict(status, sectors=dict(status["sectors"], front_right=0.279))))
        self.assertIsNone(bridge.turn_after_stopped_backoff(
            reason, dict(status, moving=None)))
        self.assertIsNone(bridge.turn_after_stopped_backoff(
            reason, dict(status, fall_suspected=True)))
        self.assertIsNone(bridge.turn_after_stopped_backoff(
            {"ok": False, "reason": "scan_stale_during_lidar_walk"}, status))

    def test_multiple_stopped_backoff_handoffs_and_forward_reset(self):
        for actions, expected_counts in (
                (["back", "stop", "back", "stop", "stop"], [1, 2]),
                (["back", "stop", "forward", "back", "stop", "stop"], [1, 1])):
            with self.subTest(actions=actions):
                sector = {"front": 0.456, "front_left": 0.456, "front_right": 0.456,
                          "left": 0.6, "right": 0.8, "rear": 0.9}
                def status():
                    return {"moving": False, "scan_seen": True, "scan_age_s": .01,
                            "sectors": dict(sector), "pose": {"yaw": 0.0},
                            "slam": {"usable": True, "pose_valid": True,
                                     "pose_age_s": .01, "map_age_s": .01}}
                def turn(**kwargs):
                    sector["front"] += .1
                    sector["front_left"] += .1
                    sector["front_right"] += .1
                    return {"ok": True, "reason": "target_reached:20deg", "elapsed_s": 1.0}
                decisions = [(a, 1.0, "test") if a != "stop" else
                             ("stop", 0.0, "test_done") for a in actions]
                with patch.object(bridge, "snapshot", side_effect=status), \
                        patch.object(bridge, "choose_explore_action", side_effect=decisions), \
                        patch.object(bridge, "continuous_reverse_escape", return_value={
                            "ok": False, "reason": "front_clearance_worsening_during_backoff",
                            "elapsed_s": .4}), \
                        patch.object(bridge, "guarded_slam_turn", side_effect=turn), \
                        patch.object(bridge, "apply_navigation_course", return_value=("forward", True)), \
                        patch.object(bridge, "supervise_lidar_forward", return_value={
                            "ok": True, "reason": "window_complete", "elapsed_s": .5}), \
                        patch.object(bridge, "pose_copy", return_value={"yaw": 0.0}), \
                        patch.object(bridge, "stop_burst"), patch.object(bridge.time, "sleep"):
                    result = bridge.lidar_walk(max_duration=5, save=False)
                handoffs = [x["consecutive_handoffs"] for x in result["trace"]
                            if x.get("next_turn")]
                self.assertEqual(handoffs, expected_counts)
                self.assertEqual(result["reason"], "test_done")

    def test_backward_supervisor_tolerates_sub_eight_cm_triplet_change(self):
        baseline = dict(FRONT_BLOCKED_REAR_CLEAR, front=0.87, front_left=0.412, front_right=0.719)
        worsened = dict(baseline, front=0.84, front_left=0.40, front_right=0.719)
        state = {"scan_seen": True, "scan_age_s": 0.01,
                 "slam": {"usable": True}, "sectors": worsened,
                 "pose": {"yaw": 0.0}}
        with patch.object(bridge, "snapshot", return_value=state):
            result = bridge.supervise_lidar_motion(
                "back", 0.01, baseline_sectors=baseline, baseline_pose={"yaw": 0.0})
        self.assertTrue(result["ok"])
        self.assertEqual(result["reason"], "window_complete")

    def test_supervised_reverse_and_exploration_use_captured_app_scale(self):
        self.assertEqual(bridge.BACKOFF_MAX_FRONT_LOSS_M, 0.12)
        self.assertEqual(math.degrees(bridge.BACKOFF_MAX_HEADING_DRIFT_RAD), 20.0)
        self.assertEqual(bridge.REVERSE_ESCAPE_STEP, 10)
        self.assertEqual(bridge.app_joystick_percent(bridge.REVERSE_ESCAPE_STEP), 100)
        self.assertEqual(bridge.explore_step_for("backward"), bridge.REVERSE_ESCAPE_STEP)
        state = {"sectors": FRONT_BLOCKED_REAR_CLEAR, "pose": {"yaw": 0.0}}
        with patch.object(bridge, "snapshot", return_value=state), \
                patch.object(bridge, "motor_send") as send, \
                patch.object(bridge, "supervise_lidar_motion", return_value={"ok": True, "reason": "window_complete"}), \
                patch.object(bridge, "stop_burst"):
            bridge.run_supervised_calibration("backward", 0.3)
        send.assert_called_once_with("backward", step=bridge.REVERSE_ESCAPE_STEP)

    def test_backward_supervisor_stops_on_unexpected_yaw(self):
        baseline = dict(FRONT_BLOCKED_REAR_CLEAR, front=0.87, front_left=0.412, front_right=0.719)
        state = {
            "scan_seen": True,
            "scan_age_s": 0.01,
            "slam": {"usable": True},
            "sectors": baseline,
            "pose": {"yaw": math.radians(21.0)},
        }
        with patch.object(bridge, "snapshot", return_value=state):
            result = bridge.supervise_lidar_motion(
                "back", 0.30, baseline_sectors=baseline, baseline_pose={"yaw": 0.0})
        self.assertFalse(result["ok"])
        self.assertEqual(result["reason"], "heading_drift_during_backoff")
        self.assertGreater(result["heading_drift_degrees"], 20.0)

    def test_backward_supervisor_stops_as_soon_as_clearance_gain_is_reached(self):
        baseline = dict(FRONT_BLOCKED_REAR_CLEAR, front=0.50, front_left=0.50, front_right=0.50)
        improved = dict(baseline, front=0.59, front_left=0.59, front_right=0.59)
        state = {
            "scan_seen": True,
            "scan_age_s": 0.01,
            "slam": {"usable": True},
            "sectors": improved,
            "pose": {"yaw": 0.0},
        }
        with patch.object(bridge, "snapshot", return_value=state):
            result = bridge.supervise_lidar_motion(
                "back", bridge.REVERSE_ESCAPE_MAX_S,
                baseline_sectors=baseline, baseline_pose={"yaw": 0.0},
                stop_on_front_gain_m=bridge.ESCAPE_CLEARANCE_GAIN_M)
        self.assertTrue(result["ok"])
        self.assertEqual(result["reason"], "reverse_clearance_gain_reached")
        self.assertGreaterEqual(result["front_clearance_gain_m"], 0.08)
        self.assertLess(result["elapsed_s"], bridge.REVERSE_ESCAPE_MAX_S)

    def test_reverse_recovery_uses_one_continuous_command(self):
        baseline = {
            "front": 0.64, "front_left": 0.63, "front_right": 0.48,
            "left": 0.57, "right": 0.44, "rear": 1.12,
        }

        def state(sectors, yaw):
            return {
                "scan_seen": True, "scan_age_s": 0.01, "sectors": sectors,
                "pose": {"yaw": yaw},
                "slam": {"usable": True, "pose_valid": True,
                         "pose_age_s": 0.01, "map_age_s": 0.01},
            }

        gained = dict(baseline, front=0.73, front_left=0.72, front_right=0.56)
        segment_results = [
            {"ok": True, "reason": "reverse_clearance_gain_reached", "elapsed_s": 0.8,
             "front_clearance_gain_m": bridge.ESCAPE_CLEARANCE_GAIN_M},
        ]
        turn_result = {"ok": True, "reason": "target_reached:5.1deg", "elapsed_s": 0.5}
        with patch.object(bridge, "snapshot", side_effect=[
                state(baseline, 0.0), state(gained, 0.0),
                ]), \
                patch.object(bridge, "supervise_lidar_motion", side_effect=segment_results), \
                patch.object(bridge, "guarded_slam_turn", return_value=turn_result) as correction, \
                patch.object(bridge, "motor_send") as motor, \
                patch.object(bridge, "stop_burst"):
            result = bridge.continuous_reverse_escape(baseline, {"yaw": 0.0})
        self.assertTrue(result["ok"])
        self.assertEqual(result["reason"], "reverse_clearance_gain_reached")
        self.assertEqual(result["control"], "single_continuous_reverse")
        self.assertEqual(motor.call_count, 1)
        self.assertEqual(motor.call_args.args, ("back",))
        self.assertEqual(motor.call_args.kwargs["step"], bridge.REVERSE_ESCAPE_STEP)
        correction.assert_not_called()

    def test_reverse_worsening_aborts_without_turn_retry(self):
        baseline = {
            "front": 0.62, "front_left": 0.62, "front_right": 0.50,
            "left": 0.57, "right": 0.43, "rear": 1.0,
        }
        opened = {
            "front": 0.54, "front_left": 0.52, "front_right": 0.56,
            "left": 0.60, "right": 0.50, "rear": 1.0,
        }
        before = {
            "scan_seen": True, "scan_age_s": 0.01, "sectors": baseline,
            "pose": {"yaw": 0.0}, "slam": {"usable": True, "pose_valid": True},
        }
        after = {
            "scan_seen": True, "scan_age_s": 0.01, "sectors": opened,
            "pose": {"yaw": 0.0}, "slam": {"usable": True, "pose_valid": True},
        }
        turn_result = {"ok": True, "reason": "target_reached:16.0deg",
                       "elapsed_s": 1.0, "status": after}
        with patch.object(bridge, "snapshot", side_effect=[before, after]), \
                patch.object(bridge, "supervise_lidar_motion", return_value={
                    "ok": False, "reason": "front_clearance_worsening_during_backoff",
                    "elapsed_s": 0.4}), \
                patch.object(bridge, "guarded_slam_turn", return_value=turn_result) as turn, \
                patch.object(bridge, "motor_send"), patch.object(bridge, "stop_burst"):
            result = bridge.continuous_reverse_escape(baseline, {"yaw": 0.0})
        self.assertFalse(result["ok"])
        self.assertEqual(result["reason"], "front_clearance_worsening_during_backoff")
        turn.assert_not_called()

    def test_autonomous_mission_refuses_boxed_in_preflight(self):
        status = {
            "scan_seen": True,
            "scan_age_s": 0.01,
            "sectors": dict(WALL_CONTACT, rear=0.30, right=0.31),
            "slam": {"active": True, "pose_valid": True, "usable": True,
                     "pose_age_s": 0.01, "map_age_s": 0.01},
        }
        with patch.object(bridge, "snapshot", return_value=status):
            result = bridge.start_autonomous_mission(mode="local", duration_s=30)
        self.assertFalse(result["ok"])
        self.assertEqual(result["reason"], "footprint_clearance_rejected_before_start")
        self.assertEqual(result["safety_reason"], "footprint_boxed_in_stop")

    def test_autonomous_mission_refuses_missing_sector_without_reverse_command(self):
        status = {
            "scan_seen": True,
            "scan_age_s": 0.01,
            "slam": {"usable": True},
            "sectors": {
                "front_left": 0.8, "front_right": 0.8,
                "left": 0.7, "right": 0.7, "rear": 0.7,
            },
            "slam": {"active": True, "pose_valid": True, "usable": True,
                     "pose_age_s": 0.01, "map_age_s": 0.01},
        }
        with patch.object(bridge, "snapshot", return_value=status), \
                patch.object(bridge, "motor_send") as motor:
            result = bridge.start_autonomous_mission(mode="local", duration_s=30)
        self.assertFalse(result["ok"])
        self.assertEqual(result["reason"], "footprint_clearance_rejected_before_start")
        self.assertEqual(result["safety_reason"], "lidar_sector_missing_or_invalid")
        motor.assert_not_called()


if __name__ == "__main__":
    unittest.main()
