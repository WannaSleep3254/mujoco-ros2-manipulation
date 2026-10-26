"""Review presentation facts cannot change approval or execute a saved plan."""
import unittest

from mujoco_ros2_manipulation.operator_console.review_summary import summarize_review
from mujoco_ros2_manipulation.workcells.motion_request import command


class ReviewSummaryTests(unittest.TestCase):
    def record(self):
        return dict(command=command(dict(l=.3,r=-.4),strategy='departure_timing'),
            initial_state_rad=dict(l=.1,r=.2),
            planned_clearance=dict(complete=True,duration_sec=2.,
                minimum=dict(distance_m=.04,time_sec=.75,bodies=['left','right'])))

    def test_signed_radian_endpoint_changes_and_checked_path_duration(self):
        record=self.record();summary=summarize_review(record)
        self.assertEqual(summary['strategy'],'departure_timing')
        self.assertEqual(summary['duration_sec'],2.)
        self.assertAlmostEqual(summary['target_delta_rad']['l'],.2)
        self.assertAlmostEqual(summary['target_delta_rad']['r'],-.6)
        self.assertAlmostEqual(summary['max_abs_target_delta_rad'],.6)
        self.assertEqual(summary['minimum'],dict(distance_m=.04,time_sec=.75,pair=['left','right']))
        summary['minimum']['pair'].clear();summary['target_delta_rad'].clear()
        self.assertEqual(record['planned_clearance']['minimum']['bodies'],['left','right'])
        self.assertEqual(len(summarize_review(record)['target_delta_rad']),2)

    def test_incomplete_checks_malformed_states_and_nonfinite_values_do_not_get_summary(self):
        mutations=(lambda r:r['planned_clearance'].update(complete=False),
            lambda r:r['planned_clearance'].update(duration_sec=float('inf')),
            lambda r:r['planned_clearance']['minimum'].update(distance_m=float('nan')),
            lambda r:r.update(initial_state_rad={}),lambda r:r['initial_state_rad'].pop('r'),
            lambda r:r['command']['target_rad'].update(l=True),
            lambda r:r['command']['target_rad'].update(l=float('nan')),
            lambda r:r['command'].update(strategy='unknown'),
            lambda r:r['command']['target_rad'].update(l=10**1000),
            lambda r:(r['initial_state_rad'].update(l=-1e308),r['command']['target_rad'].update(l=1e308)))
        for mutate in mutations:
            with self.subTest(mutate=mutate):
                record=self.record();mutate(record)
                self.assertIsNone(summarize_review(record))

    def test_older_or_invalid_minimum_time_disables_only_jump_information(self):
        for stamp in (None,True,-.01,2.001,float('nan'),float('inf')):
            with self.subTest(stamp=stamp):
                record=self.record();record['planned_clearance']['minimum'].update(time_sec=stamp,bodies=['only-one'])
                summary=summarize_review(record)
                self.assertIsNone(summary['minimum']['time_sec'])
                self.assertIsNone(summary['minimum']['pair'])
                self.assertEqual(summary['minimum']['distance_m'],.04)


if __name__=='__main__':unittest.main()
