"""Recording persistence cannot delay cancellation or skip ROS cleanup."""
from contextlib import redirect_stdout
import io
import signal
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

from mujoco_ros2_manipulation.operator_console.application import ConsoleApplication
from mujoco_ros2_manipulation.operator_console.__main__ import shutdown_console


def application(*, clean=True, attached_active=False):
    app = object.__new__(ConsoleApplication)
    events = []
    app.closing = threading.Event()
    app.worker = None
    app.files = []
    app.children = {}
    identifier = 'a'*32
    app.state = NS(lock=threading.RLock(), active=identifier if attached_active else None,
                   ui_ids={identifier} if attached_active else set())
    app.suspend_dispatch = Mock(side_effect=lambda:events.append('suspend'))
    app.demos = NS(stop_sync=Mock(side_effect=lambda:events.append('demo_stop')))
    app.views = NS(close=Mock(side_effect=lambda:events.append('view_close')))
    app.monitor = NS(join=Mock(side_effect=lambda **_:events.append('monitor_join')))
    def cancel(active):
        events.append('cancel')
        app.state.active = None
    app.cancel = Mock(side_effect=cancel)
    app.stop = Mock(side_effect=lambda:events.append('native_stop') or {'cleanup_errors':[]})
    app.audit = Mock(side_effect=lambda action,**details:events.append((action,details)))
    def join_recording():
        # Simulate a recorder saving/fsyncing. It may wait only after native
        # ownership and cancellation have been handled.
        if 'native_stop' not in events or 'suspend' not in events:
            raise AssertionError('Recording join preceded owned process stop')
        if attached_active and 'cancel' not in events:
            raise AssertionError('Recording join preceded attached action cancel')
        events.append('recording_join')
        return clean
    app.sync_recordings = NS(request_close=Mock(side_effect=lambda:events.append('recording_stop_request')),
                            close=Mock(side_effect=join_recording))
    return app,events


class ConsoleRecordingShutdownTests(unittest.TestCase):
    def test_native_stop_and_attached_cancel_precede_wait_for_saving_recorder(self):
        app,events=application(attached_active=True)
        self.assertTrue(app.close())
        self.assertTrue(app.closing.is_set())
        self.assertLess(events.index('recording_stop_request'),events.index('suspend'))
        self.assertLess(events.index('cancel'),events.index('recording_join'))
        self.assertLess(events.index('native_stop'),events.index('recording_join'))
        app.audit.assert_any_call('synchronized_capture_cleanup',clean=True)

    def test_unclean_recorder_is_reported_after_native_cleanup(self):
        app,events=application(clean=False)
        self.assertFalse(app.close())
        self.assertIn('native_stop',events)
        app.audit.assert_any_call('synchronized_capture_cleanup',clean=False)
        errors=next(call.kwargs['errors'] for call in app.audit.call_args_list if call.args[0]=='console_cleanup_failed')
        self.assertTrue(any(item['error_type']=='UncleanRecordingWorker' for item in errors))

    def test_demo_or_view_failure_does_not_skip_owned_stop_or_recording_join(self):
        for broken in ('demo','view'):
            with self.subTest(broken=broken):
                app,events=application()
                target=app.demos.stop_sync if broken=='demo' else app.views.close
                target.side_effect=RuntimeError('diagnostic cleanup failure')
                self.assertFalse(app.close())
                self.assertIn('native_stop',events)
                self.assertIn('recording_join',events)

    def test_legacy_fixture_without_recordings_still_cleans_native_ownership(self):
        app,events=application()
        del app.sync_recordings
        self.assertTrue(app.close())
        self.assertIn('native_stop',events)

    def test_owned_process_cleanup_errors_make_close_unclean(self):
        app,events=application()
        app.stop.side_effect=lambda:events.append('native_stop') or {'cleanup_errors':['simulation']}
        self.assertFalse(app.close())
        self.assertIn('recording_join',events)


class MainCleanupTests(unittest.TestCase):
    def fixture(self):
        events=[]
        server=NS(shutdown=Mock(side_effect=lambda:events.append('http_shutdown')),
                  server_close=Mock(side_effect=lambda:events.append('http_close')))
        app=NS(close=Mock(side_effect=lambda:events.append('app_close') or True))
        stopped=threading.Event()
        thread=NS(join=Mock(side_effect=lambda **_:events.append('ros_join')),is_alive=lambda:False)
        node=NS(destroy_node=Mock(side_effect=lambda:events.append('node_destroy')))
        ros=NS(ok=lambda:True,shutdown=Mock(side_effect=lambda:events.append('ros_shutdown')))
        return server,app,stopped,thread,node,ros,events

    def test_false_app_close_returns_failure_after_ros_cleanup(self):
        server,app,stopped,thread,node,ros,events=self.fixture()
        app.close.side_effect=lambda:events.append('app_close') or False
        self.assertFalse(shutdown_console(server,app,stopped,thread,node,ros,{}))
        self.assertTrue(stopped.is_set())
        self.assertEqual(events,['http_shutdown','http_close','app_close','ros_join','node_destroy','ros_shutdown'])

    def test_app_exception_and_node_exception_still_shutdown_context_and_restore_signals(self):
        server,app,stopped,thread,node,ros,events=self.fixture()
        app.close.side_effect=RuntimeError('recording persistence failure')
        node.destroy_node.side_effect=RuntimeError('node cleanup failure')
        with redirect_stdout(io.StringIO()),patch('mujoco_ros2_manipulation.operator_console.__main__.signal.signal') as restore:
            self.assertFalse(shutdown_console(server,app,stopped,thread,node,ros,{signal.SIGTERM:signal.SIG_DFL}))
        self.assertTrue(stopped.is_set())
        thread.join.assert_called_once_with(timeout=3)
        ros.shutdown.assert_called_once_with()
        restore.assert_called_once_with(signal.SIGTERM,signal.SIG_DFL)

    def test_http_shutdown_error_still_closes_application_and_ros(self):
        server,app,stopped,thread,node,ros,events=self.fixture()
        server.shutdown.side_effect=RuntimeError('server cleanup failure')
        with redirect_stdout(io.StringIO()):
            self.assertFalse(shutdown_console(server,app,stopped,thread,node,ros,{}))
        server.server_close.assert_called_once_with()
        app.close.assert_called_once_with()
        ros.shutdown.assert_called_once_with()

    def test_live_ros_thread_is_an_unclean_exit_without_skipping_node_cleanup(self):
        server,app,stopped,thread,node,ros,events=self.fixture()
        thread.is_alive=lambda:True
        self.assertFalse(shutdown_console(server,app,stopped,thread,node,ros,{}))
        node.destroy_node.assert_called_once_with()
        ros.shutdown.assert_called_once_with()


if __name__=='__main__':unittest.main()
