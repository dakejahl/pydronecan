#
# Copyright (C) 2014-2015  UAVCAN Development Team  <uavcan.org>
#
# This software is distributed under the terms of the MIT License.
#
# Author: Ben Dyer <ben_dyer@mac.com>
#         Pavel Kirienko <pavel.kirienko@zubax.com>
#

import unittest
try:
    from unittest.mock import Mock, ANY
except ImportError:
    from mock import Mock, ANY

import dronecan
from dronecan.node import Node
import dronecan.transport as transport
from dronecan.driver.common import CANFrame

import os.path

DEFAULT_TRANSFER_PRIORITY = 20

class TestException(RuntimeError):
    pass


class HandlerExceptionHandling(unittest.TestCase):
    def setUp(self):
        dronecan.load_dsdl()
        msg = dronecan.uavcan.protocol.debug.KeyValue()
        msg.key = 'foo'
        msg.value = 42

        transfer = transport.Transfer(
            payload=msg,
            source_node_id=42,
            transfer_id=10,
            transfer_priority=DEFAULT_TRANSFER_PRIORITY,
            service_not_message=False)

        self.frames = [
            CANFrame(can_id=f.message_id, data=f.bytes, extended=True)
            for f in transfer.to_frames()
        ]
        self.driver = Mock()
        self.driver.receive.side_effect = self.frames + [None]

    def _raise_callback(self, event):
        raise TestException("test exception")

    def _spin(self):
        try:
            self.node.spin()
        except StopIteration:  # due to using mocks for can frames
            pass

    def test_exceptions_are_caught_by_spin(self):
        self.node = Node(self.driver)
        self.node.add_handler(dronecan.uavcan.protocol.debug.KeyValue,
                              self._raise_callback)

        self._spin()

    def test_exceptions_are_not_caught_by_spin(self):
        self.node = Node(self.driver, catch_handler_exceptions=False)
        self.node.add_handler(dronecan.uavcan.protocol.debug.KeyValue,
                              self._raise_callback)

        with self.assertRaises(RuntimeError):
            self._spin()

    def test_second_handler_is_called_after_first_one_raises(self):
        mock_cb = Mock(return_value=None)
        self.node = Node(self.driver)
        self.node.add_handler(dronecan.uavcan.protocol.debug.KeyValue,
                              self._raise_callback)
        self.node.add_handler(dronecan.uavcan.protocol.debug.KeyValue,
                              mock_cb)

        self._spin()
        mock_cb.assert_any_call(ANY)


class TransferErrorHandling(unittest.TestCase):
    UNKNOWN_DTID = 20999

    def setUp(self):
        dronecan.load_dsdl()
        self.assertNotIn((self.UNKNOWN_DTID, dronecan.dsdl.CompoundType.KIND_MESSAGE), dronecan.DATATYPES)

        msg = dronecan.uavcan.protocol.debug.KeyValue()
        msg.key = 'foo'
        msg.value = 42
        transfer = transport.Transfer(
            payload=msg,
            source_node_id=42,
            transfer_id=10,
            transfer_priority=DEFAULT_TRANSFER_PRIORITY,
            service_not_message=False)
        self.good_frames = [
            CANFrame(can_id=f.message_id, data=f.bytes, extended=True)
            for f in transfer.to_frames()
        ]
        self.driver = Mock()

    def _single_frame(self, dtid, payload, transfer_id=0):
        can_id = (DEFAULT_TRANSFER_PRIORITY << 24) | (dtid << 8) | 42
        return CANFrame(can_id=can_id, data=bytearray(payload) + bytearray([0xC0 | transfer_id]), extended=True)

    def test_unknown_data_type_is_dropped(self):
        frames = [self._single_frame(self.UNKNOWN_DTID, b'\x01\x02\x03')] + self.good_frames
        self.driver.receive.side_effect = frames + [None]
        mock_cb = Mock(return_value=None)
        node = Node(self.driver)
        node.add_handler(dronecan.uavcan.protocol.debug.KeyValue, mock_cb)

        with self.assertLogs('dronecan.node', 'WARNING'):
            self.assertEqual(node.spin(0), len(frames))
        mock_cb.assert_called_once_with(ANY)

    def test_unknown_data_type_raises_when_not_caught(self):
        self.driver.receive.side_effect = [self._single_frame(self.UNKNOWN_DTID, b'\x01\x02\x03'), None]
        node = Node(self.driver, catch_transfer_errors=False)

        with self.assertRaises(transport.TransferError):
            node.spin(0)

    def test_short_payload_raises_transfer_error(self):
        dtid = dronecan.uavcan.protocol.NodeStatus.default_dtid
        self.driver.receive.side_effect = [self._single_frame(dtid, b'\x01\x02'), None]
        node = Node(self.driver, catch_transfer_errors=False)

        with self.assertRaises(transport.TransferError):
            node.spin(0)

    def test_repeated_errors_are_rate_limited(self):
        frames = [self._single_frame(self.UNKNOWN_DTID, b'\x01', tid) for tid in range(5)]
        self.driver.receive.side_effect = frames + [None]
        node = Node(self.driver)

        with self.assertLogs('dronecan.node', 'WARNING') as logs:
            node.spin(0)
        self.assertEqual(len(logs.records), 1)


if __name__ == '__main__':
    unittest.main()
