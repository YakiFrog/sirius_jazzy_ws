#!/usr/bin/env python3
"""Publish the newest PointCloud2 at a bounded display rate.

This display-only relay preserves timestamps and point fields. Optional spatial
sampling keeps every ring. Only the newest input is retained.
"""
import argparse
import copy
import math
import time


def decimate_cloud(message, stride):
    """Keep every Nth point within each ring, preserving selected point bytes."""
    if stride < 1:
        raise ValueError('point_stride must be positive')
    if stride == 1 or message.width * message.height == 0:
        return message
    import numpy as np

    shape = (message.height, message.width)
    strides = (message.row_step, message.point_step)
    records = np.ndarray(shape, dtype=f'V{message.point_step}',
                         buffer=message.data, strides=strides).reshape(-1)
    ring = next((field for field in message.fields if field.name == 'ring'), None)
    types = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4'}
    if ring is not None:
        if ring.count != 1 or ring.datatype not in types:
            raise ValueError('ring must be a scalar integer field')
        dtype = np.dtype(('>' if message.is_bigendian else '<') + types[ring.datatype])
        if ring.offset < 0 or ring.offset + dtype.itemsize > message.point_step:
            raise ValueError('ring field lies outside point_step')
        rings = np.ndarray(shape, dtype=dtype, buffer=message.data,
                           offset=ring.offset, strides=strides).reshape(-1)
        indices = np.sort(np.concatenate([
            np.flatnonzero(rings == value)[::stride] for value in np.unique(rings)
        ]))
    else:
        indices = np.arange(0, records.size, stride)
    result = copy.copy(message)
    result.height = 1
    result.width = len(indices)
    result.row_step = result.width * result.point_step
    result.data = records[indices].tobytes()
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', nargs='?', default='/velodyne_points_leveled')
    parser.add_argument('--output', default='/velodyne_points_preview')
    parser.add_argument('--rate', type=float, default=5.)
    parser.add_argument('--point-stride', type=int, default=2,
                        help='Keep every Nth point per ring, display only (default: half the points)')
    parser.add_argument('--max-age', type=float, default=.5,
                        help='Discard samples waiting in this relay longer than this many seconds')
    args, ros_args = parser.parse_known_args(argv)
    if (not math.isfinite(args.rate) or args.rate <= 0 or
            not math.isfinite(args.max_age) or args.max_age <= 0):
        parser.error('--rate and --max-age must be finite and positive')
    if args.input == args.output:
        parser.error('Input and output must be different topics')
    if args.point_stride < 1:
        parser.error('--point-stride must be positive')
    import rclpy
    from rclpy.serialization import deserialize_message
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rcl_interfaces.msg import SetParametersResult
    from sensor_msgs.msg import PointCloud2

    rclpy.init(args=ros_args)
    node = rclpy.create_node('sirius_pointcloud_preview')
    node.declare_parameter('point_stride', args.point_stride)
    def validate_parameters(parameters):
        for parameter in parameters:
            if parameter.name == 'point_stride' and (
                    type(parameter.value) is not int or parameter.value < 1):
                return SetParametersResult(successful=False,
                                           reason='point_stride must be a positive integer')
        return SetParametersResult(successful=True)
    node.add_on_set_parameters_callback(validate_parameters)
    latest = None
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)
    publisher = node.create_publisher(PointCloud2, args.output, qos)
    if node.resolve_topic_name(args.input) == node.resolve_topic_name(args.output):
        node.destroy_node(); rclpy.shutdown()
        parser.error('Resolved input and output must be different topics')

    def receive(serialized):
        nonlocal latest
        latest = (serialized, time.monotonic())

    def tick():
        nonlocal latest
        sample, latest = latest, None
        if sample is not None and time.monotonic()-sample[1] <= args.max_age:
            stride = node.get_parameter('point_stride').value
            if stride == 1:
                publisher.publish(sample[0])
            else:
                try:
                    publisher.publish(decimate_cloud(
                        deserialize_message(sample[0], PointCloud2), stride))
                except ValueError as error:
                    node.get_logger().warning(f'Cannot sample preview: {error}',
                                              throttle_duration_sec=10.)

    subscription = node.create_subscription(PointCloud2, args.input, receive, qos, raw=True)
    timer = node.create_timer(1./args.rate, tick)
    node.get_logger().info(
        f'Display preview: {args.input} -> {args.output}, max {args.rate:g} Hz; '
        f'latest sample only, original timestamps; point_stride={args.point_stride}')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
