"""Sensor consistency and calibration observability; never calibrate from appearance."""
import numpy as np
from scipy.spatial.transform import Rotation


def check_imu(args, reader, wheel_times, wheel_poses):
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Imu
    r = reader(args.bag, [args.imu_topic])
    stationary, gyro = [], []
    first = None
    for_topic = any(t.topic_metadata.name == args.imu_topic and t.message_count
                    for t in r.get_metadata().topics_with_message_count)
    if not for_topic:
        return {'available': False, 'absolute_attitude_usable': False, 'warnings': []}
    while r.has_next():
        _, data, _ = r.read_next()
        msg = deserialize_message(data, Imu)
        t = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if first is None:
            first = t
        g = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        a = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        if np.isfinite(g).all():
            gyro.append(g)
        k = int(np.searchsorted(wheel_times, t))
        if (t-first > 30 or np.linalg.norm(g) > .02 or not 0 < k < len(wheel_times)
                or max(t-wheel_times[k-1], wheel_times[k]-t) > args.max_sync_gap):
            continue
        dt = wheel_times[k]-wheel_times[k-1]
        speed = np.linalg.norm(wheel_poses[k][:3, 3]-wheel_poses[k-1][:3, 3])/dt
        q = msg.orientation
        if speed > .02 or msg.orientation_covariance[0] < 0 or not .5 < np.linalg.norm([q.x, q.y, q.z, q.w]) < 1.5:
            continue
        if not np.isfinite(a).all() or not 5. < np.linalg.norm(a) < 15.:
            continue
        up = Rotation.from_quat([q.x, q.y, q.z, q.w]).inv().apply([0, 0, 1.])
        stationary.append((g, a, up))
    report = {'available': True, 'stationary_samples': len(stationary), 'warnings': [],
              'absolute_attitude_usable': False, 'calibration_applied': False}
    if len(stationary) >= 20:
        samples = np.asarray(stationary)
        angles = np.degrees(np.arccos(np.clip(np.sum(samples[:, 1]*samples[:, 2], axis=1)
                                             /np.linalg.norm(samples[:, 1], axis=1), -1., 1.)))
        angle = float(np.median(angles))
        report['absolute_attitude_usable'] = angle <= 2.
        report.update(orientation_acceleration_angle_deg=angle,
                      stationary_acceleration_m_s2=np.median(samples[:, 1], axis=0).tolist(),
                      quaternion_gravity_direction=np.median(samples[:, 2], axis=0).tolist())
        if angle > 2.:
            report['absolute_attitude_usable'] = False
            report['warnings'].append('Stationary IMU quaternion and measured gravity disagree by more than 2 degrees; absolute map vertical is not validated')
    else:
        report['attitude_validation'] = 'insufficient stationary data; quaternion consistency unverified'
    if len(gyro) >= 20:
        g = np.asarray(gyro)
        g = g[np.linalg.norm(g, axis=1) > .05]
        if len(g) >= 20:
            values, vectors = np.linalg.eigh(g.T@g/len(g))
            rank = int(np.sum(values > .01*values[-1]))
            axis = vectors[:, -1]
            axis *= np.sign(axis[2]) or 1.
            report.update(rotation_excitation_rank=rank, rotation_energy_eigenvalues=values.tolist(),
                          dominant_gyro_axis=axis.tolist(),
                          full_rotation_calibration_observable=rank >= 2)
            if rank < 2:
                report['calibration_observability'] = 'Mostly one-axis rotation: full LiDAR/IMU rotation and bias calibration cannot be uniquely inferred'
    return report
