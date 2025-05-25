/*
 * quaternion.c
 *
 *  Created on: Apr 4, 2025
 *      Author: filip
 */


//class Quaternion:
//    """
//        Quaternion representation.
//        If no parameters are passed to the constructor it becomes the unit quaternion.
//    """
//
//    def __init__(self, w=1, x=0, y=0, z=0):
//        self.w = w
//        self.x = x
//        self.y = y
//        self.z = z
//    
//    def __repr__(self):
//        return f"{self.w}, {self.x}, {self.y}, {self.z}"
//
//
//def deg2rad(deg) -> float:
//    """
//        Converts degrees to radians
//    """
//    return deg * np.pi/180.0
//
//def rad2deg(rad) -> float:
//    """
//        Converts radians to degrees
//    """
//    return rad * 180.0/np.pi
//
//def quaternion_hamilton_product(a:Quaternion, b:Quaternion) -> Quaternion:
//    """
//        Calculates Hamilton product of two quaternions.
//        Rotates one quaternion by the other.
//    """
//    q = Quaternion()
//    q.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z
//    q.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y
//    q.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x
//    q.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
//
//    return q
//
//def quaterion2euler(q:Quaternion) -> tuple[float, float, float]:
//    """
//        Calculates Euler angles in radians from the provided quaternion rotation.
//    """
//    sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
//    cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
//    roll = np.atan2(sinr_cosp, cosr_cosp)
//
//    sinp = np.sqrt(1 + 2 * (q.w * q.y - q.x * q.z))
//    cosp = np.sqrt(1 - 2 * (q.w * q.y - q.x * q.z))
//    pitch = 2 * np.atan2(sinp, cosp) - np.pi / 2
//
//    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
//    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
//    yaw = np.atan2(siny_cosp, cosy_cosp)
//
//    return roll, pitch, yaw