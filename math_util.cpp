#include "math_util.h"

btQuaternion MathUtil::rot_between(const btVector3& src, const btVector3& dest) {
    if ((src.x() == 0.0 &&
         src.y() == 0.0 &&
         src.z() == 0.0) ||
        (dest.x() == 0.0 &&
         dest.y() == 0.0 &&
         dest.z() == 0.0)) {
        return btQuaternion(0.0, 0.0, 0.0, 1.0);
    }
    
    btVector3 axis = src.cross(dest);
    btScalar angle = src.angle(dest);

    return btQuaternion(axis, angle);
}

btVector3 MathUtil::find_head_axis(const btVector3& face, const btVector3& up,  const btScalar& tilt) {
    if (face == up) {
        return btVector3(1.0, 0.0, 0.0);
    }
    
    btScalar factor = face.dot(up) / face.dot(face);
    btVector3 no_tilt_axis =  (up - factor * face).normalize();
    
    // add tilt
    btVector3 tilt_axis =  no_tilt_axis.rotate(face, tilt);
    
    // point up
    return tilt_axis.dot(up) > 0 ? tilt_axis : -tilt_axis;
}
