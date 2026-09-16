#pragma

#include "joint.hpp"
#include "AngularAcceleration.hpp"

class SafetyLayer
{
    public:
        bool validate(const Joint& joint, AngularAcceleration requested) const;

};