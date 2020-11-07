#ifndef QUADRUPED_DESCRIPTION_H
#define QUADRUPED_DESCRIPTION_H

#include <quadruped_base/quadruped_base.h>

namespace champ
{
    namespace URDF
    {
        void loadFromHeader(champ::QuadrupedBase &base)
        {
      base.lf.hip.setOrigin(0.258, 0.04832, -0.0115, 0.0, 0.0, 0.0);
base.lf.upper_leg.setOrigin(0.0, 0.10214, -0.0, 0.0, 0.0, 0.0);
base.lf.lower_leg.setOrigin(0.0161607423811, -0.000530000000205, -0.228483810585, 0.0, 0.0, 0.0);
     base.lf.foot.setOrigin(0.033866559203, 0.000299999999809, -0.226407593336, 0.0, 0.0, 0.0);

      base.rf.hip.setOrigin(0.258, -0.04832, -0.0115, 0.0, 0.0, 0.0);
base.rf.upper_leg.setOrigin(0.0, -0.10214, -0.0, 0.0, 0.0, 0.0);
base.rf.lower_leg.setOrigin(0.016160742381, 0.00053, -0.228483810585, 0.0, 0.0, 0.0);
     base.rf.foot.setOrigin(0.0338546104548, -0.000299999999977, -0.226407593336, 0.0, 0.0, 0.0);

      base.lh.hip.setOrigin(-0.258, 0.04832, -0.0115, 0.0, 0.0, 0.0);
base.lh.upper_leg.setOrigin(0.0, 0.102139998505, -0.0, 0.0, 0.0, 0.0);
base.lh.lower_leg.setOrigin(0.016160742381, -0.00053, -0.228483810585, 0.0, 0.0, 0.0);
     base.lh.foot.setOrigin(0.0338665592031, 0.000300000000012, -0.226407593336, 0.0, 0.0, 0.0);

      base.rh.hip.setOrigin(-0.258, -0.04832, -0.0115, 0.0, 0.0, 0.0);
base.rh.upper_leg.setOrigin(0.0, -0.0898, -0.0, 0.0, 0.0, 0.0);
base.rh.lower_leg.setOrigin(0.016160742381, -0.0118099985047, -0.228483810585, 0.0, 0.0, 0.0);
     base.rh.foot.setOrigin(0.0338546104548, -0.000299999999976, -0.226407593336, 0.0, 0.0, 0.0);
        }
    }
}
#endif