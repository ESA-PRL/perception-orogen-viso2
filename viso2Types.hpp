#ifndef viso2_TYPES_HPP
#define viso2_TYPES_HPP

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

#include <base/Time.hpp>
#include <base/Eigen.hpp>
#include <viso2/matcher.h>
#include <viso2/viso.h>

namespace viso2 {

    /** Currently oroGen does not support class inheritance in ports/property definitions **/
    /** This is a plain struct of the viso2 parameters which have inheritance **/
    /** Have a look at viso2/viso_stereo.h viso2/viso.h and viso2/matcher.h **/

    struct StereoOdometerParameters
    {
        //double  base;           // baseline (meters) COMMENTED it will take the baseline from FrameHelper calibration parameters
        int32_t ransac_iters;     // number of RANSAC iterations
        double  inlier_threshold; // fundamental matrix inlier threshold
        bool    reweighting;      // lower border weights (more robust to calibration errors)
        Matcher::parameters         match;            // matching parameters
        VisualOdometry::bucketing   bucket;           // bucketing parameters

        StereoOdometerParameters ()
        {
          ransac_iters     = 200;
          inlier_threshold = 1.5;
          reweighting      = true;

        }
    };

    struct Viso2Info
    {
        base::Time time;
        double num_matches;
        double num_inliers;
        double ratio_inliers;
        base::Time compute_time;
    };


}

#endif

