// read_params: mbInertial from params.sensor (and System.thFarPoints to the mapper), to the key `inertial` (the tree's branches) and the local mapper (the
// leaves).
#include "fw/fw.hpp"
#include "params/orb_tracking_params.hpp"
#include "types/local_mapping/local_mapper.hpp"

namespace {

fw::bt::Status read_params(const OrbTrackingParams& params, fw::Write<local_mapping::LocalMapper> mapper,
                           bool& inertial) {
    inertial = params.sensor == "stereo_inertial";
    mapper->inertial = inertial;
    mapper->far_points = params.th_far_points > 0;  // mbFarPoints, mThFarPoints
    mapper->th_far_points = float(params.th_far_points);
    return fw::bt::Status::SUCCESS;
}

}  // namespace

FW_REGISTER_BT_ACTION(read_params, INPUTS(("params", OrbTrackingParams, latest)),
                      RESOURCES(("local_mapper", local_mapping::LocalMapper, write)), OUTPUTS(("inertial", bool)));
