package worldstatestore

import (
	commonpb "go.viam.com/api/common/v1"
	pb "go.viam.com/api/service/worldstatestore/v1"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/services/worldstatestore"
	"go.viam.com/rdk/spatialmath"
)

// diffTransforms compares two snapshots of the pose-tracker's frames and returns the
// set of TransformChanges needed to bring a client that has `prev` up to `cur`:
// frames present only in cur are ADDED, frames present only in prev are REMOVED, and
// frames present in both whose pose or parent changed are UPDATED. Unchanged frames
// produce no change.
func diffTransforms(prev, cur referenceframe.FrameSystemPoses) []worldstatestore.TransformChange {
	var changes []worldstatestore.TransformChange

	for name, pif := range cur {
		prevPIF, ok := prev[name]
		if !ok {
			changes = append(changes, worldstatestore.TransformChange{
				ChangeType: pb.TransformChangeType_TRANSFORM_CHANGE_TYPE_ADDED,
				Transform:  frameToTransform(name, pif),
			})
			continue
		}
		if prevPIF.Parent() != pif.Parent() || !spatialmath.PoseAlmostEqual(prevPIF.Pose(), pif.Pose()) {
			changes = append(changes, worldstatestore.TransformChange{
				ChangeType:    pb.TransformChangeType_TRANSFORM_CHANGE_TYPE_UPDATED,
				Transform:     frameToTransform(name, pif),
				UpdatedFields: []string{"pose_in_observer_frame"},
			})
		}
	}

	for name := range prev {
		if _, ok := cur[name]; !ok {
			changes = append(changes, worldstatestore.TransformChange{
				ChangeType: pb.TransformChangeType_TRANSFORM_CHANGE_TYPE_REMOVED,
				Transform: &commonpb.Transform{
					ReferenceFrame: name,
					Uuid:           uuidForName(name),
				},
			})
		}
	}

	return changes
}
